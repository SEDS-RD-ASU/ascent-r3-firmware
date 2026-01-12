/**
 * @file sensor_manager.c
 * @author Abdul Zia
 * @brief Code responssible for processing sensor data during flight.
 */

#include "sensor_manager.h"

static const char *TAG = "SENSOR MANAGER";

static bool simulator = false; // simulating flight?


// MARK: SENSOR INITIALIZATION
esp_err_t initialize_sensors(void)
{
    esp_err_t ret;
    i2c_port_t BMP390_I2C_PORT = R3_I2C1_PORT;
    i2c_port_t SAM_M10Q_I2C_PORT = R3_I2C0_PORT;

    #ifdef R2
    ESP_LOGW(TAG, "YOU ARE USING R2 PINS FOR SENSORS. REMOVE #define R2 IF THIS IS NOT A R2 BOARD.");
    BMP390_I2C_PORT = R2_I2C0_PORT;
    SAM_M10Q_I2C_PORT = R2_I2C0_PORT;
    #endif
    
    // ret = bmp390_flight_init(BMP390_I2C_PORT);
    // if(ret != ESP_OK) {ESP_LOGE(TAG, "FAILED TO INITIALIZE BMP390"); return ret;}

    // vTaskDelay(pdMS_TO_TICKS(1000));
    
    // ret = GPS_init(SAM_M10Q_I2C_PORT);
    // if(ret != ESP_OK) {ESP_LOGE(TAG, "FAILED TO INITIALIZE SAM-M10Q"); return ret;}

    ret = lsm_flight_init(SPI2_HOST);
    if(ret != ESP_OK) {ESP_LOGE(TAG, "FAILED TO INITIALIZE LSM"); return ret;}

    ESP_LOGI(TAG, "SUCCESSFULLY INITIALIZED ALL SENSORS");
    return ESP_OK;
}

// MARK: SENSOR POLLING
esp_err_t poll_barometer(barometer_sample_t *baro)
{
    if(simulator) {
        ;
    }
    else { 
        baro_double_t bmp390_out;
        bmp390_get_local(&bmp390_out);
        baro->timestamp = esp_timer_get_time();
        baro->pressure = bmp390_out.pressure;
        baro->temperature = bmp390_out.temperature;
        baro->altitude_agl = bmp390_out.alt;
        baro->ground_altitude = bmp390_ground_altitude();
    }

    return ESP_OK;
}

esp_err_t poll_gps(gps_sample_t *gps)
{
    if(simulator) {
        ;
    }
    else {
        GPS_data_t sam_m10q_data;
        GPS_read(&sam_m10q_data);
        gps->timestamp = esp_timer_get_time();
        gps->UTCtstamp = sam_m10q_data.UTCtstamp;
        gps->lat = sam_m10q_data.lat;
        gps->lon = sam_m10q_data.lon;
        gps->altitude_ellipsoid = sam_m10q_data.height;
        gps->altitude_msl = sam_m10q_data.hMSL;
        gps->fixType = sam_m10q_data.fixType;
        gps->num_sats = sam_m10q_data.numSV;
    }

    return ESP_OK;
}

// MARK: BAROMETRIC VELOCITY CALCULATION
#define HISTORY_SIZE 3
#define VELOCITY_HISTORY_SIZE 10
#define DT 0.01f

static float barometric_agl;
static float barometric_velocity;
static float average_barometric_velocity;

void baro_update(barometer_sample_t baro, float *agl, float *vel, float *avg_vel)
{
    static float agl_history[HISTORY_SIZE] = {0};  // Store the last 5 AGL readings

    static float velocity_samples[VELOCITY_HISTORY_SIZE] = {0};
    static int sample_index = 0;

    // Shift history
    for (int i = HISTORY_SIZE - 1; i > 0; i--) {
        agl_history[i] = agl_history[i - 1];
    }

    // Update with latest AGL
    agl_history[0] = baro.altitude_agl;
    barometric_agl = agl_history[0];

    // Compute first-order backward finite difference
    if (agl_history[1] != 0) {
        barometric_velocity = barometric_velocity*0.2 + ((agl_history[0] - agl_history[1]) / DT)*0.8;
    }

    // Update velocity samples
    velocity_samples[sample_index] = barometric_velocity;
    sample_index = (sample_index + 1) % VELOCITY_HISTORY_SIZE;

    float sum = 0;
    // Calculate the sum of all samples
    for (int i = 0; i < 10; i++) {
        sum += velocity_samples[i];
    }

    // Calculate the average velocity
    average_barometric_velocity = sum / 10.0;

    // printf("AGL: %f, Pressure: %f, Velocity: %f\n", agl_history[0], baro.pressure, barometric_velocity);

    *agl = barometric_agl;
    *vel = barometric_velocity;
    *avg_vel = average_barometric_velocity;
}