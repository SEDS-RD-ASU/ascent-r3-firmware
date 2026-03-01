/**
 * @file sensor_manager.c
 * @brief Code responssible for processing sensor data during flight.
 */

#include "sensor_manager.h"

static const char *TAG = "SENSOR MANAGER";

static bool simulator = false; // simulating flight?

static bmp_context_t bmp_ctx = {0};

bool barometer_data_ready = false;
bool imu_data_ready = true; // todo: implement ISR for IMU
bool gps_data_ready = true; // todo: implement ISR for GPS

// FAKE SENSOR DATA VARIABLES (FOR SOFTWARE-IN-THE-LOOP)
_Atomic barometer_sample_t sim_baro;
_Atomic barometer_velocity_t sim_baro_vel;
_Atomic acc_sample_t sim_low_g_acc;
_Atomic acc_sample_t sim_high_g_acc;
_Atomic gyr_sample_t sim_gyr;
_Atomic gps_sample_t sim_gps;

// MARK: SENSOR INITIALIZATION
esp_err_t initialize_sensors(bool is_simulator)
{
    if(is_simulator) {
        simulator = true;
        barometer_data_ready = true;
        imu_data_ready = true;
        gps_data_ready = true;
    } else {
        esp_err_t ret;
        i2c_port_t BMP390_I2C_PORT = R3_I2C1_PORT;
        i2c_port_t SAM_M10Q_I2C_PORT = R3_I2C0_PORT;
        spi_host_device_t LSM_SPI_HOST = SPI3_HOST;

        // Install ISR service
        // ret = gpio_install_isr_service(0);
        // if (ret != ESP_OK) {
        //     ESP_LOGE(TAG, "Failed to install ISR service");
        //     return ret;
        // }

        // ret = bmp390_flight_init(BMP390_I2C_PORT, barometer_int_callback, &bmp_ctx);
        // if(ret != ESP_OK) {ESP_LOGE(TAG, "FAILED TO INITIALIZE BMP390"); return ret;}

        // vTaskDelay(pdMS_TO_TICKS(1000));

        ret = GPS_init(SAM_M10Q_I2C_PORT);
        if(ret != ESP_OK) {ESP_LOGE(TAG, "FAILED TO INITIALIZE SAM-M10Q"); return ret;}

        // ret = lsm_flight_init(LSM_SPI_HOST);
        // if(ret != ESP_OK) {ESP_LOGE(TAG, "FAILED TO INITIALIZE LSM6DSV320X"); return ret;}

        ESP_LOGI(TAG, "SUCCESSFULLY INITIALIZED ALL SENSORS");
    }
    return ESP_OK;
}

void poll_baro(barometer_sample_t *pBaro)
{
    if(simulator) {
        barometer_sample_t temp_baro = atomic_load(&sim_baro);
        pBaro->timestamp = esp_timer_get_time();
        pBaro->pressure = temp_baro.pressure;
        pBaro->temperature = temp_baro.temperature;
        pBaro->altitude_agl = temp_baro.altitude_agl;
        pBaro->ground_altitude = temp_baro.ground_altitude;
    }
    else { 
        baro_double_t bmp390_out;
        bmp390_get_local(&bmp390_out);
        pBaro->timestamp = esp_timer_get_time();
        pBaro->pressure = bmp390_out.pressure;
        pBaro->temperature = bmp390_out.temperature;
        pBaro->altitude_agl = bmp390_out.alt;
        pBaro->ground_altitude = bmp390_ground_altitude();
    }
}

void barometer_int_callback(void *args)
{
    barometer_data_ready = true;
    portYIELD_FROM_ISR();
}

esp_err_t poll_gps(gps_sample_t *gps)
{
    if(simulator) {
        gps_sample_t temp_sam_m10q_data = atomic_load(&sim_gps);
        gps->timestamp = esp_timer_get_time();
        gps->UTCtstamp = temp_sam_m10q_data.UTCtstamp;
        gps->lat = temp_sam_m10q_data.lat;
        gps->lon = temp_sam_m10q_data.lon;
        gps->altitude_ellipsoid = temp_sam_m10q_data.altitude_ellipsoid;
        gps->altitude_msl = temp_sam_m10q_data.altitude_msl;
        gps->fixType = temp_sam_m10q_data.fixType;
        gps->num_sats = temp_sam_m10q_data.num_sats;
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

static esp_err_t poll_imu(acc_sample_t *high_g, acc_sample_t *low_g, gyr_sample_t *gyr)
{
    if(simulator) {
        acc_sample_t temp_low_acc_data = atomic_load(&sim_low_g_acc);
        acc_sample_t temp_high_acc_data = atomic_load(&sim_high_g_acc);
        gyr_sample_t temp_gyr_sample = atomic_load(&sim_gyr);
        int64_t timestamp = esp_timer_get_time();
        
        low_g->timestamp = timestamp;
        low_g->acc_x = temp_low_acc_data.acc_x;
        low_g->acc_y = temp_low_acc_data.acc_y;
        low_g->acc_z = temp_low_acc_data.acc_z;

        high_g->timestamp = timestamp;
        high_g->acc_x = temp_high_acc_data.acc_x;
        high_g->acc_y = temp_high_acc_data.acc_y;
        high_g->acc_z = temp_high_acc_data.acc_z;

        gyr->timestamp = timestamp;
        gyr->gyr_x = temp_gyr_sample.gyr_x;
        gyr->gyr_y = temp_gyr_sample.gyr_y;
        gyr->gyr_z = temp_gyr_sample.gyr_z;
    }
    else {
        lsm_raw_data_t raw_imu_data;
        esp_err_t ret = lsm_get_local(&raw_imu_data);
        if(ret) return ret;
        int64_t timestamp = esp_timer_get_time();

        high_g->timestamp = timestamp;
        high_g->acc_x = raw_imu_data.highacc_x;
        high_g->acc_y = raw_imu_data.highacc_y;
        high_g->acc_z = raw_imu_data.highacc_z;

        low_g->timestamp = timestamp;
        low_g->acc_x = raw_imu_data.lowacc_x;
        low_g->acc_y = raw_imu_data.lowacc_y;
        low_g->acc_z = raw_imu_data.lowacc_z;

        gyr->timestamp = timestamp;
        gyr->gyr_x = raw_imu_data.gyr_x;
        gyr->gyr_y = raw_imu_data.gyr_y;
        gyr->gyr_z = raw_imu_data.gyr_z;
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

void baro_update(barometer_sample_t baro, barometer_velocity_t *baro_vel)
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

    baro_vel->altitude_agl = barometric_agl;
    baro_vel->velocity = barometric_velocity;
    baro_vel->average_velocity = average_barometric_velocity;
}

void poll_sensors(barometer_sample_t *pBaro, barometer_velocity_t *pBaro_vel, acc_sample_t *high_g, acc_sample_t *low_g, gyr_sample_t *gyr, gps_sample_t *gps)
{
    if (barometer_data_ready) {
        poll_baro(pBaro);
        barometer_data_ready = false;
        if(simulator)
        {
            barometer_data_ready = true;
        }
    }
    if (imu_data_ready){
        poll_imu(high_g, low_g, gyr);
    }
    if (gps_data_ready){
        // poll_gps(gps);
    }
}

void feed_fake_flight_data(barometer_sample_t baro, barometer_velocity_t baro_vel, acc_sample_t high_g, acc_sample_t low_g, gyr_sample_t gyr, gps_sample_t gps)
{
    // Atomic store all these variables
    atomic_store(&sim_baro, baro);
    atomic_store(&sim_baro_vel, baro_vel);
    atomic_store(&sim_high_g_acc, high_g);
    atomic_store(&sim_low_g_acc, low_g);
    atomic_store(&sim_gyr, gyr);
    atomic_store(&sim_gps, gps);
}