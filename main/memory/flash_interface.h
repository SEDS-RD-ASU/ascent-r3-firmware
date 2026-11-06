#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#include "stdint.h"
#include "sensor_manager.h"

typedef struct {
    uint64_t n;
    int64_t timestamp; // return from esp_timer_get_time
    float bat_voltage;
    uint8_t flight_state;
    uint8_t pyro_cont;

    //barometer_sample_t
    double pressure;
    double temperature;
    double altitude_agl;
    double ground_altitude;
    //barometer_velocity_t
    double baro_vel;
    double avg_baro_vel;

    //gps_sample_t
    uint32_t UTCtstamp;
    uint32_t lat;
    uint32_t lon;
    uint32_t altitude_ellipsoid;
    uint32_t altitude_msl;
    uint8_t fixType;
    uint8_t num_sats;

    //acc_sample_t
    float acc_x;
    float acc_y;
    float acc_z;

    //acc_sample_t
    float hacc_x;
    float hacc_y;
    float hacc_z;

    //gyr_sample_t
    float gyr_x;
    float gyr_y;
    float gyr_z;

} flash_packet;

uint32_t flash_get_addr();

esp_err_t flash_flight_init(void);

bool flash_erase_next_bank_no_advance(int64_t max_time, int32_t* resume);

bool flash_prepare_for_flight(void);

void flash_dump_to_serial(int bank);

void flash_write_packet(flash_packet *packet);

void flash_queue_packet(flash_packet *packet);

void flash_write_queue(int64_t max_time);

int32_t flash_get_last_used_bank();

void flash_print_stats();

void flash_blank_slate();

void try_to_dump_data();

void flash_erase_jingle(void);

void print_flash_packet(flash_packet *fp);

#endif