#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#include "stdint.h"
#include "interface_bmp390l.h"

typedef struct {
    uint32_t n;
    int64_t timestamp;
    float bat_voltage;
} flash_packet;

uint32_t flash_get_addr();

void flash_flight_init(void);

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