#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#include "stdint.h"
#include "interface_bmp390l.h"

/**
 * @brief Packet structure persisted to external flash during flight.
 */
typedef struct {
    uint32_t n;          /**< Sequential packet number. */
    int64_t timestamp;   /**< Timestamp associated with the packet. */
    float bat_voltage;   /**< Battery voltage reading at the time of capture. */
} flash_packet;

/**
 * @brief Get the current flash write address.
 */
uint32_t flash_get_addr();

/**
 * @brief Initialize flash storage and bookkeeping for flight logging.
 */
void flash_flight_init(void);

/**
 * @brief Erase the next flash bank without advancing the active bank pointer.
 *
 * @param max_time Maximum time allowed for the erase operation (microseconds).
 * @param resume Pointer to progress marker for resuming erasure.
 * @return true if the bank was erased successfully, false otherwise.
 */
bool flash_erase_next_bank_no_advance(int64_t max_time, int32_t* resume);

/**
 * @brief Prepare flash storage for a new flight by erasing the active bank.
 *
 * @return true if preparation succeeded, false otherwise.
 */
bool flash_prepare_for_flight(void);

/**
 * @brief Dump the contents of a bank to the serial console.
 *
 * @param bank Bank index to print.
 */
void flash_dump_to_serial(int bank);

/**
 * @brief Write a packet immediately to flash.
 *
 * @param packet Packet to persist.
 */
void flash_write_packet(flash_packet *packet);

/**
 * @brief Queue a packet for deferred writing to flash.
 *
 * @param packet Packet to enqueue.
 */
void flash_queue_packet(flash_packet *packet);

/**
 * @brief Flush queued packets to flash until the time budget expires.
 *
 * @param max_time Maximum time allowed for writing (microseconds).
 */
void flash_write_queue(int64_t max_time);

/**
 * @brief Retrieve the index of the last bank that contained flight data.
 *
 * @return int32_t Bank index.
 */
int32_t flash_get_last_used_bank();

/**
 * @brief Print statistics about flash usage to the console.
 */
void flash_print_stats();

/**
 * @brief Reset flash bookkeeping and erase state.
 */
void flash_blank_slate();

/**
 * @brief Attempt to automatically dump buffered flight data.
 */
void try_to_dump_data();

/**
 * @brief Play an audible jingle indicating a flash erase is in progress.
 */
void flash_erase_jingle(void);

/**
 * @brief Print a human-readable representation of a flash packet.
 */
void print_flash_packet(flash_packet *fp);

#endif
