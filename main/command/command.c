#include "command.h"
#include "goober.h"
#include "nvs_interface.h"
#include "flash_interface.h"
#include "ble.h"
#include "onboard_led.h"

_Atomic bool TXLOCK = false;

static uint8_t command_packet_buf[250];

esp_err_t enable_txlock(void)
{
    int ret;

    led_purple();

    ret = flash_prepare_for_flight();

    ble_stop();

    if (!ret) return ESP_FAIL;

    led_green();

    return ESP_OK;
}

esp_err_t process_command(uint8_t msg_class)
{
    switch(msg_class) {
        case(PING): {
            break;
        }
        case(PONG): {
            break;
        }
        case(ACK): {
            break;
        }
        case(NAK): {
            break;
        }
        case(REBOOT): {
            esp_restart();
            break; // this would never be reached...
        }
        case(PYRO1): {
            break;
        }
        case(PYRO2): {
            break;
        }
        case(PYRO3): {
            break;
        }
        case(PYRO4): {
            break;
        }
        case(AUX_ON): {
            break;
        }
        case(AUX_OFF): {
            break;
        }
        case(SLEEP): {
            break;
        }
        case(WAKE_UP): {
            break;
        }
        case(TXLOCK_ON): {
            enable_txlock();
            break;
        }
        case(TXLOCK_OFF): {
            break;
        }
        case(DUMP_CONFIG): {
            break;
        }
        case(EDIT_CONFIG): {
            break;
        }
        case(DUMP_RADIO): {
            break;
        }
        case(EDIT_RADIO): {
            break;
        }
        case(SIMULATOR_ON): {
            break;
        }
        case(VOLTAGE): {
            break;
        }
        case(TELEMETRY): {
            break;
        }
        case(ERASE_FLASH): {
            flash_blank_slate();
            break;
        }
        default: {
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    return ESP_OK;
}

bool is_tx_lock()
{
    return atomic_load(&TXLOCK);
}

uint8_t next_sequence_id()
{
    static uint8_t seq_counter = 0x00;
    
    if (seq_counter == 0xFF) {
		seq_counter = 0x00;
        return seq_counter;
	}

    seq_counter++;

    return seq_counter;
}

esp_err_t single_byte_response(uint16_t resp_msg_cls, uint8_t response_payload)
{
    int ret;
    uint8_t resp_payload_buf = 0;
    size_t serialized_packet_length = 0;

    uint8_t transmission_mode = 1;
    goober_header_t response_header;

    board_information_t board_info;
    nvs_retrieve_board_info(&board_info);
    
    response_header.dev_id = board_info.serial_number;
    response_header.dev_mode = goober_device_mode(transmission_mode, false, false, false, false);
    response_header.msg_cls = resp_msg_cls;
    response_header.seq_id = next_sequence_id();

    memset(&resp_payload_buf, response_payload, 1); // i know this is overkill and stupid for one byte payload

    ret = goober_serialize(response_header, &resp_payload_buf, sizeof(resp_payload_buf), command_packet_buf, sizeof(command_packet_buf), &serialized_packet_length);
    if(ret) return ret;

    // todo: send these bytes over UART

    return ESP_OK;
}