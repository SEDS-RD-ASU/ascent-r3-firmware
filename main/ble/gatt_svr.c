/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "bleprph.h"
#include "ble.h"
#include "goober.h"
#include "command.h"
#include "driver_psu.h"

static const ble_uuid16_t gatt_svr_svc_uuid =
    BLE_UUID16_INIT(0x0000);

/* Device Information Service UUID */
static const ble_uuid16_t device_info_svc_uuid =
    BLE_UUID16_INIT(0x180A);

/* Device Information Service Characteristics */
static const ble_uuid16_t manufacturer_name_uuid =
    BLE_UUID16_INIT(0x2A29);
static const ble_uuid16_t model_number_uuid =
    BLE_UUID16_INIT(0x2A24);
static const ble_uuid16_t firmware_revision_uuid =
    BLE_UUID16_INIT(0x2A26);

/* A characteristic that can be read/written */
static uint8_t gatt_svr_chr_val;
static uint16_t gatt_svr_chr_val_handle;
static const ble_uuid16_t gatt_svr_chr_uuid =
    BLE_UUID16_INIT(0x0001);

/* Device Information strings */
static const char *manufacturer_name = "SEDS-RD-ASU";
static const char *model_number = "SN00x";
static const char *firmware_revision = "1.0.0";


static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt,
                void *arg);

static int device_info_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Custom Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                .uuid = &gatt_svr_chr_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .val_handle = &gatt_svr_chr_val_handle,
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        /*** Device Information Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &device_info_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Manufacturer Name String */
                .uuid = &manufacturer_name_uuid.u,
                .access_cb = device_info_access,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                /* Model Number String */
                .uuid = &model_number_uuid.u,
                .access_cb = device_info_access,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                /* Firmware Revision String */
                .uuid = &firmware_revision_uuid.u,
                .access_cb = device_info_access,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        0, /* No more services. */
    },
};

static int gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

/**
 * Access callback for Device Information Service characteristics.
 */
static int device_info_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    int rc;
    
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        switch (uuid) {
        case 0x2A29: /* Manufacturer Name String */
            rc = os_mbuf_append(ctxt->om, manufacturer_name, strlen(manufacturer_name));
            return rc == 0 ? 0 : BLE_ATT_ERR_UNLIKELY;
            
        case 0x2A24: /* Model Number String */
            rc = os_mbuf_append(ctxt->om, model_number, strlen(model_number));
            return rc == 0 ? 0 : BLE_ATT_ERR_UNLIKELY;
            
        case 0x2A26: /* Firmware Revision String */
            rc = os_mbuf_append(ctxt->om, firmware_revision, strlen(firmware_revision));
            return rc == 0 ? 0 : BLE_ATT_ERR_UNLIKELY;
            
        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
        }

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Access callback whenever a characteristic is read or written to.
 */
static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        // Removed logging here to prevent UART blocking during BLE operations
        // MODLOG_DFLT(INFO, "Characteristic read; conn_handle=%d attr_handle=%d\n",
        //             conn_handle, attr_handle);
        if (attr_handle == gatt_svr_chr_val_handle) {
            
            // Append the serialized data to the output mbuf
            uint8_t voltage = (uint8_t)(50.0f * psu_read_battery_voltage());
            rc = os_mbuf_append(ctxt->om, &voltage, 1);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "Failed to append data to mbuf; rc=%d\n", rc);
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            // Removed logging here to prevent UART blocking during BLE operations
            // MODLOG_DFLT(INFO, "Sending goober packet response (size=%d)\n", serialized_buffer_length);
            return 0;
        }
        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // Removed logging here to prevent UART blocking during BLE operations
        // MODLOG_DFLT(INFO, "Characteristic write; conn_handle=%d attr_handle=%d\n",
        //             conn_handle, attr_handle);
        if (attr_handle == gatt_svr_chr_val_handle) {
            uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
            uint8_t rx_buffer[256];
            uint16_t len;
            int rc;
            
            // Read the incoming data from the mbuf
            if (om_len > sizeof(rx_buffer)) {
                MODLOG_DFLT(ERROR, "Write data too large (%d bytes)\n", om_len);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            
            rc = ble_hs_mbuf_to_flat(ctxt->om, rx_buffer, sizeof(rx_buffer), &len);
            if (rc != 0) {
                MODLOG_DFLT(ERROR, "Failed to read write data; rc=%d\n", rc);
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            printf("Received BLE packet: ");
            for(int i = 0; i < len; i++) {
                printf("0x%02X ", rx_buffer[i]);
            }
            printf("\n");

            process_command(rx_buffer[0]);
            
            return 0;
        }
        break;

    default:
        break;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    default:
        assert(0);
        break;
    }
}

int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}