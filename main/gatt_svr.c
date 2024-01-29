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
#include "services/ans/ble_svc_ans.h"
#include "driver/uart.h"
/**
 * The vendor specific security test service consists of two characteristics:
 *     o random-number-generator: generates a random 32-bit number each time
 *       it is read.  This characteristic can only be read over an encrypted
 *       connection.
 *     o static-value: a single-byte characteristic that can always be read,
 *       but can only be written over an encrypted connection.
 */
/* add from server file */
uint16_t ble_svc_gatt_read_val_handle, ble_spp_svc_gatt_read_val_handle, ble_spp_svc_gatt_write_val_handle;
extern const char *master_tag;
extern char recvbuf[129];
extern char tranbuf[129];
extern uint16_t numberOfByteToUart;

/*end*/

/* 59462f12-9543-9999-12c8-58b459a2712d */
static const ble_uuid128_t gatt_svr_svc_sec_test_uuid =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                     0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);
/* 5c3a659e-897e-45e1-b016-007107c96df6 */
static const ble_uuid128_t gatt_svr_chr_sec_test_rand_uuid =
    BLE_UUID128_INIT(0xf6, 0x6d, 0xc9, 0x07, 0x71, 0x00, 0x16, 0xb0,
                     0xe1, 0x45, 0x7e, 0x89, 0x9e, 0x65, 0x3a, 0x5c);

/* 5c3a659e-897e-45e1-b016-007107c96df7 */
static const ble_uuid128_t gatt_svr_chr_sec_test_static_uuid =
    BLE_UUID128_INIT(0xf7, 0x6d, 0xc9, 0x07, 0x71, 0x00, 0x16, 0xb0,
                     0xe1, 0x45, 0x7e, 0x89, 0x9e, 0x65, 0x3a, 0x5c);

//static const ble_uuid_t * service1 =
//		BLE_UUID128_DECLARE(0x90, 0x89, 0x81, 0x40, 0xec, 0xc4, 0x96, 0xb7, 0xed, 0x45, 0x63, 0xcc, 0x1d, 0x46, 0xdb, 0xf9);
//static const ble_uuid_t * char1 =
//		BLE_UUID128_DECLARE(0x79, 0x24, 0x39, 0xf7, 0x54, 0xe5, 0xce, 0x8a, 0x44, 0x4a, 0xf6, 0xe0, 0xf3, 0x90, 0x22, 0x97);
//static const ble_uuid_t * char2 =
//		BLE_UUID128_DECLARE(0xe2, 0xa2, 0xe5, 0x9c, 0x72, 0x24, 0xa7, 0x99, 0xc6, 0x46, 0x50, 0x69, 0x11, 0x65, 0xa7, 0xb8);
static uint8_t gatt_svr_sec_test_static_val;
//extern const ble_uuid_t * service1;
//extern const ble_uuid_t * char1;
//extern const ble_uuid_t * char2;
ble_uuid128_t service1 =
    {
        .u.type = BLE_UUID_TYPE_128,
        .value = {0x90, 0x89, 0x81, 0x40, 0xec, 0xc4, 0x96, 0xb7, 0xed, 0x45, 0x63, 0xcc, 0x1d, 0x46, 0xdb, 0xf9}};
ble_uuid128_t char1 =
    {
        .u.type = BLE_UUID_TYPE_128,
        .value = {0x79, 0x24, 0x39, 0xf7, 0x54, 0xe5, 0xce, 0x8a, 0x44, 0x4a, 0xf6, 0xe0, 0xf3, 0x90, 0x22, 0x97}};
ble_uuid128_t char2 =
    {
        .u.type = BLE_UUID_TYPE_128,
        .value = {0xe2, 0xa2, 0xe5, 0x9c, 0x72, 0x24, 0xa7, 0x99, 0xc6, 0x46, 0x50, 0x69, 0x11, 0x65, 0xa7, 0xb8}};


//static uint8_t gatt_svr_sec_test_static_val;

static int
gatt_svr_chr_access_sec_test(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);
/* add from server */
static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
//	struct os_mbuf *om;
//	memcpy(recvbuf,0,strlen(recvbuf));
    switch (ctxt->op)
    {
    /*don't enter to BLE_GATT_ACCESS_OP_READ_CHR*/
    case BLE_GATT_ACCESS_OP_READ_CHR:
//        ESP_LOGI(tag, "Callback for read");
//    	ESP_LOGI(master_tag, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
//        struct os_mbuf *txom;
//        txom = ble_hs_mbuf_from_flat(recvbuf, strlen(recvbuf));
//        txom = ble_hs_mbuf_from_flat(&num, sizeof(num));
//        memset(recvbuf, 0, sizeof(recvbuf));

        /********************************************************/
//        int rc = ble_gattc_notify_custom(conn_handle, ble_spp_svc_gatt_read_val_handle, txom);
//        if (rc == 0)
//        {
//            ESP_LOGI(master_tag, "Notification sent successfully");
//            ESP_LOGI(master_tag,"Received: %s\n", recvbuf);
//        }
//        else
//        {
//            ESP_LOGI(master_tag, "Error in sending notification");
//        }
//        os_mbuf_append(ctxt->om, pDataTransmitUart,
//                                        sizeof(pDataTransmitUart));
//        om = ble_hs_mbuf_from_flat(pDataTransmitUart, 10);
//
//        /*send data available in pDataTransmitUart which I already received by SPI from STM32
//         *
//         */
////        int rc = os_mbuf_append(ctxt->om, pDataTransmitUart, 10);
//        struct os_mbuf *txom;
//        txom = ble_hs_mbuf_from_flat(pDataReciveedUart, len);
//        memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
//
//        /********************************************************/
//        rc = ble_gattc_notify_custom(connection_handle, ble_spp_svc_gatt_read_val_handle, txom);
//        if (rc == 0)
//        {
//            ESP_LOGI(tag, "Notification sent successfully");
//        }
//        else
//        {
//            ESP_LOGI(tag, "Error in sending notification");
//        }

        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(master_tag, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
        /***************************************/
        ble_hs_mbuf_to_flat(ctxt->om, tranbuf/*pDataTransmitUart*/, ctxt->om->om_len/*10*/, &numberOfByteToUart);
        ESP_LOGI(master_tag,"tranbuf = %s",tranbuf);
        uint8_t rc = 1;
        uart_write_bytes(0, &rc, 1);
        /*send data by SPI to STM32

        */


//        uint8_t len = uart_write_bytes(UART_PORT_x, (const char *)pDataTransmitUart, (size_t)numberOfByteToUart);
//
//        for (int i = 0; i < numberOfByteToUart; i++)
//        {
//            ESP_LOGI(tag, "datarecved: %d , length recived BLE=%d ,length transmit UART", pDataTransmitUart[i], numberOfByteToUart, len);
//        }
//        memset(pDataTransmitUart, 0, sizeof(pDataTransmitUart));

        /***************************************/
        break;

    default:
        ESP_LOGI(master_tag, "\nDefault Callback");
        break;
    }
    return 0;
}

/* Define new custom service */
const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
//        /*** Service: GATT */
//        .type = BLE_GATT_SVC_TYPE_PRIMARY,
//        .uuid = BLE_UUID16_DECLARE(BLE_SVC_ANS_UUID16),
//        .characteristics = (struct ble_gatt_chr_def[]){{
//                                                           /* Support new alert category */
//                                                           .uuid = BLE_UUID16_DECLARE(BLE_SVC_ANS_CHR_UUID16_SUP_NEW_ALERT_CAT),
//                                                           .access_cb = ble_svc_gatt_handler,
//                                                           .val_handle = &ble_svc_gatt_read_val_handle,
//                                                           .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
//                                                       },
//                                                       {
//                                                           0, /* No more characteristics */
//                                                       }},
//    },
//    {
        /*** Service: SPP */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t *)(&service1),
        .characteristics = (struct ble_gatt_chr_def[]){{
                                                           /* Support SPP service */
                                                           // .uuid = BLE_UUID128_DECLARE(BLE_SVC_SPP_CHR_UUID128),
                                                           .uuid = (ble_uuid_t *)(&char1),
                                                           .access_cb = ble_svc_gatt_handler,
                                                           .val_handle = &ble_spp_svc_gatt_read_val_handle,
                                                           .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                                                       },
                                                       {
                                                           /* Support SPP service */
                                                           // .uuid = BLE_UUID128_DECLARE(BLE_SVC_SPP_CHR_UUID128),
                                                           .uuid = (ble_uuid_t *)(&char2),
                                                           .access_cb = ble_svc_gatt_handler,
                                                           .val_handle = &ble_spp_svc_gatt_write_val_handle,
                                                           .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                       },
                                                       {
                                                           0, /* No more characteristics */
                                                       }},
    },
    {
        0, /* No more services. */
    },
};

/* end */

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Service: Security test. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_sec_test_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /*** Characteristic: Random number generator. */
                .uuid = &gatt_svr_chr_sec_test_rand_uuid.u,
                .access_cb = gatt_svr_chr_access_sec_test,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
            }, {
                /*** Characteristic: Static value. */
                .uuid = &gatt_svr_chr_sec_test_static_uuid.u,
                .access_cb = gatt_svr_chr_access_sec_test,
                .flags = BLE_GATT_CHR_F_READ |
                BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        0, /* No more services. */
    },
};


static int
gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
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

static int
gatt_svr_chr_access_sec_test(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    const ble_uuid_t *uuid;
    int rand_num;
    int rc;

    uuid = ctxt->chr->uuid;

    /* Determine which characteristic is being accessed by examining its
     * 128-bit UUID.
     */

    if (ble_uuid_cmp(uuid, &gatt_svr_chr_sec_test_rand_uuid.u) == 0) {
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);

        /* Respond with a 32-bit random number. */
        rand_num = rand();
        rc = os_mbuf_append(ctxt->om, &rand_num, sizeof rand_num);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ble_uuid_cmp(uuid, &gatt_svr_chr_sec_test_static_uuid.u) == 0) {
        switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &gatt_svr_sec_test_static_val,
                                sizeof gatt_svr_sec_test_static_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = gatt_svr_chr_write(ctxt->om,
                                    sizeof gatt_svr_sec_test_static_val,
                                    sizeof gatt_svr_sec_test_static_val,
                                    &gatt_svr_sec_test_static_val, NULL);
            return rc;

        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    /* Unknown characteristic; the nimble stack should not have called this
     * function.
     */
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
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

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;
    ESP_LOGI(master_tag,"================hi===================");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_ans_init();

    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

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
