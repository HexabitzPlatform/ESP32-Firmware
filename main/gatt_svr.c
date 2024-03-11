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
#include "connection_driver.h"
/*** Maximum number of characteristics with the notify flag ***/
#define MAX_NOTIFY 5
uint8_t Data[20];
uint8_t BleBuffer[20]={0};
uint8_t BleReadBuffer[20]={0};
uint16_t ble_svc_gatt_read_val_handle, ble_spp_svc_gatt_read_val_handle, ble_spp_svc_gatt_write_val_handle;
//extern const char *master_tag;
//extern char recvbuf[129];
//extern char tranbuf[129];
//extern uint16_t numberOfByteToUart;
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

static const ble_uuid128_t gatt_svr_svc_sec_test_uuid =
    BLE_UUID128_INIT(0x90, 0x89, 0x81, 0x40, 0xec, 0xc4, 0x96, 0xb7,
                     0xed, 0x45, 0x63, 0xcc, 0x1d, 0x46, 0xdb, 0xf9);

/* 5c3a659e-897e-45e1-b016-007107c96df6 */
/* b8a76511-6950-46c6-99a7-24729ce5a2e2 */
static const ble_uuid128_t gatt_svr_chr_sec_test_rand_uuid =
    BLE_UUID128_INIT(0x79, 0x24, 0x39, 0xf7, 0x54, 0xe5, 0xce, 0x8a,
            0x8a, 0x44, 0x4a, 0xe0, 0xf3, 0x90, 0x22, 0x97);

/* 5c3a659e-897e-45e1-b016-007107c96df7 */
/* 972290f3-e0f6-4a44-8ace-e554f7392479 */
static const ble_uuid128_t gatt_svr_chr_sec_test_static_uuid =
    BLE_UUID128_INIT(0xe2, 0xa2, 0xe5, 0x9c, 0x72, 0x24, 0xa7, 0x99,
            0xc6, 0x46, 0x50, 0x69, 0x11, 0x65, 0xa7, 0xb8);

//static uint8_t gatt_svr_sec_test_static_val;
//
//static const ble_uuid128_t gatt_svr_svc_uuid =
//    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
//                     0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);
//
///* A characteristic that can be subscribed to */
//static uint8_t gatt_svr_chr_val;
//static uint16_t gatt_svr_chr_val_handle;
//static const ble_uuid128_t gatt_svr_chr_uuid =
//    BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x11, 0x11,
//                     0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33);
//
///* A custom descriptor */
//static uint8_t gatt_svr_dsc_val;
//static const ble_uuid128_t gatt_svr_dsc_uuid =
//    BLE_UUID128_INIT(0x01, 0x01, 0x01, 0x01, 0x12, 0x12, 0x12, 0x12,
//                     0x23, 0x23, 0x23, 0x23, 0x34, 0x34, 0x34, 0x34);

static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt,
                void *arg);
/* add from server */
//static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
//{
////	struct os_mbuf *om;
////	memcpy(recvbuf,0,strlen(recvbuf));
//    switch (ctxt->op)
//    {
//    /*don't enter to BLE_GATT_ACCESS_OP_READ_CHR*/
//    case BLE_GATT_ACCESS_OP_READ_CHR:
////        ESP_LOGI(tag, "Callback for read");
////    	ESP_LOGI(master_tag, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
////        struct os_mbuf *txom;
////        txom = ble_hs_mbuf_from_flat(recvbuf, strlen(recvbuf));
////        txom = ble_hs_mbuf_from_flat(&num, sizeof(num));
////        memset(recvbuf, 0, sizeof(recvbuf));
//
//        /********************************************************/
////        int rc = ble_gattc_notify_custom(conn_handle, ble_spp_svc_gatt_read_val_handle, txom);
////        if (rc == 0)
////        {
////            ESP_LOGI(master_tag, "Notification sent successfully");
////            ESP_LOGI(master_tag,"Received: %s\n", recvbuf);
////        }
////        else
////        {
////            ESP_LOGI(master_tag, "Error in sending notification");
////        }
////        os_mbuf_append(ctxt->om, pDataTransmitUart,
////                                        sizeof(pDataTransmitUart));
////        om = ble_hs_mbuf_from_flat(pDataTransmitUart, 10);
////
////        /*send data available in pDataTransmitUart which I already received by SPI from STM32
////         *
////         */
//////        int rc = os_mbuf_append(ctxt->om, pDataTransmitUart, 10);
////        struct os_mbuf *txom;
////        txom = ble_hs_mbuf_from_flat(pDataReciveedUart, len);
////        memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
////
////        /********************************************************/
////        rc = ble_gattc_notify_custom(connection_handle, ble_spp_svc_gatt_read_val_handle, txom);
////        if (rc == 0)
////        {
////            ESP_LOGI(tag, "Notification sent successfully");
////        }
////        else
////        {
////            ESP_LOGI(tag, "Error in sending notification");
////        }
//
//        break;
//
//    case BLE_GATT_ACCESS_OP_WRITE_CHR:
//        ESP_LOGI(master_tag, "Data received in write event,conn_handle = %x,attr_handle = %x", conn_handle, attr_handle);
//        /***************************************/
//        ble_hs_mbuf_to_flat(ctxt->om, tranbuf/*pDataTransmitUart*/, ctxt->om->om_len/*10*/, &numberOfByteToUart);
//        ESP_LOGI(master_tag,"tranbuf = %s",tranbuf);
//        uint8_t rc = 1;
//        uart_write_bytes(0, &rc, 1);
//        /*send data by SPI to STM32
//
//        */
//
//
////        uint8_t len = uart_write_bytes(UART_PORT_x, (const char *)pDataTransmitUart, (size_t)numberOfByteToUart);
////
////        for (int i = 0; i < numberOfByteToUart; i++)
////        {
////            ESP_LOGI(tag, "datarecved: %d , length recived BLE=%d ,length transmit UART", pDataTransmitUart[i], numberOfByteToUart, len);
////        }
////        memset(pDataTransmitUart, 0, sizeof(pDataTransmitUart));
//
//        /***************************************/
//        break;
//
//    default:
//        ESP_LOGI(master_tag, "\nDefault Callback");
//        break;
//    }
//    return 0;
//}
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
	    {
	        /*** Service: Security test. */
	        .type = BLE_GATT_SVC_TYPE_PRIMARY,
	        .uuid = &gatt_svr_svc_sec_test_uuid.u,
	        .characteristics = (struct ble_gatt_chr_def[])
	        { {
	                /*** Characteristic: Random number generator. */
	                .uuid = &gatt_svr_chr_sec_test_rand_uuid.u,
	                .access_cb = gatt_svc_access,
	                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
	            }, {
	                /*** Characteristic: Static value. */
	                .uuid = &gatt_svr_chr_sec_test_static_uuid.u,
	                .access_cb = gatt_svc_access,
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
gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
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
 * Access callback whenever a characteristic/descriptor is read or written to.
 * Here reads and writes need to be handled.
 * ctxt->op tells weather the operation is read or write and
 * weather it is on a characteristic or descriptor,
 * ctxt->dsc->uuid tells which characteristic/descriptor is accessed.
 * attr_handle give the value handle of the attribute being accessed.
 * Accordingly do:
 *     Append the value to ctxt->om if the operation is READ
 *     Write ctxt->om to the value if the operation is WRITE
 **/

static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
 {
    const ble_uuid_t *uuid;
    int rc;
    uuid = ctxt->chr->uuid;

    /* Determine which characteristic is being accessed by examining its
     * 128-bit UUID.
     */

    if (ble_uuid_cmp(uuid, &gatt_svr_chr_sec_test_rand_uuid.u) == 0) {
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);

        /* Respond with a 32-bit random number. */
		rc = os_mbuf_append(ctxt->om, BleReadBuffer,
				strlen((const char*) BleReadBuffer));
		Data[0]=1;
		uart_write_bytes(EX_UART_NUM, (const char*) Data, 20);
		memset(BleReadBuffer, 0, sizeof(BleReadBuffer));
		memset(Data, 0, sizeof(Data));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ble_uuid_cmp(uuid, &gatt_svr_chr_sec_test_static_uuid.u) == 0) {
        switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
    		rc = os_mbuf_append(ctxt->om, BleBuffer,
    				strlen((const char*) BleBuffer));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
    		rc = gatt_svr_write(ctxt->om, 0, 18, BleBuffer, 0);
    		for (int i = 0; i < 20; ++i) {
    			ESP_LOGI("dfdsf","gfh %d",BleBuffer[i]);
    		}
    		Data[0] = 'H';
    		Data[1] = 'Z';
    		memcpy(&Data[2], BleBuffer, 18);
    		uart_write_bytes(EX_UART_NUM, (const char*) Data, 20);

    		memset(Data, 0, sizeof(Data));
    		memset(BleBuffer, 0, sizeof(BleBuffer));

    		ble_gatts_chr_updated(attr_handle);
    		MODLOG_DFLT(INFO,
    				"Notification/Indication scheduled for " "all subscribed peers.\n");
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

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_ans_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* Setting a value for the read-only descriptor */
//    gatt_svr_dsc_val = 0x99;

    return 0;
}
