////slave

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

#include "esp_log.h"
#include "nvs_flash.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_bt.h"
#include "blecent.h"
#include "bleprph.h"
/* peripheral */
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "freertos/task.h"

/*wifi*/
#include "connection_driver.h"


/* uart pins */
#define UART_PORT_x UART_NUM_0
#define UART_TX_PIN_ST GPIO_NUM_19
#define UART_RX_PIN_ST GPIO_NUM_18

/* SPI pins */
#define GPIO_MOSI 7
#define GPIO_MISO 6
#define GPIO_SCLK 5
#define GPIO_CS 1

static const ble_uuid_t * service1 =
		BLE_UUID128_DECLARE(0x90, 0x89, 0x81, 0x40, 0xec, 0xc4, 0x96, 0xb7, 0xed, 0x45, 0x63, 0xcc, 0x1d, 0x46, 0xdb, 0xf9);
static const ble_uuid_t * char1 =
		BLE_UUID128_DECLARE(0x79, 0x24, 0x39, 0xf7, 0x54, 0xe5, 0xce, 0x8a, 0x44, 0x4a, 0xf6, 0xe0, 0xf3, 0x90, 0x22, 0x97);
static const ble_uuid_t * char2 =
		BLE_UUID128_DECLARE(0xe2, 0xa2, 0xe5, 0x9c, 0x72, 0x24, 0xa7, 0x99, 0xc6, 0x46, 0x50, 0x69, 0x11, 0x65, 0xa7, 0xb8);


/*client*/
const char *tag = "NimBLE_BLE_CENT";
QueueHandle_t spp_common_uart_queue = NULL;
uint16_t numberOfByteToUart;
uint16_t conn_handle;
uint8_t flag;
char recbuf[129]={0};
uint8_t choose = 0;
char serverName[50];

static int blecent_gap_event(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);
/*end*/

/*add from server*/
const char *master_tag = "NimBLE_BLE_PRPH";
static uint8_t own_addr_type;
uint8_t pDataTransmitUart[10] = {0};
uint16_t numberOfByteToUart = 0;
uint8_t num = 1;
char recvbuf[129]={0};
char tranbuf[129]={0};
char dummy[129]={0};
spi_slave_transaction_t t;
uint16_t read_handle;
//uint16_t ble_svc_gatt_read_val_handle, ble_spp_svc_gatt_read_val_handle, ble_spp_svc_gatt_write_val_handle;
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
static void bleprph_print_conn_desc(struct ble_gap_conn_desc *desc);
static void bleprph_advertise(void);
void spi_init();
void receive_data(char *buf);
void send_data(char *buf);
void send_receive_data(char *send,char *rec);
int write_data(char *data);
int read_data();
static void ble_uart_init(void);
void ble_server_uart_task(void *pvParameters);
/*end*/
extern uint16_t ble_spp_svc_gatt_read_val_handle;
/*-------------------------------------------------------------server functions---------------------------------------------*/
char ssid[50] = {0};
char password[50] = {"12345678"};
char clent_name[50] = {"hexa"};
char server_name[50] = {"Reda"};
uint8_t pDataReciveedUart[512] = {0};


void parseData(char *str1, char *str2, uint8_t numofstr)
{
    int i=0;

    for(; i<pDataReciveedUart[1] ; i++)
    {
    	str1/*wifi_settings.ssid*/[i] = pDataReciveedUart[i+2];
    }

    ESP_LOGI(tag, "str1 : %s",str1/*wifi_settings.ssid*/);

    if(numofstr == 2)
    {
		int str2Size = pDataReciveedUart[i+2];
		i+=3;

		for(int j=0; j<str2Size ; j++)
		{
			str2/*wifi_settings.password*/[j] = pDataReciveedUart[i++];
		}
	    ESP_LOGI(tag, "str2 : %s",str2/*wifi_settings.ssid*/);
    }
}

static void ble_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    // Install UART driver, and get the queue.
    uart_driver_install(UART_PORT_x, 4096, 8192, 10, &spp_common_uart_queue, 0);
    // Set UART parameters
    uart_param_config(UART_PORT_x, &uart_config);
    // Set UART pins
    uart_set_pin(UART_PORT_x, UART_TX_PIN_ST, UART_RX_PIN_ST, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(ble_server_uart_task, "uTask", 4096, (void *)UART_PORT_x, 8, NULL);
}


void spi_init()
{
    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode=0,
        .spics_io_num=GPIO_CS,
        .queue_size=1,
        .flags=0,
    };

    //Initialize SPI slave interface
    spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
}

void send_data(char *buf)
{
	t.length=5*8;
	t.tx_buffer=buf;
	spi_slave_transmit(SPI2_HOST, &t, 1000/*portMAX_DELAY*/);
	ESP_LOGI(tag,"send IN FUN: %s\n", buf);
}
// Statically allocate and initialize the spinlock
//static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
void receive_data(char *buf)
{

	t.length=5*8;
	t.rx_buffer=buf;
	ESP_LOGI(tag,"Received IN FUN: %s\n", buf);
//	taskENTER_CRITICAL(&my_spinlock);
	spi_slave_transmit(SPI2_HOST, &t, /*1000*/portMAX_DELAY);

//	taskEXIT_CRITICAL(&my_spinlock);
	ESP_LOGI(tag,"Received IN FUN1: %s\n", buf);
}

void send_receive_data(char *send,char *rec)
{

	t.length=5*8;
	t.rx_buffer=rec;
	t.tx_buffer=send;
	spi_slave_transmit(SPI2_HOST, &t, /*1000*/portMAX_DELAY);
}
int write_data(char *data)
{
	const struct peer *peer;
	const struct peer_chr *chr;
	int rc;
	 peer = peer_find(conn_handle);
	    chr = peer_chr_find_uuid(peer,
	    		service1,
				char2);
	    if (chr == NULL) {
	        MODLOG_DFLT(ERROR, "Error: Peer doesn't have the subscribable characteristic\n");
	        return 1;
	    }
	rc = ble_gattc_write_flat(conn_handle, chr->chr.val_handle,
			data/*value*/, strlen(data)/*6*/, NULL/*blecent_on_custom_write*/, NULL);
	if (rc != 0) {
		MODLOG_DFLT(ERROR,
					"Error: Failed to write to the subscribable characteristic; "
					"rc=%d\n", rc);
	}
	return 0;
}

int read_data()
{
	const struct peer *peer;
	const struct peer_chr *chr;
	int rc;
	 peer = peer_find(conn_handle);
	    chr = peer_chr_find_uuid(peer,
	    		service1,
				char1
				);
	    if (chr == NULL) {
	        MODLOG_DFLT(ERROR, "Error: Peer doesn't have the subscribable characteristic\n");
	        return 1;
	    }
	rc = ble_gattc_read(conn_handle, chr->chr.val_handle,
						NULL/*blecent_on_custom_read*/, NULL);

	if (rc != 0) {
		MODLOG_DFLT(ERROR,
					"Error: Failed to read the custom subscribable characteristic; "
					"rc=%d\n", rc);
	}
	return 0;
}






void ble_store_config_init(void);

/**
 * Logs information about a connection to the console.
 */
static void
bleprph_print_conn_desc(struct ble_gap_conn_desc *desc)
{
    MODLOG_DFLT(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    print_addr(desc->our_ota_addr.val);
    MODLOG_DFLT(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    print_addr(desc->our_id_addr.val);
    MODLOG_DFLT(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    print_addr(desc->peer_ota_addr.val);
    MODLOG_DFLT(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    print_addr(desc->peer_id_addr.val);
    MODLOG_DFLT(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

#if CONFIG_EXAMPLE_EXTENDED_ADV
/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ext_bleprph_advertise(void)
{
    struct ble_gap_ext_adv_params params;
    struct os_mbuf *data;
    uint8_t instance = 0;
    int rc;

    /* First check if any instance is already active */
    if(ble_gap_ext_adv_active(instance)) {
        return;
    }

    /* use defaults for non-set params */
    memset (&params, 0, sizeof(params));

    /* enable connectable advertising */
    params.connectable = 1;

    /* advertise using random addr */
    params.own_addr_type = BLE_OWN_ADDR_PUBLIC;

    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_2M;
    //params.tx_power = 127;
    params.sid = 1;

    params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MIN;

    /* configure instance 0 */
    rc = ble_gap_ext_adv_configure(instance, &params, NULL,
                                   bleprph_gap_event, NULL);
    assert (rc == 0);

    /* in this case only scan response is allowed */

    /* get mbuf for scan rsp data */
    data = os_msys_get_pkthdr(sizeof(ext_adv_pattern_1), 0);
    assert(data);

    /* fill mbuf with scan rsp data */
    rc = os_mbuf_append(data, ext_adv_pattern_1, sizeof(ext_adv_pattern_1));
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_data(instance, data);
    assert (rc == 0);

    /* start advertising */
    rc = ble_gap_ext_adv_start(instance, 0, 0);
    assert (rc == 0);
}
#else
/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}
#endif

#if MYNEWT_VAL(BLE_POWER_CONTROL)
static void bleprph_power_control(uint16_t conn_handle)
{
    int rc;

    rc = ble_gap_read_remote_transmit_power_level(conn_handle, 0x01 );  // Attempting on LE 1M phy
    assert (rc == 0);

    rc = ble_gap_set_transmit_power_reporting_enable(conn_handle, 0x1, 0x1);
    assert (rc == 0);
}
#endif

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unused by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);
        }
        MODLOG_DFLT(INFO, "\n");

        if (event->connect.status != 0) {
            /* Connection failed; resume advertising. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
            ext_bleprph_advertise();
#else
            bleprph_advertise();
#endif
        }

#if MYNEWT_VAL(BLE_POWER_CONTROL)
	bleprph_power_control(event->connect.conn_handle);
#endif
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");

        /* Connection terminated; resume advertising. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
        ext_bleprph_advertise();
#else
        bleprph_advertise();
#endif
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        MODLOG_DFLT(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d",
                    event->adv_complete.reason);
#if CONFIG_EXAMPLE_EXTENDED_ADV
        ext_bleprph_advertise();
#else
        bleprph_advertise();
#endif
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        MODLOG_DFLT(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        MODLOG_DFLT(INFO, "notify_tx event; conn_handle=%d attr_handle=%d "
                    "status=%d is_indication=%d",
                    event->notify_tx.conn_handle,
                    event->notify_tx.attr_handle,
                    event->notify_tx.status,
                    event->notify_tx.indication);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                    "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(tag, "PASSKEY_ACTION_EVENT started \n");
        struct ble_sm_io pkey = {0};
        int key = 0;

        if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
            pkey.action = event->passkey.params.action;
            pkey.passkey = 123456; // This is the passkey to be entered on peer
            ESP_LOGI(tag, "Enter passkey %" PRIu32 "on the peer side", pkey.passkey);
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(tag, "ble_sm_inject_io result: %d\n", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            ESP_LOGI(tag, "Passkey on device's display: %" PRIu32 , event->passkey.params.numcmp);
            ESP_LOGI(tag, "Accept or reject the passkey through console in this format -> key Y or key N");
            pkey.action = event->passkey.params.action;
            if (scli_receive_key(&key)) {
                pkey.numcmp_accept = key;
            } else {
                pkey.numcmp_accept = 0;
                ESP_LOGE(tag, "Timeout! Rejecting the key");
            }
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(tag, "ble_sm_inject_io result: %d\n", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_OOB) {
            static uint8_t tem_oob[16] = {0};
            pkey.action = event->passkey.params.action;
            for (int i = 0; i < 16; i++) {
                pkey.oob[i] = tem_oob[i];
            }
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(tag, "ble_sm_inject_io result: %d\n", rc);
        } else if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
            ESP_LOGI(tag, "Enter the passkey through console in this format-> key 123456");
            pkey.action = event->passkey.params.action;
            if (scli_receive_key(&key)) {
                pkey.passkey = key;
            } else {
                pkey.passkey = 0;
                ESP_LOGE(tag, "Timeout! Passing 0 as the key");
            }
            rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
            ESP_LOGI(tag, "ble_sm_inject_io result: %d\n", rc);
        }
        return 0;

#if MYNEWT_VAL(BLE_POWER_CONTROL)
    case BLE_GAP_EVENT_TRANSMIT_POWER:
        MODLOG_DFLT(INFO, "Transmit power event : status=%d conn_handle=%d reason=%d "
                           "phy=%d power_level=%x power_level_flag=%d delta=%d",
                     event->transmit_power.status,
                     event->transmit_power.conn_handle,
                     event->transmit_power.reason,
                     event->transmit_power.phy,
                     event->transmit_power.transmit_power_level,
                     event->transmit_power.transmit_power_level_flag,
                     event->transmit_power.delta);
        return 0;

     case BLE_GAP_EVENT_PATHLOSS_THRESHOLD:
        MODLOG_DFLT(INFO, "Pathloss threshold event : conn_handle=%d current path loss=%d "
                           "zone_entered =%d",
                     event->pathloss_threshold.conn_handle,
                     event->pathloss_threshold.current_path_loss,
                     event->pathloss_threshold.zone_entered);
        return 0;
#endif
    }

    return 0;
}

static void
bleprph_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

#if CONFIG_EXAMPLE_RANDOM_ADDR
static void
ble_app_set_addr(void)
{
    ble_addr_t addr;
    int rc;

    /* generate new non-resolvable private address */
    rc = ble_hs_id_gen_rnd(0, &addr);
    assert(rc == 0);

    /* set generated address */
    rc = ble_hs_id_set_rnd(addr.val);

    assert(rc == 0);
}
#endif

static void
bleprph_on_sync(void)
{
    int rc;

#if CONFIG_EXAMPLE_RANDOM_ADDR
    /* Generate a non-resolvable private address. */
    ble_app_set_addr();
#endif

    /* Make sure we have proper identity address set (public preferred) */
#if CONFIG_EXAMPLE_RANDOM_ADDR
    rc = ble_hs_util_ensure_addr(1);
#else
    rc = ble_hs_util_ensure_addr(0);
#endif
    assert(rc == 0);

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Printing ADDR */
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    /* Begin advertising. */
#if CONFIG_EXAMPLE_EXTENDED_ADV
    ext_bleprph_advertise();
#else
    bleprph_advertise();
#endif
}

void bleprph_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}
void ble_server_uart_task(void *pvParameters)
{
    ESP_LOGI(tag, "BLE server UART_task started\n");
    uart_event_t event;
    int rc = 0;
//    uint8_t pDataReciveedUart[512] = {0};

    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(spp_common_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            switch (event.type)
            {
            // Event of UART receving data
            case UART_DATA:
            	/*START HERE*/
                	ESP_LOGI(tag, "enter to take data");
                    uart_read_bytes(UART_PORT_x, &pDataReciveedUart[0], event.size, portMAX_DELAY / portTICK_PERIOD_MS);
                    ESP_LOGI(tag, "size : %d",event.size);
                    ESP_LOGI(tag, "pDataReciveedUart = %d",pDataReciveedUart[0]);
                    ESP_LOGI(tag, "pDataReciveedUart = %d",pDataReciveedUart[1]);
                    /* ble as client */
                    if(pDataReciveedUart[0] == 1)
                    {
//                    	parseData(server_name, clent_name, 2);
//                        /* Configure the host. */
//                        ble_hs_cfg.reset_cb = blecent_on_reset;
//                        ble_hs_cfg.sync_cb = blecent_on_sync;
//                        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
//
//                        /* Initialize data structures to track connected peers. */
//                        rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
//                        assert(rc == 0);
//
//                        /* Set the default device name. */
////                        rc = ble_svc_gap_device_name_set("nimble-blecent");
//                        rc = ble_svc_gap_device_name_set(clent_name);
//                        assert(rc == 0);
//
//                        /* XXX Need to have template for store */
//                        ble_store_config_init();
//
//                        nimble_port_freertos_init(blecent_host_task);
//                        choose = 1;
                    }
                    /* ble as server */
                    else if(pDataReciveedUart[0] == 2)
                    {
                    	parseData(server_name, NULL, 1);
                    	    /* Initialize the NimBLE host configuration. */
                    	    ble_hs_cfg.reset_cb = bleprph_on_reset;
                    	    ble_hs_cfg.sync_cb = bleprph_on_sync;
                    	    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
                    	    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

                    	    ble_hs_cfg.sm_io_cap = 3;
                    	#ifdef CONFIG_EXAMPLE_BONDING
                    	    ble_hs_cfg.sm_bonding = 1;
                    	#endif
                    	#ifdef CONFIG_EXAMPLE_MITM
                    	    ble_hs_cfg.sm_mitm = 1;
                    	#endif
                    	#ifdef CONFIG_EXAMPLE_USE_SC
                    	    ble_hs_cfg.sm_sc = 1;
                    	#else
                    	    ble_hs_cfg.sm_sc = 0;
                    	#endif
                    	#ifdef CONFIG_EXAMPLE_BONDING
                    	    ble_hs_cfg.sm_our_key_dist = 1;
                    	    ble_hs_cfg.sm_their_key_dist = 1;
                    	#endif


                    	    rc = gatt_svr_init();
                    	    assert(rc == 0);

                    	    /* Set the default device name. */
//                    	    rc = ble_svc_gap_device_name_set("nimble-bleprph");
                    	    rc = ble_svc_gap_device_name_set(server_name);
                    	    assert(rc == 0);

                    	    /* XXX Need to have template for store */
                    	    ble_store_config_init();

                    	    nimble_port_freertos_init(bleprph_host_task);
                    	    choose = 2;
                    }
                    /* send data by spi */
//                    else if(pDataReciveedUart[0] == 3)
//                    {
//                    	 ESP_LOGI(tag, "--------------------------------------------------------Enter to send-----------------");
//                    	 send_receive_data(tranbuf,dummy);
//                    	 ESP_LOGI(tag,"tranbuf: %s\n", tranbuf);
//                    	 ESP_LOGI(tag,"dummy: %s\n", dummy);
//
//                    }
                    /* receive data by spi */
//                    else if(pDataReciveedUart[0] == 4)
//                    {
//                    	 ESP_LOGI(tag, "-------------------------------------------------------Enter to receive---------------");
//                    	 send_receive_data(dummy,recvbuf);
//                    	 ESP_LOGI(tag,"recvbuf: %s\n", recvbuf);
//                    	if(choose == 1)
//                    	{
//                    		write_data(recvbuf);
//                    	}
//                    	if(choose == 2)
//                    	{
//                    		/* the client will read data from BLE_GAP_EVENT_NOTIFY_RX */
//                    		 struct os_mbuf *txom;
//                    		 txom = ble_hs_mbuf_from_flat(recvbuf, strlen(recvbuf));
//                    		 int rc = ble_gattc_notify_custom(conn_handle, ble_spp_svc_gatt_read_val_handle, txom);
//                    	        if (rc == 0)
//                    	        {
//                    	            ESP_LOGI(master_tag, "Notification sent successfully");
//                    	        }
//                    	        else
//                    	        {
//                    	            ESP_LOGI(master_tag, "Error in sending notification");
//                    	        }
//                    	}
//
//
//                    }
                    /* disable BLE */
//                    else if(pDataReciveedUart[0] == 5)
//                    {
//                    	ESP_LOGI(tag, "3");
//                    	esp_err_t ret;
//                        if ((ret = esp_bt_controller_deinit()) != ESP_OK) {
//                        	ESP_LOGI(tag, "error in controller_deinit");
//                        }
//                        if ((ret = esp_bt_controller_disable()) != ESP_OK) {
//                        	ESP_LOGI(tag, "error in disable ble");
//                        }
//                    	esp_bt_controller_disable();
//                    }
                    /* enable  BLE */
//                    else if(pDataReciveedUart[0] == 6)
//                    {
//                    	ESP_LOGI(tag, "4");
//                    	esp_err_t ret;
//                    	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//                        if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
//                        	ESP_LOGI(tag, "error in controller_init");
//                        }
//                        if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
//                        	ESP_LOGI(tag, "error in enable ble");
//                        }
//
//                    }
                    /* set wifi */
                    else if(pDataReciveedUart[0] == 7)
                    {

//                    int i=0;
//
//                    for(; i<pDataReciveedUart[2] ; i++)
//                    {
//                    	ssid/*wifi_settings.ssid*/[i] = pDataReciveedUart[i+3];
//                    }
//
//                    ESP_LOGI(tag, "ssid : %s",ssid/*wifi_settings.ssid*/);
//
//                    int passSize = pDataReciveedUart[i+3];
//                    i+=4;
//
//                    for(int j=0; j<passSize ; j++)
//				    {
//                    	password/*wifi_settings.password*/[j] = pDataReciveedUart[i++];
//				    }
                    	parseData(ssid, password, 2);
				    ESP_LOGI(tag, "password : %s",password/*wifi_settings.password*/);
				    if(pDataReciveedUart[1] == WiFi_AP_MODE)
				    	startConnectionInit(WiFi_AP_MODE,0);
				    else if(pDataReciveedUart[1] == WiFi_STATION_MODE)
				    	 startConnectionInit(WiFi_STATION_MODE,0);
                }
                    /* enable wifi */
                    else if(pDataReciveedUart[0] == 8)
                    {
                    	if(pDataReciveedUart[1] == WiFi_STATION_MODE)
                    		esp_wifi_connect();

                    }
                    /* disable wifi */
                    else if(pDataReciveedUart[0] == 9)
                    {
                    	if(pDataReciveedUart[1] == WiFi_STATION_MODE)
                    		esp_wifi_disconnect();
                    }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}



int rc ;
char d[5]="hello";
void
app_main(void)
{

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if  (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
//    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ble_uart_init();
    spi_init();
    nimble_port_init();

	parseData(server_name, NULL, 1);
	    /* Initialize the NimBLE host configuration. */
	    ble_hs_cfg.reset_cb = bleprph_on_reset;
	    ble_hs_cfg.sync_cb = bleprph_on_sync;
	    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
	    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

	    ble_hs_cfg.sm_io_cap = 3;
	#ifdef CONFIG_EXAMPLE_BONDING
	    ble_hs_cfg.sm_bonding = 1;
	#endif
	#ifdef CONFIG_EXAMPLE_MITM
	    ble_hs_cfg.sm_mitm = 1;
	#endif
	#ifdef CONFIG_EXAMPLE_USE_SC
	    ble_hs_cfg.sm_sc = 1;
	#else
	    ble_hs_cfg.sm_sc = 0;
	#endif
	#ifdef CONFIG_EXAMPLE_BONDING
	    ble_hs_cfg.sm_our_key_dist = 1;
	    ble_hs_cfg.sm_their_key_dist = 1;
	#endif


	    rc = gatt_svr_init();
	    assert(rc == 0);

	    /* Set the default device name. */
//                    	    rc = ble_svc_gap_device_name_set("nimble-bleprph");
	    rc = ble_svc_gap_device_name_set(server_name);
	    assert(rc == 0);

	    /* XXX Need to have template for store */
	    ble_store_config_init();

	    nimble_port_freertos_init(bleprph_host_task);
	    choose = 2;

    // Set up interrupt handler for SPI receive events
//       spi_isr_register(spi_slave_receive_handler, NULL, 0, NULL, 0);
//    /* adding  */
//    while(choose != 1  && choose != 2)
//    {
//    uart_read_bytes(UART_PORT_x, &choose, sizeof(choose), portMAX_DELAY / portTICK_RATE_MS);
//    ESP_LOGI(master_tag,"choose = %d",choose);
//    }
//    /*end*/
//    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

//    nimble_port_init();
//    if (ret != ESP_OK) {
//            ESP_LOGE(tag, "Failed to init nimble %d ", ret);
//            return;
//        }
//
//	parseData(ssid, password, 2);
//				    ESP_LOGI(tag, "password : %s",password/*wifi_settings.password*/);
//				    if(pDataReciveedUart[1] == WiFi_AP_MODE)
//    wifi_init_softap();
//				    	startConnectionInit(WiFi_STATION_MODE,0);
//				    else if(pDataReciveedUart[1] == WiFi_STATION_MODE)
//				    	 startConnectionInit(WiFi_STATION_MODE,0);

    /* ----------------------------------either client-------------------------------- */
//    if(choose == 1)
//    {
//    /* Configure the host. */
//    ble_hs_cfg.reset_cb = blecent_on_reset;
//    ble_hs_cfg.sync_cb = blecent_on_sync;
//    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
//
//    /* Initialize data structures to track connected peers. */
//    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
//    assert(rc == 0);
//
//    /* Set the default device name. */
//    rc = ble_svc_gap_device_name_set("nimble-blecent");
//    assert(rc == 0);
//
//    /* XXX Need to have template for store */
//    ble_store_config_init();
//
//    nimble_port_freertos_init(blecent_host_task);
//    }
    /* ----------------------------------or master-------------------------------- */
//    else if(choose == 2)
//    {
////        ble_uart_init();
////        spi_init();
//    /* Initialize the NimBLE host configuration. */
//    ble_hs_cfg.reset_cb = bleprph_on_reset;
//    ble_hs_cfg.sync_cb = bleprph_on_sync;
//    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
//    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
//
//    ble_hs_cfg.sm_io_cap = 3;
//#ifdef CONFIG_EXAMPLE_BONDING
//    ble_hs_cfg.sm_bonding = 1;
//#endif
//#ifdef CONFIG_EXAMPLE_MITM
//    ble_hs_cfg.sm_mitm = 1;
//#endif
//#ifdef CONFIG_EXAMPLE_USE_SC
//    ble_hs_cfg.sm_sc = 1;
//#else
//    ble_hs_cfg.sm_sc = 0;
//#endif
//#ifdef CONFIG_EXAMPLE_BONDING
//    ble_hs_cfg.sm_our_key_dist = 1;
//    ble_hs_cfg.sm_their_key_dist = 1;
//#endif
//
//
//    rc = gatt_svr_init();
////    gatt_svr_register();
//    assert(rc == 0);
//
//    /* Set the default device name. */
//    rc = ble_svc_gap_device_name_set("nimble-bleprph");
//    assert(rc == 0);
//
//    /* XXX Need to have template for store */
//    ble_store_config_init();
//
//    nimble_port_freertos_init(bleprph_host_task);
//    xTaskCreate(ble_server_uart_task, "uTask", 4096, (void *)UART_PORT_x, 8, NULL);
//    }

//    while(1)
//    {
////    	if(choose == 1)
////    	{
////    	if(flag == 1)
////    	{
//////    	write_data(d);
//////    	vTaskDelay(3000 / portTICK_PERIOD_MS);
////    	read_data();
////    	vTaskDelay(3000 / portTICK_PERIOD_MS);
////    	}
////    	}
//    }

//    send_data(d);
//    vTaskDelay(3000 / portTICK_PERIOD_MS);

}




