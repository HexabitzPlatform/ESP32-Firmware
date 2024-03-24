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

#include "Station_WiFi_AP_Driver.h"
/* peripheral */
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"
#include "freertos/task.h"

/*wifi*/
#include "connection_driver.h"

/* uart pins */

#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;
/* SPI pins */
#define GPIO_MOSI 7
#define GPIO_MISO 6
#define GPIO_SCLK 5
#define GPIO_CS 1

/*client*/
const char *tag = "NimBLE_BLE_CENT";
QueueHandle_t spp_common_uart_queue = NULL;
uint16_t numberOfByteToUart;
uint8_t flag = 0;
char recbuf[129] = { 0 };
uint8_t choose = 0;
char serverName[50];
extern uint8_t BleReadBuffer[20];
extern uint8_t Data[20];
/*end*/

/*add from server*/
const char *master_tag = "NimBLE_BLE_PRPH";
uint8_t pDataTransmitUart[10] = { 0 };
uint16_t numberOfByteToUart = 0;
uint8_t num = 1;
char recvbuf[129] = { 0 };
char tranbuf[129] = { 0 };
char dummy[129] = { 0 };
spi_slave_transaction_t t;
uint16_t read_handle;
void spi_init();
void receive_data(char *buf);
void send_data(char *buf);
void send_receive_data(char *send, char *rec);
//int write_data(char *data);
int read_data();
static void ble_uart_init(void);
void uart_event_task(void *pvParameters);
/*end*/
//extern uint16_t ble_spp_svc_gatt_read_val_handle;
/*-------------------------------------------------------------server functions---------------------------------------------*/
char ssid[50] = { 0 };
char password[50] = { "12345678" };
char clent_name[50];
char server_name[50];
uint8_t pDataReciveedUart[512] = { 0 };
uint8_t BleReadDataClient[20];
uint8_t wififlag ;
char WifiData[100];

static void ble_uart_init(void) {
	esp_log_level_set("uart_events", ESP_LOG_INFO);

	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config = { .baud_rate = 921600, .data_bits =
			UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
			UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_DEFAULT, };
//Install UART driver, and get the queue.
	uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20,
			&uart0_queue, 0);
	uart_param_config(EX_UART_NUM, &uart_config);

//Set UART log level
	esp_log_level_set("uart_events", ESP_LOG_INFO);
//Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
			UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

//Set uart pattern detect function.
	uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0,
			0);
//Reset the pattern queue length to record at most 20 pattern positions.
	uart_pattern_queue_reset(EX_UART_NUM, 20);

//Create a task to handler UART event from ISR
	xTaskCreate(uart_event_task, "uTask", 4096, (void*) UART_PORT_x, 8, NULL);
//xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

void spi_init() {
	//Configuration for the SPI bus
	spi_bus_config_t buscfg = { .mosi_io_num = GPIO_MOSI, .miso_io_num =
			GPIO_MISO, .sclk_io_num = GPIO_SCLK, .quadwp_io_num = -1,
			.quadhd_io_num = -1, };

	//Configuration for the SPI slave interface
	spi_slave_interface_config_t slvcfg = { .mode = 0, .spics_io_num = GPIO_CS,
			.queue_size = 1, .flags = 0, };

	//Initialize SPI slave interface
	spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
}

void send_data(char *buf) {
	t.length = 5 * 8;
	t.tx_buffer = buf;
	spi_slave_transmit(SPI2_HOST, &t, 1000/*portMAX_DELAY*/);
	ESP_LOGI(tag, "send IN FUN: %s\n", buf);
}
// Statically allocate and initialize the spinlock
//static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
void receive_data(char *buf) {

	t.length = 5 * 8;
	t.rx_buffer = buf;
	ESP_LOGI(tag, "Received IN FUN: %s\n", buf);
//	taskENTER_CRITICAL(&my_spinlock);
	spi_slave_transmit(SPI2_HOST, &t, /*1000*/portMAX_DELAY);

//	taskEXIT_CRITICAL(&my_spinlock);
	ESP_LOGI(tag, "Received IN FUN1: %s\n", buf);
}

void send_receive_data(char *send, char *rec) {

	t.length = 5 * 8;
	t.rx_buffer = rec;
	t.tx_buffer = send;
	spi_slave_transmit(SPI2_HOST, &t, /*1000*/portMAX_DELAY);
}

void uart_event_task(void *pvParameters) {
	uart_event_t event;
	uint8_t *dtmp = (uint8_t*) malloc(RD_BUF_SIZE);

	for (;;) {

		// Waiting for UART event.
		if (xQueueReceive(uart0_queue, (void*) &event,
				(TickType_t) portMAX_DELAY)) {
			bzero(dtmp, RD_BUF_SIZE);
			ESP_LOGI("uart_events", "uart[%d] event:", EX_UART_NUM);
			switch (event.type) {

			// Event of UART receving data
			case UART_DATA:
				/*START HERE*/
				ESP_LOGI(tag, "enter to take data");
				uart_read_bytes(UART_PORT_x, &pDataReciveedUart[0], event.size,
						portMAX_DELAY / portTICK_PERIOD_MS);
				esp_log_level_set("NimBLE_BLE_CENT", ESP_LOG_INFO);
				ESP_LOGI(tag, "size : %d", event.size);
				ESP_LOGI(tag, "pDataReciveedUart = %d", pDataReciveedUart[0]);
				ESP_LOGI(tag, "pDataReciveedUart = %d", pDataReciveedUart[1]);
//		    	uart_write_bytes(EX_UART_NUM, (const char*) pDataReciveedUart,event.size);

				/* ble as client */
				if (1 == pDataReciveedUart[0]) {
					memcpy(clent_name, &pDataReciveedUart[3],
							pDataReciveedUart[1]);
					memcpy(server_name,
							&pDataReciveedUart[3 + pDataReciveedUart[1]],
							pDataReciveedUart[2]);
					ESP_LOGI("client_Name", " : %s", clent_name);
					ESP_LOGI("server_name", " : %s", server_name);
					/* Configure the host. */
					BLECLient(clent_name);
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
				}
				/* ble as server */
				else if (2 == pDataReciveedUart[0]) {
					memcpy(server_name, &pDataReciveedUart[2],
							pDataReciveedUart[1]);
					ESP_LOGI("server_name", " : %s", server_name);
					/* Initialize the NimBLE host configuration. */
					BLEServer(server_name);
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));

				} else if (3 == pDataReciveedUart[0]) {
					memcpy(ssid, &pDataReciveedUart[3], pDataReciveedUart[1]);
					memcpy(password,
							&pDataReciveedUart[3 + pDataReciveedUart[1]],
							pDataReciveedUart[2]);
					ESP_LOGI("ssid", " : %s", ssid);
					ESP_LOGI("password", " : %s", password);
					wifi_init_softap();
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
				} else if (4 == pDataReciveedUart[0]) {
					memcpy(ssid, &pDataReciveedUart[3], pDataReciveedUart[1]);
					memcpy(password,
							&pDataReciveedUart[3 + pDataReciveedUart[1]],
							pDataReciveedUart[2]);
					ESP_LOGI("ssid", " : %s", ssid);
					ESP_LOGI("password", " : %s", password);
					wifi_init_sta();
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
				} else if (5 == pDataReciveedUart[0]) {
					memcpy(BleReadBuffer, &pDataReciveedUart[2],
							pDataReciveedUart[1]);
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));

				} else if (6 == pDataReciveedUart[0]) {
					uint8_t BleClientBuffer[20];
					memcpy(BleClientBuffer, &pDataReciveedUart[2],
							pDataReciveedUart[1]);
					write_data(BleClientBuffer);
					Data[0] = 1;
					uart_write_bytes(EX_UART_NUM, (const char*) Data, 22);
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
					vTaskDelay(100);
				} else if (7 == pDataReciveedUart[0]) {
					read_data();
					Data[0] = 'H';
					Data[1] = 'Z';
					memcpy(&Data[2], BleReadDataClient, 18);
					uart_write_bytes(EX_UART_NUM, (const char*) Data, 22);
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
					memset(BleReadDataClient, 0, sizeof(BleReadDataClient));
					memset(Data, 0, sizeof(Data));
					vTaskDelay(50);
					Data[0] = 1;
					uart_write_bytes(EX_UART_NUM, (const char*) Data, 22);
					vTaskDelay(100);
				} else if (8 == pDataReciveedUart[0]) {
					memcpy(ssid, &pDataReciveedUart[3], pDataReciveedUart[1]);
					memcpy(password,
							&pDataReciveedUart[3 + pDataReciveedUart[1]],
							pDataReciveedUart[2]);
					ESP_LOGI("ssid", " : %s", ssid);
					ESP_LOGI("password", " : %s", password);
					WiFi_init();
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
				}
				else if (9 == pDataReciveedUart[0]) {
					memcpy(WifiData, &pDataReciveedUart[2],
							pDataReciveedUart[1]);
					wififlag = 1;
					memset(pDataReciveedUart, 0, sizeof(pDataReciveedUart));
				}

				break;
			default:
				break;
			}
		}
	}
	free(dtmp);
	dtmp = NULL;
	vTaskDelete(NULL);

}
int rc;
void app_main(void) {

	/* Initialize NVS — it is used to store PHY calibration data */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(esp_netif_init());
	// Create default event loop that running in background
//    ESP_ERROR_CHECK(esp_event_loop_create_default());

	ble_uart_init();
	nimble_port_init();

}

