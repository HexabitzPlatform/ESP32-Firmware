/*
 *  Description: file’s functionality
 *  Created on: 1/12/2021
 *      Author: ABDALRHMAN HAMMAL @ Hexabitz
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 Hexabitz.
 * All rights reserved.
 *
 ******************************************************************************
 */

#pragma once
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_eth.h"
#include "esp_mac.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include <lwip/ip4_addr.h>



#define TAG_COMM "CONNECTION_TAG"

typedef enum
{
  WiFi_AP_STATIONt_MODE =0U,
  WiFi_AP_MODE = 1U,
  WiFi_STATION_MODE = 2U,
  ETHERNET_MODE = 3U,
  WiFi_ETHERNET_MODE = 4U,
  OTHER_MODE = 5U,
} connection_mode_type_t;





void startConnectionInit(connection_mode_type_t setMode,uint8_t flag);
void wifi_init_softap(void);
void wifi_init_sta(void);
void eth_init_enc28j60(uint8_t flag);
void parse_ip(char *str,uint8_t *arr);
void check_con(void);


//void connection_mode(void);
/***********************SOFT AP WIFI**********************/
#define EXAMPLE_ESP_WIFI_SSID      "Hexabiz_wifi"
#define EXAMPLE_ESP_WIFI_PASS      "Hexabiz_wifi"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       4

/******************************************************/
#define UART_PORT_x UART_NUM_0
#define EX_UART_NUM UART_NUM_0

/***********************STACTION WIFI**********************/

//	Interface_Info.NetworkName = _NetworkName->valuestring;
//	Interface_Info.NetworkPassword = _NetworkPassword->valuestring;
#define EXAMPLE_ESP_MAXIMUM_RETRY  5
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER ""
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#define MY_STATION_WIFI_SSID      /*"kaadan"*/"abdo"/*"Hexabitz_interns"*//*"Hexabitz_interns"*//*"karim"*/
#define MY_STATION_WIFI_PASS      /*"qwer1234@"*/"mnmu12345"/*"HexaSTM32f091rc"*//*"sly.man123123123"*/
#define MY_STATION_MAXIMUM_RETRY  10


#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
//EventGroupHandle_t s_wifi_event_group;

/******************************************************/

/***********************ETHERNET PHYSICAL**********************/
#define MY_ETHERNET_GPIO_RANGE_MIN 0
#define MY_ETHERNET_GPIO_RANGE_MAX 33
#define MY_ETHERNET_ENC28J60_SPI_HOST SPI2_HOST
#define MY_ETHERNET_ENC28J60_SCLK_GPIO 7
#define MY_ETHERNET_ENC28J60_MOSI_GPIO 6
#define MY_ETHERNET_ENC28J60_MISO_GPIO 5
#define MY_ETHERNET_ENC28J60_CS_GPIO 1
#define MY_ETHERNET_ENC28J60_SPI_CLOCK_MHZ 6
#define MY_ETHERNET_ENC28J60_INT_GPIO 3
#define MY_ETHERNET_ENC28J60_RST_GPIO 0

//#define MY_ETHERNET_ENC28J60_DUPLEX_HALF 1
#define MY_ETHERNET_ENC28J60_DUPLEX_FULL 1

#define MY_ETHERNET_MAC_ADD_0 0x02
#define MY_ETHERNET_MAC_ADD_1 0x00
#define MY_ETHERNET_MAC_ADD_2 0x00
#define MY_ETHERNET_MAC_ADD_3 0x12
#define MY_ETHERNET_MAC_ADD_4 0x34
#define MY_ETHERNET_MAC_ADD_5 0x56
