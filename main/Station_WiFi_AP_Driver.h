/*
 * Station_WiFi_AP_Driver.h
 *  Description: file’s functionality
 *  Created on: 1/3/2022
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
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <errno.h> 
#include "fcntl.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <sys/types.h>
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <fcntl.h> /* Added for the nonblocking socket */




#define MY_ESP_WIFI_SSID    "WiFiStation"
#define MY_ESP_WIFI_PASS    "12345678"
#define MY_ESP_WIFI_CHANNEL 1
#define MY_MAX_STA_CONN     4

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data);
void wifi_init_softap(void);




/*******************Unbloking TCP Server**********/
// Access Point over 192.168.4.1
// typedef enum{
//     NOTREADY    =   -1,
//     READY       =   0,
//     SEND_MODE   =   1,
//     RECIVE_MODE =   2
// }soucket_ctrl_t;

// soucket_ctrl_t soucket_mode=NOTREADY;

#define MY_TCP_SERVER_BIND_ADDRESS  "0.0.0.0" //any IP 
#define MY_TCP_SERVER_BIND_PORT     "3344"    //Port Number

void log_socket_error(const char *tag, const int sock, const int err, const char *message);
int try_receive(const char *tag, const int sock, char * data, size_t max_len);
int socket_send(const char *tag, const int sock, const char * data, const size_t len);
char* get_clients_address(struct sockaddr_storage *source_addr);
void tcp_server_task_NONBlocking(void *pvParameters);

/***************Bracelet Function***************/
void WiFi_init(void);

/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
