/*
 *  Description: file’s functionality
 *  Created on: 2/4/2021
 *      Author: ABDALRHMAN HAMMAL @ Hexabitz
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 Hexabitz.
 * All rights reserved.
 *
 ******************************************************************************
*/

#include <stdio.h>
#include "connection_driver.h"
//extern Interface_Info_t Interface_Info;
EventGroupHandle_t s_wifi_event_group;
int s_active_interfaces = 0;
int s_retry_num = 0;
uint8_t wifi_connect=0;
extern char ssid[50];
extern char password[50];
void startConnectionInit(connection_mode_type_t setMode,uint8_t flag)
{

if (setMode == WiFi_AP_MODE || WiFi_STATION_MODE)
{

        if(setMode == WiFi_AP_MODE)
        {
            ESP_LOGI(TAG_COMM, "ESP_WIFI_MODE_AP");
            wifi_init_softap();

        }
        else if(setMode == WiFi_STATION_MODE)
        {
            ESP_LOGI(TAG_COMM, "ESP_WIFI_MODE_STA");
             wifi_init_sta(flag);
        }
        s_active_interfaces++;
}
}


/***********************SOFT AP WIFI**********************/
static void my_wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
//        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
//        ESP_LOGI(TAG_COMM, "station "MACSTR" join, AID=%d",MAC2STR(event->mac), event->aid);
//
//    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
//        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
//        ESP_LOGI(TAG_COMM, "station "MACSTR" leave, AID=%d",
//                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &my_wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
//            .ssid = MY_AP_WIFI_SSID,
//            .ssid_len = strlen(MY_AP_WIFI_SSID),
            .channel = MY_AP_WIFI_CHANNEL,
            .password = MY_AP_WIFI_PASS,
            .max_connection = MY_AP_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    memcpy(wifi_config.sta.ssid,(uint8_t *)ssid,strlen(ssid));
    memcpy(wifi_config.sta.password,(uint8_t *)password,strlen(password));


    if (strlen(MY_AP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_COMM, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
    		ssid, password, MY_AP_WIFI_CHANNEL);

}

/***********************STATION WIFI**********************/

static void my_station_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    	ESP_LOGI(TAG_COMM, "1");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MY_STATION_MAXIMUM_RETRY) {
        	ESP_LOGI(TAG_COMM, "2");
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_COMM, "retry to connect to the AP");
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG_COMM,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_COMM, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
//    ESP_LOGI(TAG_COMM, "s_retry_num=%d",s_retry_num);
}

void wifi_init_sta(uint8_t flag)
{

	esp_netif_t *my_sta=NULL;
	s_wifi_event_group = xEventGroupCreate();
    my_sta = esp_netif_create_default_wifi_sta();
//    if(flag == 1)
//    {
//    esp_netif_dhcpc_stop(my_sta);
//    esp_netif_ip_info_t ip_info;
////    char *str=Interface_Info.ipv4;
//    uint8_t final[4];
//    parse_ip(Interface_Info.ipv4,final);
////    ESP_LOGI(TAG_COMM, "ip=%s",Interface_Info.ipv4);
//    IP4_ADDR(&ip_info.ip, final[0], final[1], final[2], final[3]);
////    IP4_ADDR(&ip_info.ip, 192, 168, 15, 22);
//    IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
//    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
//    esp_netif_set_ip_info(my_sta, &ip_info);
//    }


    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//	}

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &my_station_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &my_station_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    memcpy(wifi_config.sta.ssid,(uint8_t *)ssid,strlen(ssid));
    memcpy(wifi_config.sta.password,(uint8_t *)password,strlen(password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by my_station_event_handler() (see above) */

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_COMM, "connected to ap SSID:%s password:%s",
        		ssid, password);
        wifi_connect = 1;
//        gpio_set_level(RED_LED, LED_RED_ON);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_COMM, "Failed to connect to SSID:%s, password:%s",
        		ssid, password);
    } else {
        ESP_LOGE(TAG_COMM, "UNEXPECTED EVENT");
    }
    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);


}

void check_con(void)
{
	s_wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &my_station_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &my_station_event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
			pdTRUE/*pdFALSE*/,
            pdFALSE,
            portMAX_DELAY);
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_COMM, "connected to ap SSID:%s password:%s",
                 MY_STATION_WIFI_SSID, MY_STATION_WIFI_PASS);

    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_COMM, "Failed to connect to SSID:%s, password:%s",
                 MY_STATION_WIFI_SSID, MY_STATION_WIFI_PASS);
    } else {
        ESP_LOGE(TAG_COMM, "UNEXPECTED EVENT");
    }
    xEventGroupClearBits( s_wifi_event_group,
    		WIFI_CONNECTED_BIT | WIFI_FAIL_BIT );
    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

//void wifi_init_sta(void)
//{
//    s_wifi_event_group = xEventGroupCreate();
//
//    ESP_ERROR_CHECK(esp_netif_init());
//
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
//    esp_netif_create_default_wifi_sta();
//
//    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//
//    esp_event_handler_instance_t instance_any_id;
//    esp_event_handler_instance_t instance_got_ip;
//    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
//                                                        ESP_EVENT_ANY_ID,
//                                                        &my_station_event_handler,
//                                                        NULL,
//                                                        &instance_any_id));
//    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
//                                                        IP_EVENT_STA_GOT_IP,
//                                                        &my_station_event_handler,
//                                                        NULL,
//                                                        &instance_got_ip));
//
//    wifi_config_t wifi_config = {
//        .sta = {
//            .ssid = MY_STATION_WIFI_SSID,
//            .password = MY_STATION_WIFI_PASS,
//            /* Setting a password implies station will connect to all security modes including WEP/WPA.
//             * However these modes are deprecated and not advisable to be used. Incase your Access point
//             * doesn't support WPA2, these mode can be enabled by commenting below line */
//	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
//
//            .pmf_cfg = {
//                .capable = true,
//                .required = false
//            },
//        },
//    };
//    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
//    ESP_ERROR_CHECK(esp_wifi_start());
//
//    ESP_LOGI(TAG_COMM, "wifi_init_sta finished.");
//
//    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
//     * number of re-tries (WIFI_FAIL_BIT). The bits are set by my_station_event_handler() (see above) */
//    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
//            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
//            pdFALSE,
//            pdFALSE,
//            portMAX_DELAY);
//
//    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
//     * happened. */
//    if (bits & WIFI_CONNECTED_BIT) {
//        ESP_LOGI(TAG_COMM, "connected to ap SSID:%s password:%s",
//                 MY_STATION_WIFI_SSID, MY_STATION_WIFI_PASS);
//        gpio_set_level(RED_LED, LED_RED_ON);
//
//    } else if (bits & WIFI_FAIL_BIT) {
//        ESP_LOGI(TAG_COMM, "Failed to connect to SSID:%s, password:%s",
//                 MY_STATION_WIFI_SSID, MY_STATION_WIFI_PASS);
//    } else {
//        ESP_LOGE(TAG_COMM, "UNEXPECTED EVENT");
//    }
//
//    /* The event will not be processed after unregister */
//    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
//    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
//
//    vEventGroupDelete(s_wifi_event_group);
//}
/******************************************************/

///***********************Ethernet**********************/
//static void eth_event_handler(void *arg, esp_event_base_t event_base,
//                              int32_t event_id, void *event_data)
//{
//    uint8_t mac_addr[6] = {0};
//    /* we can get the ethernet driver handle from event data */
//    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
//
//    switch (event_id) {
//    case ETHERNET_EVENT_CONNECTED:
//        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
//        ESP_LOGI(TAG_COMM, "Ethernet Link Up");
//        ESP_LOGI(TAG_COMM, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
//                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//        break;
//    case ETHERNET_EVENT_DISCONNECTED:
//        ESP_LOGI(TAG_COMM, "Ethernet Link Down");
//        break;
//    case ETHERNET_EVENT_START:
//        ESP_LOGI(TAG_COMM, "Ethernet Started");
//        break;
//    case ETHERNET_EVENT_STOP:
//        ESP_LOGI(TAG_COMM, "Ethernet Stopped");
//        break;
//    default:
//        break;
//    }
//}
//
///** Event handler for IP_EVENT_ETH_GOT_IP */
//static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
//                                 int32_t event_id, void *event_data)
//{
//    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
//    const esp_netif_ip_info_t *ip_info = &event->ip_info;
//
//    ESP_LOGI(TAG_COMM, "Ethernet Got IP Address");
//    ESP_LOGI(TAG_COMM, "~~~~~~~~~~~");
//    ESP_LOGI(TAG_COMM, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
//    ESP_LOGI(TAG_COMM, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
//    ESP_LOGI(TAG_COMM, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
//    ESP_LOGI(TAG_COMM, "~~~~~~~~~~~");
//}
//
//
//void eth_init_enc28j60(uint8_t flag)
//{
////    ESP_ERROR_CHECK(gpio_install_isr_service(0));  // isr is alredy installed
//    // Initialize TCP/IP network interface (should be called only once in application)
//
//
////    ESP_ERROR_CHECK(esp_netif_init());
////    // Create default event loop that running in background
////    ESP_ERROR_CHECK(esp_event_loop_create_default());
//
//
////    esp_netif_t *my_sta = esp_netif_create_default_wifi_sta();
////    esp_netif_dhcpc_stop(my_sta);
////    esp_netif_ip_info_t ip_info;
////    IP4_ADDR(&ip_info.ip, 192, 168, 15, 22);
////       	IP4_ADDR(&ip_info.gw, 192, 168, 15, 1);
////       	IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
////    esp_netif_set_ip_info(my_sta, &ip_info);
//
//
//    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
//    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
//    if(flag == 1)
//    {
//		esp_netif_dhcpc_stop(eth_netif);
//		esp_netif_ip_info_t ip_info;
//		uint8_t final[4];
//		parse_ip(Interface_Info.ipv4,final);
//		IP4_ADDR(&ip_info.ip, final[0], final[1], final[2], final[3]);
//		IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
//		IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
//		esp_netif_set_ip_info(eth_netif, &ip_info);
//    }
//    else if(flag == 2)
//    {
//		esp_netif_dhcpc_stop(eth_netif);
//		esp_netif_ip_info_t ip_info;
//		IP4_ADDR(&ip_info.ip, 192, 168, 1, 2);
//		IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
//		IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
//		esp_netif_set_ip_info(eth_netif, &ip_info);
//    }
//    spi_bus_config_t buscfg = {
//        .miso_io_num = MY_ETHERNET_ENC28J60_MISO_GPIO,
//        .mosi_io_num = MY_ETHERNET_ENC28J60_MOSI_GPIO,
//        .sclk_io_num = MY_ETHERNET_ENC28J60_SCLK_GPIO,
//        .quadwp_io_num = -1,
//        .quadhd_io_num = -1,
//    };
//    ESP_ERROR_CHECK(spi_bus_initialize(MY_ETHERNET_ENC28J60_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
//    /* ENC28J60 ethernet driver is based on spi driver */
//    spi_device_interface_config_t devcfg = {
//        .command_bits = 3,
//        .address_bits = 5,
//        .mode = 0,
//        .clock_speed_hz = MY_ETHERNET_ENC28J60_SPI_CLOCK_MHZ * 1000 * 1000,
//        .spics_io_num = MY_ETHERNET_ENC28J60_CS_GPIO,
//        .queue_size = 20,
//        .cs_ena_posttrans = enc28j60_cal_spi_cs_hold_time(MY_ETHERNET_ENC28J60_SPI_CLOCK_MHZ),
//    };
//
//    spi_device_handle_t spi_handle = NULL;
//    ESP_ERROR_CHECK(spi_bus_add_device(MY_ETHERNET_ENC28J60_SPI_HOST, &devcfg, &spi_handle));
//    eth_enc28j60_config_t enc28j60_config = ETH_ENC28J60_DEFAULT_CONFIG(spi_handle);
//    enc28j60_config.int_gpio_num = MY_ETHERNET_ENC28J60_INT_GPIO;
//
//    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
////    mac_config.smi_mdc_gpio_num = -1;  // ENC28J60 doesn't have SMI interface
////    mac_config.smi_mdio_gpio_num = -1;
//    esp_eth_mac_t *mac = esp_eth_mac_new_enc28j60(&enc28j60_config, &mac_config);
//    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
//    phy_config.autonego_timeout_ms = 0; // ENC28J60 doesn't support auto-negotiation
//    phy_config.reset_gpio_num = -1; // ENC28J60 doesn't have a pin to reset internal PHY
//    esp_eth_phy_t *phy = esp_eth_phy_new_enc28j60(&phy_config);
//    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
//    esp_eth_handle_t eth_handle = NULL;
//    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
//    /* ENC28J60 doesn't burn any factory MAC address, we need to set it manually.
//       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
//    */
//    mac->set_addr(mac, (uint8_t[]) {
//        MY_ETHERNET_MAC_ADD_0, MY_ETHERNET_MAC_ADD_1, MY_ETHERNET_MAC_ADD_2, MY_ETHERNET_MAC_ADD_3, MY_ETHERNET_MAC_ADD_4, MY_ETHERNET_MAC_ADD_5
//    });
//    // ENC28J60 Errata #1 check
//    if (emac_enc28j60_get_chip_info(mac) < ENC28J60_REV_B5 && MY_ETHERNET_ENC28J60_SPI_CLOCK_MHZ < 8) {
//        ESP_LOGE(TAG_COMM, "SPI frequency must be at least 8 MHz for chip revision less than 5");
//        ESP_ERROR_CHECK(ESP_FAIL);
//    }
//    /* attach Ethernet driver to TCP/IP stack */
//    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
//    // Register user defined event handers
//    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
//    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
//    /* start Ethernet driver state machine */
//    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
//    /* It is recommended to use ENC28J60 in Full Duplex mode since multiple errata exist to the Half Duplex mode */
//#if MY_ETHERNET_ENC28J60_DUPLEX_FULL
//    /* Set duplex needs to be called after esp_eth_start since the driver is started with auto-negotiation by default */
//    enc28j60_set_phy_duplex(phy, ETH_DUPLEX_FULL);
//#endif
//}

//void parse_ip(char *str,uint8_t *arr)
//{
//	char write[50];
//	snprintf(write,sizeof(write),"%s.", str);
//    char* ip= write;
//    uint8_t count=0,number=0;
//    int array[20];
//    int size = strlen(ip);
//    ESP_LOGI(TAG_HTTP_CLIENT,"size = %d\n",size);
//    for(int i=0;i<size;i++)
//    {
//     	array[i]= (*ip++)-'0';
//    	count++;
//    	if(array[i] == -2)
//    	{
//    		count--;
//    			if(count == 3)
//    			{
//    				number = array[i-3]*100 + array[i-2]*10 + array[i-1];
//    			}
//    			else if(count == 2)
//    			{
//    				number = array[i-2]*10 + array[i-1];
//    			}
//    			else if(count == 1)
//    			{
//    				number = array[i-1];
//    			}
//    			*(arr++)=number;
//        	count=0;
//    	}
//    }
//
//}



//void parse_ip(char *str,uint8_t *arr)
//{
//	char write[50];
//	snprintf(write,sizeof(write),"%s.", str);
//    char* ip= write;
//    uint8_t count=0,count1=0,i=0,j=0,number=0;
//    int array[4];
//
//    start:
//    while(count<4)
//    {
//
//    	array[i]= (*ip++)-'0';
//    	count1++;
//    	if(array[i] == -2)
//    	{
//    		count1--;
//    			if(count1 == 3)
//    			{
//    				number = array[0]*100 + array[1]*10 + array[2];
//    			}
//    			else if(count1 == 2)
//    			{
//    				number = array[0]*10 + array[1];
//    			}
//    			else if(count1 == 1)
//    			{
//    				number = array[0];
//    			}
//    			*(arr++)=number;
//    			j++;
//
//        	memset(&array,0,sizeof(array));
//        	i=0;
//        	count1=0;
//        	count++;
//        	goto start;
//    	}
//    	i++;
//    }
//
//}
//void parse_ip(char *str,uint8_t *arr)
//{
//	char write[50];
//	snprintf(write,sizeof(write),"%s.", str);
//    char* ip= write;
//    uint8_t count=0,number=0;
//    int array[4];
//    int size = strlen(ip);
//
//    for(int i=0;i<size;i++)
//    {
//     	array[i]= (*ip++)-'0';
//    	count++;
//    	if(array[i] == -2)
//    	{
//    		count--;
//    			if(count == 3)
//    			{
//    				number = array[i-1]*100 + array[i-2]*10 + array[i-3];
//    			}
//    			else if(count == 2)
//    			{
//    				number = array[i-1]*10 + array[i-2];
//    			}
//    			else if(count == 1)
//    			{
//    				number = array[i-1];
//    			}
//    			*(arr++)=number;
//        	count=0;
//    	}
//    }
//
//}



//void eth_init_enc28j60(void)
//{
////    ESP_ERROR_CHECK(gpio_install_isr_service(0));  // isr is alredy installed
//    // Initialize TCP/IP network interface (should be called only once in application)
//
//
//    ESP_ERROR_CHECK(esp_netif_init());
//    ESP_LOGI(TAG_COMM, "1");
//    // Create default event loop that running in background
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
//    ESP_LOGI(TAG_COMM, "2");
//
//
//
//
//    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
//    ESP_LOGI(TAG_COMM, "3");
//    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
//    ESP_LOGI(TAG_COMM, "4");
//    spi_bus_config_t buscfg = {
//        .miso_io_num = MY_ETHERNET_ENC28J60_MISO_GPIO,
//        .mosi_io_num = MY_ETHERNET_ENC28J60_MOSI_GPIO,
//        .sclk_io_num = MY_ETHERNET_ENC28J60_SCLK_GPIO,
//        .quadwp_io_num = -1,
//        .quadhd_io_num = -1,
//    };
//    ESP_ERROR_CHECK(spi_bus_initialize(MY_ETHERNET_ENC28J60_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
//    ESP_LOGI(TAG_COMM, "5");
//    /* ENC28J60 ethernet driver is based on spi driver */
//    spi_device_interface_config_t devcfg = {
//        .command_bits = 3,
//        .address_bits = 5,
//        .mode = 0,
//        .clock_speed_hz = MY_ETHERNET_ENC28J60_SPI_CLOCK_MHZ * 1000 * 1000,
//        .spics_io_num = MY_ETHERNET_ENC28J60_CS_GPIO,
//        .queue_size = 20,
//        .cs_ena_posttrans = enc28j60_cal_spi_cs_hold_time(MY_ETHERNET_ENC28J60_SPI_CLOCK_MHZ),
//    };
//
//    spi_device_handle_t spi_handle = NULL;
//    ESP_ERROR_CHECK(spi_bus_add_device(MY_ETHERNET_ENC28J60_SPI_HOST, &devcfg, &spi_handle));
//    ESP_LOGI(TAG_COMM, "6");
//    eth_enc28j60_config_t enc28j60_config = ETH_ENC28J60_DEFAULT_CONFIG(spi_handle);
//    ESP_LOGI(TAG_COMM, "7");
//    enc28j60_config.int_gpio_num = MY_ETHERNET_ENC28J60_INT_GPIO;
//
//    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
//    ESP_LOGI(TAG_COMM, "8");
////    mac_config.smi_mdc_gpio_num = -1;  // ENC28J60 doesn't have SMI interface
////    mac_config.smi_mdio_gpio_num = -1;
//    esp_eth_mac_t *mac = esp_eth_mac_new_enc28j60(&enc28j60_config, &mac_config);
//    ESP_LOGI(TAG_COMM, "9");
//    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
//    ESP_LOGI(TAG_COMM, "10");
//    phy_config.autonego_timeout_ms = 0; // ENC28J60 doesn't support auto-negotiation
//    phy_config.reset_gpio_num = -1; // ENC28J60 doesn't have a pin to reset internal PHY
//    esp_eth_phy_t *phy = esp_eth_phy_new_enc28j60(&phy_config);
//    ESP_LOGI(TAG_COMM, "11");
//    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
//    ESP_LOGI(TAG_COMM, "12");
//    esp_eth_handle_t eth_handle = NULL;
//    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
//    ESP_LOGI(TAG_COMM, "13");
//    /* ENC28J60 doesn't burn any factory MAC address, we need to set it manually.
//       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
//    */
//    mac->set_addr(mac, (uint8_t[]) {
//        MY_ETHERNET_MAC_ADD_0, MY_ETHERNET_MAC_ADD_1, MY_ETHERNET_MAC_ADD_2, MY_ETHERNET_MAC_ADD_3, MY_ETHERNET_MAC_ADD_4, MY_ETHERNET_MAC_ADD_5
//    });
//    ESP_LOGI(TAG_COMM, "14");
//    // ENC28J60 Errata #1 check
//    if (emac_enc28j60_get_chip_info(mac) < ENC28J60_REV_B5 && MY_ETHERNET_ENC28J60_SPI_CLOCK_MHZ < 8) {
//        ESP_LOGE(TAG_COMM, "SPI frequency must be at least 8 MHz for chip revision less than 5");
//        ESP_ERROR_CHECK(ESP_FAIL);
//    }
//    ESP_LOGI(TAG_COMM, "15");
//    /* attach Ethernet driver to TCP/IP stack */
//    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
//    ESP_LOGI(TAG_COMM, "16");
//    // Register user defined event handers
//    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
//    ESP_LOGI(TAG_COMM, "17");
//    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
//    ESP_LOGI(TAG_COMM, "18");
//    /* start Ethernet driver state machine */
//    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
//    ESP_LOGI(TAG_COMM, "19");
//    /* It is recommended to use ENC28J60 in Full Duplex mode since multiple errata exist to the Half Duplex mode */
//#if MY_ETHERNET_ENC28J60_DUPLEX_FULL
//    /* Set duplex needs to be called after esp_eth_start since the driver is started with auto-negotiation by default */
//    enc28j60_set_phy_duplex(phy, ETH_DUPLEX_FULL);
//    ESP_LOGI(TAG_COMM, "20");
//#endif
//}


