/*
 * MQTT_ESP.h
 *  Created on: August 2, 2020
 *      Author: Marlon Frassi
 */

#include <main.h>

// === CONFIG ===
#define UART_ESP &huart1
#define FREERTOS    0
#define CMD_DELAY   2000
// ==============

void ESP_RxCallBack(void);

int ESP_SendCommand(char *command, char *reply, uint16_t delay);

int ESP_Init(void);

int ESP_Connect(char *SSID, char *PASSWD);

int MQTT_Connect(char *host, uint16_t port, char *username, char *pass,
                 char *clientID, unsigned short keepAliveInterval);

int MQTT_Pub(char *topic, char *payload);

int MQTT_PingReq(void);
