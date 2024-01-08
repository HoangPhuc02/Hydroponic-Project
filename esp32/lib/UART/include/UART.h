#ifndef __UART_H
#define __UART_H
/** UART config*/
/*Pin */
#include <Arduino.h>

#include <stdio.h>
#include <String>
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include "Sensor_Actuator.h"






/*========================= UART DECLARE =======================*/
/** UART config*/

/*UART channel*/
#define UART UART_NUM_2

/*Pin */
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

/* Pattern check*/
#define PATTERN_CHR_NUM (3)
/* Buffer*/
// static uint8_t data_print[16] = {0};
static char buffer[50] = {0};

/*Buffer size*/
static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 1024;
static const int DHT_DATA_SIZE = 4;

/* Interrupt queue handler*/
extern QueueHandle_t uart2_queue ;
extern String sensor_data[MAX_SENSOR_DATA];

/* FUNCTION */
void uart_init(void);
int sendData(const char *logName, const char *data);

/**Receive data*/
void tx_task(void *pvParameters);

/**Receive data*/
void rx_task(void *pvParameters);

void config_data_sensor(char* data);

void update_sensor_data_from_uart(void);
/*===============================================================*/

#endif