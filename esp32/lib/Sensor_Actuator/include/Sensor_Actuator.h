#ifndef __SENSOR_ACTUATOR_H
#define __SENSOR_ACTUATOR_H
#include <Arduino.h>

#include <stdio.h>
#include <string.h>
#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdlib.h>


/*======================= SENSOR Declare ========================*/
//#define MAX_SENSOR_DATA 6

typedef enum {
  Temp,
  Humi,
  ADC1,
  ADC2,
  ADC3,
  ADC4,
  MAX_SENSOR_DATA   
} sensor_data_type_t;
// String sensor_data[MAX_SENSOR_DATA] = {};


uint8_t sensor_check(String sensor_data[], int size);
/*================================================================*/


/*======================== ACTUATOR DECLARE ======================*/
/*Actuator control*/
typedef enum
{
    Relay = 0,
    Servo,
    Led,
    MAX_DEVICE
} Actuator_Type_t;
typedef enum
{
    ACTUATOR_OFF = 0,
    ACTUATOR_ON
} Actuator_State_t;

typedef struct
{
    Actuator_Type_t ActuatorId;
    Actuator_State_t ActuatorState;
    /* data */
} Actuator_Init_t;



// extern char actuator_name[3][6];
// extern char actuator_state[2][4] ;

// queue handler to control actuator
extern QueueHandle_t actuator_control_queue;

// Todo update real state of actuator everytime we have reset
/*
Exam problem
if Actuator 1 is on
and we reset the state when reset will be off so when check condition we can turn off Acuator 1

Idea
using uart get state of every actuators and update it after uart init

*/
extern Actuator_Init_t actuators[MAX_DEVICE];

/* FUNCTION*/
void actuator_init(void);

void actuator_config(Actuator_Init_t *actuator, Actuator_Type_t type, Actuator_State_t state);

void set_state_actuator(Actuator_Init_t *actuator, Actuator_State_t state);
Actuator_State_t get_state_actuator(Actuator_Init_t *actuator);

/*TODO queue send to activate actuator*/

/*===============================================================*/
#endif