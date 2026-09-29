#pragma once
#include "driver/gpio.h"

// --- Hall Effect sensors ---
#define PIN_HALL_A1 GPIO_NUM_33
#define PIN_HALL_A2 GPIO_NUM_32
#define PIN_HALL_B1 GPIO_NUM_36
#define PIN_HALL_B2 GPIO_NUM_39

// --- DC Drivers ---
#define PIN_DC_A1 GPIO_NUM_18 
#define PIN_DC_A2 GPIO_NUM_17
#define PIN_DC_B1 GPIO_NUM_19
#define PIN_DC_B2 GPIO_NUM_21

// --- Servomotors ---
#define PIN_SERVO_1 GPIO_NUM_26
#define PIN_SERVO_2 GPIO_NUM_27

// --- Ultrasonic sensor ---
// Board labels are inverted for HC-SR04 usage: RX drives TRIG, TX reads ECHO.
#define PIN_US_TRIG GPIO_NUM_16
#define PIN_US_ECHO GPIO_NUM_35

// --- Utility ---
#define PIN_BATTERY_LEVEL GPIO_NUM_34
#define PIN_BOOT GPIO_NUM_0         
#define PIN_SW_TIRETTE GPIO_NUM_23
#define PIN_SW_TEAM GPIO_NUM_25
#define PIN_TEAM_RGB GPIO_NUM_22
#define PIN_STATUS_LED GPIO_NUM_4
