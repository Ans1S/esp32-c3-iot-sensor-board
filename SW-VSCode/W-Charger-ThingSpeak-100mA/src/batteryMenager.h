#ifndef BATTERYMENAGER_H_INCLUDED
#define BATTERYMENAGER_H_INCLUDED
#include <Arduino.h>
#include <ThingSpeak.h>
#include <WiFi.h>
#include <esp_adc_cal.h>
#include <WiFiManager.h>
#include "config.h"  // Include configuration file
/*
*    DEFINE SECTION
*/

static int volt = 0;
static int battery_p = 0;

void batteryMenagement(void *parameters);
uint32_t readADC_Cal();
void set_status(int8_t s);
void blinkTask(void *parameter);

#endif