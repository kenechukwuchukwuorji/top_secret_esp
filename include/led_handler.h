#ifndef LED_HANDLER_H
#define LED_HANDLER_H

#include <Arduino.h>
#include <FastLED.h>
#include "Config.h"

// Configuration for Lidar LEDs
#define LIDAR_NUM_LEDS    29
#define LED_TYPE          WS2812B
#define COLOR_ORDER       GRB

void led_init_early();
void led_animation();
void led_set_cyan();
void led_set_yellow();
void led_set_ready();
void led_set_start();
void led_set_reset();
void led_update_from_lidar(int* values, int count); // New function

#endif