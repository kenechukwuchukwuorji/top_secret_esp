#ifndef ACTION_HANDLER_H
#define ACTION_HANDLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Robot color enum (determines homing corner)
enum RobotColor {
    COLOR_BLUE,
    COLOR_YELLOW
};

// Queue for action commands
extern QueueHandle_t actionQueue;

void action_handler_init();
void action_loop_task(void* parameter);

// Homing sequence
void homing();

// Robot color management
void set_robot_color(RobotColor color);
RobotColor get_robot_color();

#endif