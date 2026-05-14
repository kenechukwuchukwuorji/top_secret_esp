#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "serial_handler.h"
#include "motor_controller.h"
#include "odometry.h"
#include "Config.h"
#include "servos_handler.h"
#include "action_handler.h"

const float pi = 3.14159;
const float LEFT = pi / 2;
const float RIGHT = -pi / 2;

int blue = 1;
/*
    if blue = -1 then we are yellow
    if blue = 1 then we are blue
*/

float degree90Left = blue * LEFT;
float degree90right = blue * RIGHT;


void task_distance(MotorController motorController, float distance) {
    char buffer[30];
    sprintf(buffer, "debug [main] Strategy: moving %f", distance);
    Serial.println(buffer);
    motorController.moveDistance(distance);
    vTaskDelay(pdMS_TO_TICKS(200));
}

// angle in radiants
void task_angle(MotorController motorController, float angle) {
    char buffer[30];
    sprintf(buffer, "debug [main] Strategy: rotating %f", angle);
    Serial.println(buffer);
    motorController.rotate(angle);
    vTaskDelay(pdMS_TO_TICKS(200));
}


// Starting point (2700, 1800)
void strategy1(MotorController motorController) {
    task_distance(motorController, 0.9);
    task_distance(motorController, -0.2);              // move backwards
    task_angle(motorController, degree90right);
    task_distance(motorController, 0.4);
    task_angle(motorController, degree90Left);
    task_distance(motorController, 0.3);
    task_angle(motorController, degree90right);
    task_distance(motorController, 0.9);              
    task_distance(motorController, -1.0);              // move backwards
    task_distance(motorController, degree90right);
    task_distance(motorController, 1.0);
}