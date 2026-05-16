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


/*
    if blue = -1 then we are yellow
    if blue = 1 then we are blue
*/



// Starting point (2700, 1800)
void strategy2(MotorController &motorController, int blue) {
    
    float degree90Left = blue * LEFT;
    float degree90right = blue * RIGHT;
    float adjust = 0.436 * blue;
   


    task_distance(motorController, 0.85);        // 0.9
    task_distance(motorController, -0.2);              // move backwards
    task_angle(motorController, degree90right);
    task_distance(motorController, 0.3);
    task_angle(motorController, degree90Left);
    task_distance(motorController, 0.4);
    task_angle(motorController, degree90right);
    task_distance(motorController, 0.75);              
    task_distance(motorController, -0.5);              // move backwards
    task_angle(motorController, degree90Left);
    task_distance(motorController, 0.5);
    task_angle(motorController, degree90right);
    task_distance(motorController, 0.5);
    task_distance(motorController, -0.8);
    task_angle(motorController, degree90right - adjust);
    task_distance(motorController, 1.65);
}
