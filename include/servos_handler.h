#ifndef SERVOS_HANDLER_H
#define SERVOS_HANDLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void servos_init();

void grab_blocks();
void grab_blocks2();
void start_transport_mode();
void stop_transport_mode();
void release_blocks();
void release_blocks2();
void push_one_block();
void push_four_block();
void sort_one_block(bool flip);
void sort_two_blocks();
void sort_four_blocks();

void servos_loop();
void servos_set_zero_position(byte servoIdToCalibrate);
void servos_set_position(byte servoId, int angle, bool waitForCompletion);
void wait_for_servo(byte servoId);

void arm_push();
void arm_flush();

#endif