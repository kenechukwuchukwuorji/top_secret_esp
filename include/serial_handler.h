#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <Arduino.h>

// Initialize serial communication
void init_serial_communication();

// Process incoming serial commands
void process_serial_command();
void parse_and_execute_command(String command);

// Command handlers
void handle_cmd_vel_command(String data);
void handle_action_command(String action);
void handle_teleport_command(String data);
void handle_button_command(String data);
void handle_lidar_led_command(String data); // New handler

// Data senders
void send_button_event(String button_name);
void send_action_status(String status);
void send_odom_data(float x, float y, float theta);
void send_error(String error_msg);

#endif