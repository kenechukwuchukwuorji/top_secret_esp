#include "serial_handler.h"
#include "motor_controller.h"
#include "servos_handler.h"
#include "odometry.h"
#include "led_handler.h"
#include "action_handler.h"
#include <stdlib.h>

// External reference to the motor controller
extern MotorController motorController;
extern QueueHandle_t actionQueue;
extern Odometry odometry;

// Binary semaphore for thread-safe serial communication
SemaphoreHandle_t serialSemaphore = NULL;

// Speed multiplier is managed by MotorController directly

void init_serial_communication() {
    Serial.begin(115200);
    
    // Create binary semaphore for serial protection
    serialSemaphore = xSemaphoreCreateBinary();
    if (serialSemaphore != NULL) {
        xSemaphoreGive(serialSemaphore);  // Initially available
    }
    
    Serial.println("debug [serial_handler] ESP32 Serial Communication Initialized");
}

void process_serial_command() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        Serial.print(command);

        if (command.length() > 0) {
            // If the line contains no spaces, allow numeric multiplier values like "0.6"

            if (command.indexOf(' ') == -1) {
                const char* s = command.c_str();
                char* endptr = NULL;
                double v = strtod(s, &endptr);
                if (endptr != s && *endptr == '\0') {
                    // valid number
                    if (v < 0.0) v = 0.0;
                    if (v > 1.0) v = 1.0;
                    motorController.setSpeedMultiplier((float)v);
                    Serial.print("debug [serial_handler] speed multiplier set to ");
                    Serial.println(motorController.getSpeedMultiplier());
                    return;
                }
            }

            parse_and_execute_command(command);
        }
    }
}

void parse_and_execute_command(String command) {
    // Split command into parts
    int first_space = command.indexOf(' ');
    if (first_space == -1) return;
    
    String topic_type = command.substring(0, first_space);
    String remaining = command.substring(first_space + 1);
    // Serial.println("debug [serial_handler] received_command " + topic_type + " " + remaining);
    
    if (topic_type == "cmd_vel") {
        handle_cmd_vel_command(remaining);
    } else if (topic_type == "game_action") {
        handle_action_command(remaining);
    } else if (topic_type == "teleport") {
        handle_teleport_command(remaining);
    } else if (topic_type == "button_topic") {
        handle_button_command(remaining);
    } else if (topic_type == "lidar_led") {
        handle_lidar_led_command(remaining);
    } else {
        Serial.println("error [serial_handler] unknown_command " + topic_type);
    }
}

void handle_lidar_led_command(String data) {
    int values[LIDAR_NUM_LEDS];
    int count = 0;
    int start_index = 0;
    int end_index = data.indexOf(' ');

    // Parse space-separated integers
    while (end_index != -1 && count < LIDAR_NUM_LEDS) {
        String valStr = data.substring(start_index, end_index);
        values[count++] = valStr.toInt();
        start_index = end_index + 1;
        end_index = data.indexOf(' ', start_index);
    }
    
    // Capture the last value
    if (count < LIDAR_NUM_LEDS && start_index < data.length()) {
        values[count++] = data.substring(start_index).toInt();
    }

    if (count > 0) {
        led_update_from_lidar(values, count);
    }
}

void handle_cmd_vel_command(String data) {
    // Parse: "linear_x angular_z"
    int space_pos = data.indexOf(' ');
    if (space_pos == -1) return;
    
    float linear_x = data.substring(0, space_pos).toFloat();
    float angular_z = data.substring(space_pos + 1).toFloat();
    
    // Convert to wheel speeds using your track width from Config.h
    float left_speed = linear_x - (angular_z * trackWidth) / 2.0;
    float right_speed = linear_x + (angular_z * trackWidth) / 2.0;
    
    // Use the new motor controller
    motorController.setMotorSpeeds(left_speed, right_speed);
}

void handle_button_command(String data) {
    // Map team colors to LED colors and store robot color
    if (data == "yellow") {
        set_robot_color(COLOR_YELLOW);
        led_set_yellow();
        Serial.println("debug [serial_handler] set LED to yellow");
    } else if (data == "blue") {
        set_robot_color(COLOR_BLUE);
        led_set_cyan();
    } else if (data == "ready") {
        led_set_ready();
        Serial.println("debug [serial_handler] set LED to ready (pulsing)");
    } else if (data == "start") {
        led_set_start();
        Serial.println("debug [serial_handler] set LED to start");
    } else if (data == "reset") {
        led_set_reset();
        Serial.println("debug [serial_handler] set LED to reset");
    } else {
        // Serial.println("error [serial_handler] unknown_button_color " + data);
    }
}

void handle_action_command(String action) {
    Serial.println("debug [serial_handler] action received: " + action);

    if (action == "stop") {
        motorController.stop();
    } else if (action == "reset_esp") {
        Serial.println("debug [serial_handler] resetting ESP32...");
        delay(100);
        ESP.restart();
    } else if (action.startsWith("move ")) {
        // Parse: "move <distance_in_meters>"
        float distance = action.substring(5).toFloat();
        Serial.print("debug [serial_handler] moving distance: ");
        Serial.println(distance);
        motorController.moveDistance(distance);
    } else if (action.startsWith("rotate ")) {
        // Parse: "rotate <angle_in_radians>"
        float angle = action.substring(7).toFloat();
        Serial.print("debug [serial_handler] rotating angle: ");
        Serial.println(angle);
        motorController.rotate(angle);
    } else if (action == "grab_block" ||
               action == "grab_block2" ||
               action == "release_block" ||
               action == "release_block2" ||
               action == "push_one_block" ||
               action == "push_four_block" ||
               action == "flip" ||
               action == "sort" ||
               action == "sort_two_blocks" ||
               action == "sort_four_blocks" ||
               action == "start_transport" ||
               action == "stop_transport" ||
               action == "homing" ||
               action == "arm_push" ||
               action == "arm_flush") {
        // Send action to queue for servo task to process
        char actionBuffer[32];
        action.toCharArray(actionBuffer, 32);

        Serial.println("debug [serial_handler] queuing action: " + action);

        if (xQueueSend(actionQueue, &actionBuffer, pdMS_TO_TICKS(100)) != pdTRUE) {
            Serial.println("error [serial_handler] Failed to queue action: " + action);
        }
    } else {
        Serial.println("error [serial_handler] unknown_action " + action);
    }
}

void handle_teleport_command(String data) {
    // Parse: "x y theta"
    int first_space = data.indexOf(' ');
    int second_space = data.indexOf(' ', first_space + 1);
    
    if (first_space == -1 || second_space == -1) {
        Serial.println("error [serial_handler] Invalid teleport format");
        return;
    }
    
    float x = data.substring(0, first_space).toFloat();
    float y = data.substring(first_space + 1, second_space).toFloat();
    float theta = data.substring(second_space + 1).toFloat();
    
    Serial.print("debug [serial_handler] Teleport command: x=");
    Serial.print(x);
    Serial.print(", y=");
    Serial.print(y);
    Serial.print(", theta=");
    Serial.println(theta);
    
    // Call odometry setPose to teleport the robot
    odometry.setPose(x, y, theta);
}

// Functions to send data to Raspberry Pi
void send_button_event(String button_name) {
    if (xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("button_topic " + button_name);
        xSemaphoreGive(serialSemaphore);
    }
}

void send_action_status(String status) {
    if (xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("action_status " + status);
        xSemaphoreGive(serialSemaphore);
    }
}

void send_odom_data(float x, float y, float theta) {
    if (xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("odom " + String(x, 3) + " " + String(y, 3) + " " + String(theta, 3));
        xSemaphoreGive(serialSemaphore);
    }
}

void send_error(String error_msg) {
    if (xSemaphoreTake(serialSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.println("error " + error_msg);
        xSemaphoreGive(serialSemaphore);
    }
}
