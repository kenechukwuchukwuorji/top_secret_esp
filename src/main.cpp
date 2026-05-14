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
#include "led_handler.h"
#include "Strategy1.h"

// Global objects
MotorController motorController;
Odometry odometry;  // Make sure this is declared before serial_handler includes

// Forward declaration for the background strategy task
void strategy_task(void* parameter);

void setup() {
    // Initialize serial communication
    init_serial_communication();

    Serial2.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);

    // Initialize LEDs early (turn everything off)
    led_init_early();
    
    // Initialize I2C bus (shared by multiple devices)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.println("debug [main] I2C initialized");
    
    // Initialize odometry system
    // odometry.init();
    
    // Link motor controller to odometry
    // odometry.setMotorController(&motorController);
    
    // Initialize motor controller
    motorController.init();

    // Initialize servos
    servos_init();

    // Initialize action handler
    action_handler_init();

    // Run LED animation at the end
    led_animation();
    
    Serial.println("debug [main] ESP32 Robot System Initialized");

    // setup pins for limit switches
    pinMode(PIN_LIMIT_RIGHT, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_LEFT, INPUT_PULLDOWN);
    // while(1){
    //     Serial.print("Limit Right: ");
    //     Serial.print(digitalRead(PIN_LIMIT_RIGHT));
    //     Serial.print(" | Limit Left: ");
    //     Serial.println(digitalRead(PIN_LIMIT_LEFT));
    // }

    // Start a background strategy task that moves forward 0.5m then back 0.5m
    xTaskCreate(
        strategy_task,   // task function
        "Strategy",     // name
        4096,            // stack size
        NULL,            // parameters
        1,               // priority
        NULL             // task handle
    );
    // input both limit pins at while (1) serial print them just for temperory debugging
    
}

// Background strategy: simple forward then backward movement
void strategy_task(void* parameter) {
    // small delay to allow system to fully initialize
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Wait for right limit switch to be consistently HIGH (debounce)
    const int DEBOUNCE_COUNT = 50;
    int rightCount = 0;

    Serial.println("debug [main] Strategy: waiting for right limit switch (debounce)");
    while (rightCount < DEBOUNCE_COUNT) {
        int rightState = digitalRead(PIN_LIMIT_LEFT);
        Serial.print("debug [main] Strategy: right limit raw=");
        Serial.print(rightState);
        Serial.print(" count=");
        Serial.println(rightCount);

        if (rightState == LOW) {
            rightCount++;
        } else {
            rightCount = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("debug [main] Strategy: right limit confirmed, executing moves");
    // Serial.println("debug [main] Strategy: moving forward 1.2m");
    // motorController.moveDistance(1.2);
    // vTaskDelay(pdMS_TO_TICKS(200));
    // Serial.println("debug [main] Strategy: moving backward 1.0m");
    // motorController.moveDistance(-1.0);

    strategy1(motorController);
    Serial.println("debug [main] Strategy: completed");
    vTaskDelete(NULL);
}



void loop() {
    // Process incoming serial commands
    process_serial_command();
    // servos_loop();
    delay(10);  // 100Hz main loop
}
