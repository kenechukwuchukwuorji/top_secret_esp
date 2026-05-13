#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Configuration selection
#define SIMA false  // Set to true to use SIMA values, false for alternative values


#define PIN_ELEVATOR_LIMIT_SWITCH 35
#define PIN_LIMIT_RIGHT 34 // limit switch at theh bottom of the robot for homing and reseting the odometry
#define PIN_LIMIT_LEFT 35 // same pin as elevator but can be touch when elevator is up so they won't trigger at the same time

// I2C Configuration
#define I2C_SDA_PIN 14
#define I2C_SCL_PIN 27

// Motor interface type for AccelStepper
#define motorInterfaceType 1

// Serial interface pin for servo board
#define S_RXD 16
#define S_TXD 17

// led pins
#define LIDAR_LED_PIN 13

// Robot physical parameters
#if SIMA
    // Motor pins
    
    #define dirPin1 25   // Left motor direction
    #define stepPin2 33  // Right motor step
    #define dirPin2 32   // Right motor direction

    const float wheelDiameter = 0.0675;      // meters
    const float trackWidth = 0.108;          // meters  
    const int stepsPerRevolution = 200;

    // OTOS Calibration Values (currently not handled on the sima)
    const float otosOffsetX = 0.0015;         // 1.5mm to the left
    const float otosOffsetY = 0.03785;        // 37.85mm back
    const float otosOffsetH = -180.0;         // 180 degrees
    const float otosAngularScalar = 0.989;
    const float otosLinearScalar = 0.973;
    
    // Robot physical offsets
    const float frontSensorOffset = 0.05;    // Distance from front sensor to wheel axis (meters)
    const float backSensorOffset = 0.05;     // Distance from back sensor to wheel axis (meters)

    // Wall calibration sensor pins
    #define frontRightWallSensorPin 5    
    #define frontLeftWallSensorPin 5
    #define backRightWallSensorPin 5
    #define backLeftWallSensorPin 5

    // Motor performance settings
    const float maxMotorSpeed = 20000;       // steps/s
    const float motorAcceleration = 2000;    // steps/s²

    // Calibration settings
    const float calibrationLinearSpeed = 0.05;  // m/s - slow approach speed for wall calibration
#else
    // Motor pins
    #define stepPin2 26  // Left motor step
    #define dirPin2 25   // Left motor direction
    #define stepPin1 33  // Right motor step
    #define dirPin1 32   // Right motor direction

    // Alternative configuration 
    const float wheelDiameter = 0.083;      // meters
    const float trackWidth = 0.210;          // meters  
    const int stepsPerRevolution = 200;

    // OTOS Calibration Values
    const float otosOffsetX = 0.112;   // found by trial and error (mesuring didn't work ...)
    const float otosOffsetY = -0.0631;
    const float otosOffsetH = -90.0;         // 90 degrees
    const float otosAngularScalar = 0.989;
    const float otosLinearScalar = 0.998;
    
    // Robot physical offsets
    const float frontSensorOffset = 0.05;    // Distance from front sensor to wheel axis (meters)
    const float backSensorOffset = 0.05;     // Distance from back sensor to wheel axis (meters)

    // Wall calibration sensor pins
    #define frontRightWallSensorPin 5    
    #define frontLeftWallSensorPin 5
    #define backRightWallSensorPin 5
    #define backLeftWallSensorPin 5

    // Motor performance settings
    const float maxMotorSpeed = 300;       // steps/s
    const float motorAcceleration = 100;    // steps/s²

    // Calibration settings
    const float calibrationLinearSpeed = 0.05;  // m/s - slow approach speed for wall calibration
#endif

#endif