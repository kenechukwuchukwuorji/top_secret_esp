#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "../../include/Config.h"
#include "../../include/serial_handler.h"

// Thin subclass of AccelStepper that overrides run() to return whether a step
// was actually taken (i.e., the result of runSpeed()), instead of whether the
// motor still has distance left to travel. This avoids modifying the library.
class TopSecretStepper : public AccelStepper {
public:
    TopSecretStepper(uint8_t interface, uint8_t pin1, uint8_t pin2)
        : AccelStepper(interface, pin1, pin2) {}

    boolean run() {
        boolean result = runSpeed();
        if (result) computeNewSpeed();
        return result;
    }
};

enum WallDirection {
    WALL_FRONT,
    WALL_BACK,
    WALL_LEFT,
    WALL_RIGHT
};

enum CalibrationDirection {
    FORWARD,
    BACKWARD
};

enum ControlMode {
    SPEED_MODE,      // Normal operation - Raspberry Pi controls via setMotorSpeeds()
    POSITION_MODE    // Autonomous moves - calibration, moveDistance, rotate
};

class MotorController {
private:
    // AccelStepper instances
    TopSecretStepper motorL;
    TopSecretStepper motorR;
    
    // Control mode
    ControlMode currentMode;
    
    // Odometry state (protected by mutex)
    float robotX, robotY, robotTheta;
    SemaphoreHandle_t odometryMutex;
    
    // Task handles
    TaskHandle_t motorTaskHandle;
    
    // Motor speeds (steps/s) - for speed mode
    volatile float targetLeftSpeed;
    volatile float targetRightSpeed;

    // Multiplier applied to all outgoing speeds (0.0 - 1.0)
    float speedMultiplier;
    
    // Calibration state
    bool calibrationInProgress;

    //Kaycee code
    unsigned long currentTime_ = millis();
    
public:
    // Constructor & Destructor
    MotorController();
    
    // Initialization
    void init();
    
    // Speed control (primary interface for Raspberry Pi)
    void setMotorSpeeds(float leftSpeedMs, float rightSpeedMs);
    void stop();
    
    // Autonomous movement commands (automatically switch modes)
    void moveDistance(float distance);
    void rotate(float angle);
    void calibrateAgainstWall(CalibrationDirection direction, WallDirection wall, bool reverse = false);
    
    // Odometry
    void getCurrentPose(float &x, float &y, float &theta);
    void setCurrentPose(float x, float y, float theta);
    
    // Status
    bool isMoving();
    bool isCalibrating();
    ControlMode getCurrentMode();
    
    // Task functions (static for FreeRTOS)
    static void motorControlTask(void* parameter);

    // Speed multiplier (0.0 - 1.0) applied to all speed outputs
    void setSpeedMultiplier(float m);
    float getSpeedMultiplier() const;
    
private:
    // Mode management
    void switchToSpeedMode();
    void switchToPositionMode();
    
    // Internal motor control
    void updateSpeedMode();
    void updatePositionMode();
    void updateOdometry(bool leftStep, bool rightStep);
    
    // Unit conversions (using config.h constants)
    float msToStepsPerSec(float ms);
    int metersToSteps(float meters);
    float stepsToMeters(int steps);
};

#endif