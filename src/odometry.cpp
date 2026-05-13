#include "odometry.h"
#include "motor_controller.h"
#include "Config.h"

Odometry::Odometry() : 
    otosAvailable(false),
    activeSource(STEPPER_ONLY),
    motorController(nullptr)
{
}

bool Odometry::init() {
    Serial.println("debug [odometry] Initializing odometry system");
    
    // Try to initialize OTOS
    if (initOTOS()) {
        Serial.println("debug [odometry] OTOS available");
        setOdometrySource(OTOS_ONLY);  // Prefer OTOS when available
    } else {
        Serial.println("warn [odometry] OTOS not available, using stepper only");
        setOdometrySource(STEPPER_ONLY);
    }
    
    // Start odometry reporting task (replaces motor controller's task)
    xTaskCreatePinnedToCore(
        odometryReportTask,   // Task function
        "OdometryReport",     // Name
        2048,                 // Stack size
        this,                 // Parameters (pass this instance)
        1,                    // Priority (lower than motor control)
        &odometryTaskHandle,  // Task handle
        0                     // CPU core 0
    );
    
    return true;
}

bool Odometry::initOTOS() {
    // Wire should already be initialized in main.cpp
    
    if (otos.begin(Wire)) {
        Serial.println("debug [odometry] OTOS sensor connected");
        
        // Calibrate IMU
        Serial.println("debug [odometry] Calibrating OTOS IMU...");
        otos.calibrateImu();
        
        // Set units and calibration values from config
        otos.setLinearUnit(kSfeOtosLinearUnitMeters);
        
        sfe_otos_pose2d_t offset = {otosOffsetX, otosOffsetY, otosOffsetH};
        otos.setOffset(offset);
        
        otos.setAngularScalar(otosAngularScalar);
        otos.setLinearScalar(otosLinearScalar);
        
        otos.resetTracking();
        
        otosAvailable = true;
        return true;
    }
    
    Serial.println("error [odometry] OTOS sensor not found");
    otosAvailable = false;
    return false;
}

void Odometry::setMotorController(MotorController* controller) {
    motorController = controller;
}

void Odometry::setOdometrySource(OdometrySource source) {
    activeSource = source;
    Serial.print("debug [odometry] Switched to ");
    Serial.println(source == OTOS_ONLY ? "OTOS_ONLY" : "STEPPER_ONLY");
}

void Odometry::setPose(float x, float y, float theta) {
    Serial.print("debug [odometry] Teleporting to x=");
    Serial.print(x);
    Serial.print(", y=");
    Serial.print(y);
    Serial.print(", theta=");
    Serial.println(theta);
    
    if (activeSource == OTOS_ONLY && otosAvailable) {
        // Set OTOS position
        // Note: OTOS uses different coordinate frame, so swap x/y and negate x
        sfe_otos_pose2d_t position;
        position.x = -y;  // OTOS x is robot -y
        position.y = x;   // OTOS y is robot x
        position.h = theta * 180.0f / M_PI;  // Convert radians to degrees
        
        otos.setPosition(position);
        Serial.println("debug [odometry] OTOS position updated");
    }
    
    // Always update stepper position as backup
    if (motorController != nullptr) {
        motorController->setCurrentPose(x, y, theta);
        Serial.println("debug [odometry] Stepper position updated");
    }
}

void Odometry::odometryReportTask(void* parameter) {
    // Cast parameter to Odometry instance
    Odometry* odometry = static_cast<Odometry*>(parameter);

    // Add a small startup delay to let OTOS stabilize
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1 second delay
    
    // Continuous loop - this task runs forever
    while (1) {
        float x, y, theta;
        
        // THIS IS WHERE THE FUTURE FUSION/SWITCHING WILL HAPPEN
        if (odometry->activeSource == OTOS_ONLY && odometry->otosAvailable) {
            // Get data from OTOS sensor
            sfe_otos_pose2d_t position;

            // Check if sensor is ready and get position
            if (odometry->otos.getPosition(position) == 0) {  // 0 indicates success
                x = position.y;
                y = - position.x;
                theta = position.h * M_PI / 180.0f;  // Convert degrees to radians
            } else {
                // OTOS read failed, fall back to stepper
                odometry->motorController->getCurrentPose(x, y, theta);
            }
        } else {
            // Get data from motor controller (stepper-based)
            odometry->motorController->getCurrentPose(x, y, theta);
        }
        
        // Send odometry data via serial handler
        send_odom_data(x, y, theta);
        
        // Run at 20Hz (50ms delay)
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}