#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "serial_handler.h"

enum OdometrySource {
    STEPPER_ONLY,
    OTOS_ONLY
};

class Odometry {
private:
    // OTOS sensor
    QwiicOTOS otos;
    bool otosAvailable;
    
    // Current odometry source
    OdometrySource activeSource;
    
    // Motor controller reference for stepper data
    class MotorController* motorController;
    
    // Task handle
    TaskHandle_t odometryTaskHandle;

public:
    Odometry();
    
    // Initialization
    bool init();
    bool initOTOS();
    
    // Configuration
    void setMotorController(class MotorController* controller);
    void setOdometrySource(OdometrySource source);
    
    // Status
    bool isOTOSAvailable() const { return otosAvailable; }
    OdometrySource getActiveSource() const { return activeSource; }
    
    // Pose manipulation
    void setPose(float x, float y, float theta);
    
    // Task function - this replaces the motor controller's odometryReportTask
    static void odometryReportTask(void* parameter);
};

#endif