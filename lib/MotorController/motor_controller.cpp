#include "motor_controller.h"

// Constructor - initialize the AccelStepper objects directly
MotorController::MotorController() : motorL(motorInterfaceType, stepPin1, dirPin1),
                                     motorR(motorInterfaceType, stepPin2, dirPin2)
{
    // Set initial control mode
    currentMode = SPEED_MODE;

    // Initialize odometry
    robotX = 0.0;
    robotY = 0.0;
    robotTheta = 0.0;

    // Initialize handles to null
    odometryMutex = nullptr;
    motorTaskHandle = nullptr;

    // Initialize motor speeds
    targetLeftSpeed = 0.0;
    targetRightSpeed = 0.0;

    // Initialize calibration state
    calibrationInProgress = false;
    // default speed multiplier
    speedMultiplier = 1.0f;
}

// Public Methods - Initialization
void MotorController::init()
{
    // Configure motor settings (no need to create objects)
    motorL.setMaxSpeed(maxMotorSpeed);
    motorL.setAcceleration(motorAcceleration);
    motorR.setMaxSpeed(maxMotorSpeed);
    motorR.setAcceleration(motorAcceleration);

    // Set motor inversions
    motorL.setPinsInverted(true, false, false);

    // Set initial positions
    motorL.setCurrentPosition(0);
    motorR.setCurrentPosition(0);

    // Set initial speeds
    motorL.setSpeed(0);
    motorR.setSpeed(0);

    // Initialize wall sensor pins - buttons connected to ground
    pinMode(frontRightWallSensorPin, INPUT_PULLUP);
    pinMode(frontLeftWallSensorPin, INPUT_PULLUP);
    pinMode(backRightWallSensorPin, INPUT_PULLUP);
    pinMode(backLeftWallSensorPin, INPUT_PULLUP);

    // Create mutex for odometry protection
    odometryMutex = xSemaphoreCreateMutex();
    if (odometryMutex == NULL)
    {
        Serial.println("error [motor_controller] failed to create odometry mutex");
        return;
    }

    // Create motor control task
    xTaskCreatePinnedToCore(
        motorControlTask, // Task function
        "MotorControl",   // Name
        2048,             // Stack size
        this,             // Parameters (pass this instance)
        2,                // Priority (high for motor control)
        &motorTaskHandle, // Task handle
        1                 // CPU core 1
    );

    Serial.println("debug [motor_controller] initialized successfully");
}

void MotorController::setSpeedMultiplier(float m) {
    if (m < 0.0f) m = 0.0f;
    if (m > 1.0f) m = 1.0f;
    speedMultiplier = m;

    // Apply to motor max speeds immediately
    motorL.setMaxSpeed(maxMotorSpeed * speedMultiplier);
    motorR.setMaxSpeed(maxMotorSpeed * speedMultiplier);
}

float MotorController::getSpeedMultiplier() const {
    return speedMultiplier;
}

// Public Methods - Speed Control
void MotorController::setMotorSpeeds(float leftSpeedMs, float rightSpeedMs)
{
    // Only respond to speed commands in SPEED_MODE
    if (currentMode != SPEED_MODE)
    {
        return; // Ignore speed commands during autonomous movements
    }

    // Apply multiplier to requested speeds, then convert m/s to steps/s and update target speeds
    targetLeftSpeed = msToStepsPerSec(leftSpeedMs * speedMultiplier);
    targetRightSpeed = msToStepsPerSec(rightSpeedMs * speedMultiplier);

    // Apply the speeds to the motors
    motorL.setSpeed(targetLeftSpeed);
    motorR.setSpeed(targetRightSpeed);
}

void MotorController::stop()
{
    // Emergency stop - works in both modes
    targetLeftSpeed = 0.0;
    targetRightSpeed = 0.0;

    // Stop motors immediately
    motorL.setSpeed(0);
    motorR.setSpeed(0);

    // In position mode, also stop any ongoing movements
    if (currentMode == POSITION_MODE)
    {
        motorL.stop(); // AccelStepper stop() method
        motorR.stop();
    }
}

// Public Methods - Autonomous Movement
void MotorController::moveDistance(float distance)
{
    switchToPositionMode();

    int steps = metersToSteps(distance);
    // scale max speed for this position move according to multiplier
    motorL.setMaxSpeed(maxMotorSpeed * speedMultiplier);
    motorR.setMaxSpeed(maxMotorSpeed * speedMultiplier);

    motorL.move(steps);
    motorR.move(steps);

    unsigned long lastMoveTime = millis();
    // Always wait for completion
    while (motorL.isRunning() || motorR.isRunning())
    {
        
        if(std::isnan(motorL.speed())){
            motorL.setSpeed(0);
        }
        if(std::isnan(motorR.speed())){
            motorR.setSpeed(0);
        }
        
        Serial.print("Current speed: ");
        Serial.println(motorL.speed());
        if(millis() - lastMoveTime > 500){
            lastMoveTime = millis();
            if((motorL.speed() == 0 || motorR.speed()==0) && speedMultiplier > 0.5){
            Serial.println(speedMultiplier);
            Serial.println("Broke out of loop");
            break;
            }
        
        }
        delay(10);
        Serial.println("I am running");
        // if(motorL.targetPosition() == motorL.currentPosition() || motorR.targetPosition() == motorR.currentPosition()){
        //     break;
        // }
        

    }

    motorL.stop();
    motorR.stop();

    // restore max speeds
    motorL.setMaxSpeed(maxMotorSpeed);
    motorR.setMaxSpeed(maxMotorSpeed);

    switchToSpeedMode();

//     switchToPositionMode();

//     int steps = metersToSteps(distance);
//     motorL.setMaxSpeed(maxMotorSpeed * speedMultiplier);
//     motorR.setMaxSpeed(maxMotorSpeed * speedMultiplier);
//     motorL.move(steps);
//     motorR.move(steps);

//     // --- TIMEOUT LOGIC START ---
//     unsigned long lastMoveTime = millis();
//     long lastPosL = motorL.currentPosition();
    
//     while (motorL.isRunning() || motorR.isRunning())
//     {
//         // Check if we are physically moving
//         long currentPosL = motorL.currentPosition();
        
//         if (currentPosL != lastPosL) {
//             lastPosL = currentPosL;
//             lastMoveTime = millis(); // Reset timer because we are moving
//         }

//         // If we haven't moved a single step in 2 seconds...
//         if (millis() - lastMoveTime > 2000) {
//             // Check if we are "close enough" (e.g., within 5cm)
//             float remainingDist = stepsToMeters(motorL.distanceToGo());
//             if (abs(remainingDist) < 0.10) { 
//                 Serial.println("debug [Motor] Obstacle blocking, but close enough. Finishing.");
//                 break; 
//             } else {
//                 // If we are far away, we stay in this loop until the obstacle clears
//                 // OR you can add a global 'strategy timeout' here.
//                 Serial.println("debug [Motor] Waiting for obstacle...");
//             }
//         }
        
//         delay(10); 
//     }
//     // --- TIMEOUT LOGIC END ---

//     // Force stop motors so they don't try to "jump" to the old target 
//     // once we switch modes or the obstacle leaves.
//     motorL.stop();
//     motorR.stop();

//     motorL.setMaxSpeed(maxMotorSpeed);
//     motorR.setMaxSpeed(maxMotorSpeed);
//     switchToSpeedMode();
}

void MotorController::rotate(float angle)
// angle in radian
{
    switchToPositionMode();

    float arcLength = angle * trackWidth / 2.0;
    int steps = metersToSteps(arcLength);

    // scale max speed for this position move according to multiplier
    motorL.setMaxSpeed(maxMotorSpeed * speedMultiplier);
    motorR.setMaxSpeed(maxMotorSpeed * speedMultiplier);

    motorL.move(-steps);
    motorR.move(steps);


    unsigned long lastMoveTime = millis();
    // Always wait for completion
    while (motorL.isRunning() || motorR.isRunning())
    {
        if(std::isnan(motorL.speed())){
            motorL.setSpeed(0);
        }
        if(std::isnan(motorR.speed())){
            motorR.setSpeed(0);
        }

        Serial.print("Current speed: ");
        Serial.println(motorL.speed());
        if(millis() - lastMoveTime > 500){
            lastMoveTime = millis();
            if((motorL.speed() == 0 || motorR.speed()==0) && speedMultiplier > 0.5){
            Serial.println(speedMultiplier);
            Serial.println("Broke out of loop");
            break;
            }
        
        }
        delay(10);
        Serial.println("I am running");
        // if(motorL.targetPosition() == motorL.currentPosition() || motorR.targetPosition() == motorR.currentPosition()){
        //     break;
        // }
       

        
    }

    motorL.stop();
    motorR.stop();

    // restore max speeds
    motorL.setMaxSpeed(maxMotorSpeed);
    motorR.setMaxSpeed(maxMotorSpeed);

    switchToSpeedMode();





    // switchToPositionMode();

    // float arcLength = angle * trackWidth / 2.0;
    // int steps = metersToSteps(arcLength);

    // // scale max speed for this position move according to multiplier
    // motorL.setMaxSpeed(maxMotorSpeed * speedMultiplier);
    // motorR.setMaxSpeed(maxMotorSpeed * speedMultiplier);

    // motorL.move(-steps);
    // motorR.move(steps);

    // // --- TIMEOUT LOGIC START ---
    // unsigned long lastMoveTime = millis();
    // long lastPosL = motorL.currentPosition();
    
    // while (motorL.isRunning() || motorR.isRunning())
    // {
    //     // Check if we are physically moving
    //     long currentPosL = motorL.currentPosition();
        
    //     if (currentPosL != lastPosL) {
    //         lastPosL = currentPosL;
    //         lastMoveTime = millis(); // Reset timer because we are moving
    //     }

    //     // If we haven't moved a single step in 2 seconds...
    //     if (millis() - lastMoveTime > 2000) {
    //         // Check if we are "close enough" (e.g., within 5cm)
    //         float remainingDist = stepsToMeters(motorL.distanceToGo());
    //         if (abs(remainingDist) < 0.1) { 
    //             Serial.println("debug [Motor] Obstacle blocking, but close enough. Finishing.");
    //             break; 
    //         } else {
    //             // If we are far away, we stay in this loop until the obstacle clears
    //             // OR you can add a global 'strategy timeout' here.
    //             Serial.println("debug [Motor] Waiting for obstacle...");
    //         }
    //     }
        
    //     delay(10); 
    // }
    // // --- TIMEOUT LOGIC END ---

    // // Force stop motors so they don't try to "jump" to the old target 
    // // once we switch modes or the obstacle leaves.
    // motorL.stop();
    // motorR.stop();

    // // restore max speeds
    // motorL.setMaxSpeed(maxMotorSpeed);
    // motorR.setMaxSpeed(maxMotorSpeed);

    // switchToSpeedMode();

}

void MotorController::calibrateAgainstWall(CalibrationDirection direction, WallDirection wall, bool reverse)
{
    calibrationInProgress = true;

    // Get current robot orientation
    float currentX, currentY, currentTheta;
    getCurrentPose(currentX, currentY, currentTheta);

    // Calculate target absolute angle to face the wall
    float targetAbsoluteAngle = 0.0;
    switch (wall)
    {
    case WALL_FRONT:
        targetAbsoluteAngle = reverse ? M_PI : 0.0; // 180° if reverse, 0° if forward
        break;
    case WALL_BACK:
        targetAbsoluteAngle = reverse ? 0.0 : M_PI; // 0° if reverse, 180° if forward
        break;
    case WALL_LEFT:
        targetAbsoluteAngle = reverse ? M_PI / 2 : -M_PI / 2; // 90° if reverse, -90° if forward
        break;
    case WALL_RIGHT:
        targetAbsoluteAngle = reverse ? -M_PI / 2 : M_PI / 2; // -90° if reverse, 90° if forward
        break;
    }

    // Calculate the relative angle needed to turn
    float relativeAngle = targetAbsoluteAngle - currentTheta;

    // Normalize angle to [-π, π] range
    while (relativeAngle > M_PI)
        relativeAngle -= 2.0 * M_PI;
    while (relativeAngle < -M_PI)
        relativeAngle += 2.0 * M_PI;

    // Rotate by the relative angle to face the wall
    rotate(relativeAngle);

    // Switch to speed mode for slow approach
    switchToSpeedMode();

    // Determine sensor pins - simple: front sensors if not reverse, back sensors if reverse
    int leftSensorPin, rightSensorPin;
    if (reverse)
    {
        leftSensorPin = backLeftWallSensorPin;
        rightSensorPin = backRightWallSensorPin;
    }
    else
    {
        leftSensorPin = frontLeftWallSensorPin;
        rightSensorPin = frontRightWallSensorPin;
    }

    pinMode(leftSensorPin, INPUT_PULLDOWN);
    pinMode(rightSensorPin, INPUT_PULLDOWN);

    // Move slowly towards the wall using config speed
    float approachSpeed = reverse ? -calibrationLinearSpeed : calibrationLinearSpeed;
    setMotorSpeeds(approachSpeed, approachSpeed);

    // Wait for sensor hit - for now just check if either sensor is triggered
    // TODO: Implement dual sensor logic for orientation correction
    int consecutiveHighCount = 0;
    const int requiredHighCount = 5;

    while (consecutiveHighCount < requiredHighCount)
    {
        int leftReading = digitalRead(leftSensorPin);
        int rightReading = digitalRead(rightSensorPin);

        // For now, trigger if either sensor hits
        if (leftReading == LOW || rightReading == LOW)
        {
            consecutiveHighCount++;
        }
        else
        {
            consecutiveHighCount = 0; // Reset if both sensors are LOW
        }

        delay(5); // Small delay between readings
    }

    // Stop immediately when wall is hit
    stop();

    // Reset the appropriate coordinate based on wall orientation
    // Origin (0,0) is bottom-left, right wall at 3m, top wall at 2m
    getCurrentPose(currentX, currentY, currentTheta);

    float newY = 0;
    float newX = 0;

    switch (wall)
    {
    case WALL_FRONT: // Top wall (Y = 2m)
        newY = reverse ? (2.0 - backSensorOffset) : (2.0 - frontSensorOffset);
        setCurrentPose(currentX, newY, currentTheta);
        break;
    case WALL_BACK: // Bottom wall (Y = 0m)
        newY = reverse ? (0.0 + frontSensorOffset) : (0.0 + backSensorOffset);
        setCurrentPose(currentX, newY, currentTheta);
        break;
    case WALL_LEFT:                     // Left wall (X = 0m)
        newX = 0.0 + frontSensorOffset; // Always use front sensor for side walls
        setCurrentPose(newX, currentY, currentTheta);
        break;
    case WALL_RIGHT:                    // Right wall (X = 3m)
        newX = 3.0 - frontSensorOffset; // Always use front sensor for side walls
        setCurrentPose(newX, currentY, currentTheta);
        break;
    }

    calibrationInProgress = false;

    Serial.println("debug [motor_controller] wall calibration completed");
}

// Public Methods - Odometry
void MotorController::getCurrentPose(float &x, float &y, float &theta)
{
    // Use mutex to safely read odometry values
    if (xSemaphoreTake(odometryMutex, portMAX_DELAY) == pdTRUE)
    {
        x = robotX;
        y = robotY;
        theta = robotTheta;
        xSemaphoreGive(odometryMutex);
    }
    else
    {
        // Fallback if mutex fails (shouldn't happen with portMAX_DELAY)
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        Serial.println("error [motor_controller] failed to take odometry mutex in getCurrentPose");
    }
}

void MotorController::setCurrentPose(float x, float y, float theta)
{
    // Use mutex to safely set odometry values
    if (xSemaphoreTake(odometryMutex, portMAX_DELAY) == pdTRUE)
    {
        robotX = x;
        robotY = y;
        robotTheta = theta;
        xSemaphoreGive(odometryMutex);
    }
    else
    {
        Serial.println("error [motor_controller] failed to take odometry mutex in setCurrentPose");
    }
}

// Public Methods - Status
bool MotorController::isMoving()
{
    // Check if either motor is running
    return motorL.isRunning() || motorR.isRunning();
}

bool MotorController::isCalibrating()
{
    // Return calibration state
    return calibrationInProgress;
}

ControlMode MotorController::getCurrentMode() {
    // Return current control mode
    return currentMode;
}

// Static Task Functions
void MotorController::motorControlTask(void *parameter)
{
    // Cast the parameter back to MotorController instance
    MotorController *controller = (MotorController *)parameter;

    // Continuous loop - this task runs forever
    while (1)
    {
        // Check current control mode and call appropriate update function
        if (controller->currentMode == SPEED_MODE)
        {
            controller->updateSpeedMode();
        }
        else if (controller->currentMode == POSITION_MODE)
        {
            controller->updatePositionMode();
        }

        // Small delay to prevent CPU overload and let other tasks run
        // This determines the motor control frequency (1ms = 1kHz)
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// Private Methods - Mode Management
void MotorController::switchToSpeedMode()
{
    // Set current mode
    currentMode = SPEED_MODE;

    // Configure motors for speed control
    // In speed mode, we use the target speeds set by setMotorSpeeds()
    motorL.setSpeed(targetLeftSpeed);
    motorR.setSpeed(targetRightSpeed);

    Serial.println("debug [motor_controller] switched to SPEED_MODE");
}

void MotorController::switchToPositionMode()
{
    // Set current mode
    currentMode = POSITION_MODE;

    // Stop any current speed-based movement
    motorL.setSpeed(0);
    motorR.setSpeed(0);

    // Reset target speeds for safety
    targetLeftSpeed = 0.0;
    targetRightSpeed = 0.0;

    Serial.println("debug [motor_controller] switched to POSITION_MODE");
}

// Private Methods - Internal Motor Control
void MotorController::updateSpeedMode()
{
    // Call runSpeed() on both motors - returns true if motor stepped
    bool leftStep = motorL.runSpeed();
    bool rightStep = motorR.runSpeed();

    // Update odometry if either motor stepped
    if (leftStep || rightStep)
    {
        updateOdometry(leftStep, rightStep);
    }
}

void MotorController::updatePositionMode()
{
    // Call run() on both motors - returns true if motor stepped
    bool leftStep = motorL.run();
    bool rightStep = motorR.run();

    // kaycee code
    // if(millis() - this->currentTime_ > 200){
    //     Serial.print("leftstep: ");
    //     Serial.println(leftStep);
    //     Serial.print("rightstep: ");
    //     Serial.println(rightStep);
    //     this->currentTime_ = millis();

    // }


    // Update odometry if either motor stepped
    if (leftStep || rightStep)
    {
        updateOdometry(leftStep, rightStep);
    }
}

void MotorController::updateOdometry(bool leftStep, bool rightStep)
{
    // Calculate distance traveled by each wheel
    float leftDistance = 0.0;
    float rightDistance = 0.0;

    if (leftStep)
    {
        // Check direction of left motor
        if (motorL.speed() >= 0)
        {
            leftDistance = (M_PI * wheelDiameter) / stepsPerRevolution; // Forward step
        }
        else
        {
            leftDistance = -(M_PI * wheelDiameter) / stepsPerRevolution; // Backward step
        }
    }

    if (rightStep)
    {
        // Check direction of right motor
        if (motorR.speed() >= 0)
        {
            rightDistance = (M_PI * wheelDiameter) / stepsPerRevolution; // Forward step
        }
        else
        {
            rightDistance = -(M_PI * wheelDiameter) / stepsPerRevolution; // Backward step
        }
    }

    // Calculate robot movement using differential drive kinematics
    float deltaDistance = (leftDistance + rightDistance) / 2.0;     // Average distance
    float deltaTheta = (rightDistance - leftDistance) / trackWidth; // Angular change

    // Update robot pose with mutex protection
    if (xSemaphoreTake(odometryMutex, portMAX_DELAY) == pdTRUE)
    {
        // Update orientation first
        robotTheta += deltaTheta;

        // Normalize angle to [-π, π] range
        while (robotTheta > M_PI)
            robotTheta -= 2.0 * M_PI;
        while (robotTheta < -M_PI)
            robotTheta += 2.0 * M_PI;

        // Update position using current orientation
        robotX += deltaDistance * cos(robotTheta);
        robotY += deltaDistance * sin(robotTheta);

        xSemaphoreGive(odometryMutex);
    }
    else
    {
        Serial.println("error [motor_controller] failed to take odometry mutex in updateOdometry");
    }
}

// Private Methods - Unit Conversions
float MotorController::msToStepsPerSec(float ms)
{
    // Calculate meters per step: (pi * diameter) / steps_per_revolution
    const float metersPerStep = (M_PI * wheelDiameter) / stepsPerRevolution;

    // Convert m/s to steps/s
    return ms / metersPerStep;
}

int MotorController::metersToSteps(float meters)
{
    // Calculate meters per step: (pi * diameter) / steps_per_revolution
    const float metersPerStep = (M_PI * wheelDiameter) / stepsPerRevolution;

    // Convert meters to steps
    return (int)(meters / metersPerStep);
}

float MotorController::stepsToMeters(int steps)
{
    // Calculate meters per step: (pi * diameter) / steps_per_revolution
    const float metersPerStep = (M_PI * wheelDiameter) / stepsPerRevolution;

    // Convert steps to meters
    return steps * metersPerStep;
}