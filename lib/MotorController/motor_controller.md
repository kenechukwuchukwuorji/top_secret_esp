# MotorController Library

A FreeRTOS-based differential drive motor controller for ESP32 using AccelStepper library.

**Warning :** this library isn't standalone, it needs a config.h with certain variable (see Configuration subsection for more detail)

## Features

- **Dual Mode Operation**:
  - **Speed Mode**: Continuous speed control (meant for Raspberry Pi teleoperation for general mouvement)
  - **Position Mode**: Precise movements for autonomous actions: calibration, distance moves, rotation (meant for game actions)

- **Real-time Odometry**: Tracks robot position (x, y, θ) with mutex-protected access (**Warning**: this isn't real odometry but just track how much the motor is supposed to move)

- **Wall Calibration**: Automatic odometry reset by moving until endstop sensors hit walls

- **FreeRTOS Integration**: 
  - Dedicated motor control task (high frequency)
  - Separate odometry reporting task (20Hz) *to send position to the rasbery*

## Usage

### Basic Setup

```cpp
#include "motor_controller.h"

MotorController motorController;

void setup() {
    init_serial_communication();
    motorController.init();  // Creates FreeRTOS tasks automatically
}
```

### Speed Control (Primary Interface)
```cpp
// Called from process_serial_command() when receiving Raspberry Pi commands
motorController.setMotorSpeeds(0.2, 0.2);  // Both motors at 0.2 m/s forward
motorController.setMotorSpeeds(0.1, -0.1); // Turn right
motorController.setMotorSpeeds(0.0, 0.0);  // Stop
motorController.stop();  
```

### Autonomous Movements

```cpp
// These automatically switch from SPEED_MODE to POSITION_MODE and back
motorController.moveDistance(0.5, true);           // Move 50cm forward, wait for completion
motorController.rotate(M_PI/2, false);             // Rotate 90°, don't wait
motorController.calibrateAgainstWall(FORWARD, WALL_FRONT);  // Reset odometry against front wall
```

### Odometry

```cpp
float x, y, theta;
motorController.getCurrentPose(x, y, theta);
motorController.setCurrentPose(0, 0, 0);  // Reset to origin
```

### Status Checking
```cpp
if (motorController.isMoving()) {
    Serial.println("Motors are running");
}

if (motorController.isCalibrating()) {
    Serial.println("Wall calibration in progress");
}

if (motorController.getCurrentMode() == SPEED_MODE) {
    Serial.println("Ready for Raspberry Pi control");
}
```

## Configuration

Uses an existing `Config.h` file for all hardware parameters:
- Pin assignments (`stepPin1`, `dirPin1`, `stepPin2`, `dirPin2`)
- Physical parameters (`wheelDiameter`, `trackWidth`, `stepsPerRevolution`)
- Motor settings (`maxSpeed`, `acceleration`)
- Sensor pins for wall calibration

## Architecture

### FreeRTOS Tasks

1. **Motor Control Task**: Runs continuously, handles both speed and position control modes
2. **Odometry Report Task**: Runs at 20Hz, sends odometry data via `send_odom_data()`

### Thread Safety

- Odometry data protected by mutex for safe access between tasks
- Volatile variables for speed targets to ensure real-time updates

### Mode Switching

The controller automatically switches between modes:
- **SPEED_MODE**: Normal operation, responds to `setMotorSpeeds()` (use the `runSpeed()` function from accelStepper)
- **POSITION_MODE**: Temporary mode for autonomous actions, ignores speed commands (use the `run()` function from accelStepper)

