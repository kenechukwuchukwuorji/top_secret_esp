#include "action_handler.h"
#include "servos_handler.h"
#include "motor_controller.h"
#include "serial_handler.h"
#include "odometry.h"
#include "Config.h"

extern MotorController motorController;
extern Odometry odometry;

// Robot color (set from serial_handler, determines homing corner)
static RobotColor robotColor = COLOR_BLUE;

// Homing configuration
static const float homingApproachSpeed = 0.15;   // m/s - slow approach speed
static const float homingBackDistance = 0.5;      // 500mm - back up after first wall hit
static const float sensorWallDistance = 0.1;      // 100mm - distance from sensor to wall when triggered
static const float homingX = 0.2;                 // X position relative to corner (meters) - adjust as needed
static const float homingY = 0.2;                 // Y position relative to corner (meters) - adjust as needed
static const int HOMING_DEBOUNCE_COUNT = 5;       // Number of consecutive HIGH readings required

// Create queue for action commands
QueueHandle_t actionQueue;

void action_handler_init() {
    // Create queue for action commands (can hold up to 10 strings)
    actionQueue = xQueueCreate(10, sizeof(char) * 32);
    
    // Create FreeRTOS task for action loop
    xTaskCreate(
        action_loop_task,    // Task function
        "ActionLoop",        // Task name
        4096,               // Stack size (bytes)
        NULL,               // Parameters
        1,                  // Priority
        NULL                // Task handle
    );
    
    Serial.println("debug [action_handler] Action loop task created");
}

void action_loop_task(void* parameter) {
    char actionBuffer[32];
    
    Serial.println("debug [action_handler] Action loop task started");
    
    while (true) {
        // Wait for action command from queue
        if (xQueueReceive(actionQueue, &actionBuffer, portMAX_DELAY) == pdTRUE) {
            String action = String(actionBuffer);
            Serial.println("debug [action_handler] Processing action: " + action);
            
            if (action == "grab_block") {
                grab_blocks();
                } else if (action == "grab_block2") {
                    grab_blocks2();
            } else if (action == "release_block") {
                release_blocks();
                } else if (action == "release_block2") {
                    release_blocks2();
            } else if (action == "push_one_block") {
                push_one_block();
            } else if (action == "push_four_block") {
                push_four_block();
            } else if (action == "flip") {
                sort_one_block(true);
            } else if (action == "sort") {
                sort_one_block(false);
            } else if (action == "sort_four_blocks") {
                sort_four_blocks();
            } else if (action == "sort_two_blocks") {
                sort_two_blocks();
            } else if (action == "start_transport") {
                start_transport_mode();
            } else if (action == "stop_transport") {
                stop_transport_mode();
            } else if (action == "homing") {
                homing();
            } else if (action == "arm_push") {
                arm_push();
            } else if (action == "arm_flush") {
                arm_flush();
            } else {
                Serial.println("error [action_handler] Unknown action: " + action);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms delay using FreeRTOS
    }
}

// ---- Robot color management ----

void set_robot_color(RobotColor color) {
    robotColor = color;
    Serial.println("debug [action_handler] Robot color set to " + String(color == COLOR_BLUE ? "blue" : "yellow"));
}

RobotColor get_robot_color() {
    return robotColor;
}

// ---- Homing helpers ----

/**
 * Drive forward slowly until BOTH limit switches are triggered (debounced).
 * Uses the same debounce logic as main.cpp: requires HOMING_DEBOUNCE_COUNT
 * consecutive HIGH readings on each sensor before considering it triggered.
 */
static void drive_until_wall_hit() {
    // Configure sensor pins
    pinMode(PIN_LIMIT_RIGHT, INPUT_PULLDOWN);
    pinMode(PIN_LIMIT_LEFT, INPUT_PULLDOWN);

    // Start driving forward slowly
    motorController.setMotorSpeeds(homingApproachSpeed, homingApproachSpeed);

    int leftConsecutive = 0;
    int rightConsecutive = 0;

    // Wait until BOTH sensors have HOMING_DEBOUNCE_COUNT consecutive HIGH readings
    while (leftConsecutive < HOMING_DEBOUNCE_COUNT || rightConsecutive < HOMING_DEBOUNCE_COUNT) {
        motorController.setMotorSpeeds(homingApproachSpeed, homingApproachSpeed);
        if (digitalRead(PIN_LIMIT_LEFT) == HIGH) {
            leftConsecutive++;
        } else {
            leftConsecutive = 0;
        }

        if (digitalRead(PIN_LIMIT_RIGHT) == HIGH) {
            rightConsecutive++;
        } else {
            rightConsecutive = 0;
        }

        delay(10);  // 10ms between readings (same as main.cpp)
    }

    // Stop immediately when both sensors are confirmed
    motorController.setMotorSpeeds(0, 0);
    motorController.stop();
}

// ---- Homing sequence ----

/**
 * Homing sequence to position the robot in a known corner.
 *
 * Steps:
 *   1. Drive forward until both limit switches hit the wall
 *   2. Back up 500mm
 *   3. Turn 90° (direction depends on robot color)
 *   4. Drive forward until both limit switches hit the second wall
 *   5. Back up by (homingX + sensorWallDistance)
 *   6. Turn 90° the other way
 *   7. Back up by (homingY + sensorWallDistance)
 *
 * Blue  = one corner (turns are +90° then -90°)
 * Yellow = other corner (turns are -90° then +90°)
 */
void homing() {
    Serial.println("debug [action_handler] Starting homing sequence");
    send_action_status("homing_started");

    // Enter transport mode before starting the homing sequence
    start_transport_mode();

    // Turn direction: +1 for blue, -1 for yellow
    float turnSign = (robotColor == COLOR_BLUE) ? -1.0 : 1.0;

    // Step 1: Drive forward until both sensors hit the wall
    Serial.println("debug [action_handler] Homing step 1: approaching first wall");
    drive_until_wall_hit();
    Serial.println("debug [action_handler] First wall hit");

    odometry.setPose(0, 2-sensorWallDistance, (M_PI / 2.0));

    // Step 2: Back up 500mm
    Serial.println("debug [action_handler] Homing step 2: backing up");
    motorController.moveDistance(-homingBackDistance);

    // Step 3: Turn 90 degrees
    Serial.println("debug [action_handler] Homing step 3: turning 90 degrees");
    motorController.rotate(turnSign * (M_PI / 2.0));

    // Step 4: Drive forward until second wall hit
    Serial.println("debug [action_handler] Homing step 4: approaching second wall");
    drive_until_wall_hit();
    Serial.println("debug [action_handler] Second wall hit");

    // get current odometry
    float _, y;
    motorController.getCurrentPose(_, y, _); 
    Serial.print("CURRENT y : ");
    Serial.println(y);
    float x =  (robotColor == COLOR_BLUE) ? 3.0f-sensorWallDistance : 0.0f+sensorWallDistance;
    odometry.setPose(x, y, turnSign*M_PI + M_PI);


    // Step 5: Back up by X offset + sensor-to-wall distance
    float backDistanceX = homingX + sensorWallDistance;
    Serial.print("debug [action_handler] Homing step 5: backing up ");
    Serial.print(backDistanceX * 1000);
    Serial.println("mm");
    motorController.moveDistance(-backDistanceX);

    // Step 6: Turn 90 degrees the other way
    Serial.println("debug [action_handler] Homing step 6: turning -90 degrees");
    motorController.rotate(turnSign * (M_PI / 2.0));

    // Step 7: Back up by Y offset + sensor-to-wall distance
    float backDistanceY = homingY + sensorWallDistance;
    Serial.print("debug [action_handler] Homing step 7: backing up ");
    Serial.print(backDistanceY * 1000);
    Serial.println("mm");
    motorController.moveDistance(-backDistanceY);

    motorController.setMotorSpeeds(0, 0);
    motorController.stop();

    Serial.println("debug [action_handler] Homing sequence complete");
    send_action_status("homing_completed");

    motorController.setMotorSpeeds(0, 0);
    motorController.stop();
}