#include "servos_handler.h"
#include "STSServoDriver.h"
#include "Config.h"
#include "motor_controller.h"

extern MotorController motorController;

STSServoDriver servos;

// Mutex for servo communication
SemaphoreHandle_t servoMutex;

bool blocks_currently_sorting = false;
bool blocks_currently_waiting = false;
bool all_blocks_sorted = true;

// Sort completion ID counters
int sort_four_blocks_id = 1;
int sort_two_blocks_id = 1;

// Servo position constants (in degrees)
// Servo 7 - Grabber
const int GRABBER_GRAB_ANGLE = -50;
const int GRABBER_INTERMEDIATE_ANGLE = 70;
const int GRABBER_RELEASE_ANGLE = 330;

// Servo 6 - Rotator
const int ROTATOR_PLATFORM_ANGLE = 270;
const int ROTATOR_INTERMEDIATE_ANGLE = 180;
const int ROTATOR_RELEASE_ANGLE = 90;

// Servo 5 - Elevator
// TODO do a proper offset calculation but I couldn't make it work the proper way
int ELEVATOR_LOW_ANGLE = 90;
int ELEVATOR_INTERMEDIATE_ANGLE = -350;
int ELEVATOR_PLATFORM_ANGLE = -430;
int ELEVATOR_HIGH_ANGLE = -450;

// Servo 4 - Pusher
const int PUSHER_DEFAULT_ANGLE = 5;
const int PUSHER_INTERMEDIATE_ANGLE = 120;
const int PUSHER_PUSH_ANGLE = 195;

// Servo 3 - Distributor
const int DISTRIBUTOR_HORIZONTAL = 95;
const int DISTRIBUTOR_DISTRIBUTING = 210;

// Servo 2 - Sorter
const int SORTER_UP = 72;
const int SORTER_MIDDLE = 95;
const int SORTER_FLIP = 105;
const int SORTER_DOWN = 130;
const int SORTER_DISTRIBUTING = 153;

// Servo 8 - Pusher
const int SORTER_PUSHER_RETRACTED = 62;
const int SORTER_PUSHER_INTERMEDIATE = 95;
const int SORTER_PUSHER_PUSHING = 140;

// Servo 9 - Arm Right (SCS0009, base-1024)
// Raw positions: flush=512, push=245
// Inverse of map(angle, 0, 360, 0, 4095): angle = raw * 360 / 4095
const int ARM_RIGHT_FLUSH = 48;    // → map gives 511 ≈ raw 512
const int ARM_RIGHT_PUSH = 20;     // → map gives 250 ≈ raw 245

// Servo 16 Grabber 2
const int GRABBER2_GRAB_ANGLE = 291;
const int GRABBER2_RELEASE_ANGLE = 95;


// servo 15 big rotator
const int ROTATOR2_RELEASE_ANGLE = 345;
const int ROTATOR2_GRAB_ANGLE = 263;


void servos_init(){
    // Create mutex for servo communication
    servoMutex = xSemaphoreCreateMutex();
    
    delay(1000);
    if (!servos.init(&Serial2))
    {
      Serial.println("error [servos_handler] Failed to initialize servo driver");
      return; // Return immediately if driver init fails
    }

    // add 30 degree of offset to servo 15
    servos.setPositionOffset(15, map(30, 0, 360, 0, 4095));
    
    // limit servo 16 maximum torque to ~10%
    servos.writeTwoBytesRegister(16, STSRegisters::TORQUE_PROTECTION_TH, 15, false);
    servos.writeTwoBytesRegister(16, STSRegisters::OVERLOAD_TORQUE, 15, false);



    // while(1){
    //     // display position of servo 16 and 15 (degrees)
    //     Serial.print("Servo 16 pos: ");
    //     Serial.print(servos.getCurrentPosition(16)/4096.0*360.0);
    //     Serial.print(" Servo 15 pos: ");
    //     Serial.println(servos.getCurrentPosition(15)/4096.0*360.0);
    // }

    // move servo 15 and 16 to release
    servos_set_position(15, ROTATOR2_RELEASE_ANGLE, false);
    servos_set_position(16, GRABBER2_RELEASE_ANGLE, true);

    // Check if servos are actually powered (Ping elevator servo)
    // If Ping fails (-1), we assume E-STOP is active or power is cut.
    if (servos.ping(5) == -1) {
        Serial.println("error [servos_handler] Servo 5 not responding (E-STOP active?). Aborting servo init.");
        return;
    }

    delay(1000);
    
    Serial.println("debug [servos_handler] Servo initialization completed");
}

void grab_blocks(){
    Serial.print("debug [servos_handler] blocks_currently_sorting: ");
    Serial.println(blocks_currently_sorting);
    Serial.print("debug [servos_handler] blocks_currently_waiting: ");
    Serial.println(blocks_currently_waiting);

    servos_set_position(7, GRABBER_GRAB_ANGLE, true);
    servos_set_position(6, ROTATOR_INTERMEDIATE_ANGLE, false);
    servos_set_position(5, ELEVATOR_INTERMEDIATE_ANGLE, false);

    vTaskDelay(200);
    Serial.println("action_status grab_blocks completed");
    wait_for_servo(5);

    if(blocks_currently_sorting){
        Serial.println("debug [servos_handler] Currently sorting, setting waiting flag");
        blocks_currently_waiting = true;
        return;
    }
    servos_set_position(5, ELEVATOR_HIGH_ANGLE, false);

    servos_set_position(6, ROTATOR_PLATFORM_ANGLE, true);
    wait_for_servo(5);
    servos_set_position(5, ELEVATOR_PLATFORM_ANGLE, true);

    servos_set_position(7, GRABBER_INTERMEDIATE_ANGLE, true);

    servos_set_position(6, ROTATOR_RELEASE_ANGLE, false);
    servos_set_position(5, ELEVATOR_LOW_ANGLE, false);

    wait_for_servo(6);
    servos_set_position(7, GRABBER_RELEASE_ANGLE, false);
    wait_for_servo(5);
    blocks_currently_sorting = true;
}

void start_transport_mode(){
    servos_set_position(6, ROTATOR_INTERMEDIATE_ANGLE, false);
    servos_set_position(5, ELEVATOR_HIGH_ANGLE, true);
    servos_set_position(6, ROTATOR_PLATFORM_ANGLE, true);
    Serial.println("debug [servos_handler] Transport mode started");
}

void stop_transport_mode(){
    servos_set_position(6, ROTATOR_RELEASE_ANGLE, true);
    servos_set_position(7, GRABBER_RELEASE_ANGLE, true);
    servos_set_position(6, ROTATOR_RELEASE_ANGLE, false);
    servos_set_position(5, ELEVATOR_LOW_ANGLE, false);
    Serial.println("debug [servos_handler] Transport mode stopped");
}

void release_blocks(){
    servos_set_position(8, SORTER_PUSHER_RETRACTED, false);
    servos_set_position(2, SORTER_DISTRIBUTING, true);
    servos_set_position(3, DISTRIBUTOR_DISTRIBUTING, true);
    // move back 0.05m
    vTaskDelay(300);
    motorController.moveDistance(0.1);
    servos_set_position(3, DISTRIBUTOR_HORIZONTAL, false);
    motorController.moveDistance(-0.20);
    motorController.moveDistance(0.20);
    // add a delay to ensure the elevator is up before rotating
    // vTaskDelay(500);

    Serial.println("action_status release_block completed");
    if(!all_blocks_sorted){
        sort_two_blocks();
    }
}


void push_one_block(){
    servos_set_position(4, PUSHER_PUSH_ANGLE, true);
    servos_set_position(4, PUSHER_DEFAULT_ANGLE, true);
    Serial.println("action_status push_block completed");
}

void push_four_block(){
    for(int i=0; i<4; i++){
        servos_set_position(4, PUSHER_PUSH_ANGLE, true);
        servos_set_position(4, PUSHER_DEFAULT_ANGLE, true);
    }
    Serial.println("action_status push_block completed");
}

void sort_one_block(bool flip){
    if (flip){
        servos_set_position(8, SORTER_PUSHER_RETRACTED, true);
        servos_set_position(2, SORTER_FLIP, true);
        // push_one_block();
        servos_set_position(4, PUSHER_INTERMEDIATE_ANGLE, true);
        servos_set_position(2, SORTER_DOWN, true);
        servos_set_position(4, PUSHER_PUSH_ANGLE, true);
        servos_set_position(4, PUSHER_DEFAULT_ANGLE, false);

    }else{
        servos_set_position(8, SORTER_PUSHER_RETRACTED, false);
        servos_set_position(2, SORTER_UP, true);
        servos_set_position(4, PUSHER_INTERMEDIATE_ANGLE, true);
        servos_set_position(2, SORTER_MIDDLE, true);
        servos_set_position(2, SORTER_DOWN, false);
        // push_one_block();
        servos_set_position(4, PUSHER_PUSH_ANGLE, true);
        servos_set_position(4, PUSHER_DEFAULT_ANGLE, false);

    }

    // servos_set_position(8, SORTER_PUSHER_INTERMEDIATE, true);
    delay(100);
    servos_set_position(8, SORTER_PUSHER_RETRACTED, true);
    servos_set_position(8, SORTER_PUSHER_PUSHING, true);
    wait_for_servo(4);
}

void sort_four_blocks(){
    sort_one_block(false);
    sort_one_block(false);
    sort_one_block(true);
    sort_one_block(true);
    Serial.println("action_status sort_four_blocks completed sort4_" + String(sort_four_blocks_id));
    sort_four_blocks_id++;
    blocks_currently_sorting = false;
    if(blocks_currently_waiting){

        servos_set_position(6, ROTATOR_PLATFORM_ANGLE, true);
        servos_set_position(5, ELEVATOR_PLATFORM_ANGLE, true);

        servos_set_position(7, GRABBER_INTERMEDIATE_ANGLE, true);

        servos_set_position(6, ROTATOR_RELEASE_ANGLE, false);
        servos_set_position(5, ELEVATOR_LOW_ANGLE, false);

        wait_for_servo(6);
        servos_set_position(7, GRABBER_RELEASE_ANGLE, false);
        wait_for_servo(5);
        blocks_currently_waiting = false;
    }
}

void sort_two_blocks(){
    if (all_blocks_sorted){
        sort_one_block(false);
        sort_one_block(false);
        all_blocks_sorted = false;
    }else{
        sort_one_block(true);
        sort_one_block(true);
        all_blocks_sorted = true;
    }
    Serial.println("action_status sort_two_blocks completed sort2_" + String(sort_two_blocks_id));
    sort_two_blocks_id++;
    blocks_currently_sorting = false;
    if(blocks_currently_waiting){
        servos_set_position(6, ROTATOR_PLATFORM_ANGLE, true);
        servos_set_position(5, ELEVATOR_PLATFORM_ANGLE, true);

        servos_set_position(7, GRABBER_INTERMEDIATE_ANGLE, true);

        servos_set_position(6, ROTATOR_RELEASE_ANGLE, false);
        servos_set_position(5, ELEVATOR_LOW_ANGLE, false);

        wait_for_servo(6);
        servos_set_position(7, GRABBER_RELEASE_ANGLE, false);
        wait_for_servo(5);
        blocks_currently_waiting = false;
    }
}

void servos_loop(){
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.print(" Servo 1 pos: ");
        Serial.print(servos.getCurrentPosition(1));
        Serial.print(",");
        Serial.print(servos.getCurrentPosition(1)/4096.0*360.0);

        Serial.print(" Servo 2 pos: ");
        Serial.print(servos.getCurrentPosition(2));
        Serial.print(",");
        Serial.print(servos.getCurrentPosition(2)/4096.0*360.0);

        Serial.print(" Servo 3 pos: ");
        Serial.print(servos.getCurrentPosition(3));
        Serial.print(",");
        Serial.print(servos.getCurrentPosition(3)/4096.0*360.0);

        Serial.print(" Servo 4 pos: ");
        Serial.print(servos.getCurrentPosition(4));
        Serial.print(",");
        Serial.println(servos.getCurrentPosition(4)/4096.0*360.0);
        
        xSemaphoreGive(servoMutex);
    }
    delay(100);
}

void servos_set_position(byte servoId, int angle, bool waitForCompletion){
    // Take mutex before servo operation
    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // angle in degree
        int position = map(angle, 0, 360, 0, 4095);
        servos.setTargetPosition(servoId, position);
        vTaskDelay(100);
        
        // Release mutex after communication
        xSemaphoreGive(servoMutex);
        
        if (waitForCompletion) {
            wait_for_servo(servoId);
        }
    } else {
        Serial.println("error [servos_handler] Failed to acquire servo mutex for servo " + String(servoId));
    }
}

void wait_for_servo(byte servoId) {
    while (true) {
        bool isMoving = false;
        
        // Check if servo is moving (requires mutex)
        if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            isMoving = servos.isMoving(servoId);
            xSemaphoreGive(servoMutex);
        }
        
        if (!isMoving) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void arm_push() {
    servos_set_position(1, ARM_RIGHT_PUSH, true);
    Serial.println("debug [servos_handler] arm_push completed");
}

void arm_flush() {
    servos_set_position(1, ARM_RIGHT_FLUSH, true);
    Serial.println("debug [servos_handler] arm_flush completed");
}

// New: variant 2 of grab/release actions
// grab_blocks2: rotate first, then close grabber and wait for completion
void grab_blocks2(){
    // rotate rotator2 to grab position, then close grabber2 and wait
    servos_set_position(15, ROTATOR2_GRAB_ANGLE, true);
    servos_set_position(16, GRABBER2_GRAB_ANGLE, true);
    Serial.println("action_status grab_blocks2 completed");
}

// release_blocks2: rotate first, then open grabber and wait for completion
void release_blocks2(){
    // rotate rotator2 to release position, then open grabber2 and wait
    servos_set_position(15, ROTATOR2_RELEASE_ANGLE, true);
    servos_set_position(16, GRABBER2_RELEASE_ANGLE, true);
    Serial.println("action_status release_blocks2 completed");
}

void servos_set_zero_position(byte servoIdToCalibrate){

        // 1. remove current offset and get current position
        servos.setPositionOffset(servoIdToCalibrate, 0);
        delay(100);

        int currentPosition = servos.getCurrentPosition(servoIdToCalibrate); 

        Serial.print("Servo ");
        Serial.print(servoIdToCalibrate);
        Serial.print(" is at absolute position: ");
        Serial.println(currentPosition);

        // 2. Calculate the offset, constrained to valid range (-2047 to 2047)
        int positionOffset;
        if (currentPosition <= 2047) {
            positionOffset = currentPosition;
        } else {
            positionOffset = 2047 + 4096 - currentPosition;
        }
        
        Serial.print("Calculated offset: ");
        Serial.println(positionOffset);

        // 3. Write this offset using the dedicated library function which handles the EEPROM lock.
        if (servos.setPositionOffset(servoIdToCalibrate, positionOffset)) {
            Serial.println("Successfully set new zero position.");

            // 4. Verify by reading the position again. It should now be 0 (or very close).
            delay(100); // Delay for the servo to update its internal state
            int newPosition = servos.getCurrentPosition(servoIdToCalibrate);
            Serial.print("Servo ");
            Serial.print(servoIdToCalibrate);
            Serial.print(" new logical position: ");
            Serial.println(newPosition);
        } else {
            Serial.println("Failed to set position offset.");
        }
}
