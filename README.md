# Things To Improve
* There is a bug in the speed control of the stepper motors. The Accelstepper.cpp file calculates the speed of the motors, however, in some cases there is a division by zero which results in the speed being NAN. This creates a lot of issues with operation. Right now, there is a quick fix embedded in the MotorController::moveDistance
and MotorController::rotate functions but it should be improved.
* Odometry needs to be implemented and made as robust and resistant to noise as possible
