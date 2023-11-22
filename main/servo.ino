// Teensy Flight Controller - QuickDrehm
// Authors: Kevin Plaizer
// Version: Alpha 1.0

//========================================================================================================================//
//                                                      SERVO SETUP                                                       //
//========================================================================================================================//

// PWM servo pins
const int servo1Pin = 0;
const int servo2Pin = 1;
const int servo3Pin = 5;
const int servo4Pin = 6;
const int servo5Pin = 7;
const int servo6Pin = 10;
const int servo7Pin = 11;
const int servo8Pin = 12;
const int servo9Pin = 14;
const int servo10Pin = 15;

const int servoPins[MAX_SERVO_COUNT] = {0, 1, 5, 6, 10, 11, 12, 14, 15}; // Weird servo ordering due to hardware requirements of dshot

PWMServo servo[SERVO_COUNT];  // Create servo objects to control a servo or ESC with PWM

// comment servos you are not using
void initServoScales(boundedRangeScaler_t servoScales[]) {
  servoScalerInitHelper(
    servoScales[SERVO_0], // update roll rc_scaler
    50.0f, -90.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.1, 1.0 // use values of 0.0-1.0, change these if you want to limit servo movement 
  );

  servoScalerInitHelper(
    servoScales[SERVO_1], // update roll rc_scaler
    -90.0f, 50.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.0, 0.9 // use values of 0.0-1.0, change these if you want to limit servo movement 
  );

  servoScalerInitHelper(
    servoScales[SERVO_2], // update roll rc_scaler
    -45.0f, 45.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.05, 0.95 // use values of 0.0-1.0, change these if you want to limit servo movement 
  );

  servoScalerInitHelper(
    servoScales[SERVO_3], // update roll rc_scaler
    45.0f, -45.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.05, 0.95 // use values of 0.0-1.0, change these if you want to limit servo movement 
  );
  
  servoScalerInitHelper(
    servoScales[SERVO_4], // update roll rc_scaler
    0.0f, 90.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.48, 0.92 // use values of 0.0-1.0, change these if you want to limit servo movement 
  ); 

  servoScalerInitHelper(
    servoScales[SERVO_5], // update roll rc_scaler
    90.0f, 0.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.07, 0.5 // use values of 0.0-1.0, change these if you want to limit servo movement 
  );

  servoScalerInitHelper(
    servoScales[SERVO_6], // update roll rc_scaler
    -90.0f, 90.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.0, 1.0 // use values of 0.0-1.0, change these if you want to limit servo movement 
  );

  servoScalerInitHelper(
    servoScales[SERVO_7], // update roll rc_scaler
    -90.0f, 90.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.0, 1.0 // use values of 0.0-1.0, change these if you want to limit servo movement 
  ); 

  servoScalerInitHelper(
    servoScales[SERVO_8], // update roll rc_scaler
    -90.0f, 90.0f, // input min and max set to servo/control surface angles, use a higher then lower number to swap directions
    0.0, 1.0 // use values of 0.0-1.0, change these if you want to limit servo movement 
  );
}

void initServos(boundedRangeScaler_t servoScales[]) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    // these values should work for most servos
    servo[i].attach(servoPins[i], 900, 2100); // Pin, min PWM value, max PWM value
  }

  initServoScales(servoScales);
}

void servoScalerInitHelper(boundedRangeScaler_t &servoScales, float angle_min, float angle_max, float range_min, float range_max) {
    boundedRangeScalerInit(
    &servoScales, // update roll rc_scaler
    angle_min, angle_max, // input min and max set to servo/control surface angles
    range_min * 180.0f, range_max * 180.0f // output min and max don't change as this relates to servo library code
  );
}

void sendServoCommands(boundedRangeScaler_t servoScales[], float servo_commands[]) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    float scaled_command = boundedRangeScalerApply(servoScales[i], servo_commands[i]);

    int command = constrain(scaled_command, 0, 180);
    servo[i].write(command); // Writes PWM value to servo object takes 0-180
  }
}