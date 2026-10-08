// #include "Arduino.h"
// #include "motor.h"
// // #include "lightring.h"

// static void StartMotor(uint8_t dir, uint8_t speed, uint32_t duration);

// uint8_t motor_active = 0;
// uint8_t motor_enable = 1;
// uint32_t motion_duration = 0;
// uint32_t motion_start = 0;

// void MotorInit(void)
// {
//   digitalWrite(MOTOR_SPEED_PIN,  LOW);
//   digitalWrite(MOTOR_DIR_PIN,  LOW);
//   digitalWrite(MOTOR_DRV_PIN,  LOW);
//   digitalWrite(MOTOR_SLEEP_PIN,  HIGH);

//   pinMode(MOTOR_SPEED_PIN,  OUTPUT);
//   pinMode(MOTOR_DIR_PIN,  OUTPUT);
//   pinMode(MOTOR_DRV_PIN,  OUTPUT);
//   pinMode(MOTOR_SLEEP_PIN, OUTPUT);
//   pinMode(MOTOR_FAULT_PIN, INPUT);
  
//   delay(1000);
//   MotorReset();
// }

// //**********************************************************************************
// // Enable or Disable the motor driver (Using the DRVOFF pin the motor is active low)
// //**********************************************************************************
// void MotorEnableWrite(uint8_t enable)
// {
//   if(enable)
//   {
//     digitalWrite(MOTOR_DRV_PIN,  0);
//     motor_enable = 1;
//   }
//   else
//   {
//     digitalWrite(MOTOR_DRV_PIN,  1);
//     motor_enable = 0;
//   }
// }

// uint8_t MotorEnableRead(void)
// {
//   return(motor_enable);
// }
  
// //******************************************************
// // Check if a motor driver fault is present (active low)
// //******************************************************
// uint8_t MotorFaultRead(void)
// {
//   if(digitalRead(MOTOR_FAULT_PIN))
//     return(0);

//   return(1);
// }

// //*********************** 
// // Clear any motor faults
// //***********************
// void MotorReset()
// {
//   digitalWrite(MOTOR_SLEEP_PIN,  LOW); 
//   CyDelayUs(10);
//   digitalWrite(MOTOR_SLEEP_PIN,  HIGH);
// }

// //***********************************************
// // Check the state of the current motor animation
// //*********************************************** 
// void UpdateMotor(void)
// {
//   if(!motor_active) return;

//   //Check if the motion is completed
//   if(millis() - motion_start > motion_duration)
//   {
//     gumbandPrint("Stopping Motor!\n");
//     //LedStop();
//     //Turn off the motor and set inactive
//     analogWrite(MOTOR_SPEED_PIN, 0);
//     motor_active = 0;
//   }
// }

// //************************************************************************
// // Take in a click strength and time for each haptic 'click' or detent
// // These control the intensity (pwm duty cycle) and duration respectively
// // Drive the motor in either the forwards or backwards direction
// //*********************************************************************************
// void ForwardClick(uint8_t click_strength, uint8_t click_time) 
// {
//   StartMotor(MOTOR_FORWARD, click_strength, click_time);
// }

// void BackwardClick(uint8_t click_strength, uint8_t click_time) 
// {
//   StartMotor(MOTOR_BACKWARD, click_strength, click_time);
// }

// //******************************************************
// // Set the motor parameters and start the motion profile
// //******************************************************
// static void StartMotor(uint8_t dir, uint8_t speed, uint32_t duration)
// {
//   if(motor_active) return;

//   gumbandPrint("Starting Motor!");
//   //LedStart();

//   //Disable any current motion
//   analogWrite(MOTOR_SPEED_PIN, 0);

//   //Configure the new duration
//   motion_duration = duration;
//   motion_start = millis();
//   motor_active = 1;

//   //Set the new motor direction
//   digitalWrite(MOTOR_DIR_PIN, dir);

//   //Set the motor speed
//   analogWrite(MOTOR_SPEED_PIN, speed);
// }


