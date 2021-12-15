/*
 * Control.cpp
 *
 *  Created on: 25.09.2017
 *      Author: anonymous
 */

#include <Arduino.h>
#include "defines.h"
#include "globals.h"

// PD controller implementation(Proportional, derivative). DT in seconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}


// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
  return (output);
}


float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * float(speedM);
  return (output);
}

