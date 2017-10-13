/*
 * MPU6050.cpp
 *
 *  Created on: 22.09.2017
 *      Author: anonymous
 */

#include "MPU6050.h"
#include "Wire.h"

// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// MPU9250/6050 IMU code
// Read the accel and gyro registers and calculate a complementary filter for sensor fusion between gyros and accel

// Code based on arduino.cc MPU6050 sample
// Open Source / Public Domain
//
// Documentation:"MPU-6000 and MPU-6050 Register Map and Descriptions": RM-MPU-6000A.pdf

// MPU6050 Register map


// Global MPU6050 IMU variables
accel_t_gyro_union accel_t_gyro;
float x_gyro_value;   // in deg/seg units
float x_gyro_offset = 0.0;
float accel_angle;  // in degree units
float angle;

// Util function to swap byte values
uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap


// This function implements a complementary filter to fusion gyro and accel info
float MPU6050_getAngle(float dt)
{
  accel_angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
  x_gyro_value = (accel_t_gyro.value.x_gyro - x_gyro_offset) / 65.5;  // Accel scale at 500deg/seg  => 65.5 LSB/deg/s

  // Complementary filter
  // We integrate the gyro rate value to obtain the angle in the short term and we take the accelerometer angle with a low pass filter in the long term...
  angle = 0.99 * (angle + x_gyro_value * dt) + 0.01 * accel_angle;  // Time constant = 0.99*0.01(100hz)/(1-0.99) = 0.99, around 1 sec.

  // Gyro bias correction
  // We supose that the long term mean of the gyro_value should tend to zero (gyro_offset). This means that the robot is not continuosly rotating.
  int16_t correction = constrain(accel_t_gyro.value.x_gyro, x_gyro_offset - 10, x_gyro_offset + 10); // limit corrections...
  x_gyro_offset = x_gyro_offset * 0.9995 + correction * 0.0005; // Time constant of this correction is around 20 sec.

  //Serial.print(angle);
  //Serial.print(" ");
  //Serial.println(x_gyro_offset);

  return angle;
}

// Calibrate function. Take 100 readings (over 2 seconds) to calculate the gyro offset value. IMU should be steady in this process...
void MPU6050_calibrate()
{
  int i;
  long value = 0;
  float dev;
  int16_t values[100];
  bool gyro_cal_ok = false;

  delay(500);
  while (!gyro_cal_ok){
    Serial.println("Gyro calibration... DONT MOVE!");
    // we take 100 measurements in 4 seconds
    for (i = 0; i < 100; i++)
    {
      MPU6050_read_3axis();
      values[i] = accel_t_gyro.value.x_gyro;
      value += accel_t_gyro.value.x_gyro;
      delay(25);
    }
    // mean value
    value = value / 100;
    // calculate the standard deviation
    dev = 0;
    for (i = 0; i < 100; i++)
      dev += (values[i] - value) * (values[i] - value);
    dev = sqrt((1 / 100.0) * dev);
    Serial.print("offset: ");
    Serial.print(value);
    Serial.print("  stddev: ");
    Serial.println(dev);
    if (dev < 50.0)
      gyro_cal_ok = true;
    else
      Serial.println("Repeat, DONT MOVE!");
  }
  x_gyro_offset = value;
  // Take the first reading of angle from accels
  angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
}

void MPU6050_setup()
{
  int error;
  uint8_t c;

  error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
  Serial.print("WHO_AM_I : ");
  Serial.print(c, HEX);
  Serial.print(", error = ");
  Serial.println(error, DEC);

  // RESET chip
  MPU6050_write_reg(MPU6050_PWR_MGMT_1, bit(MPU6050_DEVICE_RESET));
  delay(125);
  // Clear the 'sleep' bit to start the sensor and select clock source
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0x01);
  //MPU6050_write_reg(MPU6050_PWR_MGMT_1,MPU6050_CLKSEL_Z);

  // Config Gyro scale (500deg/seg)
  MPU6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_500);
  // Config Accel scale (2g)
  MPU6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_2G);
  // Config Digital Low Pass Filter 10Hz
  MPU6050_write_reg(MPU6050_CONFIG, MPU6050_DLPF_10HZ);
  // Set Sample Rate to 100Hz
  MPU6050_write_reg(MPU6050_SMPLRT_DIV, 9);  // 100Hz : Sample Rate = 1000 / (1 + SMPLRT_DIV) Hz
  // Data ready interrupt enable
  MPU6050_write_reg(MPU6050_INT_ENABLE, MPU6050_DATA_RDY_EN);
  // Clear the 'sleep' bit to start the sensor (and select clock source).
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0x01);

  // Clear the 'sleep' bit to start the sensor.
  //MPU6050_write_reg(MPU6050_PWR_MGMT_1,MPU6050_CLKSEL_Z);
  //MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
}

void MPU6050_read_3axis()
{
  int error;

  // read 14 bytes (gyros, temp and accels)
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  if (error != 0) {
    //Serial.print("MPU6050 Error:");
    //Serial.println(error);
  }
  // swap bytes
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

  /*
    // Print the raw acceleration values
    Serial.print("ACC:");
    Serial.print(accel_t_gyro.value.x_accel, DEC);
    Serial.print(",");
    Serial.print(accel_t_gyro.value.y_accel, DEC);
    Serial.print(",");
    Serial.print(accel_t_gyro.value.z_accel, DEC);
    Serial.println();

    // Print the raw gyro values.
    Serial.print("GY:");
    Serial.print(accel_t_gyro.value.x_gyro, DEC);
    Serial.print(",");
    Serial.print(accel_t_gyro.value.y_gyro, DEC);
    Serial.print(",");
    Serial.print(accel_t_gyro.value.z_gyro, DEC);
    Serial.print(",");
    Serial.println();
  */
}

void MPU6050_read_1axis()
{
  int error;

  // read X accel
  error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro.reg.x_accel_h, 6);
  if (error != 0) {
    //Serial.print("MPU6050 Error:");
    //Serial.println(error);
  }
  // read X gyro
  error = MPU6050_read(MPU6050_GYRO_XOUT_H, (uint8_t *) &accel_t_gyro.reg.x_gyro_h, 2);
  if (error != 0) {
    //Serial.print("MPU6050 Error:");
    //Serial.println(error);
  }
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);

  // Print values
  Serial.print("axis:");
  Serial.print(accel_t_gyro.value.y_accel, DEC);
  Serial.print(",");
  Serial.println(accel_t_gyro.value.x_gyro, DEC);
}

// return true on new data available
bool MPU6050_newData()
{
  uint8_t status;
  int error;

  error = MPU6050_read(MPU6050_INT_STATUS, &status, 1);
  if (error != 0) {
    //Serial.print("MPU6050 Error:");
    //Serial.println(error);
  }
  if (status & (0b00000001)) // Data ready?
    return true;
  else
    return false;
}

// MPU6050_read n bytes
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom((uint8_t) MPU6050_I2C_ADDRESS, (size_t) size, (bool) true);
  i = 0;
  while (Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// MPU6050_write n bytes
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg (only 1 byte)
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}
