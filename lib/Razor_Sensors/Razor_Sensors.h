#ifndef RAZOR_SENSORS_H
#define RAZOR_SENSORS_H

#include "arduino.h"
#include "Razor_Math.h"
#include <Wire.h>

class Accelerometer
{
private:
  int num_accel_errors;
  byte buff[6];
  unsigned long timestamp;
  unsigned long timestamp_old;
  float G_Dt;
  float accel_timestamp;
  float accel_timestamp_old;

public:
  Accelerometer();
  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
  float accel[3];
  float accel_min[3];
  float accel_max[3];
  void init();
  int read();
  void calibrate();
  void compensate_errors();
};

class Gyroscope
{
private:
  int num_gyro_errors;
  unsigned long timestamp;
  unsigned long timestamp_old;
  float G_Dt;
  
public:
  Gyroscope();
  float gyro[3];
  float gyro_average[3];
  int gyro_num_samples;
  float yaw;
  float pitch;
  float roll;
  
  // Store the gyros turn rate in a vector
  float Gyro_Vector[3]; 
  void init();
  int read();
  void calibrate();
  void compensate_errors();
  void Euler_angles(float [3][3]);
  
};

class Magnetometer
{
private:
  int num_magn_errors;
  unsigned long timestamp;
  unsigned long timestamp_old;
  float G_Dt;
  
public:
  Magnetometer();
  float magnetom[3];
  float magnetom_min[3];
  float magnetom_max[3];
  float magnetom_tmp[3];
  float MAG_Heading;

  void init();
  int read();
  void calibrate();
  void compensate_errors();
  void Compass_Heading(Gyroscope&);
  void Drift_correction();  
};

// Functions
void Matrix_update(Gyroscope&, Accelerometer&, Magnetometer&, float, float[][3]);
void sensors_init(Accelerometer& , Gyroscope& , Magnetometer&);
void init_rotation_matrix(Gyroscope&, float [][3]);
void normalize(float[][3]);

#endif
