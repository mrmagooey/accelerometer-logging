
#include "Razor_Sensors.h"
 
// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 
#else
#define WIRE_SEND(b) Wire.send(b)
#define WIRE_RECEIVE() Wire.receive() 
#endif

#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)

// Magnetometer (standard calibration)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

// Magnetometer (extended calibration)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Gyroscope

// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false

/*
// Calibration example:

// "accel x,y,z (min/max) = -278.00/270.00  -254.00/284.00  -294.00/235.00"
#define ACCEL_X_MIN ((float) -278)
#define ACCEL_X_MAX ((float) 270)
#define ACCEL_Y_MIN ((float) -254)
#define ACCEL_Y_MAX ((float) 284)
#define ACCEL_Z_MIN ((float) -294)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
//#define MAGN_X_MIN ((float) -511)
//#define MAGN_X_MAX ((float) 581)
//#define MAGN_Y_MIN ((float) -516)
//#define MAGN_Y_MAX ((float) 568)
//#define MAGN_Z_MIN ((float) -489)
//#define MAGN_Z_MAX ((float) 486)

// Extended magn
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {91.5, -13.5, -48.1};
const float magn_ellipsoid_transform[3][3] = {{0.902, -0.00354, 0.000636}, {-0.00354, 0.9, -0.00599}, {0.000636, -0.00599, 1}};

// Extended magn (with Sennheiser HD 485 headphones)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {72.3360, 23.0954, 53.6261};
//const float magn_ellipsoid_transform[3][3] = {{0.879685, 0.000540833, -0.0106054}, {0.000540833, 0.891086, -0.0130338}, {-0.0106054, -0.0130338, 0.997494}};

//"gyro x,y,z (current/average) = -32.00/-34.82  102.00/100.41  -16.00/-16.38"
#define GYRO_AVERAGE_OFFSET_X ((float) -34.82)
#define GYRO_AVERAGE_OFFSET_Y ((float) 100.41)
#define GYRO_AVERAGE_OFFSET_Z ((float) -16.38)
*/

#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f
// DCM variables
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// Timing
#define GYRO_UPDATE_TIME = 20
#define ACCEL_UPDATE_TIME = 20
#define MAGNETO_UPDATE_TIME = 20

Accelerometer::Accelerometer()
{
  num_accel_errors = 0;
};

Gyroscope::Gyroscope() 
{
  gyro_num_samples = 0;
};

Magnetometer::Magnetometer()
{

}

// Methods
void Accelerometer::init ()
{
  // Set Power Control
  Wire.beginTransmission(ACCEL_ADDRESS);
  // POWER_CTL
  WIRE_SEND(0x2D);  
  // +-----+-----+-----+----------+-------+-----+-----+-----+
  // | D7  | D6  | D5  |    D4    |  D3   | D2  | D1  | D0  |
  // +-----+-----+-----+----------+-------+-----+-----+-----+
  // |  0  |  0  |Link |AUTO_SLEEP|Measure|Sleep|  Wakeup   |
  // +-----+-----+-----+----------+-------+-----+-----------+
  // Set Measurement mode 
  // #0x08 => 00001000
  WIRE_SEND(0x08);  
  Wire.endTransmission();
  
  delay(5);
  
  // Set the data format
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x31);  
  // Data format register
  // +---------+-----+----------+-----+--------+-------+-----+-----+
  // |   D7    | D6  |    D5    | D4  |   D3   |  D2   | D1  | D0  |
  // +---------+-----+----------+-----+--------+-------+-----+-----+
  // |Self-Test| SPI |INT_INVERT|  0  |FULL_RES|Justify|   Range   |
  // +---------+-----+----------+-----+--------+-------+-----------+
  // Set to full resolution
  // #0x08 => 00001000
  WIRE_SEND(0x08);  
  Wire.endTransmission();
  
  delay(5);

  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x2C);  // Rate
  // WIRE_SEND(0x09);  // Set to 50Hz
  WIRE_SEND(0xE); // Set to 1600Hz
  Wire.endTransmission();
  delay(5);
  accel_timestamp = millis();
}

void Magnetometer::init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Gyroscope::init()
{
  // Power up reset defaults
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);

  // Select full-scale range of the gyro sensors
  // Set LP filter bandwidth to 42Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x16);
  WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);

  // Set sample rato to 50Hz
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x15);
  WIRE_SEND(0x03);  //  SMPLRT_DIV = 3 (333Hz)
  Wire.endTransmission();
  delay(5);

  // Set clock to PLL with z gyro reference
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x3E);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
  
  // initialise gyro to zero
  Gyro_Vector[0] = 0;
  Gyro_Vector[1] = 0;
  Gyro_Vector[2] = 0;
}


// Reads x, y and z accelerometer registers
int Accelerometer::read()
{
  int i = 0;
  // Sensor timing
  accel_timestamp_old = accel_timestamp;
  G_Dt = (float) (millis() - accel_timestamp_old)/1000.0f;
  // wait time
  if (G_Dt < 0.001) {
    return 0; // accel not updated
  }
  // reset the timestamp
  accel_timestamp = millis(); 
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x32);  // Send address to read from
  Wire.endTransmission();
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
    { 
      buff[i] = WIRE_RECEIVE();  // Read one byte
      i++;
    }
  Wire.endTransmission();
  if (i == 6)  // All bytes received?
    {
      // No multiply by -1 for coordinate system transformation here, because of double negation:
      // We want the gravity vector, which is negated acceleration vector.
      
      // Concatenate two bytes together
      accel[0] = (((int) buff[3]) << 8) | buff[2];  // X axis (internal sensor y axis)
      accel[1] = (((int) buff[1]) << 8) | buff[0];  // Y axis (internal sensor x axis)
      accel[2] = (((int) buff[5]) << 8) | buff[4];  // Z axis (internal sensor z axis)
    }
  else
    {
      num_accel_errors++;
      // if (output_errors) Serial.println("!ERR: reading accelerometer");
    }
  return 1;
}

int Magnetometer::read()
{
  int i = 0;
  byte buff[6];
  
  // Sensor timing
  timestamp_old = timestamp;
  timestamp = millis();
  G_Dt = (float) (timestamp - timestamp_old) / 1000.0f;
  // 20 milliseconds wait time
  if (G_Dt < 20) {
    return 0; // gyro not updated
  }

  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();

  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
    { 
      buff[i] = WIRE_RECEIVE();  // Read one byte
      i++;
    }
  Wire.endTransmission();

  if (i == 6)  // All bytes received?
    {
      // 9DOF Razor IMU SEN-10125 using HMC5843 magnetometer
#if HW__VERSION_CODE == 10125
      // MSB byte first, then LSB; X, Y, Z
      magnetom[0] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // X axis (internal sensor -y axis)
      magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
      magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
      // 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10736
      // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
      magnetom[0] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // X axis (internal sensor -y axis)
      magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
      magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
      // 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
#elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
      // MSB byte first, then LSB; X, Y, Z
      magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
      magnetom[1] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Y axis (internal sensor -y axis)
      magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
      // 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10724
      // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
      magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
      magnetom[1] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
      magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
#endif
    }
  else
    {
      num_magn_errors++;
      // if (output_errors) Serial.println("!ERR: reading magnetometer");
    }
  return 1;
}


// Reads x, y and z gyroscope registers
int Gyroscope::read()
{
  // Gyroscope::Read()
  // 0. Checks timestamp and returns 0 if early
  // 1. Reads data from the gyro sensor
  // 2. Scales the data and converts from degrees/sec to radians/sec
  // 3. Multiplies the radian data by time to get radians (from radians/sec)
  // 4. Adds this radian figure to the total number of radians rotated since the
  //    Gyro_Vector was last read and cleared.
  // 5. 
  
  int i = 0;
  // store raw gyro data
  byte buff[6];
  // Sensor timing
  timestamp_old = timestamp;
  timestamp = millis();
  G_Dt = (float) (timestamp - timestamp_old) / 1000.0f;
  // 20 milliseconds wait time
  if (G_Dt < 20) {
    return 0; // gyro not updated
  }
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(0x1D);  // Sends address to read from
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
    { 
      buff[i] = WIRE_RECEIVE();  // Read one byte
      i++;
    }
  Wire.endTransmission();

  if (i == 6)  // All bytes received?
    {
      // concatenate each two buffers
      gyro[0] = -1 * ((((int) buff[2]) << 8) | buff[3]);    // X axis (internal sensor -y axis)
      gyro[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);    // Y axis (internal sensor -x axis)
      gyro[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);    // Z axis (internal sensor -z axis)
    }
  else
    {
      num_gyro_errors++;
      // if (output_errors) Serial.println("!ERR: reading gyroscope");
    }
  // This is to change from degrees per second into degrees
  // and represents the change of orientation in degrees
  
  Gyro_Vector[0]+=G_Dt*GYRO_SCALED_RAD(gyro[0]); // gyro x roll
  Gyro_Vector[1]+=G_Dt*GYRO_SCALED_RAD(gyro[1]); // gyro y pitch
  Gyro_Vector[2]+=G_Dt*GYRO_SCALED_RAD(gyro[2]); // gyro z yaw
  
  return 1;
}

#if OUTPUT__MODE_CALIBRATE_SENSORS

void Accelerometer::calibrate()
{
  Serial.print("accel x,y,z (min/max) = ");
  for (int i = 0; i < 3; i++) {
    if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
    if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
    Serial.print(accel_min[i]);
    Serial.print("/");
    Serial.print(accel_max[i]);
    if (i < 2) Serial.print("  ");
    else Serial.println();
  }
  if (reset_calibration_session_flag) 
    {
      for (int i = 0; i < 3; i++) {
        accel_min[i] = accel_max[i] = accel[i];
      }
    }
}

void Magnetometer::calibrate()
{
  Serial.print("magn x,y,z (min/max) = ");
  for (int i = 0; i < 3; i++) {
    if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
    if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
    Serial.print(magnetom_min[i]);
    Serial.print("/");
    Serial.print(magnetom_max[i]);
    if (i < 2) Serial.print("  ");
    else Serial.println();
  }
  if (reset_calibration_session_flag) 
    {
      for (int i = 0; i < 3; i++) {
        magnetom_min[i] = magnetom_max[i] = magnetom[i];
      }
    }
}

void Gyroscope::calibrate()
{
  // Average gyro values
  for (int i = 0; i < 3; i++)
    gyro_average[i] += gyro[i];
  gyro_num_samples++;

  // Output current and averaged gyroscope values
  Serial.print("gyro x,y,z (current/average) = ");
  for (int i = 0; i < 3; i++) {
    Serial.print(gyro[i]);
    Serial.print("/");
    Serial.print(gyro_average[i] / (float) gyro_num_samples);
    if (i < 2) Serial.print("  ");
    else Serial.println();
  }

  if (reset_calibration_session_flag) 
    {
      // Reset gyro calibration variables
      gyro_num_samples = 0;  // Reset gyro calibration averaging
      gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
    }
}

#endif

// Apply calibration to raw sensor readings
void Accelerometer::compensate_errors()
{
  accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
}

void Magnetometer::compensate_errors()
{
#if CALIBRATION__MAGN_USE_EXTENDED == true
  for (int i = 0; i < 3; i++)
    magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
  Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
  magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
  magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
  magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif
}

void Gyroscope::compensate_errors()
{
  gyro[0] -= GYRO_AVERAGE_OFFSET_X;
  gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
  gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

void Magnetometer::Compass_Heading(Gyroscope& gyro)
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(gyro.roll);
  sin_roll = sin(gyro.roll);
  cos_pitch = cos(gyro.pitch);
  sin_pitch = sin(gyro.pitch);

  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch 
    + magnetom[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  // Magnetic Heading
  MAG_Heading = (float)atan2(-mag_y, mag_x);
}

void Gyroscope::Euler_angles(float DCM_Matrix[3][3])
{
  pitch = (float)-asin(DCM_Matrix[2][0]);
  roll = (float)atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = (float)atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

void normalize(float DCM_Matrix[3][3])
{
  float error=0;
  float temporary[3][3];
  float renorm=0;

  error= -Vector_Dot_Product(DCM_Matrix[0],DCM_Matrix[1])*.5; //eq.19
  
  Vector_Scale(temporary[0], DCM_Matrix[1], error); //eq.19
  Vector_Scale(temporary[1], DCM_Matrix[0], error); //eq.19

  Vector_Add(temporary[0], temporary[0], DCM_Matrix[0]);//eq.19
  Vector_Add(temporary[1], temporary[1], DCM_Matrix[1]);//eq.19

  Vector_Cross_Product(temporary[2], temporary[0], temporary[1]); // c= a x b //eq.20

  renorm= .5 *(3 - Vector_Dot_Product(temporary[0], temporary[0])); //eq.21
  Vector_Scale(DCM_Matrix[0], temporary[0], renorm);

  renorm= .5 *(3 - Vector_Dot_Product(temporary[1], temporary[1])); //eq.21
  Vector_Scale(DCM_Matrix[1], temporary[1], renorm);

  renorm= .5 *(3 - Vector_Dot_Product(temporary[2], temporary[2])); //eq.21
  Vector_Scale(DCM_Matrix[2], temporary[2], renorm);
}

void init_rotation_matrix(Gyroscope & gyro, float DCM_Matrix[][3])
{
  float c1 = cos(gyro.roll);
  float s1 = sin(gyro.roll);
  float c2 = cos(gyro.pitch);
  float s2 = sin(gyro.pitch);
  float c3 = cos(gyro.yaw);
  float s3 = sin(gyro.yaw);
  
  // Euler angles, right-handed, intrinsic, XYZ convention
  // aka Tait-Bryan ZY'X'' passive
  // (which means: rotate around body axes Z, Y', X'') 
  DCM_Matrix[0][0] = c2 * c3;
  DCM_Matrix[0][1] = c3 * s1 * s2 - c1 * s3;
  DCM_Matrix[0][2] = s1 * s3 + c1 * c3 * s2;

  DCM_Matrix[1][0] = c2 * s3;
  DCM_Matrix[1][1] = c1 * c3 + s1 * s2 * s3;
  DCM_Matrix[1][2] = c1 * s2 * s3 - c3 * s1;

  DCM_Matrix[2][0] = -s2;
  DCM_Matrix[2][1] = c2 * s1;
  DCM_Matrix[2][2] = c1 * c2;
}

// Init DCM with unfiltered orientation
void reset_sensor_fusion(Accelerometer& accel, Gyroscope& gyro, Magnetometer& magneto)
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};
  accel.read();
  magneto.read();
  gyro.read();    
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  gyro.pitch = (float)-atan2(accel.accel[0], sqrt(accel.accel[1] * accel.accel[1] 
                                                  + accel.accel[2] * accel.accel[2]));
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel.accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  gyro.roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  magneto.Compass_Heading(gyro);
  gyro.yaw = magneto.MAG_Heading;
  
}

void sensors_init(Accelerometer& accel, Gyroscope& gyro, Magnetometer& magneto)
{
  Wire.begin();
  delay(20);  // Give sensors enough time to collect data
  accel.init();
  gyro.init();
  magneto.init();
  reset_sensor_fusion(accel, gyro, magneto);
}

void Matrix_update(Gyroscope& gyro, Accelerometer& accel, Magnetometer&magneto, float G_Dt, float DCM_Matrix[][3])
{
  float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
  
#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*gyro.Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*gyro.Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*gyro.Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*gyro.Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*gyro.Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*gyro.Gyro_Vector[0];
  Update_Matrix[2][2]=0;
  
#else // Use drift correction
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  
  //Compensation the Roll, Pitch and Yaw drift.
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  float errorRollPitch[3] = {0, 0, 0};
  float Omega_P[3] = {0,0,0};
  float Omega_I[3] = {0,0,0};
  float Omega[3] = {0,0,0};
  float errorYaw[3] = {0, 0, 0};
  float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data

  
  //*****Roll and Pitch***************
  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(accel.accel[0]*accel.accel[0] + 
                         accel.accel[1]*accel.accel[1] + 
                         accel.accel[2]*accel.accel[2]);
  // Scale to gravity.
  Accel_magnitude = Accel_magnitude / GRAVITY; 
  
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);
  
  //adjust the ground of reference
  Vector_Cross_Product(errorRollPitch, accel.accel, DCM_Matrix[2]); 
  Vector_Scale(Omega_P, errorRollPitch, Kp_ROLLPITCH*Accel_weight);
  Vector_Scale(Scaled_Omega_I, errorRollPitch, Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);

  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
  mag_heading_x = cos(magneto.MAG_Heading);
  mag_heading_y = sin(magneto.MAG_Heading);
  // Calculating YAW error
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  
  // Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  Vector_Scale(errorYaw, DCM_Matrix[2], errorCourse);
  // .01proportional of YAW.
  Vector_Scale(Scaled_Omega_P, errorYaw, Kp_YAW);
  // Adding Proportional.
  Vector_Add(Omega_P, Omega_P, Scaled_Omega_P);
  // .00001Integrator
  Vector_Scale(Scaled_Omega_I, errorYaw, Ki_YAW);
  // Adding integrator to the Omega_I
  Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);
  // Adding Integrator term
  Vector_Add(Omega, gyro.Gyro_Vector, Omega_I); 
  // Adding proportional term
  Vector_Add(Omega_Vector, Omega, Omega_P);
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] =- G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2] = G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0] = G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0] = -G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1] = G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2] = 0;
#endif
  // Update the DCM_Matrix with the new readings  
  Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c
  for(int x=0; x<3; x++) //Matrix Addition (update)
    {
      for(int y=0; y<3; y++)
        {
          DCM_Matrix[x][y] += Temporary_Matrix[x][y];
        }
    }
}

