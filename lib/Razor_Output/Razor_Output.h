/* This file is part of the Razor AHRS Firmware */

#ifndef RAZOR_OUTPUT_H
#define RAZOR_OUTPUT_H

/* #include <SD.h> */
#include <SdFat.h>

#include "Razor_Sensors.h"

#define STATUS_LED_PIN 13  // Pin number of status LED

#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

boolean output_stream_on;
boolean output_single_on;

// SD chip select pin
const uint8_t chipSelect = SS;

// file system object
SdFat sd;

// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))

ofstream logfile;

void setup_SD() {
  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) sd.initErrorHalt();
  // Create a new file, making sure to not overwrite previous files
  char name[] = "LOGGER00.TXT";
  for (uint8_t i = 0; i < 100; i++) {
    name[6] = i/10 + '0';
    name[7] = i%10 + '0';
    if (sd.exists(name)) continue;
    logfile.open(name, ios::out | ios::app);
    break;
  }
  if (!logfile.is_open()) error("open failed, there may be 99 log files present");
}

void stream_text_to_SD(char *output_string){
  logfile << output_string << endl;
  logfile << " millis = " << millis() << endl;
  logfile << flush;
}

void close_SD(){
    logfile.close();
}

char angle_string[50];
char* get_angles(Gyroscope& gyro)
{
  char temp_string[10];
  // stitch together angle_string
  strcat(angle_string, "#YPR=");
  
  /* Serial.println(TO_DEG(gyro.yaw)); */
  /* Serial.println(gyro.yaw); */
  dtostrf(TO_DEG(gyro.yaw), 4, 2, temp_string);
  strcat(angle_string, temp_string);
  strcat(angle_string, ",");
  
  /* Serial.println(TO_DEG(gyro.pitch)); */
  /* Serial.println(gyro.pitch); */
  dtostrf(TO_DEG(gyro.pitch), 4, 2, temp_string);
  strcat(angle_string, temp_string);
  strcat(angle_string, ",");
  
  /* Serial.println(TO_DEG(gyro.roll)); */
  /* Serial.println(gyro.roll); */
  dtostrf(TO_DEG(gyro.roll), 4, 2, temp_string);
  strcat(angle_string, temp_string);
  strcat(angle_string, "\r\n");
  return angle_string;
}

char accel_string[50];

void stream_accel_to_SD(Accelerometer &accel){
  char temp_string[10];
  char time_string[15];
  /* clear accel_string */
  memset(&accel_string[0], 0, sizeof(accel_string));
  /* create new acceleration data string */
  itoa(millis(), time_string, 10);
  strcat(accel_string, time_string);
  strcat(accel_string, " : ");
  dtostrf(accel.accel[0],4,2, temp_string);
  strcat(accel_string, temp_string);
  strcat(accel_string, ",");
  dtostrf(accel.accel[1],4,2, temp_string);
  strcat(accel_string, temp_string);
  strcat(accel_string, ",");
  dtostrf(accel.accel[2],4,2 ,temp_string);
  strcat(accel_string, temp_string);
  strcat(accel_string,",");
  /* strcat(angle_string, "\r\n"); */
  /* Serial.print(accel_string); */
  /* Push string to logfile */
  logfile << accel_string;
  logfile << endl;
  logfile << flush;
}

void stream_angles_to_SD(Gyroscope &gyro)
{
  get_angles(gyro);
  logfile << angle_string;
  logfile << flush;
}

// Output angles: yaw, pitch, roll
void output_angles(Gyroscope& gyro)
{
  Serial.print(get_angles(gyro));
}

void output_sensors_text(Accelerometer& accel, char raw_or_calibrated)
{
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(accel.accel[0]); Serial.print(",");
  Serial.print(accel.accel[1]); Serial.print(",");
  Serial.print(accel.accel[2]); Serial.println();
}

void output_sensors_binary(Accelerometer &accel, Magnetometer &magneto, Gyroscope &gyro)
{
  Serial.write((byte*) accel.accel, 12);
  Serial.write((byte*) magneto.magnetom, 12);
  Serial.write((byte*) gyro.gyro, 12);
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

void read_control_message(int curr_calibration_sensor, bool reset_calibration_session_flag, bool output_errors){
  int num_magn_errors = 0;
  int num_gyro_errors = 0;

  if (Serial.read() == '#') // Start of new control message
    {
      int command = Serial.read(); // Commands
      if (command == 'f') // request one output _f_rame
        output_single_on = true;
      else if (command == 's') // _s_ynch request
        {
          // Read ID
          byte id[2];
          id[0] = readChar();
          id[1] = readChar();

          // Reply with synch message
          Serial.print("#SYNCH");
          Serial.write(id, 2);
          Serial.println();
        }
      else if (command == 'o') // Set _o_utput mode
        {
          char output_param = readChar();
          if (output_param == 'n')  // Calibrate _n_ext sensor
            {
              curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
              reset_calibration_session_flag = true;
            }
          else if (output_param == 't') // Output angles as _t_ext
            {
              output_mode = OUTPUT__MODE_ANGLES;
              output_format = OUTPUT__FORMAT_TEXT;
            }
          else if (output_param == 'b') // Output angles in _b_inary format
            {
              output_mode = OUTPUT__MODE_ANGLES;
              output_format = OUTPUT__FORMAT_BINARY;
            }
          else if (output_param == 'c') // Go to _c_alibration mode
            {
              output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
              reset_calibration_session_flag = true;
            }
          else if (output_param == 's') // Output _s_ensor values
            {
              char values_param = readChar();
              char format_param = readChar();
              if (values_param == 'r')  // Output _r_aw sensor values
                output_mode = OUTPUT__MODE_SENSORS_RAW;
              else if (values_param == 'c')  // Output _c_alibrated sensor values
                output_mode = OUTPUT__MODE_SENSORS_CALIB;
              else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
                output_mode = OUTPUT__MODE_SENSORS_BOTH;

              if (format_param == 't') // Output values as _t_text
                output_format = OUTPUT__FORMAT_TEXT;
              else if (format_param == 'b') // Output values in _b_inary format
                output_format = OUTPUT__FORMAT_BINARY;
            }
          else if (output_param == '0') // Disable continuous streaming output
            {
              turn_output_stream_off();
              reset_calibration_session_flag = true;
            }
          else if (output_param == '1') // Enable continuous streaming output
            {
              reset_calibration_session_flag = true;
              turn_output_stream_on();
            }
          else if (output_param == 'e') // _e_rror output settings
            {
              char error_param = readChar();
              if (error_param == '0') output_errors = false;
              else if (error_param == '1') output_errors = true;
              else if (error_param == 'c') // get error count
                {
                  Serial.print("#AMG-ERR:");
                  /* Serial.print(num_accel_errors); Serial.print(","); */
                  Serial.print(num_magn_errors); Serial.print(",");
                  Serial.println(num_gyro_errors);
                }
            }
        }
#if OUTPUT__HAS_RN_BLUETOOTH == true
      // Read messages from bluetooth module
      // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
      else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
        turn_output_stream_on();
      else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
        turn_output_stream_off();
#endif // OUTPUT__HAS_RN_BLUETOOTH == true
    }
  else
    { } // Skip character
}
#endif
