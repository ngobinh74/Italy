#include "helper_3dmath.h"

/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/
// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
// #define OUTPUT__BAUD_RATE 9600
// #define OUTPUT__BAUD_RATE 19200
// #define OUTPUT__BAUD_RATE 38400
//#define OUTPUT__BAUD_RATE 57600
// #define OUTPUT__BAUD_RATE 748800
#define OUTPUT__BAUD_RATE 115200
// #define OUTPUT__BAUD_RATE 230400
// #define OUTPUT__BAUD_RATE 250000

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Upper speed limit for the cycle
#define OUTPUT_CYCLELIMIT 20 // in milliseconds - Request one output frame, Though #f only requests one reply, replies are still
                             // bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.

// state definitions (do not change)
// Output mode definitions (do not change)
enum TYPE_OUTPUT
{
  OUTPUT_UNKNOWN            = 0,
  OUTPUT_NONE               = 1,  //non output
  OUTPUT_RESUME             = 2,  //non output
  OUTPUT_CALIBRATE_SENSORS  = 3,  // Outputs sensor min/max values as text for manual calibration
  OUTPUT_ANGLES             = 4,  // Outputs yaw/pitch/roll in degrees  
  OUTPUT_SENSORS_CALIB      = 5,  // Outputs calibrated sensor values for all 9 axes
  OUTPUT_SENSORS_RAW        = 6,  // Outputs raw (uncalibrated) sensor values for all 9 axes
  OUTPUT_SENSORS_BOTH       = 7,  // Outputs calibrated AND raw sensor values for all 9 axes
  OUTPUT_CUSTOM             = 8,  // Outputs data in custom format    
  OUTPUT_RAW_ACCEL          = 9,

  START_PROCESS             = 10,
  STOP_PROCESS              = 11
};

// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
unsigned char output_mode = OUTPUT_CUSTOM;  //original
//int output_mode = OUTPUT_RAW_ACCEL_SENSORS;  //adding by Tuan
//int output_mode = OUTPUT_ANGLES;
unsigned char next_output_mode;
unsigned char prev_mode;

// int output_format = OUTPUT__FORMAT_BINARY;
int output_format = OUTPUT__FORMAT_TEXT;    // in Project , following Binh NGO

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -276)
#define ACCEL_X_MAX ((float) 271)
#define ACCEL_Y_MIN ((float) -260)
#define ACCEL_Y_MAX ((float) 271)
#define ACCEL_Z_MIN ((float) -286)
#define ACCEL_Z_MAX ((float) 246)

// Magnetometer (standard calibration)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
#define MAGN_X_MIN ((float) -511)
#define MAGN_X_MAX ((float) 581)
#define MAGN_Y_MIN ((float) -516)
#define MAGN_Y_MAX ((float) 568)
#define MAGN_Z_MIN ((float) -489)
#define MAGN_Z_MAX ((float) 486)

// Magnetometer (extended calibration)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {55.2973, 11.1152, -12.7898};
const float magn_ellipsoid_transform[3][3] = {{0.835372, 0.00834115, -0.0300560}, {0.00834115, 0.836436, -0.0237541}, {-0.0300560, -0.0237541, 0.990596}};
// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 31.79)
#define GYRO_AVERAGE_OFFSET_Y ((float) 64.22)
#define GYRO_AVERAGE_OFFSET_Z ((float) 16.59)

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

// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false

/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/

// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
  // Generate compile error
  #error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.pde (or .ino)!
#endif

#include <Wire.h>

// Stuff
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration -> tuong duong 1G trong c�c d? li?u th� t? c�c gia t?c 
                       // ADXL345 - Accelerometer: this equivalent to 1G in the raw data coming from the accelerometer 
                       // ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
                        // Tested value : 248   ;))
                        
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

// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
// #f - Request one output frame, internal 20ms (50Hz) time raster ???
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

/////////////////////////////////////////////////////////////////////////////////////
// Sensor variables
float accel_raw[3];
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float mag_raw[3];
float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro_raw[3];
float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm -> Th?i gian tích h?p cho các thu?t toán DCM

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

//custom define for specific board 
//----IMU_BOARD---
//#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)
#if defined(ARDUINO_AVR_DUEMILANOVE)  //ARDUINO_AVR_UNO or ARDUINO_AVR_PRO
  // Arduino Duemilanove or Deicimila
  #define BOARD_MEGA        false
  #define BOARD_IMU_UNO     true    //using IMU Arduino Board      
//#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#elif defined(ARDUINO_AVR_MEGA2560)
  #define BOARD_MEGA        true   //using arduino mega 2560 and shield
  #define BOARD_IMU_UNO     false
#endif 

#define LOGGING_SERIAL  true
#define LOGGING_RF      true

#if BOARD_MEGA
  #define LCD_LOGGING true
#else
  #define LCD_LOGGING false
#endif

#if BOARD_IMU_UNO
  #define LED1_BOARD  8
  const int LED2_BOARD = 16;  //A2 in Arduino UNO Board
  const int LED3_BOARD = 17;  //A3 in Arduino UNO Board

  const int BATTERY_CTRL = 4;  //pin to control enable power supply by battery

  #define NUM_BUTTON 1
  int But_timeout[NUM_BUTTON] = { 0 };       // how many times we have seen new value
  int But[NUM_BUTTON];           // the current value read from the input pin
  int But_state[NUM_BUTTON] = { 1 };    // the debounced input value
  bool But_Press[NUM_BUTTON] = {false};

  const int BUT_PINS[] = {3};  
#endif

#if BOARD_MEGA  
  //#define LED1_BOARD  10    //maybe confict with LCD control pins
  
  const int BATTERY_CTRL = 8;

  #define NUM_BUTTON 3
  int But_timeout[NUM_BUTTON] = { 0 };       // how many times we have seen new value
  int But[NUM_BUTTON];           // the current value read from the input pin
  int But_state[NUM_BUTTON] = { 1 };    // the debounced input value
  bool But_Press[NUM_BUTTON] = {false};
  
  //const int BUT_PINS[] = { A5, A6, A7 };
  const int BUT_PINS[] = { 3, 4, 5 }; 
#endif

// the following variable is a long because the time, measured in milliseconds,
// will quickly become a bigger number than can be stored in an int.
long time_buttons_scan = 0;         // the last time the output pin was sampled
int debounce_count = 10; // number of millis/samples to consider before declaring a debounced input

#define NUMBER_STATE  2
unsigned char state_board = 0;
unsigned char num_sampling = 0;

#define FREQ_PROCESS    1 //multiple by 20ms
unsigned char process_counter = 0;

unsigned int time_counter = 0;
unsigned int tick_1s = 0;

// [w, x, y, z]  quaternion orientation of chip to reference frame
VectorFloat angle = VectorFloat(0, 0, 0);
Quaternion orientationChip;        
Quaternion oldOrientationChip = Quaternion(1, 0, 0, 0); //[w,x,y,z]      of chip orientation
VectorFloat sensor2tip;                             //[x,y,z]        from sensor to tip
VectorFloat normal;                                 //normal         of probe on surface                 
VectorFloat gravity;                                // [x, y, z]     gravity vector
Quaternion quadRotate;                              //[w,x,y,z]      of chip rotation during interval
VectorInt16 aaRaw;                              // [x, y, z]     accel sensor measurements
VectorInt16 aaReal;                                 // [x, y, z]     gravity-free accel sensor measurements
VectorInt16 aaWorld;                                // [x, y, z]     reference-frame accel sensor measurements
VectorFloat aaWorldF;

VectorFloat aaOffset; 
VectorFloat aaSum; 

// Low pass filter variables
VectorFloat new_afilt = VectorFloat(0,0,0);
VectorFloat old_afilt;
VectorFloat new_aLP = VectorFloat(0, 0, 0);
VectorFloat old_aLP;
VectorFloat new_vfilt = VectorFloat(0,0,0);
VectorFloat old_vfilt;
VectorFloat new_vLP = VectorFloat(0, 0, 0);
VectorFloat old_vLP;
VectorFloat new_sfilt = VectorFloat(0,0,0);
VectorFloat old_sfilt;
VectorFloat new_sLP = VectorFloat(0, 0, 0);
VectorFloat old_sLP;

// Translation variables
VectorFloat old_a;
VectorFloat new_a;
VectorFloat old_v;
VectorFloat new_v;
VectorFloat old_s;
VectorFloat new_s;

VectorFloat sensorstart = VectorFloat(1, 1, 1);     //starting position of sensor relative to tip
VectorFloat tip;                                    //[x,y,z]        of probe tip
VectorFloat sensor;                                 //[x,y,z]        of sensor

#include <SPI.h>        //using SPI interface, with nRF chip
#include <nRF24L01.h>   //control wireless interface thought nRF24L01
#include <RF24.h>

//------variable used in nRF communication-----//
/*Create a unique pipe out. The receiver has to wear the same unique code*/
const uint64_t pipeOut = 0xE8E8F0F0E1LL;
const uint64_t pipeIn = 0xDEADBEEFF1L;  //This will be the other device

#if BOARD_IMU_UNO
  RF24 radio(9, 10);
#elif BOARD_MEGA
  RF24 radio(49, 48); // select (CE, CSN)  pin
#endif 

unsigned long lastRecvTime = 0;
bool bRecv = false; //flag indicate receiver acknowleger
bool recv_state = false;

struct SensorInfo{
    byte node_id;
    byte mode;
    byte state;
    float X1; //4 byte
    float Y1;
    float Z1;
    float X2;
    float Y2;
    float Z2;
    byte G_Dt;  //1 byte
    unsigned long time_tick;  //4 byte
};
SensorInfo  SensorXYZ;        //receiving
SensorInfo  CtrlSensorXYZ;    //sending

struct AckPayload {
  float vol_bat;  //voltage of battery in car
  unsigned long time_tick;  //timer tick in car's processing
  byte mode;   //current speed of car
  byte state;   //state of car
};
AckPayload ack;     //for receiving
AckPayload CtrlAck; //for sending

#if (LCD_LOGGING == true) && (BOARD_MEGA == true)
  #include <Adafruit_GFX.h>
  #include <Adafruit_PCD8544.h>
  
  #define LCD_RST   9     //LCD reset (RST)
  #define LCD_CS    10    //A6 - LCD chip select (CS) - connect to GND thought 10K
  #define LCD_DC    11     //Data/Command select (D/C)
  #define LCD_DIN   12     //Serial data out (DIN)
  #define LCD_SCK   13     //Serial clock out (SCLK)
  
  //#define LCD_CONSTRACT 60
  #define LCD_CONSTRACT 40  //60
    
  Adafruit_PCD8544 display = Adafruit_PCD8544(LCD_SCK, LCD_DIN, LCD_DC, LCD_CS, LCD_RST);
#endif  
