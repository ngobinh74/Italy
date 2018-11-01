#if defined(ARDUINO_AVR_DUEMILANOVE)  // Arduino Duemilanove or Deicimila
  #define BOARD_ARDUINO     false   //using Arduino UNO/Decimilar 
  #define BOARD_ARDUINO_DIY false   //using Tuan DIY Arduino board
  #define BOARD_IMU_UNO     true    //using IMU Arduino Board
  #define BOARD_MEGA        false
#elif defined(ARDUINO_AVR_MEGA2560)
  #define BOARD_ARDUINO     false   //using Arduino UNO/Decimilar 
  #define BOARD_ARDUINO_DIY false   //using Tuan DIY Arduino board
  #define BOARD_IMU_UNO     false    //using IMU Arduino Board
  #define BOARD_MEGA        true   //using arduino mega 2560 and shield
#endif  

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

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
static unsigned char output_mode = OUTPUT_NONE;
unsigned char next_mode;
unsigned char prev_mode;

// int output_format = OUTPUT__FORMAT_BINARY;
unsigned char output_format = OUTPUT__FORMAT_TEXT;    // in Project , following Binh NGO
unsigned char output_state;
unsigned char next_state;
unsigned char prev_state;

int curr_calibration_sensor = 0;

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

//////////////////////////////////////////////////////////////
//define for nRF Interface
const uint64_t pipeIn = 0xE8E8F0F0E1LL; //Remember that this code is the same as in the transmitter
const uint64_t pipeOut = 0xDEADBEEFF1L;//This will be the other device

// select (CE, CSN)  pin
//CS- pin 8
//CSN - pin 7
#if BOARD_ARDUINO_DIY
  RF24 radio(8, 7);
#elif BOARD_ARDUINO
  RF24 radio(10, 9);
#elif BOARD_IMU_UNO
  RF24 radio(9, 10);
#elif BOARD_MEGA
  RF24 radio(49, 48); // select (CE, CSN)  pin
#endif 

unsigned char num_send = 0;

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

SensorInfo SensorXYZ;
SensorInfo  RemoteSensorXYZ;    //sending

struct AckPayload {
  float vol_bat;  //voltage of battery in car
  unsigned long time_tick;  //timer tick in car's processing
  byte mode;   //current speed of car
  byte state;   //state of car
};
AckPayload ack;
AckPayload RemoteAck; //for sending

unsigned long lastRecvTime = 0;
bool bRecv = false;
bool recv_state = false;

bool transmission_state = true;
unsigned long prev_transmission = 0;

//////////////////////////////////////////////////////////
//define for board interface
#if BOARD_IMU_UNO
  const int BATTERY_CTRL = 4;  //pin to control enable power supply by battery
  const int LED1_BOARD = 8;
  const int LED2_BOARD = 16;  //A2 in Arduino UNO Board
  const int LED3_BOARD = 17;  //A3 in Arduino UNO Board  

  #define NUM_BUTTON 1
  int But_timeout[NUM_BUTTON] = { 0 };       // how many times we have seen new value
  int But[NUM_BUTTON];           // the current value read from the input pin
  int But_state[NUM_BUTTON] = { 1 };    // the debounced input value
  bool But_Press[NUM_BUTTON] = {false};

  const int BUT_PINS[] = {3};    
#endif

#if BOARD_MEGA
  const int BATTERY_CTRL = 8;
  const int LED1_BOARD = 14;
  const int LED2_BOARD = 40;
  const int LED3_BOARD = 41;
  
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

/////////////////////////////////////////////////////////
//define for LCD display
#if BOARD_MEGA
  #define LCD_LOGGING true  //or false
#else
  #define LCD_LOGGING false
#endif

#if (LCD_LOGGING == true) && (BOARD_MEGA == true)
  #include <Adafruit_GFX.h>
  #include <Adafruit_PCD8544.h>
  
  #define LCD_RST   9     //LCD reset (RST)
  #define LCD_CS    10    //A6 - LCD chip select (CS) - connect to GND thought 10K
  #define LCD_DC    11     //Data/Command select (D/C)
  #define LCD_DIN   12     //Serial data out (DIN)
  #define LCD_SCK   13     //Serial clock out (SCLK)
  
  #define LCD_CONSTRACT 60
  //#define LCD_CONSTRACT 40  //60
    
  Adafruit_PCD8544 display = Adafruit_PCD8544(LCD_SCK, LCD_DIN, LCD_DC, LCD_CS, LCD_RST);
    
  // For 1st display mode:
  static unsigned char Leters[] = { 'N' , 'E' , 'S' , 'W' }; 
  static unsigned char arrow_bmp[] ={B00100000, B00100000, B01110000, B01110000, B11111000,};  
#endif  

unsigned long timestamp;
unsigned int time_counter = 0;
unsigned int tick_1s = 0;
bool led1_state;

bool bplotting = true;
///////////////////////////////////////////////////
///variables for main function
///////////////////////////////////////////////////
float Dt = 0;

float ox,oy,oz;
float Axyz;

float offset[3];
float accel_raw[3];
float accel_real[3];
float accel_filtered[3];
float accel[3];  
float accel_min[3];
float accel_max[3];

float vel_filtered[3];
float pos_filtered[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// Euler angles
float yaw;
float pitch;
float roll;

float q0,q1,q2,q3;
