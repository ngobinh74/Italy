#include "HW_Config.h"

void check_cmd()
{
  byte mode = SensorXYZ.mode;  
  
  //set output mode when received command from master node
  switch(mode)
  {
  case OUTPUT_CALIBRATE_SENSORS: //#0c or #on
    next_output_mode = OUTPUT_CALIBRATE_SENSORS;
    curr_calibration_sensor = SensorXYZ.state;    
    reset_calibration_session_flag = true;
    break;    
  
  case OUTPUT_CUSTOM:    
    next_output_mode = OUTPUT_CUSTOM;      
    break;
    
  case OUTPUT_SENSORS_RAW:    
    next_output_mode = OUTPUT_SENSORS_RAW;
    if(SensorXYZ.state == 0)  //text
      output_format = OUTPUT__FORMAT_TEXT;
    else  //binary
      output_format = OUTPUT__FORMAT_BINARY;
    break; 
       
  case OUTPUT_SENSORS_CALIB:    
    next_output_mode = OUTPUT_SENSORS_CALIB;
    if(SensorXYZ.state == 0)  //text
      output_format = OUTPUT__FORMAT_TEXT;
    else  //binary
      output_format = OUTPUT__FORMAT_BINARY;      
    break;
          
  case OUTPUT_RAW_ACCEL:  //new by Tuan    
    next_output_mode = OUTPUT_RAW_ACCEL;
    output_format = OUTPUT__FORMAT_TEXT; 
    break;

  case OUTPUT_ANGLES:    
    next_output_mode = OUTPUT_ANGLES;
    output_format = OUTPUT__FORMAT_TEXT;
    break;
    
  case OUTPUT_NONE:    
    turn_output_stream_off();
    reset_calibration_session_flag = true;
    break;
    
  case OUTPUT_RESUME:    
    reset_calibration_session_flag = true;
    turn_output_stream_on();
    break;

  case START_PROCESS:
    next_output_mode = START_PROCESS;
    process_counter = 0;
    state_board = 1;  //start process
    break;
  case STOP_PROCESS:
    next_output_mode = OUTPUT_NONE;    
    state_board = 0;  //stop process
    break;    
  }   
}

void imu_process()
{
  // Update sensor readings
  read_sensors();

  //save raw acceleronmeter values
  aaRaw.x = (int)accel_raw[0];
  aaRaw.y = (int)accel_raw[1];
  aaRaw.z = (int)accel_raw[2];

  //filtering with raw acceleronmeter data
  //....
  //aaFilteredF   = noiseFilter(&aaFilteredF,   &aaRaw,   &aaFilteredF,  noiseParaAccelLow, &noiseParaAccelSen, &noiseParaAccel);

  // Apply sensor calibration
  compensate_sensor_errors(); 
  
  // Run DCM algorithm
  Compass_Heading(); // Calculate magnetic heading
  Matrix_update();
  Normalize();  
  Drift_correction();
  Euler_angles(); //calculate Euler angles (Pitch/Roll/Yaw)

  GetQuat();      //calculate quanternion
    
  //rotating
  // finding the rotation shift during the interval, disp * q1 = q2 --> disp = q2*inv(q1)
  quadRotate = orientationChip.getProduct(oldOrientationChip.getConjugate()); 
  sensor2tip.rotate(&quadRotate);         //rotate the sensor to tip translation vector with the new rotation
  normal.rotate(&quadRotate);             //rotate the normal with the new rotation
  oldOrientationChip = orientationChip;   //store the orientation

  GetGravity(&gravity, &orientationChip);         //get gravity in gravity(x,y,z)      
    
  //Get real/liner accelermeter
  aaReal.x = aaRaw.x - gravity.x * 256; //3.9mG - 1 LSB ==> 255 * 3.9 mG = 1000 mG = 1G
  aaReal.y = aaRaw.y - gravity.y * 256;
  aaReal.z = aaRaw.z - gravity.z * 256;

  //get world accelermeter
  aaWorld = aaReal.getRotated(&orientationChip);

  switch(state_board)
  {
  case 0: //init
    num_sampling++;
    //aaSum.getAddition(&aaWorld);
    aaSum.x += aaWorld.x;
    aaSum.y += aaWorld.y;
    aaSum.z += aaWorld.z;
    if(num_sampling >= 64){
      aaOffset.x = aaSum.x / 64;
      aaOffset.y = aaSum.y / 64;
      aaOffset.z = aaSum.z / 64;
      num_sampling = 0;
      aaSum.x = aaSum.y = aaSum.z = 0;
      /*
      Serial.print("Offset: [");
      Serial.print((int)aaOffset.x);Serial.print("\t");
      Serial.print((int)aaOffset.y);Serial.print("\t");
      Serial.print((int)aaOffset.z);Serial.println("]");      
      */
    }
    break;
  case 1: //running
    process_counter++;    
    aaWorld.x = aaWorld.x - aaOffset.x;
    aaWorld.y = aaWorld.y - aaOffset.y;
    aaWorld.z = aaWorld.z - aaOffset.z;
    //aaSum.getAddition(&aaWorld);
    aaSum.x += aaWorld.x;
    aaSum.y += aaWorld.y;
    aaSum.z += aaWorld.z;
    if(process_counter >= FREQ_PROCESS){
      aaWorld.x = (int)(aaSum.x / process_counter);
      aaWorld.y = (int)(aaSum.y / process_counter);
      aaWorld.z = (int)(aaSum.z / process_counter);
      
      Positioning();
      
      //reset variables
      process_counter = 0;
      aaSum.x = 0;
      aaSum.y = 0;
      aaSum.z = 0;
    }
    
    break;
  }  
}

// Main loop
void loop()
{     
  recvData();
  
  if (bRecv == true){
    bRecv = false;    
    check_cmd();
  }
  
  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL) //20ms
  {
    //blink led
    time_counter++;
    if(time_counter % 25 == 0)  tick_1s++;

    #if (BOARD_MEGA == false)
    switch(state_board)
    {
    case 0: //init - fast blink
      if (time_counter % 20 == 0) digitalWrite(LED1_BOARD, LOW);
      else                        digitalWrite(LED1_BOARD, HIGH);
      break;
    case 1: //running - slow blink      
      if (tick_1s % 2 == 0) digitalWrite(LED1_BOARD, LOW);
      else                  digitalWrite(LED1_BOARD, HIGH);
      break;
    }    
    #endif
    
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    CtrlSensorXYZ.G_Dt = G_Dt;
    CtrlSensorXYZ.time_tick = timestamp;  //send 1s'tick to car
    
    imu_process();

    //output information    
    switch(output_mode)
    {
    case OUTPUT_CALIBRATE_SENSORS:  // We're in calibration mode
      check_reset_calibration_session();  // Check if this session needs a reset
      if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
      break;

    case OUTPUT_CUSTOM:  // Output custom            
      if (output_stream_on || output_single_on) output_custom();
      break;
      
    case START_PROCESS:
      output_position();      
      break;
      
    case OUTPUT_RAW_ACCEL:  //adding by Tuan
      if (output_stream_on || output_single_on) out_accel();
      break;    
      
    case OUTPUT_SENSORS_RAW:
    case OUTPUT_SENSORS_CALIB:
      if (output_stream_on || output_single_on) output_sensors();
      break;
                
    case OUTPUT_ANGLES:  // Output angles
      if (output_stream_on || output_single_on) output_angles();
      break;
                
    default: //Output sensor values
      break;
    }
    
    output_single_on = false;

    //change output mode
    if(next_output_mode != output_mode) {      
      output_mode = next_output_mode;
    }    
  }    

  if(time_counter % 10 == 0)  display_info(); //200ms
  Scan_buttons();
}

void read_sensors() {
  Read_Gyro();    // Read gyroscope
  Read_Accel();   // Read accelerometer
  Read_Magn();    // Read magnetometer
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
  // Compensate accelerometer error
  //ACCEL_X_SCALE = (256 / (ACCEL_X_MAX - ACCEL_X_OFFSET))
  accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
  
  // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
  //The magBias obtained will correct hard iron errors, the scalefactor will correct soft iron errors.
  for (int i = 0; i < 3; i++)
    magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i]; //magn_ellipsoid_center <=> magBias
  Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom); //magn_ellipsoid_transform <=> scalefactor
#else
  magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
  magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
  magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

  // Compensate gyroscope error
  gyro[0] -= GYRO_AVERAGE_OFFSET_X;
  gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
  gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel_raw[i];
    magnetom_min[i] = magnetom_max[i] = mag_raw[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
}

void turn_output_stream_off()
{
  output_stream_on = false;
}

void setup()
{
  board_init();
   
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);  
  
  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();
  
  turn_output_stream_on();
  next_output_mode = output_mode;

  state_board = 0;
  process_counter = 0;
  num_sampling = 0;
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
  
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
  
  resetVars();  //adding by tuan, for position process
}


