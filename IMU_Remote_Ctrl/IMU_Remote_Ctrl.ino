#include "HW_Config.h"

/**************************************************/
// Blocks until another byte is available on serial port
char readChar(){
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

void Send_Cmd(){
  if(next_mode == OUTPUT_UNKNOWN)  return;
  
  bool resend = false;
  if(output_mode != next_mode) {
    Serial.print("Mode ");Serial.print(output_mode);
    Serial.print(" to ");Serial.println(next_mode);
    resend = true;    
  }
  else if(next_mode == OUTPUT_CALIBRATE_SENSORS){
    if(output_state != next_state){
      Serial.print("State ");Serial.print(output_state);
      Serial.print(" to ");Serial.println(next_state);
      resend = true;
    }
  }
  else{
    output_mode = next_mode;    
    next_mode = OUTPUT_UNKNOWN;
    Serial.println("Change Complite");
  }
  
  if(resend == true)  {
    num_send++;
    RemoteSensorXYZ.mode = next_mode;
    RemoteSensorXYZ.state = next_state;
    
    Serial.print("send cmd["); Serial.print(next_mode);
    if(RF_Trans(1)) Serial.println("].");
    else            Serial.println("]*");

    if(num_send >= 10){
      num_send = 0;
      next_mode = OUTPUT_UNKNOWN;
      Serial.println("Timeout sending...");
    }
  }  
}

void loop()
{  
  Serial_Cmd(); //terminal serial cmd process
  Send_Cmd();
  
	recvData();   //rf communication with node sensor
	
	if (bRecv == true){
		bRecv = false;

		Dt = SensorXYZ.G_Dt;    
    prev_mode = output_mode;
    output_mode = SensorXYZ.mode;
    
    prev_state = output_state;
    output_state = SensorXYZ.state;

    time_counter++;
        
    if(output_mode != prev_mode){      
      Serial.print("Change mode to ");Serial.println(output_mode);        
    }
    /*
    if(output_state != prev_state){      
      Serial.print("Change state to ");Serial.println(output_state);        
    }
    */
    
    sensor_monitor();        
    //print_raw_data();
  }
  
  if(time_counter % 25 == 0) blink_led();
  if((millis() - timestamp) >= 200) //200ms for update display
  {    
    timestamp = millis();    
    display_info();
  }
  
  Scan_buttons();   
}

void sensor_monitor()
{
  switch(output_mode)
  {
  case OUTPUT_CALIBRATE_SENSORS:
    output_calibration(output_state);
    break;  
    
  case OUTPUT_CUSTOM:
    output_custom(bplotting);
    break;
  
  case START_PROCESS:
    output_position_info(bplotting);
    break;    
    
  case OUTPUT_ANGLES:  
    output_angles();
    break;
    
  case OUTPUT_RAW_ACCEL:
    out_raw_accel(bplotting);
    break;   
    
  case OUTPUT_SENSORS_RAW:
  case OUTPUT_SENSORS_CALIB:  
    output_sensors();
    break;
      
  default:  
    Serial.println("unknown cmd");
    break;
  }
}

void setup()
{
  board_init();
  
  Serial.begin(115200);
  Serial.println("Setup....");

  //init variables
  next_mode = OUTPUT_UNKNOWN;
  prev_mode = output_mode = OUTPUT_NONE;

  prev_state = output_state = 0;
  num_send = 0;

  timestamp = millis();
}

void Serial_Cmd()
{    
  char values_param, format_param, output_param;
  
  if (Serial.available() >= 2) {
    // Start of new control message
    if (Serial.read() == '#') {
      Serial.print("CMD [");
      //next_mode = OUTPUT_UNKNOWN;
      
      int command = Serial.read(); // Commands      
      
      if (command == 'o') // Set output mode
      {                
        output_param = readChar();
                
        //"#on" - When in calibration mode, go on to calibrate NEXT sensor.
        switch(output_param)
        {
        case 'n':  // Calibrate next sensor
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;          
          next_mode = OUTPUT_CALIBRATE_SENSORS;
          next_state = curr_calibration_sensor;
          Serial.print("CMD_CALIBRATION_NEXT]");
          break;
        
        //#oc" - Go to CALIBRATION output mode.
        case 'c': // Go to calibration mode
          curr_calibration_sensor = 0;
          next_mode = OUTPUT_CALIBRATE_SENSORS;
          next_state = curr_calibration_sensor;
          Serial.print("CMD_CALIBRATION]");
          break;
          
        //#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
        //       followed by carriage return and line feed [\r\n]).
        case 't': // Output angles as text
          next_mode = OUTPUT_ANGLES;
          next_state = OUTPUT__FORMAT_TEXT;           
          Serial.print("CMD_OUTPUT_ANGLES_TEXT]");
          break; 
                          
        case 'h': // Output angles custom format
          next_mode = OUTPUT_CUSTOM;
          Serial.print("CMD_OUT_CUSTOM]");     
          break;     
        /*
        // Sensor data output
        "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
        "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
        "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
                In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.        

         "#osa": output raw accelermeter
         */         
        case 's': // Output sensor values
          values_param = readChar();
          format_param = readChar();
          
          switch(values_param)  {
          case 'r':  // Output raw sensor values
            next_mode = OUTPUT_SENSORS_RAW;
            Serial.print("CMD_OUTPUT_SENSOR_RAW]");
            break;
          case 'c':  // Output calibrated sensor values
            next_mode = OUTPUT_SENSORS_CALIB;
            Serial.print("CMD_OUTPUT_SENSOR_CALIB]");
            break;          
          
          //add new command by Tuan #osat
          case 'a':  // Output both sensor values (raw and calibrated)
            next_mode = OUTPUT_RAW_ACCEL;
            Serial.print("CMD_OUTPUT_RAW_ACCEL]");
            break;
          }
          
          if (format_param == 't') // Output values as text
            next_state = OUTPUT__FORMAT_TEXT;  //text            
          else if (format_param == 'b') // Output values in binary format
            next_state = OUTPUT__FORMAT_BINARY;  //binary        
          break;
        
        //#o0" - DISABLE continuous streaming output. Also see #f below.
        case '0': // Disable continuous streaming output
          next_mode = OUTPUT_NONE;
          Serial.print("CMD_DISABLE_STREAM]");
          break;
          
        //#o1" - ENABLE continuous streaming output.
        case '1': // Enable continuous streaming output
          next_mode = OUTPUT_RESUME;
          Serial.print("CMD_ENABLE_STREAM]");
          break;
          
        default: 
          next_mode = OUTPUT_UNKNOWN;                  
          Serial.print("CMD_UNKNOWN]");
          break;
        }//end switch(output_param)
      }
      else if (command == 'b')  //start position process
      {
        next_mode = START_PROCESS;  
        Serial.print("START_PROCESS]");
      }
      else if (command == 'e')  //stop/end of position process
      {
        next_mode = STOP_PROCESS;  
        Serial.print("STOP_PROCESS]");
      }
      
      //send command to sensor node
      if(next_mode != OUTPUT_UNKNOWN){
        RemoteSensorXYZ.mode = next_mode;
        RemoteSensorXYZ.state = next_state;        
        
        Serial.print("Current mode: "); Serial.print(output_mode); 
        Serial.print(" change mode to ");Serial.println(RemoteSensorXYZ.mode);        
      }
      else	Serial.print("Unknow command...");
    }
  }  
}

