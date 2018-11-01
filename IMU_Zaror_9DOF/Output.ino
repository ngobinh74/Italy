void out_accel()
{
  #if LOGGING_RF
  //send data thought RF communication
  CtrlSensorXYZ.mode = output_mode;        
  CtrlSensorXYZ.X2 = accel_raw[0];
  CtrlSensorXYZ.Y2 = accel_raw[1];
  CtrlSensorXYZ.Z2 = accel_raw[2];
  CtrlSensorXYZ.X2 = aaWorld.x;
  CtrlSensorXYZ.Y2 = aaWorld.y;
  CtrlSensorXYZ.Z2 = aaWorld.z;  
  RF_Trans(1);
  CtrlSensorXYZ.mode = output_mode;
  CtrlSensorXYZ.state = 1;
  CtrlSensorXYZ.X1 = aaReal.x;
  CtrlSensorXYZ.Y1 = aaReal.y;
  CtrlSensorXYZ.Z1 = aaReal.z;  
  CtrlSensorXYZ.X2 = aaOffset.x;
  CtrlSensorXYZ.Y2 = aaOffset.y;
  CtrlSensorXYZ.Z2 = aaOffset.z;
  RF_Trans(1);
  #endif
}

void output_position()
{
  #if LOGGING_RF
  //send data thought RF communication
  CtrlSensorXYZ.mode = output_mode;        
  CtrlSensorXYZ.state = 0;
  CtrlSensorXYZ.X2 = accel_raw[0];
  CtrlSensorXYZ.Y2 = accel_raw[1];
  CtrlSensorXYZ.Z2 = accel_raw[2];
  CtrlSensorXYZ.X2 = aaWorld.x;
  CtrlSensorXYZ.Y2 = aaWorld.y;
  CtrlSensorXYZ.Z2 = aaWorld.z;  
  RF_Trans(1);
  CtrlSensorXYZ.mode = output_mode;
  CtrlSensorXYZ.state = 1;
  CtrlSensorXYZ.X1 = new_afilt.x;
  CtrlSensorXYZ.Y1 = new_afilt.y;
  CtrlSensorXYZ.Z1 = new_afilt.z;  
  CtrlSensorXYZ.X2 = aaOffset.x;
  CtrlSensorXYZ.Y2 = aaOffset.y;
  CtrlSensorXYZ.Z2 = aaOffset.z;
  RF_Trans(1);
  CtrlSensorXYZ.mode = output_mode;
  CtrlSensorXYZ.state = 2;
  CtrlSensorXYZ.X1 = new_vfilt.x;
  CtrlSensorXYZ.Y1 = new_vfilt.y;
  CtrlSensorXYZ.Z1 = new_vfilt.z;  
  CtrlSensorXYZ.X2 = new_sfilt.x;
  CtrlSensorXYZ.Y2 = new_sfilt.y;
  CtrlSensorXYZ.Z2 = new_sfilt.z;
  RF_Trans(1);
  #endif
}

void output_custom()
{  
  if (output_format == OUTPUT__FORMAT_TEXT)
  {
  	#if LOGGING_SERIAL     
    Serial.print("$INS");Serial.print(",");
    Serial.print("Y");Serial.print(TO_DEG(yaw));Serial.print(",");    // Blue
    Serial.print("R");Serial.print(TO_DEG(roll));Serial.print(",");  // Red
    Serial.print("P");Serial.print(TO_DEG(pitch));Serial.print(","); // Green
        
    //Serial.print("Ax");Serial.print(accel[0]);Serial.print(",");
    //Serial.print("Ay");Serial.print(accel[1]);Serial.print(",");
    //Serial.print("Az");Serial.print(accel[2]);Serial.print(",");
    Serial.print("Ax");Serial.print(aaWorld.x);Serial.print(",");
    Serial.print("Ay");Serial.print(aaWorld.y);Serial.print(",");
    Serial.print("Az");Serial.print(aaWorld.z);Serial.print(",");

    Serial.print("Gx");Serial.print(gyro[0]);Serial.print(","); 
    Serial.print("Gy");Serial.print(gyro[1]);Serial.print(",");
    Serial.print("Gz");Serial.print(gyro[2]);Serial.print(",");
    
    Serial.print("Mx");Serial.print(magnetom[0]);Serial.print(",");
    Serial.print("My");Serial.print(magnetom[1]);Serial.print(",");
    Serial.print("Mz");Serial.print(magnetom[2]);Serial.println(",*");
    #endif    
  }  
  
  #if LOGGING_RF
  //send data thought RF communication
  CtrlSensorXYZ.mode = output_mode;        
  CtrlSensorXYZ.state = 0;
  CtrlSensorXYZ.X1 = TO_DEG(yaw);
  CtrlSensorXYZ.Y1 = TO_DEG(roll);
  CtrlSensorXYZ.Z1 = TO_DEG(pitch);    
  //CtrlSensorXYZ.X2 = accel[0];
  //CtrlSensorXYZ.Y2 = accel[1];
  //CtrlSensorXYZ.Z2 = accel[2];
  CtrlSensorXYZ.X2 = aaWorld.x;
  CtrlSensorXYZ.Y2 = aaWorld.y;
  CtrlSensorXYZ.Z2 = aaWorld.z;
  RF_Trans(1);
  CtrlSensorXYZ.mode = output_mode;
  CtrlSensorXYZ.state = 1;
  CtrlSensorXYZ.X1 = gyro[0];
  CtrlSensorXYZ.Y1 = gyro[1];
  CtrlSensorXYZ.Z1 = gyro[2];  
  CtrlSensorXYZ.X2 = magnetom[0];
  CtrlSensorXYZ.Y2 = magnetom[1];
  CtrlSensorXYZ.Z2 = magnetom[2];
  RF_Trans(1);
  CtrlSensorXYZ.mode = output_mode;
  CtrlSensorXYZ.state = 2;
  CtrlSensorXYZ.X1 = orientationChip.w;
  CtrlSensorXYZ.Y1 = orientationChip.x;
  CtrlSensorXYZ.Z1 = orientationChip.y;  
  CtrlSensorXYZ.X2 = orientationChip.z;
  RF_Trans(1);
  CtrlSensorXYZ.mode = output_mode;
  CtrlSensorXYZ.state = 3;
  CtrlSensorXYZ.X1 = aaOffset.x;
  CtrlSensorXYZ.Y1 = aaOffset.y;
  CtrlSensorXYZ.Z1 = aaOffset.z;
  CtrlSensorXYZ.X2 = accel_raw[0];
  CtrlSensorXYZ.Y2 = accel_raw[1];
  CtrlSensorXYZ.Z2 = accel_raw[2];
  RF_Trans(1);
  #endif
}

void output_calibration(int calibration_sensor)
{ 
  if (calibration_sensor == 0)  // Accelerometer
  {        
    // Output MIN/MAX values
    #if LOGGING_SERIAL    
    Serial.print("accel x,y,z (min/max) = ");
    #endif
    
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];

      #if LOGGING_SERIAL
      Serial.print(accel_min[i]);
      Serial.print("/");
      Serial.print(accel_max[i]);      
      
      if (i < 2) Serial.print("  ");
      else Serial.println();
      #endif      
    }
    
    #if LOGGING_RF
    CtrlSensorXYZ.mode = OUTPUT_CALIBRATE_SENSORS;
    CtrlSensorXYZ.state = 0;
    CtrlSensorXYZ.X1 = accel_min[0];
    CtrlSensorXYZ.Y1 = accel_min[1];
    CtrlSensorXYZ.Z1 = accel_min[2];        
    CtrlSensorXYZ.X2 = accel_max[0];
    CtrlSensorXYZ.Y2 = accel_max[1];
    CtrlSensorXYZ.Z2 = accel_max[2];
    RF_Trans(1);
    #endif
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {        
    // Output MIN/MAX values
    #if LOGGING_SERIAL
    Serial.print("magn x,y,z (min/max) = ");
    #endif
    
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      
      #if LOGGING_SERIAL
      Serial.print(magnetom_min[i]);
      Serial.print("/");
      Serial.print(magnetom_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
      #endif
    }
    
    #if LOGGING_RF
    CtrlSensorXYZ.mode = OUTPUT_CALIBRATE_SENSORS;
    CtrlSensorXYZ.state = 1;
    CtrlSensorXYZ.X1 = magnetom_min[0];
    CtrlSensorXYZ.Y1 = magnetom_min[1];
    CtrlSensorXYZ.Z1 = magnetom_min[2];    
    CtrlSensorXYZ.X2 = magnetom_max[0];
    CtrlSensorXYZ.Y2 = magnetom_max[1];
    CtrlSensorXYZ.Z2 = magnetom_max[2];
    RF_Trans(1);
    #endif
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {        
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    #if LOGGING_SERIAL
    // Output current and averaged gyroscope values
    Serial.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      Serial.print(gyro[i]);
      Serial.print("/");
      Serial.print(gyro_average[i] / (float) gyro_num_samples);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
    #endif
    
    #if LOGGING_RF
    CtrlSensorXYZ.mode = OUTPUT_CALIBRATE_SENSORS;
    CtrlSensorXYZ.state = 2;
    CtrlSensorXYZ.X1 = gyro[0];
    CtrlSensorXYZ.Y1 = gyro[1];
    CtrlSensorXYZ.Z1 = gyro[2];    
    CtrlSensorXYZ.X2 = gyro_average[0] / (float) gyro_num_samples;
    CtrlSensorXYZ.Y2 = gyro_average[1] / (float) gyro_num_samples;
    CtrlSensorXYZ.Z2 = gyro_average[2] / (float) gyro_num_samples;
    RF_Trans(1);
    #endif
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  #if LOGGING_SERIAL
  switch(raw_or_calibrated)
  {
  case 'C':
    Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(accel[0]); Serial.print(",");
    Serial.print(accel[1]); Serial.print(",");
    Serial.print(accel[2]); Serial.println();

    Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(magnetom[0]); Serial.print(",");
    Serial.print(magnetom[1]); Serial.print(",");
    Serial.print(magnetom[2]); Serial.println();

    Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(gyro[0]); Serial.print(",");
    Serial.print(gyro[1]); Serial.print(",");
    Serial.print(gyro[2]); Serial.println();
    break;  
  case 'R':
    Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(accel_raw[0]); Serial.print(",");
    Serial.print(accel_raw[1]); Serial.print(",");
    Serial.print(accel_raw[2]); Serial.println();

    Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(mag_raw[0]); Serial.print(",");
    Serial.print(mag_raw[1]); Serial.print(",");
    Serial.print(mag_raw[2]); Serial.println();

    Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
    Serial.print(gyro_raw[0]); Serial.print(",");
    Serial.print(gyro_raw[1]); Serial.print(",");
    Serial.print(gyro_raw[2]); Serial.println();
    break;
  }  
  #endif
}

void Send_Sensor_Cali_Data(unsigned char type)
{
  #if LOGGING_RF 
  CtrlSensorXYZ.mode = output_mode;    
  switch(type)
  {
  case 0: 
    CtrlSensorXYZ.state = 0;    
    CtrlSensorXYZ.X1 = accel[0];
    CtrlSensorXYZ.Y1 = accel[1];
    CtrlSensorXYZ.Z1 = accel[2];    
    RF_Trans(1); 
    break;
  case 1:
    CtrlSensorXYZ.state = 1;
    CtrlSensorXYZ.X1 = magnetom[0];
    CtrlSensorXYZ.Y1 = magnetom[1];
    CtrlSensorXYZ.Z1 = magnetom[2];
    RF_Trans(1);
    break;
  case 2:
    CtrlSensorXYZ.state = 2;
    CtrlSensorXYZ.X1 = gyro[0];
    CtrlSensorXYZ.Y1 = gyro[1];
    CtrlSensorXYZ.Z1 = gyro[2];
    RF_Trans(1);
    break;
  }  
  #endif  
}

void Send_Sensor_Raw_Data(unsigned char type)
{
  #if LOGGING_RF 
  CtrlSensorXYZ.mode = output_mode;    
  switch(type)
  {
  case 0: 
    CtrlSensorXYZ.state = 0;    
    CtrlSensorXYZ.X1 = accel_raw[0];
    CtrlSensorXYZ.Y1 = accel_raw[1];
    CtrlSensorXYZ.Z1 = accel_raw[2];    
    RF_Trans(1); 
    break;
  case 1:
    CtrlSensorXYZ.state = 1;
    CtrlSensorXYZ.X1 = mag_raw[0];
    CtrlSensorXYZ.Y1 = mag_raw[1];
    CtrlSensorXYZ.Z1 = mag_raw[2];
    RF_Trans(1);
    break;
  case 2:
    CtrlSensorXYZ.state = 2;
    CtrlSensorXYZ.X1 = gyro_raw[0];
    CtrlSensorXYZ.Y1 = gyro_raw[1];
    CtrlSensorXYZ.Z1 = gyro_raw[2];
    RF_Trans(1);
    break;
  }  
  #endif  
}

void output_sensors()
{  
  //output text through serial
  if (output_format == OUTPUT__FORMAT_TEXT){
    if(output_mode == OUTPUT_SENSORS_CALIB){
      output_sensors_text('C');
      //send over RF
      Send_Sensor_Cali_Data(0);  //acc
      Send_Sensor_Cali_Data(1);  //mag
      Send_Sensor_Cali_Data(2);  //gro
    }
    else{
      output_sensors_text('R');
      //send over RF
      Send_Sensor_Raw_Data(0);  //acc
      Send_Sensor_Raw_Data(1);  //mag
      Send_Sensor_Raw_Data(2);  //gro
    }    
  }
      
  /*
  if(output_mode == OUTPUT_SENSORS_BOTH) {        
    //send calibrated data after raw data
    if (output_format == OUTPUT__FORMAT_TEXT){
      output_sensors_text('C');
      //send over RF
      Send_Sensor_Data(0);  //acc
      Send_Sensor_Data(1);  //mag
      Send_Sensor_Data(2);  //gro
    }
  } 
  */
}

// Output angles: yaw, pitch, roll
void output_angles()
{  
  if (output_format == OUTPUT__FORMAT_TEXT)
  {   
    #if LOGGING_SERIAL
    Serial.print("#YPR=");
    Serial.print(TO_DEG(yaw)); Serial.print(",");
    Serial.print(TO_DEG(pitch)); Serial.print(",");
    Serial.print(TO_DEG(roll)); Serial.println();
    #endif
  }
  
  #if LOGGING_RF  
  CtrlSensorXYZ.mode = OUTPUT_ANGLES;  
  CtrlSensorXYZ.state = OUTPUT__FORMAT_TEXT;
  
  //send data thought RF communication
  CtrlSensorXYZ.X1 = TO_DEG(yaw);
  CtrlSensorXYZ.Y1 = TO_DEG(pitch);
  CtrlSensorXYZ.Z1 = TO_DEG(roll);
  RF_Trans(1);
  #endif
}
