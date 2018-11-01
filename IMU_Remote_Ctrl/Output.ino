void out_raw_accel(bool bplot)
{  
  switch(SensorXYZ.state)
  {
  case 0:
    accel_raw[0] = SensorXYZ.X1;
    accel_raw[1] = SensorXYZ.Y1;
    accel_raw[2] = SensorXYZ.Z1;    
    accel[0] = SensorXYZ.X2;
    accel[1] = SensorXYZ.Y2;
    accel[2] = SensorXYZ.Z2;
    return;
  case 1:
    q0 = SensorXYZ.X1;
    q1 = SensorXYZ.Y1;
    q2 = SensorXYZ.Z1;      
    q3 = SensorXYZ.X2;
    break;
  }

  if(bplot == true){
    //getQuat();
    char q1_text[30];
    char q2_text[30];
    char q3_text[30];
    char q4_text[30];

    dtostrf(q0, 10, 10, q1_text);
    dtostrf(q1, 10, 10, q2_text);
    dtostrf(q2, 10, 10, q3_text);
    dtostrf(q3, 10, 10, q4_text);

    char text[166];
    snprintf(text, 166, "%s,%s,%s,%s,%d,%d,%d,%d,%d,%d", q1_text, q2_text, q3_text, q4_text,
      //(int)(accel[0] - offset[0]), (int)(accel[1] - offset[1]), (int)(accel[2] - offset[2]),
      (int)accel[0], (int)accel[1], (int)accel[2],
      (int)accel_raw[0], (int)accel_raw[1], (int)accel_raw[2]);      
    Serial.println(text);
  }
  else{
    Serial.print("Ax");Serial.print(accel[0]);Serial.print(",");
    Serial.print("Ay");Serial.print(accel[1]);Serial.print(",");
    Serial.print("Az");Serial.print(accel[2]);Serial.println(",");
  }
}

void output_position_info(bool bplot)
{   
  switch(SensorXYZ.state)
  {
  case 0:
    accel_raw[0] = SensorXYZ.X1;
    accel_raw[1] = SensorXYZ.Y1;
    accel_raw[2] = SensorXYZ.Z1;    
    accel[0] = SensorXYZ.X2;
    accel[1] = SensorXYZ.Y2;
    accel[2] = SensorXYZ.Z2;
    return;
  case 1:
    accel_filtered[0] = SensorXYZ.X1;
    accel_filtered[1] = SensorXYZ.Y1;
    accel_filtered[2] = SensorXYZ.Z1;      
    offset[0] = SensorXYZ.X2;
    offset[1] = SensorXYZ.Y2;
    offset[2] = SensorXYZ.Z2;
    return;  
  case 2:
    vel_filtered[0] = SensorXYZ.X1;
    vel_filtered[1] = SensorXYZ.Y1;
    vel_filtered[2] = SensorXYZ.Z1;      
    pos_filtered[0] = SensorXYZ.X2;
    pos_filtered[1] = SensorXYZ.Y2;
    pos_filtered[2] = SensorXYZ.Z2;
    break;
  }
  if(bplot == true){
    char text[63];
    snprintf(text, 63, "%d,%d,%d,%d,%d,%d,%d,%d,%d",(int)accel_filtered[0], (int)accel_filtered[1], (int)accel_filtered[2],
                                                    (int)vel_filtered[0], (int)vel_filtered[1], (int)vel_filtered[2], 
                                                    (int)pos_filtered[0], (int)pos_filtered[1], (int)pos_filtered[2]);
    Serial.println(text);
  }
  else{; }  
}

void output_custom(bool bplot)
{   
  switch(SensorXYZ.state)
  {
  case 0:
    yaw = SensorXYZ.X1;
    pitch = SensorXYZ.Y1;
    roll = SensorXYZ.Z1;    
    accel[0] = SensorXYZ.X2;
    accel[1] = SensorXYZ.Y2;
    accel[2] = SensorXYZ.Z2;
    return;
  case 1:
    gyro[0] = SensorXYZ.X1;
    gyro[1] = SensorXYZ.Y1;
    gyro[2] = SensorXYZ.Z1;      
    magnetom[0] = SensorXYZ.X2;
    magnetom[1] = SensorXYZ.Y2;
    magnetom[2] = SensorXYZ.Z2;
    return;
  case 2:
    q0 = SensorXYZ.X1;
    q1 = SensorXYZ.Y1;
    q2 = SensorXYZ.Z1;      
    q3 = SensorXYZ.X2;
    return;
  case 3:
    offset[0] = SensorXYZ.X1;
    offset[1] = SensorXYZ.Y1;
    offset[2] = SensorXYZ.Z1;      
    accel_raw[0] = SensorXYZ.X2;
    accel_raw[1] = SensorXYZ.Y2;
    accel_raw[2] = SensorXYZ.Z2;
    break;
  }
  if(bplot == true){
    //getQuat();
    char q1_text[30];
    char q2_text[30];
    char q3_text[30];
    char q4_text[30];

    dtostrf(q0, 10, 10, q1_text);
    dtostrf(q1, 10, 10, q2_text);
    dtostrf(q2, 10, 10, q3_text);
    dtostrf(q3, 10, 10, q4_text);

    char text[166];
    snprintf(text, 166, "%s,%s,%s,%s,%d,%d,%d,%d,%d,%d", q1_text, q2_text, q3_text, q4_text,
      //(int)(accel[0] - offset[0]), (int)(accel[1] - offset[1]), (int)(accel[2] - offset[2]),
      (int)accel[0], (int)accel[1], (int)accel[2],
      (int)accel_raw[0], (int)accel_raw[1], (int)accel_raw[2]);      
    Serial.println(text);
  }
  else{
    Serial.print("$INS");Serial.print(",");
    Serial.print("Y");Serial.print(yaw);Serial.print(",");    // Blue
    Serial.print("R");Serial.print(roll);Serial.print(",");  // Red
    Serial.print("P");Serial.print(pitch);Serial.print(","); // Green
        
    Serial.print("Ax");Serial.print(accel[0]);Serial.print(",");
    Serial.print("Ay");Serial.print(accel[1]);Serial.print(",");
    Serial.print("Az");Serial.print(accel[2]);Serial.print(",");

    Serial.print("Gx");Serial.print(gyro[0]);Serial.print(","); 
    Serial.print("Gy");Serial.print(gyro[1]);Serial.print(",");
    Serial.print("Gz");Serial.print(gyro[2]);Serial.print(",");
    
    Serial.print("Mx");Serial.print(magnetom[0]);Serial.print(",");
    Serial.print("My");Serial.print(magnetom[1]);Serial.print(",");
    Serial.print("Mz");Serial.print(magnetom[2]);Serial.println(",*");    
  }
}

// Output angles: yaw, pitch, roll
void output_angles()
{
  yaw = SensorXYZ.X1;
  pitch = SensorXYZ.Y1;
  roll = SensorXYZ.Z1; 
  
  if (output_format == OUTPUT__FORMAT_TEXT)
  {
    Serial.print("#YPR=");
    Serial.print(yaw); Serial.print(",");
    Serial.print(pitch); Serial.print(",");
    Serial.print(roll); Serial.println();    
  }
}

void output_calibration(int calibration_sensor)
{ 
  static float v1[3] = {0};
  static float v2[3] = {0};

  v1[0] = SensorXYZ.X1;//min
  v1[1] = SensorXYZ.Y1;
  v1[2] = SensorXYZ.Z1;    
  v2[0] = SensorXYZ.X2;//max
  v2[1] = SensorXYZ.Y2;
  v2[2] = SensorXYZ.Z2;
    
  // Output MIN/MAX values
  if (calibration_sensor == 0)  // Accelerometer
  { 
    Serial.print("accel x,y,z (min/max) = ");        
    for(int i = 0; i < 3; i++) {
      Serial.print(v1[i]);
      Serial.print("/");
      Serial.print(v2[i]);      
      
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }        
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    Serial.print("magn x,y,z (min/max) = ");    
    for (int i = 0; i < 3; i++) {
      Serial.print(v1[i]);
      Serial.print("/");
      Serial.print(v2[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Output current and averaged gyroscope values
    Serial.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      Serial.print(v1[i]);
      Serial.print("/");
      Serial.print(v2[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
}

void output_sensors_text(char raw_or_calibrated)
{
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
}

void output_sensors()
{  
  if(output_format == OUTPUT__FORMAT_BINARY)  return;

  switch(output_state)
  {
  case 0:
    accel[0] = SensorXYZ.X1;
    accel[1] = SensorXYZ.Y1;
    accel[2] = SensorXYZ.Z1;
    return;
  case 1:
    magnetom[0] = SensorXYZ.X1;
    magnetom[1] = SensorXYZ.Y1;
    magnetom[2] = SensorXYZ.Z1;
    return; 
  case 2:
    gyro[0] = SensorXYZ.X1;
    gyro[1] = SensorXYZ.Y1;
    gyro[2] = SensorXYZ.Z1;
    break;  
  }
  
  if(output_mode == OUTPUT_SENSORS_CALIB) output_sensors_text('C');
  else                                    output_sensors_text('R');  
}

void print_raw_data()
{
  Serial.print("time:\t" + String(SensorXYZ.time_tick) + "\t");
  Serial.print(SensorXYZ.G_Dt);Serial.print("\t");
  Serial.print("Mode: ");Serial.print((char)('0' + SensorXYZ.mode));Serial.print("\t [");
  Serial.print("State: ");Serial.print((char)('0' + SensorXYZ.state));Serial.print("\t [");
  Serial.print(SensorXYZ.X1);Serial.print("\t");
  Serial.print(SensorXYZ.Y1);Serial.print("\t");
  Serial.print(SensorXYZ.Z1);Serial.print("\t");
  Serial.print(SensorXYZ.X2);Serial.print("\t");
  Serial.print(SensorXYZ.Y2);Serial.print("\t");
  Serial.print(SensorXYZ.Z2);Serial.println("]");
}

// or use this loop if sending floats
void plotting() {  
  char text[112];  
  snprintf(text, 112, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", 
    (int)accel[0], (int)accel[1], (int)accel[2], 
    (int)gyro[0], (int)gyro[1], (int)gyro[2], 
    (int)magnetom[0], (int)magnetom[1], (int)magnetom[2], 
    (int)roll, (int)pitch, (int)yaw);    
  Serial.println(text);
}
/*
void getQuat()
{ 
  // Converting vector angle to quaternion 
  float qw, qx, qy, qz;
  float x, y, z;
  float degtorad = 3.14159 / 180; 
    
  x = roll;
  y = pitch;
  z = yaw;    
  x = x*degtorad; 
  y = y*degtorad; 
  z = z*degtorad; 
  q0 = cos(x/2) * cos(y/2) * cos(z/2) + sin(x/2) * sin(y/2) * sin(z/2); 
  q1 = sin(x/2) * cos(y/2) * cos(z/2) - cos(x/2) * sin(y/2) * sin(z/2); 
  q2 = cos(x/2) * sin(y/2) * cos(z/2) + sin(x/2) * cos(y/2) * sin(z/2); 
  q3 = cos(x/2) * cos(y/2) * sin(z/2) - sin(x/2) * sin(y/2) * cos(z/2);     
}
*/
