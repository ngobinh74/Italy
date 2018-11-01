/* This my file is my part for the Razor Stick 9-DOF Firmware */
/**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq 4.18,19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq 4.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq 4.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq 4.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq 4.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq 4.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq 4.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq 4.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq 4.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  
  //Compensation the Roll, Pitch and Yaw drift. 
  
  // PI
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
    
  //*****Roll and Pitch***************
  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity. // My e.4.27
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  

  // hieu chinh roll-pitch qua bo PI
  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference My e.4.28
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight); // my e 4.29 (scale -> tich vo huong)
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight); // My eq 4.29
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // make the gyro YAW drift correction based on compass magnetic heading -> Viết ra công thức
 
  mag_heading_x = cos(MAG_Heading);  // My eq 4.22
  mag_heading_y = sin(MAG_Heading);  // My eq 4.22
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error -> my eq4.23
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position. My eq4.24
  
  // hieu chinh yaw qua bo PI
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01 proportional of YAW. P (tỉ lệ) của Yaw // My eq 4.29
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001 Integrator -> I (tích phân) của Yaw // My eq 4.29
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

void Matrix_update(void)
{
  Gyro_Vector[0] = GYRO_SCALED_RAD(gyro[0]); //gyro x roll
  Gyro_Vector[1] = GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
  Gyro_Vector[2] = GYRO_SCALED_RAD(gyro[2]); //gyro z yaw
  
  Accel_Vector[0] = accel[0];
  Accel_Vector[1] = accel[1];
  Accel_Vector[2] = accel[2];
    
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
  
  //Accel_adjust();    //Remove centrifugal acceleration  
#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
#else // Use drift correction
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] = -G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2] = G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0] = G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0] = -G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1] = G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2] = 0;
#endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

//calculate quanternion from euler angles (adding by Tuan)
void GetQuat()
{         
  float x, y, z;
  // Converting vector angle to quaternion 
  x = roll; //rad
  y = pitch;
  z = yaw;      
  
  orientationChip.w = cos(x/2) * cos(y/2) * cos(z/2) + sin(x/2) * sin(y/2) * sin(z/2); 
  orientationChip.x = sin(x/2) * cos(y/2) * cos(z/2) - cos(x/2) * sin(y/2) * sin(z/2); 
  orientationChip.y = cos(x/2) * sin(y/2) * cos(z/2) + sin(x/2) * cos(y/2) * sin(z/2); 
  orientationChip.z = cos(x/2) * cos(y/2) * sin(z/2) - sin(x/2) * sin(y/2) * cos(z/2);    
}

//Get gravity in accleronmeter from quanternion (adding by Tuan)
void GetGravity(VectorFloat *v, Quaternion *q) {
  v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
  v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
  v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
}

int float2int(float flo) {
  int in;
  if (flo < 0) { // check for negative value
    in = (int)(flo - 0.5); // round to nearest negative value
  }
  else {
    in = (int)(flo + 0.5); // round to nearest positive value
  }
  return in;
}

float trapezoid(float new_data, float old_data, float old_out, float dT) {
  float out;
  out = old_out + dT * 0.5 * (old_data + new_data);
  return out;
}

VectorFloat lowpassfilter(VectorFloat *prevX, VectorFloat *newU, VectorFloat *prevU, float alpha) {
  VectorFloat newX;
  newX.x = ((1 - alpha) * prevX->x + alpha * prevU->x);
  newX.y = ((1 - alpha) * prevX->y + alpha * prevU->y);
  newX.z = ((1 - alpha) * prevX->z + alpha * prevU->z);
  return newX;
}

void resetVars() {
  sensor = sensorstart;                        // sensor starting position  
  oldOrientationChip = Quaternion(1, 0, 0, 0); // reset orientation
      
  //reset filter values
  new_a = VectorFloat(0.0, 0.0, 0.0);
  new_v = VectorFloat(0.0, 0.0, 0.0);
  new_s = VectorFloat(0.0, 0.0, 0.0);
  new_aLP = VectorFloat(0, 0, 0);
  new_vLP = VectorFloat(0, 0, 0);
  new_sLP = VectorFloat(0, 0, 0);
  
  new_afilt = aaRaw.getVectorFloat();
  new_vfilt = VectorFloat(0, 0, 0);
  new_sfilt = VectorFloat(0, 0, 0);

  aaSum = VectorFloat(0, 0, 0);
}

//calculate position from world(/real/raw) acceleronmeter
void Positioning()
{  
  aaWorldF.x = (float)aaWorld.x / 256;     //convert to G
  aaWorldF.y = (float)aaWorld.y / 256;
  aaWorldF.z = (float)aaWorld.z / 256;

  // updating acceleration
  old_a = new_a;                // storing old data
  new_a.x = aaWorldF.x * 981;   // convert to cm/s2
  new_a.y = aaWorldF.y * 981;
  new_a.z = aaWorldF.z * 981;    
        
  // filtering aceleration
  old_aLP = new_aLP;                // storing old drift offset
  old_afilt = new_afilt;            // storing old filtered data
  //new_aLP = lowpassfilter(&old_aLP, &new_a , &old_a, 0.05); // calculate new drift offset
  //new_afilt.x = new_a.x - new_aLP.x;// subtract drift from data
  //new_afilt.y = new_a.y - new_aLP.y;
  //new_afilt.z = new_a.z - new_aLP.z;
  
  new_afilt.x = abs(float2int(aaWorldF.x));
  new_afilt.y = abs(float2int(aaWorldF.y));
  new_afilt.z = abs(float2int(aaWorldF.z));

  // updating velocity
  old_v = new_v;                    // storing old data
  new_v.x = trapezoid(new_afilt.x, old_afilt.x, old_v.x, G_Dt); // reading out new data
  new_v.y = trapezoid(new_afilt.y, old_afilt.y, old_v.y, G_Dt);
  new_v.z = trapezoid(new_afilt.z, old_afilt.z, old_v.z, G_Dt);
  // filtering v
  old_vLP = new_vLP;                // storing old drift offset
  old_vfilt = new_vfilt;            // storing old filtered data
  //new_vLP = lowpassfilter(&old_vLP, &new_v , &old_v, 0.05); // calculate new drift offset
  new_vfilt.x = new_v.x - new_vLP.x;// subtract drift from data
  new_vfilt.y = new_v.y - new_vLP.y;
  new_vfilt.z = new_v.z - new_vLP.z;
    
  // updating displacement  
  old_s = new_s;                    // storing old data
  new_s.x = trapezoid(new_vfilt.x, old_vfilt.x, old_s.x, G_Dt); // reading out new data
  new_s.y = trapezoid(new_vfilt.y, old_vfilt.y, old_s.y, G_Dt);
  new_s.z = trapezoid(new_vfilt.z, old_vfilt.z, old_s.z, G_Dt);
  //filtering s
  old_sLP = new_sLP;                // storing old drift offset
  old_sfilt = new_sfilt;            // storing old filtered data
  //new_sLP = lowpassfilter(&old_sLP, &new_s , &old_s, 0.05); // calculate new drift offset
  new_sfilt.x = new_s.x - new_sLP.x;// subtract drift from data
  new_sfilt.y = new_s.y - new_sLP.y;
  new_sfilt.z = new_s.z - new_sLP.z;

  sensor = sensor.getAddition(new_s);      //translate the sensor in the reference frame
  tip = sensor.getAddition(sensor2tip);    //translate from sensor to tip position
}

