///////////////////////////////////////////////////////////
void resetData()
{  
  SensorXYZ.mode = 0;
  SensorXYZ.state = 0;
  SensorXYZ.X1 = 0;
  SensorXYZ.Y1 = 0;
  SensorXYZ.Z1 = 0; 
  SensorXYZ.X2 = 0;
  SensorXYZ.Y2 = 0;
  SensorXYZ.Z2 = 0; 
  SensorXYZ.G_Dt = 0;

  CtrlSensorXYZ.mode = 0;
  CtrlSensorXYZ.state = 0;
  CtrlSensorXYZ.X1 = 0;
  CtrlSensorXYZ.Y1 = 0;
  CtrlSensorXYZ.Z1 = 0; 
  CtrlSensorXYZ.X2 = 0;
  CtrlSensorXYZ.Y2 = 0;
  CtrlSensorXYZ.Z2 = 0; 
  CtrlSensorXYZ.G_Dt = 0;  
}

void RF_setup()
{
  //Start everything up
  radio.begin();
  radio.setChannel(100);  
  radio.powerUp();
  
  //radio.setPALevel(RF24_PA_HIGH);
  radio.setPALevel(RF24_PA_MAX);
  
  radio.setDataRate(RF24_1MBPS);
  //radio.setDataRate(RF24_250KBPS);
  
  //radio.setAutoAck(false);	//disable ack
  
  //radio.enableDynamicAck();
  radio.setAutoAck(true);		//enable ack, transmiter
  radio.enableAckPayload();	//enable payload in ack packet
  radio.setRetries(5,5);    //This will improve reliability
  
  radio.openWritingPipe(pipeOut); 
  radio.openReadingPipe(1, pipeIn);  //Set up the two way communications with the named device
  //radio.openReadingPipe(1, pipeOut);  //using the same pipe for send and receiver, implement broadcast

  //radio.stopListening();
  radio.startListening();
}

bool RF_Trans(bool bACK)
{
  radio.stopListening();		
  recv_state = false;
  if(bACK == true){
    if (radio.write(&CtrlSensorXYZ, sizeof(struct SensorInfo))){
      if (radio.isAckPayloadAvailable()){
        radio.read(&CtrlAck, sizeof(struct AckPayload));	//read payload of ack packet        
        recv_state = true;        
      }
      else  recv_state = false;
    }    
  }
  else{   //no ACK
    radio.write(&CtrlSensorXYZ, sizeof(struct SensorInfo));
    //radio.write(&CtrlSensorXYZ, sizeof(struct SensorInfo),1);
  }       
  radio.startListening();  
  return recv_state;
}

void recvData()
{
  while (radio.available()) {  
    ack.mode = output_mode;
    ack.state = curr_calibration_sensor;   //only using when calibration command
     
    radio.writeAckPayload(pipeIn, &ack, sizeof(struct AckPayload));
    radio.read(&SensorXYZ, sizeof(SensorInfo));
    bRecv = true;
  } 
}
