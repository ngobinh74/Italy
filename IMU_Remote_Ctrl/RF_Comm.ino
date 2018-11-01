//-----------------------------------------------------------//
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

  RemoteSensorXYZ.mode = 0;
  RemoteSensorXYZ.state = 0;
  RemoteSensorXYZ.X1 = 0;
  RemoteSensorXYZ.Y1 = 0;
  RemoteSensorXYZ.Z1 = 0; 
  RemoteSensorXYZ.X2 = 0;
  RemoteSensorXYZ.Y2 = 0;
  RemoteSensorXYZ.Z2 = 0; 
  RemoteSensorXYZ.G_Dt = 0;

  resetData();}

void RF_setup()
{
  //Start everything up
  radio.begin();
  radio.setChannel(100);
      
  //radio.setPALevel(RF24_PA_HIGH); // Set Power Amplifier (PA) level to one of four levels: 
  radio.setPALevel(RF24_PA_MAX);  //RF24_PA_HIGH
  
  radio.setDataRate(RF24_1MBPS);
  //radio.setDataRate(RF24_250KBPS);
  
  //radio.setAutoAck(false);  //disable ack
  
  //radio.enableDynamicAck();
  radio.setAutoAck(true);   //enable ack, transmiter
  radio.enableAckPayload(); //enable payload in ack packet
  radio.setRetries(5,5);    //This will improve reliability
  
  //using ack payload to resend to remote controller
  radio.openWritingPipe(pipeOut); 
  radio.openReadingPipe(1, pipeIn);  //Set up the two way communications with the named device
  //radio.openReadingPipe(1, pipeOut);  //using the same pipe for rf communication, implement broadcast
  
  //we start the radio communication, listening data from sensor node  
  radio.startListening();  
  //radio.stopListening();
}
  
/**************************************************/
void recvData()
{
	while (radio.available()) {
    ack.mode = output_mode;
    ack.state = output_state;    
		radio.writeAckPayload(pipeIn, &ack, sizeof(struct AckPayload));
   
		radio.read(&SensorXYZ, sizeof(SensorInfo));		
		bRecv = true;
	}	
}

bool RF_Trans(bool bACK)
{ 
  radio.stopListening(); 
  recv_state = false;
  if(bACK == true){  
    if (radio.write(&RemoteSensorXYZ, sizeof(struct SensorInfo))){
      if (radio.isAckPayloadAvailable()){
        radio.read(&RemoteAck, sizeof(struct AckPayload));  //read payload of ack packet      
        recv_state = true;
      }
      else recv_state = false;
    }
  } 
  else{  //no ACK
    //radio.write(&RemoteSensorXYZ, sizeof(struct SensorInfo));
    radio.write(&RemoteSensorXYZ, sizeof(struct SensorInfo),1);
  }
  radio.startListening();
  return recv_state;
}
