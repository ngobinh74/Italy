void check_buttons()
{
  //button process
  for (int i = 0; i < NUM_BUTTON; i++)
  {
    if (But_Press[i] == false)  continue;
    switch (i)
    {
    case 0: //first button
      state_board++;
      state_board = state_board % NUMBER_STATE;
      switch (state_board)
      {
      case 0: //first state        
        next_output_mode = OUTPUT_CUSTOM;
        state_board = 0;  //stop process
        break;
      case 1:   
        next_output_mode = START_PROCESS;
        state_board = 1;  //start process   
        break;
      }
      break;
    /*
    case 1: //second button
      break;
    case 2: //third button
      break;
    */
    }    
    But_Press[i] = false; //reset state of button
  }
}

void Scan_buttons()
{
  int i;
  //Debounce buttons   
  if (millis() != time_buttons_scan)  {
    for (i = 0; i < NUM_BUTTON; i++)
    {
      But[i] = digitalRead(BUT_PINS[i]);

      if (But[i] == But_state[i] && But_timeout[i] > 0){
        But_timeout[i]--;
      }
      if (But[i] != But_state[i]) {
        But_timeout[i]++;
      }
      // If the Input has shown the same value for long enough let's switch it
      if (But_timeout[i] >= debounce_count) {
        But_timeout[i] = 0;
        But_state[i] = But[i];
        if (But_state[i] == LOW) But_Press[i] = true;
      }
    }
    time_buttons_scan = millis();
  }

  check_buttons();
}

void board_init()
{
  pinMode(BATTERY_CTRL, OUTPUT);   //using on Tuan's Test Board
  digitalWrite(BATTERY_CTRL, HIGH);
  
#if BOARD_IMU_UNO  
  pinMode(LED1_BOARD, OUTPUT);
  digitalWrite(LED1_BOARD, LOW);
  
  pinMode(LED2_BOARD, OUTPUT);
  digitalWrite(LED2_BOARD, LOW);

  pinMode(LED3_BOARD, OUTPUT);
  digitalWrite(LED3_BOARD, LOW);
#endif

  state_board = 0;
  
  dislay_setup();
  RF_setup();  
}

#if BOARD_MEGA
  void dislay_setup(){
    display.begin();    // init done

    // you can change the contrast around to adapt the display
    // for the best viewing!  
    display.setContrast(LCD_CONSTRACT);
    display.display(); // show splashscreen
    delay(500);
    display.clearDisplay();   // clears the screen and buffer      

    //init for display text
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(0, 0);
    display.println("...Running...");
    display.display();
    delay(1000);
    display.clearDisplay();
    display.display();
  }

  void display_info()  
  {
    display.clearDisplay();
  
    display.setCursor(0, 0); display.print(" x    y   z  ");
  
    display.setCursor(0,  8); display.print((int)(accel[0])); 
    display.setCursor(26, 8); display.print((int)(accel[1])); 
    display.setCursor(50, 8); display.print((int)(accel[2]));   
  
    display.setCursor(0,  16); display.print((int)(magnetom[0])); 
    display.setCursor(26, 16); display.print((int)(magnetom[1])); 
    display.setCursor(50, 16); display.print((int)(magnetom[2])); 
    
    display.setCursor(0,  24); display.print((int)(gyro[0])); 
    display.setCursor(26, 24); display.print((int)(gyro[1])); 
    display.setCursor(50, 24); display.print((int)(gyro[2])); 
  
    display.setCursor(0, 32); display.print(" Y    P   R  ");
  
    display.setCursor(0,  40); display.print((int)(TO_DEG(yaw))); 
    display.setCursor(26, 40); display.print((int)(TO_DEG(pitch))); 
    display.setCursor(50, 40); display.print((int)(TO_DEG(roll)));   
  
    display.display();     
  }
#else
  void dislay_setup() {;}
  void display_info() {;}
#endif
