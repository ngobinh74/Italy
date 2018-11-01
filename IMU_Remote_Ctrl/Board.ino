#if ((BOARD_ARDUINO == true) || (BOARD_ARDUINO_DIY == true))
  void check_buttons() {;}
#else
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
        bplotting = true;        
        digitalWrite(LED2_BOARD, LOW);
        digitalWrite(LED2_BOARD, HIGH);
        break;
      case 1:              
        bplotting = false;        
        digitalWrite(LED2_BOARD, HIGH);
        digitalWrite(LED2_BOARD, LOW);
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
#endif

#if ((BOARD_ARDUINO == true) || (BOARD_ARDUINO_DIY == true))
  void Scan_buttons() {;}
#else
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
#endif

void board_init()
{
#if ((BOARD_ARDUINO == true) || (BOARD_ARDUINO_DIY == true))
  RF_setup();
#else
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
  led1_state = false;
  
  dislay_setup();
  RF_setup();
#endif    
}

void blink_led()
{
  if(led1_state == true)  led1_state  = false;
  else                    led1_state = true;
#if BOARD_IMU_UNO  
  if(led1_state == true)  digitalWrite(LED1_BOARD, HIGH);
  else                    digitalWrite(LED1_BOARD, LOW);
#endif
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
    if((output_mode == OUTPUT_UNKNOWN) || (output_mode == OUTPUT_NONE)) return;
    
    display.clearDisplay();   
      
    switch(output_mode)
    {
    case OUTPUT_CUSTOM:    
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
  
      display.setCursor(0,  40); display.print((int)yaw); 
      display.setCursor(26, 40); display.print((int)pitch); 
      display.setCursor(50, 40); display.print((int)roll);   
      break;
      
    case OUTPUT_CALIBRATE_SENSORS:
      break;

    case OUTPUT_ANGLES:
      display_compass();
      break;

    case OUTPUT_SENSORS_RAW:
      break;
    case OUTPUT_SENSORS_CALIB:
      break;
    }
  
    display.display();     
  }

  //draw compass function
  void display_compass()
  {
    /*
    float ang = TO_RAD(yaw);
    float declinationAngle = 0.22;  //<--Change 0.22 with yours. If you can't find your declination juct delete those lines ;)
    ang += declinationAngle;
    // Correct for when signs are reversed.
    if(ang < 0)   ang += 2*PI;
    // Check for wrap due to addition of declination.
    if(ang > 2*PI)ang -= 2*PI;
    // Convert radians to degrees for readability.
    float headingDegrees = ang * 180/M_PI; 
    //Convert float to int
    int yaw_angle=(int)headingDegrees;
    */  
    int yaw_angle = (int)yaw;
    DrawRow(yaw_angle); //call DrawRow function
    display.drawBitmap(40, 24,  arrow_bmp, 5, 5, 1);
    display.drawLine(42, 0, 42, 24, BLACK);
    display.setCursor(32,34);  
    display.print(yaw);
    display.setTextSize(1);
    display.print("o");  
  }  

  //Draw rows and print letters to display - 1st Display Mode
  void DrawRow(int angle) 
  {
    display.drawLine(0, 0, 84, 0, BLACK);
    display.drawLine(0, 1, 84, 1, BLACK);

    display.drawLine(0, 22, 91, 22, BLACK);
    display.drawLine(0, 23, 95, 23, BLACK);
  
    display.setTextSize(2);
    display.setTextColor(BLACK);  
  
    int start = 42 - angle / 3 ;
    if (start > 120) start += -120 ;

    int x = 0 ;
    int y = 18 ;
    for (int i=0; i<4; i++) {
      x = start + (i*30) -1;
      if (x>119) x += -120;
      display.drawPixel(x+1, y-2, 1);
      display.drawPixel(x, y, 1);
      display.drawPixel(x+1, y, 1);
      display.drawPixel(x+2, y, 1);
      display.drawPixel(x, y-1, 1);
      display.drawPixel(x+1, y-1, 1);
      display.drawPixel(x+2, y-1, 1);
      display.setCursor((x-4),(y-16)); 
      display.write(Leters[i]);
    }
    for (int i=0; i<24; i++) {
      x = start + (i*5) -1;
      if (x>119) x += -120;
      display.drawPixel(x+1, y+1, 1);
      display.drawPixel(x, y+2, 1);
      display.drawPixel(x+1, y+2, 1);
      display.drawPixel(x+2, y+2, 1);
      display.drawPixel(x, y+3, 1);
      display.drawPixel(x+1, y+3, 1);
      display.drawPixel(x+2, y+3, 1);
    };
   
    for (int i=0; i<8; i++) {
      x = start + (i*15)-1;
      if (x>119) x += -120;
      display.drawPixel(x+1, y-1, 1);

      display.drawPixel(x, y, 1);
      display.drawPixel(x+1, y, 1);
      display.drawPixel(x+2, y, 1);
      display.drawPixel(x, y+1, 1);
      display.drawPixel(x+1, y+1, 1);
      display.drawPixel(x+2, y+1, 1);
    }
  }
#else
  void dislay_setup() {;}
  void display_info() {;}
#endif
