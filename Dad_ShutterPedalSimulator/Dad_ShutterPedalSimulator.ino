void setup() {
  // put your setup code here, to run once:
  DDRD = B11111111; // set PORTD (digital 7~0) to outputs
}


void _set(int i) {
  PORTD=i<<2;
//     Serial.println(i<<2);
}



void loop() {
  // put your main code here, to run repeatedly:


  // nice speeds 50
  int iStepDelay=110;
  
  pinMode(LED_BUILTIN, OUTPUT);

  int i=0;

  while (true) {
    while (i<62) {
      _set(i);
      i++;
      
      digitalWrite(LED_BUILTIN, HIGH);
      
      delay(iStepDelay/2);
        
      digitalWrite(LED_BUILTIN, LOW);
  
      delay(iStepDelay/2);
    }


//  while (true) {
//    
//  }
    delay(10000);
    
    while (i>0) {
      _set(i);
      i=i-1;
      
      digitalWrite(LED_BUILTIN, HIGH);
      
      delay(iStepDelay/2);
        
      digitalWrite(LED_BUILTIN, LOW);
  
      delay(iStepDelay/2);
    }
  }
}
