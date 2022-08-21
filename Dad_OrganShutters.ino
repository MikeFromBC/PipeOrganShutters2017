#include <EEPROM.h>

const int LED_ERROR = LED_BUILTIN;

enum TMotorDir {CloseShutter, Stop, OpenShutter};

String sCommandBuffer;

class Motor
{
  private:
    const int icMotorTimeoutMS = 30000;
    const int dcDiffThresholdPct = 3;
    const int MaxADCValue = 1023;
    const int UseSignalFromSyndyne = -1;

    bool m_bDebug;
    bool m_bEnabled;

    int m_iCalMemoryStart;
    int m_iMotorEnablePin;
    int m_iMotorDirAPin;
    int m_iMotorDirBPin;
    int m_iActualShutterPositionAnalogInputPin;

    int m_iPedalClosedLimit;
    int m_iPedalOpenedLimit;

    int m_iShutterClosedLimit;
    int m_iShutterOpenedLimit;

    int m_iPrevDiffPct;
    
    TMotorDir m_eChosenMotorDir;
    byte m_iChosenSpeed;

    unsigned long m_iMotorStartTime;

    int readRawPedalPosition() {
      // bottom two bits conflict with use of serial port so mask them off
      const int icBitMask = B11111100;

      //Serial.println(((PIND ^ icBitMask) & icBitMask) >> 2);
      
      // invert bits & mask off serial port bits
      return ((PIND ^ icBitMask) & icBitMask) >> 2;
    }


    int readPedalPositionPct() {
      int iRawValue = readRawPedalPosition();

      if (iRawValue < m_iPedalClosedLimit)
        iRawValue = m_iPedalClosedLimit;

      if (iRawValue > m_iPedalOpenedLimit)
        iRawValue = m_iPedalOpenedLimit;

      int iRangeSize = m_iPedalOpenedLimit - m_iPedalClosedLimit;

      int iPct = round(100 * (iRawValue - m_iPedalClosedLimit) / iRangeSize);

      if (iPct < 0) iPct = 0;
      if (iPct > 100) iPct = 100;
      
      return iPct;
    }


    int readRawActualShutterPosition() {
      // about 100 us
      return analogRead(m_iActualShutterPositionAnalogInputPin);
    }


    int readActualShutterPositionPct() {
      int iRawValue = readRawActualShutterPosition();

      if (iRawValue < m_iShutterClosedLimit)
        iRawValue = m_iShutterClosedLimit;

      if (iRawValue > m_iShutterOpenedLimit)
        iRawValue = m_iShutterOpenedLimit;

      int iRangeSize = m_iShutterOpenedLimit - m_iShutterClosedLimit;

      // clutters other debugging stuff
//            if (m_bDebug) {
//              Serial.println();
//              Serial.print("readActualShutterPositionPct  Raw=");
//              Serial.print(iRawValue);
//              Serial.print("  Limit=");
//              Serial.print(m_iShutterClosedLimit);
//              Serial.print(" - ");
//              Serial.print(m_iShutterOpenedLimit);
//              Serial.print("  Range=");
//              Serial.print(iRangeSize);
//              Serial.print("  pct=");
//              Serial.println(round(100 * (iRawValue - m_iShutterClosedLimit) / iRangeSize));
//            }

      int iPct = round(100 * (iRawValue - m_iShutterClosedLimit) / iRangeSize);
      
      if (iPct < 0) iPct = 0;
      if (iPct > 100) iPct = 100;
      
      return iPct;
    }


    void loadLimitsFromEEPROM() {
      unsigned int iMemPos = m_iCalMemoryStart;
      m_iShutterClosedLimit = eeprom_read_word((unsigned int*) iMemPos);
      iMemPos += sizeof(int);
      m_iShutterOpenedLimit = eeprom_read_word((unsigned int*) iMemPos);

      iMemPos += sizeof(int) ;
      m_iPedalClosedLimit = eeprom_read_word((unsigned int*) iMemPos);
      iMemPos += sizeof(int);
      m_iPedalOpenedLimit = eeprom_read_word((unsigned int*) iMemPos);
    }


    void saveLimitsToEEPROM() {
      unsigned int iMemPos = m_iCalMemoryStart;
      eeprom_write_word((unsigned int*) iMemPos, m_iShutterClosedLimit);
      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iShutterOpenedLimit);

      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iPedalClosedLimit);
      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iPedalOpenedLimit);

      // clears any timeout
      m_iMotorStartTime = millis();
    }


    void forgetStartTime()
    {
      m_iMotorStartTime = 0;
    }

    void turnOffErrorLED()
    {
      digitalWrite(LED_ERROR, LOW);
    }


    void turnOnErrorLED()
    {
      digitalWrite(LED_ERROR, HIGH);
    }


    void configurePort()
    {
      pinMode(m_iActualShutterPositionAnalogInputPin, INPUT);

      pinMode(m_iMotorDirAPin, OUTPUT);
      pinMode(m_iMotorDirBPin, OUTPUT);
      pinMode(m_iMotorEnablePin, OUTPUT);

      // prepare input port to read from ULN2003 buffer
      // leave these alone; used by serial port:  pinMode(0, INPUT_PULLUP);
      // leave these alone; used by serial port:  pinMode(1, INPUT_PULLUP);
      pinMode(2, INPUT_PULLUP);
      pinMode(3, INPUT_PULLUP);
      pinMode(4, INPUT_PULLUP);
      pinMode(5, INPUT_PULLUP);
      pinMode(6, INPUT_PULLUP);
      pinMode(7, INPUT_PULLUP);
    }


    void setMotorSpeed() {
      switch (m_eChosenMotorDir) {
        case OpenShutter:
          digitalWrite(m_iMotorDirAPin, LOW);  // rotate forward
          digitalWrite(m_iMotorDirBPin, HIGH);
          analogWrite(m_iMotorEnablePin, m_iChosenSpeed);  // motor speed
          break;

        case CloseShutter:
          digitalWrite(m_iMotorDirAPin, HIGH);  // rotate reverse
          digitalWrite(m_iMotorDirBPin, LOW);
          analogWrite(m_iMotorEnablePin, m_iChosenSpeed);  // motor speed
          break;

        case Stop:
          analogWrite(m_iMotorEnablePin, 0);  // motor speed
          break;
      }
    }


    void decideSpeedAndDirection(int iForcedSetPct) {

      // decide speed!

      int iSetPct;

      if (iForcedSetPct == UseSignalFromSyndyne)
      iSetPct = readPedalPositionPct();
      else
        iSetPct = iForcedSetPct;

      int iDiffPct = iSetPct - readActualShutterPositionPct();

      // decide direction
      m_eChosenMotorDir = iDiffPct > 0 ? OpenShutter : CloseShutter;

      // shutter speed management affected by this; see also here

      if ((iDiffPct==0) || ((m_iPrevDiffPct<0) != (iDiffPct<0)))
        // stop!
        m_iChosenSpeed = 0;
        else      
        if (abs(iDiffPct) > dcDiffThresholdPct) {
          switch (m_eChosenMotorDir) {
            case OpenShutter:
              // opening requires more effort than closing (see graphs). 
              m_iChosenSpeed = 175;
              break;

            case CloseShutter:
              // closing requires less effort than opening (see graphs).  threshold is 100-140 or so; varies with position.
              m_iChosenSpeed = 165;
          }
        } else 
          // stop!
          m_iChosenSpeed = 0;

     m_iPrevDiffPct = iDiffPct;

     // prepare for tracking motor start time
      if ((m_iChosenSpeed) && (!m_iMotorStartTime))
        m_iMotorStartTime = millis();
      else
        m_iMotorStartTime = 0;

      // check for error conditions
      // if error, choose speed of 0 and flash the error LED

      int iFlashTmp;

      bool bTimeout = (m_iMotorStartTime > 0) && (millis() - m_iMotorStartTime > icMotorTimeoutMS);
      bool bBadFeedback = (readRawActualShutterPosition() == MaxADCValue) || (readRawActualShutterPosition() == 0);

      bool bError = bTimeout || bBadFeedback;
      if (bError) {
        iFlashTmp = millis() / 500;

        if (iFlashTmp % 2 == 1)
          turnOnErrorLED();
        else
          turnOffErrorLED();

        m_iChosenSpeed = 0;
      } else {
        turnOffErrorLED();
      }

      if (m_iChosenSpeed == 0)
        m_eChosenMotorDir = Stop;

      if (bTimeout) Serial.println("TIMEOUT");
      if (bBadFeedback) Serial.println("BAD FEEDBACK");

      if ((m_bDebug) && (m_iCalMemoryStart==0x0000)) {
        Serial.print(millis());
        Serial.print(" ms  Pedal:  ");
        Serial.print(readRawPedalPosition());
        Serial.print(" (%");
        Serial.print(iSetPct);
        Serial.print(")   Shutter:  ");
        Serial.print(readRawActualShutterPosition());
        Serial.print(" (%");
        Serial.print(readActualShutterPositionPct());
        Serial.print(")   Diff %:  ");
        Serial.print(iDiffPct);
        Serial.print("   Chosen:  ");
        switch (m_eChosenMotorDir) {
          OpenShutter:
            Serial.print("Open ");
            Serial.println(m_iChosenSpeed);
            break;
          
          CloseShutter:
            Serial.print("Close ");
            Serial.println(m_iChosenSpeed);
            break;
          
          Stop:
            Serial.println("STOPPED");
            break;         
        }
      }

      if (!m_bEnabled) {
        m_iChosenSpeed = 0;
        m_eChosenMotorDir = Stop;
      }
    }


  public:
    Motor(int iCalMemoryStart,
          int iMotorEnablePin,
          int iMotorDirAPin,
          int iMotorDirBPin,
          int iActualShutterPositionAnalogInputPin)
    {
      m_iCalMemoryStart = iCalMemoryStart;
      m_iMotorEnablePin = iMotorEnablePin;
      m_iMotorDirAPin = iMotorDirAPin;
      m_iMotorDirBPin = iMotorDirBPin;
      m_iActualShutterPositionAnalogInputPin = iActualShutterPositionAnalogInputPin;

      m_bDebug = false;

      loadLimitsFromEEPROM();

      configurePort();

      forgetStartTime();
      resetError();
      enable();
    }


    void setDebug(bool bDebug)
    {
      m_bDebug = bDebug;
    }

    void updateSpeed()
    {
      decideSpeedAndDirection(UseSignalFromSyndyne);
      setMotorSpeed();
    }


    void showInfo()
    {
      Serial.println("ROOM INFO");
      Serial.println("=========");

      if (m_bEnabled)
        Serial.println("Status: ENABLED");
      else
        Serial.println("Status: DISABLED");

      Serial.print("Raw: Pedal 'Open' limit is ");
      Serial.println(m_iPedalOpenedLimit);
      Serial.print("Raw: Pedal 'Close' limit is ");
      Serial.println(m_iPedalClosedLimit);
      Serial.print("Raw: Pedal position is ");
      Serial.println(readRawPedalPosition());
      Serial.print("Pct: Pedal position % is ");
      Serial.println(readPedalPositionPct());

      Serial.print("Raw: Shutter 'Open' limit is ");
      Serial.println(m_iShutterOpenedLimit);
      Serial.print("Raw: Shutter 'Close' limit is ");
      Serial.println(m_iShutterClosedLimit);
      Serial.print("Raw: Shutter position is ");
      Serial.println(readRawActualShutterPosition());
      Serial.print("Pct: Shutter position % is ");
      Serial.println(readActualShutterPositionPct());
    }


    void disable()
    {
      m_bEnabled = false;
      Serial.println("DISABLED");
    }


    void enable()
    {
      m_bEnabled = true;
      Serial.println("ENABLED");
    }


    void resetError() {
      m_iMotorStartTime = 0;
    }

    void setShutterOpenedLimit() {
      m_iShutterOpenedLimit = readRawActualShutterPosition();
      Serial.print("OK; new shutter 'open' limit is ");
      Serial.println(m_iShutterOpenedLimit);
      saveLimitsToEEPROM();
    }


    void setShutterClosedLimit() {
      m_iShutterClosedLimit = readRawActualShutterPosition();
      Serial.print("OK; new shutter 'close' limit is ");
      Serial.println(m_iShutterClosedLimit);
      saveLimitsToEEPROM();
    }


    void setPedalOpenedLimit() {
      m_iPedalOpenedLimit = readRawPedalPosition();
      Serial.print("OK; new pedal 'open' limit is ");
      Serial.println(m_iPedalOpenedLimit);
      saveLimitsToEEPROM();
    }


    void setPedalClosedLimit() {
      m_iPedalClosedLimit = readRawPedalPosition();
      Serial.print("OK; new pedal 'close' limit is ");
      Serial.println(m_iPedalClosedLimit);
      saveLimitsToEEPROM();
    }


    bool enterWasPressed() {
      char c = 0;
      while (Serial.available() > 0) {
        c = Serial.read();
        if (c == 13)
          return true;
      }

      return false;
    }


    void testOpenClose() {
      Serial.println("Driving to fully closed position.  Motor should finish quickly.");
      Serial.println("Press Enter when you're ready to continue.");
      do {
        decideSpeedAndDirection(0);
        setMotorSpeed();
      } while (!enterWasPressed());

      Serial.println("Driving to fully opened position.  Motor should finish quickly.");
      Serial.println("Press Enter when you're ready to continue.");
      do {
        decideSpeedAndDirection(100);
        setMotorSpeed();
      } while (!enterWasPressed());

      Serial.println("Driving to fully closed position.  Motor should finish quickly.");
      Serial.println("Press Enter when you're ready to continue.");
      do {
        decideSpeedAndDirection(0);
        setMotorSpeed();
      } while (!enterWasPressed());

      Serial.println("Returning to normal operation.");
    }


    void testDrive(int iForcedSetPos) {
      unsigned long iStartMS = millis();
      do {
        decideSpeedAndDirection(iForcedSetPos);
        setMotorSpeed();
      } while ((m_eChosenMotorDir!=Stop) || (millis() - iStartMS < 500));
      
      Serial.println("DONE!");
    }

    
    void testOverallPerformance(TMotorDir eMotorDir) {
      int iStartPct;
      int iIncPct;
      int iLimitPct;

      switch (eMotorDir) {
        case CloseShutter:
          iStartPct=100;
          iIncPct=-5;
          iLimitPct=0;
          break;
        
        case OpenShutter:
          iStartPct=0;
          iIncPct=5;
          iLimitPct=100;
          break;
      }

      while (iStartPct != iLimitPct) {
        Serial.print("Driving to ");
        Serial.println(iStartPct);
        testDrive(iStartPct);
        iStartPct += iIncPct;
      }
      
      Serial.println("Returning to normal operation.");
    }

    
    void testStaticFriction(TMotorDir eMotorDir) {
      int iStartPct;
      int iIncPct;
      int iLimitPct;

      switch (eMotorDir) {
        case CloseShutter:
          iStartPct=100;
          iIncPct=-5;
          iLimitPct=5;  // don't test past end
          break;
        
        case OpenShutter:
          iStartPct=0;
          iIncPct=5;
          iLimitPct=95;  // don't test past end
          break;
      }

      while (iStartPct != iLimitPct) {
        Serial.print("Driving to ");
        Serial.println(iStartPct);
        testDrive(iStartPct);
  
        m_eChosenMotorDir = eMotorDir;        

        int iStartPos = readActualShutterPositionPct();

        do {
          m_iChosenSpeed += 5;
          setMotorSpeed();
//          Serial.print("    Trying ");
//          Serial.println(m_iChosenSpeed);
          delay(100);

          if (m_iChosenSpeed>245) {
            Serial.println("ERROR!");
            break;
          }
            
        } while (abs(iStartPos - readActualShutterPositionPct()) < 5);

        Serial.print("    Threshold ");
        Serial.println(m_iChosenSpeed);
        
        iStartPct += iIncPct;
      }
      
      Serial.println("Returning to normal operation.");
    }
};


Motor* currentMotor;

void setup()
{
  Serial.begin(9600);

  pinMode(LED_ERROR, OUTPUT);

  sCommandBuffer = "";
}


void handleCommands(Motor *motor1) {
  while (Serial.available() > 0) {
    // get incoming byte:
    char c = Serial.read();

    // discard unused char
    if (c == 10)
      continue;

    if (c == 13) {
      Serial.println("Executing command:  " + sCommandBuffer);

      if (sCommandBuffer.equalsIgnoreCase("Motor1")) {
        currentMotor = motor1;
        Serial.println("Motor 1 selected");
//      } else if (sCommandBuffer.equalsIgnoreCase("Motor2")) {
//        currentMotor = motor2;
//        Serial.println("Motor 2 selected");
      } else if (sCommandBuffer.equalsIgnoreCase("SetPedalFullyOpenedPosition"))
        currentMotor->setPedalOpenedLimit();
      else if (sCommandBuffer.equalsIgnoreCase("SetPedalFullyClosedPosition"))
        currentMotor->setPedalClosedLimit();
      else if (sCommandBuffer.equalsIgnoreCase("SetShutterFullyOpenedPosition"))
        currentMotor->setShutterOpenedLimit();
      else if (sCommandBuffer.equalsIgnoreCase("SetShutterFullyClosedPosition"))
        currentMotor->setShutterClosedLimit();
      else if (sCommandBuffer.equalsIgnoreCase("TestOpenClose"))
        currentMotor->testOpenClose();
      else if (sCommandBuffer.equalsIgnoreCase("TestOpen"))
        currentMotor->testOverallPerformance(OpenShutter);
      else if (sCommandBuffer.equalsIgnoreCase("TestClose"))
        currentMotor->testOverallPerformance(CloseShutter);
      else if (sCommandBuffer.equalsIgnoreCase("TestOpenFric"))
        currentMotor->testStaticFriction(OpenShutter);
      else if (sCommandBuffer.equalsIgnoreCase("TestCloseFric"))
        currentMotor->testStaticFriction(CloseShutter);        
      else if (sCommandBuffer.equalsIgnoreCase("ShowInfo"))
        currentMotor->showInfo();
      else if (sCommandBuffer.equalsIgnoreCase("Disable"))
        currentMotor->disable();
      else if (sCommandBuffer.equalsIgnoreCase("Enable"))
        currentMotor->enable();
      else if (sCommandBuffer.equalsIgnoreCase("ResetError")) {
        currentMotor->resetError();
        Serial.println("OK");
      } else if (sCommandBuffer.equalsIgnoreCase("DebugOn")) {
        currentMotor->setDebug(true);
        Serial.println("Debug is now on");
      } else if (sCommandBuffer.equalsIgnoreCase("DebugOff")) {
        currentMotor->setDebug(false);
        Serial.println("Debug is now off");
      } else
        Serial.println("Unrecognized command!");

      Serial.println();

      sCommandBuffer = "";
    } else
      sCommandBuffer += c;
  }
}

void loop()
{
  //  while (1) {
  //    delay(200);
  //    digitalWrite(LED_ERROR, LOW);
  //    delay(200);
  //    digitalWrite(LED_ERROR, HIGH);
  //    Serial.println("ok");
  //  }

  Motor* motor1 = &Motor(0x0000, 9, 8, 10, 14 /* A0 */);
  //Motor* motor2 = &Motor(0x0010, 11, 12, 13, 15 /* A1 */);

  currentMotor = motor1;

  while (1) {
    motor1->updateSpeed();
    //motor2->updateSpeed();

    handleCommands(motor1);

    // shutter speed management affected by this; see also here
    // too long-->overshoot
    //delay(10);
  }
}
