#include <EEPROM.h>

// digital outputs
const int In1_MotorDirA=9;
const int In2_MotorDirB=8;
const int En1_MotorEnable=10;

const int LED_ERROR=LED_BUILTIN;

// analog inputs
const int ActualShutterPositionAnalogInputPin=A0;


enum TMotorDir {CloseShutter, Stop, OpenShutter};

String sCommandBuffer;

class Motor
{
  private:
    const int icMotorTimeoutMS = 30000;
    const int icShutterSteps=32;  // this is now an arbitrary value
    const int dcDiffThresholdPct=100 / icShutterSteps;
    const int MaxADCValue = 1023;
    const int UseSignalFromSyndyne = -1;
    const int CalMemoryStart = 0x0000;
    
    bool m_bDebug;
    bool m_bEnabled;
    
    boolean m_bMotor_HighSpeedWasUsed;
   
    int m_iPedalClosedLimit;
    int m_iPedalOpenedLimit;

    int m_iShutterClosedLimit;
    int m_iShutterOpenedLimit;

    TMotorDir m_eChosenMotorDir;
    byte m_iChosenSpeed;
  
    unsigned long m_iMotorStartTime;

    int readRawPedalPosition() {
      const int icBitMask = 0x7f - 3;
      return (PIND ^ icBitMask) & icBitMask;
    }

      
    int readPedalPositionPct() {
      int iRawValue = readRawPedalPosition();

      if (iRawValue < m_iPedalClosedLimit)
        iRawValue = m_iPedalClosedLimit;
        
      if (iRawValue > m_iPedalOpenedLimit)
        iRawValue = m_iPedalOpenedLimit;
        
      int iRangeSize = m_iPedalOpenedLimit - m_iPedalClosedLimit;
      
      return round(100 * (iRawValue - m_iPedalClosedLimit) / iRangeSize);
    }


    int readRawActualShutterPosition() {
      return analogRead(ActualShutterPositionAnalogInputPin);
    }
    
    
    int readActualShutterPositionPct() {
      int iRawValue = readRawActualShutterPosition();

      if (iRawValue < m_iShutterClosedLimit)
        iRawValue = m_iShutterClosedLimit;
        
      if (iRawValue > m_iShutterOpenedLimit)
        iRawValue = m_iShutterOpenedLimit;
        
      int iRangeSize = m_iShutterOpenedLimit - m_iShutterClosedLimit;

// clutters other debugging stuff
//      if (m_bDebug) {
//        Serial.print("Raw=");
//        Serial.print(iRawValue);
//        Serial.print("  Range=");
//        Serial.print(iRangeSize);
//        Serial.print("  pct=");
//        Serial.println(round(100 * (iRawValue - m_iClosedLimit) / iRangeSize));
//      }
      
      return round(100 * (iRawValue - m_iShutterClosedLimit) / iRangeSize);
    }


    void loadLimitsFromEEPROM() {
      unsigned int iMemPos = CalMemoryStart;
      m_iShutterClosedLimit = eeprom_read_word((unsigned int*) iMemPos);
      iMemPos += sizeof(int);
      m_iShutterOpenedLimit = eeprom_read_word((unsigned int*) iMemPos);
    
      iMemPos += sizeof(int) ;
      m_iPedalClosedLimit = eeprom_read_word((unsigned int*) iMemPos);
      iMemPos += sizeof(int);
      m_iPedalOpenedLimit = eeprom_read_word((unsigned int*) iMemPos);
    }


    void saveLimitsToEEPROM() {
      unsigned int iMemPos = CalMemoryStart;
      eeprom_write_word((unsigned int*) iMemPos, m_iShutterClosedLimit);
      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iShutterOpenedLimit);
      
      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iPedalClosedLimit);
      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iPedalOpenedLimit);

      // clears any timeout
      m_iMotorStartTime=millis();
    }


    void forgetStartTime()
    {
      m_iMotorStartTime = 0;
      m_bMotor_HighSpeedWasUsed = false;
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
      pinMode(ActualShutterPositionAnalogInputPin, INPUT);  
      
      pinMode(In1_MotorDirA, OUTPUT);
      pinMode(In2_MotorDirB, OUTPUT);  
      pinMode(En1_MotorEnable, OUTPUT);    

      pinMode(0, INPUT_PULLUP);
      pinMode(1, INPUT_PULLUP);
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
          digitalWrite(In1_MotorDirA, LOW);  // rotate forward
          digitalWrite(In2_MotorDirB, HIGH);
          analogWrite(En1_MotorEnable, m_iChosenSpeed);  // motor speed  
          break;
    
        case CloseShutter:
          digitalWrite(In1_MotorDirA, HIGH);  // rotate reverse
          digitalWrite(In2_MotorDirB, LOW);
          analogWrite(En1_MotorEnable, m_iChosenSpeed);  // motor speed  
          break;
    
        case Stop:
          analogWrite(En1_MotorEnable, 0);  // motor speed  
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

      // big change?  use high speed.
      if (abs(iDiffPct) > 25) {
        m_bMotor_HighSpeedWasUsed = true;
        
        if (m_eChosenMotorDir==CloseShutter)
          m_iChosenSpeed = 130;
          else
          m_iChosenSpeed = 180;
      } else  
        // just a short distance away?  drive slowly (or slow down)
        if (abs(iDiffPct) > dcDiffThresholdPct) {
          if (m_bMotor_HighSpeedWasUsed) {
            if (m_eChosenMotorDir==CloseShutter)
              m_iChosenSpeed = 100;
              else
              m_iChosenSpeed = 120;
          } else {
              if (m_eChosenMotorDir==CloseShutter)
                m_iChosenSpeed = 170;
                else
                m_iChosenSpeed = 190;
            }  
        } else {
            // stop!
            m_bMotor_HighSpeedWasUsed = false;
            m_iChosenSpeed = 0;
        }

      // prepare for tracking motor start time
      if ((m_iChosenSpeed) && (!m_iMotorStartTime))
          m_iMotorStartTime=millis();
          else
          m_iMotorStartTime=0;
        
      // check for error conditions
      // if error, choose speed of 0 and flash the error LED

      int iFlashTmp;

      bool bTimeout = (m_iMotorStartTime>0) && (millis() - m_iMotorStartTime > icMotorTimeoutMS);
      bool bBadFeedback = (readRawActualShutterPosition()==MaxADCValue) || (readRawActualShutterPosition()==0);
      
      bool bError=bTimeout || bBadFeedback;
      if (bError) {
        iFlashTmp = millis() / 500;
    
        if (iFlashTmp % 2==1) 
          turnOnErrorLED();
          else
          turnOffErrorLED();

        m_iChosenSpeed = 0; 
      } else {
          turnOffErrorLED();
      }
      
      if (m_iChosenSpeed==0)
        m_eChosenMotorDir=Stop;   
    
        if (bTimeout) Serial.println("TIMEOUT");
        if (bBadFeedback) Serial.println("BAD FEEDBACK");
        
      if (m_bDebug) {
        Serial.print("Pedal:  ");
        Serial.print(readRawPedalPosition());
        Serial.print(" (%");
        Serial.print(iSetPct); 
        Serial.print(")   Shutter:  ");
        Serial.print(readRawActualShutterPosition());
        Serial.print(" (%");
        Serial.print(readActualShutterPositionPct());
        Serial.print(")   Diff %:  ");
        Serial.print(iDiffPct);
        Serial.print("   Selected speed:  ");
        Serial.print(m_iChosenSpeed);
        Serial.print("   Direction:  (0=cl, 1=st, 2=op) ");
        Serial.println(m_eChosenMotorDir);
      }

      if (!m_bEnabled) {
        m_iChosenSpeed=0;
        m_eChosenMotorDir=Stop;   
      }
    }

    
  public:
    Motor() {
      loadLimitsFromEEPROM();

      configurePort();
      
      forgetStartTime();      
      resetError();
      enable();
    }    


    void setDebug(bool bDebug)
    {
      m_bDebug=bDebug;
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
      m_bEnabled=false;
      Serial.println("DISABLED");
    }


    void enable()
    {
      m_bEnabled=true;
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

    
    void test() {
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
};

             
Motor motor;

void setup()
{
  Serial.begin(9600);

  pinMode(LED_ERROR, OUTPUT);

  sCommandBuffer = "";
}


void handleCommands() {
  while (Serial.available() > 0) {
    // get incoming byte:
    char c = Serial.read();

    // discard unused char
    if (c==10)
      continue;
      
    if (c==13) { 
      Serial.println("Executing command:  " + sCommandBuffer);
      
      if (sCommandBuffer.equalsIgnoreCase("SetPedalFullyOpenedPosition"))
        motor.setPedalOpenedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("SetPedalFullyClosedPosition"))
        motor.setPedalClosedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("SetShutterFullyOpenedPosition"))
        motor.setShutterOpenedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("SetShutterFullyClosedPosition"))
        motor.setShutterClosedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("Test"))
        motor.test();
        else
      if (sCommandBuffer.equalsIgnoreCase("ShowInfo"))
        motor.showInfo();
        else
      if (sCommandBuffer.equalsIgnoreCase("Disable"))
        motor.disable();
        else
      if (sCommandBuffer.equalsIgnoreCase("Enable"))
        motor.enable();
        else
      if (sCommandBuffer.equalsIgnoreCase("ResetError")) {
        motor.resetError();
        Serial.println("OK");
      } else
      if (sCommandBuffer.equalsIgnoreCase("DebugOn")) {
        motor.setDebug(true);
        Serial.println("Debug is now on");
      } else
      if (sCommandBuffer.equalsIgnoreCase("DebugOff")) {
        motor.setDebug(false);
        Serial.println("Debug is now off");
      } else
        Serial.println("Unrecognized command!");
        
      Serial.println();
      
      sCommandBuffer="";
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
  
  while (1) {
    motor.updateSpeed();

    handleCommands();
    
    // shutter speed management affected by this; see also here
    // too long-->overshoot
    delay(10);
  }
}

