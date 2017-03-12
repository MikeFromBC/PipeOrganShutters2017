#include <EEPROM.h>

// digital outputs
const int In1_MotorDirA=9;
const int In2_MotorDirB=8;
const int En1_MotorEnable=10;

const int LED_ERROR=LED_BUILTIN;

// analog inputs
const int ActualPositionAnalogInputPin=A0;


enum TMotorDir {CloseShutter, Stop, OpenShutter};

String sCommandBuffer;

class Motor
{
  private:
    const int icMotorTimeoutMS = 30000;
    const int icShutterSteps=16;
    const int dcDiffThresholdPct=100 / icShutterSteps;
    const int MaxADCValue = 1023;
    const int UseSignalFromSyndyne = -1;
    const int CalMemoryStart = 0x0000;
    
    bool m_bDebug;
    bool m_bEnabled;
    
    boolean m_bMotor_HighSpeedWasUsed;
   
    int m_iInClosedLimit;
    int m_iInOpenedLimit;

    unsigned int m_iClosedLimit;
    unsigned int m_iOpenedLimit;

    TMotorDir m_eChosenMotorDir;
    byte m_iChosenSpeed;
  
    unsigned long m_iStartTime;

    int readRawSetPosition() {
      return PIND >> 1;
    }

      
    int readSetPositionPct() {
      int iRawValue = readRawSetPosition();

      if (iRawValue < m_iInClosedLimit)
        iRawValue = m_iInClosedLimit;
        
      if (iRawValue > m_iInOpenedLimit)
        iRawValue = m_iInOpenedLimit;
        
      int iRangeSize = m_iInOpenedLimit - m_iInClosedLimit;
      
      return round(100 * (iRawValue - m_iInClosedLimit) / iRangeSize);
    }


    int readRawActualPosition() {
      return analogRead(ActualPositionAnalogInputPin);
    }
    
    
    int readActualPositionPct() {
      int iRawValue = readRawActualPosition();

      if (iRawValue < m_iClosedLimit)
        iRawValue = m_iClosedLimit;
        
      if (iRawValue > m_iOpenedLimit)
        iRawValue = m_iOpenedLimit;
        
      int iRangeSize = m_iOpenedLimit - m_iClosedLimit;

// clutters other debugging stuff
//      if (m_bDebug) {
//        Serial.print("Raw=");
//        Serial.print(iRawValue);
//        Serial.print("  Range=");
//        Serial.print(iRangeSize);
//        Serial.print("  pct=");
//        Serial.println(round(100 * (iRawValue - m_iClosedLimit) / iRangeSize));
//      }
      
      return round(100 * (iRawValue - m_iClosedLimit) / iRangeSize);
    }


    void loadLimitsFromEEPROM() {
      unsigned int iMemPos = CalMemoryStart;
      m_iClosedLimit = eeprom_read_word((unsigned int*) iMemPos);
      iMemPos += sizeof(int);
      m_iOpenedLimit = eeprom_read_word((unsigned int*) iMemPos);
    
      iMemPos += sizeof(int) ;
      m_iInClosedLimit = eeprom_read_word((unsigned int*) iMemPos);
      iMemPos += sizeof(int);
      m_iOpenedLimit = eeprom_read_word((unsigned int*) iMemPos);
    }


    void saveLimitsToEEPROM() {
      unsigned int iMemPos = CalMemoryStart;
      eeprom_write_word((unsigned int*) iMemPos, m_iClosedLimit);
      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iOpenedLimit);
      
      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iInClosedLimit);
      iMemPos += sizeof(int);
      eeprom_write_word((unsigned int*) iMemPos, m_iInOpenedLimit);

      // clears any timeout
      m_iStartTime=millis();
    }


    void forgetStartTime()
    {
      m_iStartTime = 0;
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
      pinMode(ActualPositionAnalogInputPin, INPUT);  
      
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
        iSetPct = readSetPositionPct();
        else
        iSetPct = iForcedSetPct;
      
      int iDiffPct = iSetPct - readActualPositionPct();

      // big change?  use high speed.
      if (abs(iDiffPct) > 25) {
        m_bMotor_HighSpeedWasUsed = true;
        m_iChosenSpeed = 180;
      } else  
        // just a short distance away?  drive slowly (or slow down)
        if (abs(iDiffPct) > dcDiffThresholdPct) {
          if (m_bMotor_HighSpeedWasUsed) 
            m_iChosenSpeed = 150;
            else 
            m_iChosenSpeed = 220;
        } else {
            // stop!
            m_bMotor_HighSpeedWasUsed = false;
            m_iChosenSpeed = 0;
        }

      // prepare for tracking motor start time
      if ((m_iChosenSpeed) && (!m_iStartTime))
          m_iStartTime=millis();
          else
          m_iStartTime=0;
        
      // decide direction
      m_eChosenMotorDir = iDiffPct > 0 ? OpenShutter : CloseShutter;

      // check for error conditions
      // if error, choose speed of 0 and flash the error LED

      int iFlashTmp;

      bool bTimeout = (m_iStartTime>0) && (millis() - m_iStartTime > icMotorTimeoutMS);
      bool bBadFeedback = (readRawActualPosition()==MaxADCValue) || (readRawActualPosition()==0);
      
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
        Serial.print("Set %:  ");
        Serial.print(iSetPct); 
        Serial.print("   Actual:  ");
        Serial.print(readRawActualPosition());
        Serial.print("   Actual %:  ");
        Serial.print(readActualPositionPct());
        Serial.print("   Diff %:  ");
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
      
      Serial.print("Raw: IN 'Open' limit is ");
      Serial.println(m_iInOpenedLimit);
      Serial.print("Raw: IN 'Close' limit is ");
      Serial.println(m_iInClosedLimit);

      Serial.print("Raw: 'Open' limit is ");
      Serial.println(m_iOpenedLimit);
      Serial.print("Raw: 'Close' limit is ");
      Serial.println(m_iClosedLimit);

      Serial.print("Raw: Current set position is ");
      Serial.println(readRawSetPosition());
      Serial.print("Raw: Current position is ");
      Serial.println(readRawActualPosition());

      Serial.print("Pct: Current set position is ");
      Serial.println(readSetPositionPct());
      Serial.print("Pct: Current position is ");
      Serial.println(readActualPositionPct());
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
      m_iStartTime = 0; 
    }
       
    void setOpenedLimit() {
      m_iOpenedLimit = readRawActualPosition();
      Serial.print("OK; new 'open' limit is ");
      Serial.println(m_iOpenedLimit);
      saveLimitsToEEPROM();
    }


    void setClosedLimit() {
      m_iClosedLimit = readRawActualPosition();
      Serial.print("OK; new 'close' limit is ");
      Serial.println(m_iClosedLimit);
      saveLimitsToEEPROM();
    }


    void setInOpenedLimit() {
      m_iInOpenedLimit = readRawSetPosition();
      Serial.print("OK; new IN 'open' limit is ");
      Serial.println(m_iInOpenedLimit);
      saveLimitsToEEPROM();
    }


    void setInClosedLimit() {
      m_iInClosedLimit = readRawSetPosition();
      Serial.print("OK; new IN 'close' limit is ");
      Serial.println(m_iInClosedLimit);
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
      if (sCommandBuffer.equalsIgnoreCase("SetPedalFullyOpenedPosition"))
        motor.setInOpenedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("SetPedalFullyClosedPosition"))
        motor.setInClosedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("SetShutterFullyOpenedPosition"))
        motor.setOpenedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("SetShutterFullyClosedPosition"))
        motor.setClosedLimit();
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
  while (1) {
    motor.updateSpeed();

    handleCommands();

    // too long-->overshoot
    delay(50);
  }
}

