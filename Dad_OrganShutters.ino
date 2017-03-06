#include <EEPROM.h>

// digital outputs
const int In1_Motor1DirA=5;
const int In2_Motor1DirB=6;
const int In3_Motor2DirA=7;
const int In4_Motor2DirB=8;
const int En1_Motor1Enable=9;
const int En2_Motor2Enable=10;

const int LED_ERROR=LED_BUILTIN;

// analog inputs
const int Motor1_ActualPositionAnalogInputPin=A0;
const int Motor2_ActualPositionAnalogInputPin=A1;


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
    
    byte m_iActualPositionAnalogInputPin;
    
    // motor control particulars
    byte m_iMotor_DirPinA;
    byte m_iMotor_DirPinB;
    byte m_iMotor_EnablePin;
    boolean m_bMotor_HighSpeedWasUsed;

    int m_iCalMemoryStart;
    
    int m_iClosedLimit;
    int m_iOpenedLimit;

    TMotorDir m_eChosenMotorDir;
    byte m_iChosenSpeed;
  
    unsigned long m_iStartTime;

    int readSetPosition() {
    //  return analogRead(m_iSetPositionAnalogInputPin_TEST) / MaxADCValue);
      return PIND & 15;
    }

      
    int readSetPositionPct() {
    //  return round(100.0 * readSetPosition() / MaxADCValue);
      return round(100.0 * readSetPosition() / 15);
    }

      
     int readRawActualPosition() {
      return analogRead(m_iActualPositionAnalogInputPin);
    }
    
    
    int readActualPositionPct() {
      int iRawValue = readRawActualPosition();
      int iRangeSize = m_iOpenedLimit - m_iClosedLimit;
      return round(100 * (iRawValue - m_iClosedLimit) / iRangeSize);
    }


//    void logPositionNow() 
//    {
//      Serial.print("set value:  % ");
//      Serial.print(readSetPosition());
//  
//      Serial.print("  set pct value:  ");
//      Serial.print(readSetPositionPct());
//      Serial.print("  actual value:  ");
//      Serial.print(analogRead(m_iActualPositionAnalogInputPin));
//      Serial.print("  actual value pct:  ");
//      Serial.println(readActualPositionPct());
//      delay(100);
//    }
    

//    void logDecisionMaking() 
//    {
//      if (m_eChosenMotorDir!=Stop) {
//        if (m_bMotor_HighSpeedWasUsed) 
//          Serial.print("HS  ");
//
//        Serial.print("set pct value:  ");
//        Serial.print(readSetPositionPct());
//        Serial.print("  actual value:  ");
//        Serial.print(analogRead(m_iActualPositionAnalogInputPin));
//        Serial.print("  actual value pct:  ");
//        Serial.print(readActualPositionPct());
//        Serial.print("  speed:  ");
//        Serial.print(iSpeed);
//        
//        if (m_eChosenMotorDir==CloseShutter)
//          Serial.println("  CLOSING");
//        
//        if (m_eChosenMotorDir==OpenShutter)
//          Serial.println("  OPENING");
//      } else
//        Serial.println("STOPPED");        
//    }

    void loadLimitsFromEEPROM() {
      m_iClosedLimit = eeprom_read_word(m_iCalMemoryStart);
      m_iOpenedLimit = eeprom_read_word(m_iCalMemoryStart + sizeof(int));
    }


    void saveLimitsToEEPROM() {
      eeprom_write_word(m_iCalMemoryStart, m_iClosedLimit);
      eeprom_write_word(m_iCalMemoryStart + sizeof(int), m_iOpenedLimit);

      // clears any timeout
      m_iStartTime=millis();
    }


    void ForgetStartTime()
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
      pinMode(m_iActualPositionAnalogInputPin, INPUT);  
      
      pinMode(m_iMotor_DirPinA, OUTPUT);
      pinMode(m_iMotor_DirPinB, OUTPUT);  
      pinMode(m_iMotor_EnablePin, OUTPUT);    
    }

    
    void setMotorSpeed() {
      switch (m_eChosenMotorDir) {
        case OpenShutter:
          digitalWrite(m_iMotor_DirPinA, LOW);  // rotate forward
          digitalWrite(m_iMotor_DirPinB, HIGH);
          analogWrite(m_iMotor_EnablePin, m_iChosenSpeed);  // motor speed  
          break;
    
        case CloseShutter:
          digitalWrite(m_iMotor_DirPinA, HIGH);  // rotate reverse
          digitalWrite(m_iMotor_DirPinB, LOW);
          analogWrite(m_iMotor_EnablePin, m_iChosenSpeed);  // motor speed  
          break;
    
        case Stop:
          analogWrite(m_iMotor_EnablePin, 0);  // motor speed  
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
            m_iChosenSpeed = 100;
            else 
            m_iChosenSpeed = 190;
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
    
      bool bError=((millis() - m_iStartTime > icMotorTimeoutMS) || (readRawActualPosition()==MaxADCValue));
      if (bError) {
        iFlashTmp = millis() / 500;
    
        if (iFlashTmp % 2==1) 
          turnOnErrorLED();
          else
          turnOffErrorLED();

        m_iChosenSpeed = 0; 
      } else 
          turnOffErrorLED();

      if (m_iChosenSpeed==0)
        m_eChosenMotorDir=Stop;   
    }

    
  public:
    Motor(byte iActualPositionAnalogInputPin,
          // motor control particulars
          byte iMotor_DirPinA,
          byte iMotor_DirPinB,
          byte iMotor_EnablePin,
          int iCalMemoryStart) {
      m_iActualPositionAnalogInputPin = iActualPositionAnalogInputPin;
      
      // motor control particulars
      m_iMotor_DirPinA = iMotor_DirPinA;
      m_iMotor_DirPinB = iMotor_DirPinB;
      m_iMotor_EnablePin = iMotor_EnablePin;

      m_iCalMemoryStart = iCalMemoryStart;

      loadLimitsFromEEPROM();

      configurePort();
      
      ForgetStartTime();      
    }    


    void updateSpeed() 
    {
      decideSpeedAndDirection(UseSignalFromSyndyne);
      setMotorSpeed();
    }

    
    void setOpenedLimit() {
      m_iOpenedLimit = readRawActualPosition();
      saveLimitsToEEPROM();
    }


    void setClosedLimit() {
      m_iClosedLimit = readRawActualPosition();
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

             
Motor motor1(Motor1_ActualPositionAnalogInputPin,
             // motor control particulars
             In1_Motor1DirA,
             In2_Motor1DirB,
             En1_Motor1Enable,
             0x0000);

Motor motor2(Motor2_ActualPositionAnalogInputPin,
             // motor control particulars
             In3_Motor2DirA,
             In4_Motor2DirB,
             En2_Motor2Enable,
             0x0010);

void setup()
{
  Serial.begin(9600);

  pinMode(LED_ERROR, OUTPUT);
}


void handleCommands() {
  while (Serial.available() > 0) {
    // get incoming byte:
    char c = Serial.read();

    // discard unused char
    if (c==10)
      continue;
      
    if (c==13) { 
      if (sCommandBuffer.equalsIgnoreCase("Room1SetOpenedLimit"))
        motor1.setOpenedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("Room1SetClosedLimit"))
        motor1.setClosedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("Room1Test"))
        motor1.test();
        else
      if (sCommandBuffer.equalsIgnoreCase("Room2SetOpenedLimit"))
        motor2.setOpenedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("Room2SetClosedLimit"))
        motor2.setClosedLimit();
        else
      if (sCommandBuffer.equalsIgnoreCase("Room2Test"))
        motor2.test();
        else
        Serial.println("Unrecognized command!");
        
      sCommandBuffer="";
    } else
      sCommandBuffer += c;
  }  
}

void loop()
{ 
//  logPositions(); 
//  runTests();

  sCommandBuffer = "";
  
  while (1) {
    motor1.updateSpeed();
    motor2.updateSpeed();
    
    delay(1);
  }
}

