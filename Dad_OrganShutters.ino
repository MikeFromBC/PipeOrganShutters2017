#include <LiquidCrystal.h>

const int In1_Motor1DirA=5;
const int In2_Motor1DirB=6;
const int In3_Motor2DirA=7;
const int In4_Motor2DirB=8;
const int En1_Motor1Enable=9;
const int En2_Motor2Enable=10;
const int Motor1_PositionAnalogInputPin=11;
const int Motor2_PositionAnalogInputPin=12;

const int Motor1=1;
const int Motor2=2;

const int icSeekSpeed=128;

struct TMotor {
  byte iMotorNum;
  byte iPositionAnalogInputPin;
  
  // motor control particulars
  byte iMotor_DirPinA;
  byte iMotor_DirPinB;
  byte iMotor_EnablePin;

  int iMemory_ClosedPosition;
  int iMemory_OpenPosition;
};

enum TMotorDir {OpenShutter, Stop, CloseShutter};

TMotor rMotor1, rMotor2;

LiquidCrystal lcd(4, 10, 12, 13, 2, 3);

void configurePort(TMotor* prMotor) {
  pinMode(prMotor->iPositionAnalogInputPin, INPUT);  
  
  pinMode(prMotor->iMotor_DirPinA, OUTPUT);
  pinMode(prMotor->iMotor_DirPinB, OUTPUT);  
  pinMode(prMotor->iMotor_EnablePin, OUTPUT);    
}


void stopMotor(TMotor* prMotor) {
  analogWrite(prMotor->iMotor_EnablePin, 0);  // motor speed  
}


void setMotorSpeed(TMotor* prMotor, TMotorDir eMotorDir, byte iSpeed) {
  stopMotor(prMotor);
  
  switch (eMotorDir) {
    OpenShutter:
      digitalWrite(prMotor->iMotor_DirPinA, LOW);  // rotate forward
      digitalWrite(prMotor->iMotor_DirPinB, HIGH);
      analogWrite(prMotor->iMotor_EnablePin, iSpeed);  // motor speed  
      break;

    CloseShutter:
      digitalWrite(prMotor->iMotor_DirPinA, HIGH);  // rotate reverse
      digitalWrite(prMotor->iMotor_DirPinB, LOW);
      analogWrite(prMotor->iMotor_EnablePin, iSpeed);  // motor speed  
      break;
  }
}


void activateMotorDrive(TMotor* prMotor, int iDiff) {
  // decide speed
  int iSpeed = Stop;
  
  if (abs(iDiff) > 100)
    iSpeed=255;
    else  
    if (abs(iDiff) > 20)
      iSpeed=128;
      else
      if (abs(iDiff) > 5)
        iSpeed=100;

  // decide direction
  TMotorDir eMotorDir = iDiff > 0 ? OpenShutter : CloseShutter;
  if (iDiff==0)
    eMotorDir=Stop;
    
  setMotorSpeed(prMotor, eMotorDir, iSpeed);
}


int readActualPosition(TMotor* prMotor) {
  return 0;  //!!!!!!!!!!!!!!!!!!!
}


int readSetPosition1() {
  return 0;  //!!!!!!!!!!!!!!!!!!!
}


int readSetPosition2() {
  return 0;  //!!!!!!!!!!!!!!!!!!!
}


int readSetPosition(TMotor* prMotor) {
  switch (prMotor->iMotorNum) {
    case Motor1:  
      return readSetPosition1();
      
    case Motor2:  
      return readSetPosition2();

    default:    
      return 0;
  }
}


bool positionUnchanged(int iOldValue, int iNewValue) {
  return abs(iOldValue - iNewValue) < 5;
}


void driveUntilStopped(TMotor* prMotor, TMotorDir eMotorDir) {
  unsigned long iStarted = millis();
  const int iOverallTimeoutMS=1000;

  setMotorSpeed(prMotor, eMotorDir, icSeekSpeed); 

  int iLastPosition=-1;
  int iNewPosition=0;
  bool bTimeout;
  
  do {
    iLastPosition = iNewPosition;
    
    delay(50);

    iNewPosition = readActualPosition(prMotor);
    
    bool bTimeout = ((millis() - iStarted) > iOverallTimeoutMS);
  } while ((!bTimeout) && (!positionUnchanged(iLastPosition, iNewPosition)));

  if (bTimeout)
    return;   // hmmm, we need to show an error condition

  if (eMotorDir==OpenShutter) 
    prMotor->iMemory_OpenPosition = iNewPosition;
    else
    if (eMotorDir==CloseShutter) 
      prMotor->iMemory_ClosedPosition = iNewPosition;
}


void setup()
{
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Ready!");
  
  rMotor1.iMotorNum=Motor1;
  rMotor1.iPositionAnalogInputPin=Motor1_PositionAnalogInputPin;
  rMotor1.iMotor_DirPinA=In1_Motor1DirA;
  rMotor1.iMotor_DirPinB=In2_Motor1DirB;
  rMotor1.iMotor_EnablePin=En1_Motor1Enable;
  configurePort(&rMotor1);
  
  rMotor2.iMotorNum=Motor2;
  rMotor2.iPositionAnalogInputPin=Motor2_PositionAnalogInputPin;
  rMotor2.iMotor_DirPinA=In3_Motor2DirA;
  rMotor2.iMotor_DirPinB=In4_Motor2DirB;
  rMotor2.iMotor_EnablePin=En2_Motor2Enable;
  configurePort(&rMotor2);

  driveUntilStopped(&rMotor1, OpenShutter);
  driveUntilStopped(&rMotor2, OpenShutter);
  
  driveUntilStopped(&rMotor1, CloseShutter);
  driveUntilStopped(&rMotor2, CloseShutter);
}


int actualPositionPct(TMotor* prMotor) {
  int iMin = prMotor->iMemory_ClosedPosition;
  int iMax = prMotor->iMemory_OpenPosition;
  int iDiff = iMax - iMin;

  if (iDiff==0)
    return 0;
  
  int iPosPct = 100 * (readActualPosition(prMotor) - iMin) / iDiff;
}


int manageMotor(TMotor* prMotor) {
  int iSetPct = 0;    // !!!!!!!!!!!!!!!!
  int iPosPct = actualPositionPct(prMotor);

  activateMotorDrive(prMotor, iSetPct - iPosPct);
}


void loop()
{  
  while (1) {
    manageMotor(&rMotor1);
    manageMotor(&rMotor1);
    
    delay(50);
  }
}

