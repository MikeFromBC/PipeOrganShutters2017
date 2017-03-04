// digital outputs
const int In1_Motor1DirA=5;
const int In2_Motor1DirB=6;
const int In3_Motor2DirA=7;
const int In4_Motor2DirB=8;
const int En1_Motor1Enable=9;
const int En2_Motor2Enable=10;

// analog inputs
const int Motor1_ActualPositionAnalogInputPin=A0;
const int Motor2_ActualPositionAnalogInputPin=A1;

const int Motor1_SetPositionAnalogInputPin_TEST=A2;
const int Motor2_SetPositionAnalogInputPin_TEST=A3;

const int Motor1=1;
const int Motor2=2;

const int icShutterSteps=16;
const int icSeekSpeed=128;
const int dcThresholdPct=100 / icShutterSteps;

struct TMotor {
  byte iMotorNum;
  byte iActualPositionAnalogInputPin;
  byte iSetPositionAnalogInputPin_TEST;
  
  // motor control particulars
  byte iMotor_DirPinA;
  byte iMotor_DirPinB;
  byte iMotor_EnablePin;
  boolean bMotor_HighSpeedWasUsed;
  
  int iClosedLimit;
  int iOpenedLimit;

//  int iMemory_FullClosedPosition;
//  int iMemory_FullOpenPosition;
};

enum TMotorDir {CloseShutter, Stop, OpenShutter};

TMotor rMotor1, rMotor2;


void logPosition() {
  int i =0;
  while (i<550) {
    Serial.print("set value:  ");
    Serial.print(analogRead(Motor1_SetPositionAnalogInputPin_TEST));

    Serial.print("  set pct value:  ");
    Serial.print(readSetPositionPct(&rMotor1));
    Serial.print("  actual value:  ");
    Serial.print(analogRead(rMotor1.iActualPositionAnalogInputPin));
    Serial.print("  actual value pct:  ");
    Serial.println(readActualPositionPct(&rMotor1));
    delay(100);
    i++;
  }
}


void logPositionAndStop() {
  logPosition();
  
  while (1) {
  }    
}


void configurePort(TMotor* prMotor) {
  pinMode(prMotor->iActualPositionAnalogInputPin, INPUT);  
  pinMode(prMotor->iSetPositionAnalogInputPin_TEST, INPUT);  
  
  pinMode(prMotor->iMotor_DirPinA, OUTPUT);
  pinMode(prMotor->iMotor_DirPinB, OUTPUT);  
  pinMode(prMotor->iMotor_EnablePin, OUTPUT);    
}


void stopMotor(TMotor* prMotor) {
  analogWrite(prMotor->iMotor_EnablePin, 0);  // motor speed  
}


void setMotorSpeed(TMotor* prMotor, TMotorDir eMotorDir, byte iSpeed) {
  if (eMotorDir!=Stop) {
    if (rMotor1.bMotor_HighSpeedWasUsed) 
      Serial.print("HS  ");
//    Serial.print("set pct value:  ");
//    Serial.print(readSetPositionPct(&rMotor1));
//    Serial.print("  actual value:  ");
//    Serial.print(analogRead(rMotor1.iActualPositionAnalogInputPin));
//    Serial.print("  actual value pct:  ");
//    Serial.print(readActualPositionPct(&rMotor1));
//    Serial.print("  diff pct:  ");
//    Serial.println(iDiffPct);
    Serial.print("speed:  ");
    Serial.print(iSpeed);
    if (eMotorDir==CloseShutter)
      Serial.println("  CLOSING");
    if (eMotorDir==OpenShutter)
      Serial.println("  OPENING");
  } else
    Serial.println("STOPPED");
    
  //stopMotor(prMotor);
  
  switch (eMotorDir) {
    case OpenShutter:
      digitalWrite(prMotor->iMotor_DirPinA, LOW);  // rotate forward
      digitalWrite(prMotor->iMotor_DirPinB, HIGH);
      analogWrite(prMotor->iMotor_EnablePin, iSpeed);  // motor speed  
      break;

    case CloseShutter:
      digitalWrite(prMotor->iMotor_DirPinA, HIGH);  // rotate reverse
      digitalWrite(prMotor->iMotor_DirPinB, LOW);
      analogWrite(prMotor->iMotor_EnablePin, iSpeed);  // motor speed  
      break;

    case Stop:
      stopMotor(prMotor);
      break;
  }
}


int readActualPositionPct(TMotor* prMotor) {
  int iRawValue = analogRead(prMotor->iActualPositionAnalogInputPin);
  int iRangeSize = prMotor->iOpenedLimit - prMotor->iClosedLimit;
  return round(100 * (iRawValue - prMotor->iClosedLimit) / iRangeSize);
}


int readSetPositionPct(TMotor* prMotor) {
  return round(100.0 * analogRead(prMotor->iSetPositionAnalogInputPin_TEST) / 1024);
}


//void findLimit(TMotor* prMotor, TMotorDir eMotorDir) {
//  unsigned long iStarted = millis();
//  const int iOverallTimeoutMS=1000;
//
//  setMotorSpeed(prMotor, eMotorDir, icSeekSpeed); 
//  
//  int iLastPosition=-1;
//  int iNewPosition=0;
//  bool bTimeout;
//  bool bFinished = false;
//
//  // give it a chance to start moving before looking...
//  delay(50);
//  
//  do {
//    iLastPosition = iNewPosition;
//    
//    delay(1);
//
//    iNewPosition = readActualPosition(prMotor);
//
//    if (prMotor->iMotorNum==1) {
//      Serial.print(eMotorDir);
//      Serial.print(" new pos:  ");
//      Serial.print(iNewPosition);
//      Serial.print(" time delta:  ");
//      Serial.println(millis() - iStarted);
//    }
//
//    switch (eMotorDir) {
//      case CloseShutter:
//        if (iNewPosition < 250) {
//          bFinished=true;
//          Serial.println("CLOSE FINISHED");
//        }
//        break;
//
//      case OpenShutter:
//        if (iNewPosition > 350) {
//          bFinished=true;
//          Serial.println("OPEN FINISHED");
//        }
//        break;
//    }
//    
//    bTimeout = ((millis() - iStarted) > iOverallTimeoutMS);
//  } while ((!bTimeout) && (abs(iLastPosition - iNewPosition) > icThreshold) && (!bFinished));
//
//  if (bTimeout)
//    Serial.println("TIMEOUT!");
//
//  if (eMotorDir==OpenShutter) 
//    prMotor->iMemory_FullOpenPosition = iNewPosition;
//    else
//    if (eMotorDir==CloseShutter) 
//      prMotor->iMemory_FullClosedPosition = iNewPosition;
//
//  stopMotor(prMotor);
//}


//void motorAccelerationTestAndStop(TMotor* prMotor) {
//  unsigned long iStarted = millis();
//  const int iOverallTimeoutMS=100;
//  const int icPointCount=300;
//  word iaTime[icPointCount];
//  word iaValue[icPointCount];
//  
//  for (int i=0; i<icPointCount; i++) {
//    iaTime[i]=0;
//    iaValue[i]=0;
//  }
//
//  setMotorSpeed(prMotor, OpenShutter, icSeekSpeed); 
//  
//  int iPoint=0;
//  do {
//    iaTime[iPoint]=millis() - iStarted;
//    iaValue[iPoint]=readActualPosition(prMotor);
//    iPoint++;    
//
//    delay(10);
//    
//    if ((millis() - iStarted) > 300)
//      break;
//  } while (1);
//
//  stopMotor(prMotor);
//  
//  for (int i=0; i<icPointCount; i++) {
//    Serial.print(iaTime[i]);
//    Serial.print("\t");
//    Serial.println(iaValue[i]);
//  }
//
//  Serial.println("TEST IS FINISHED!");
//
//  while (1) {
//    
//  }
//}


void setup()
{
  Serial.begin(9600);
  
  rMotor1.iMotorNum=Motor1;
  rMotor1.iActualPositionAnalogInputPin=Motor1_ActualPositionAnalogInputPin;
  rMotor1.iSetPositionAnalogInputPin_TEST=Motor1_SetPositionAnalogInputPin_TEST;
  rMotor1.iMotor_DirPinA=In1_Motor1DirA;
  rMotor1.iMotor_DirPinB=In2_Motor1DirB;
  rMotor1.iMotor_EnablePin=En1_Motor1Enable;
  rMotor1.iClosedLimit = 646;
  rMotor1.iOpenedLimit = 834;
  configurePort(&rMotor1);
  
  rMotor2.iMotorNum=Motor2;
  rMotor2.iActualPositionAnalogInputPin=Motor2_ActualPositionAnalogInputPin;
  rMotor2.iSetPositionAnalogInputPin_TEST=Motor2_SetPositionAnalogInputPin_TEST;
  rMotor2.iMotor_DirPinA=In3_Motor2DirA;
  rMotor2.iMotor_DirPinB=In4_Motor2DirB;
  rMotor2.iMotor_EnablePin=En2_Motor2Enable;
  rMotor2.iClosedLimit = 773;
  rMotor2.iOpenedLimit = 888;
  configurePort(&rMotor2);

  //motorAccelerationTestAndStop(&rMotor1);

//  logPositionAndStop();
  
//  Serial.println("Seeking limits...");
//
//  findLimit(&rMotor1, OpenShutter);
//  findLimit(&rMotor1, CloseShutter);
//
//  findLimit(&rMotor2, OpenShutter);
//  findLimit(&rMotor2, CloseShutter);
//  
//  Serial.println("Limits found!");
//  
//  Serial.print("Motor 1 limit:  ");
//  Serial.print(rMotor1.iMemory_FullClosedPosition);
//  Serial.print(" - ");
//  Serial.println(rMotor1.iMemory_FullOpenPosition);
//
//  Serial.print("Motor 2 limit:  ");
//  Serial.print(rMotor2.iMemory_FullClosedPosition);
//  Serial.print(" - ");
//  Serial.println(rMotor2.iMemory_FullOpenPosition);
}


//int actualPositionPct(TMotor* prMotor) {
//  int iMin = prMotor->iMemory_FullClosedPosition;
//  int iMax = prMotor->iMemory_FullOpenPosition;
//  int iDiff = iMax - iMin;
//
//  if (iDiff==0)
//    return 0;
//  
//  int iPosPct = 100 * (readActualPosition(prMotor) - iMin) / iDiff;
//}


void decideDriveSpeedAndDirection(TMotor* prMotor, int iDiffPct, TMotorDir* peMotorDir_out, byte* piSpeed_out) {
  if (abs(iDiffPct) > 25) {
    prMotor->bMotor_HighSpeedWasUsed = true;
    *piSpeed_out=128;
  } else  
    // within one step?  ok!
    if (abs(iDiffPct) <= dcThresholdPct) {
        prMotor->bMotor_HighSpeedWasUsed = false;
        *piSpeed_out=0;
    } else
      if (prMotor->bMotor_HighSpeedWasUsed) 
        *piSpeed_out=80;
        else 
        *piSpeed_out=100;
  
  // decide direction
  *peMotorDir_out = iDiffPct > 0 ? OpenShutter : CloseShutter;
  if (*piSpeed_out==0)
    *peMotorDir_out=Stop;    
}


//void driveToPosition(TMotor* prMotor, int iSetPos) {
//  TMotorDir eMotorDir;
//  byte iSpeed;
//
//  do {
//    do {
//      int iDiff = iSetPos - readActualPosition(prMotor);
//      
//      decideDriveSpeedAndDirection(iDiff, &eMotorDir, &iSpeed);
//  
//      setMotorSpeed(prMotor, eMotorDir, iSpeed);
//          
//      delay(1);
//    } while ((iSpeed!=0) && (eMotorDir!=Stop));
//    
//    stopMotor(prMotor);
//  
//    Serial.print("initial stop ");
//    Serial.println(readActualPosition(prMotor));
//  
//    delay(100);
//  } while (abs(readActualPositionPct(prMotor) - iSetPos) > icThreshold);
//
//  Serial.println("STOPPED");
//}
//
//
//void runTests() {
//  while (1) {
//    driveToPosition(&rMotor1, 400);
//
//    delay(2000);
//
//    driveToPosition(&rMotor1, 390);
//
//    delay(2000);
//
//    driveToPosition(&rMotor1, 350);
//
//    delay(2000);
//
//    driveToPosition(&rMotor1, 250);
//
//    delay(2000);
//
//    driveToPosition(&rMotor1, 350);
//
//    delay(2000);
//
//    driveToPosition(&rMotor1, 390);
//
//    delay(5000);
//  }
//}

void logPositions() {
  while (1) {
    Serial.print("Shutter 1 position:  ");
    int iRawValue = analogRead(rMotor1.iActualPositionAnalogInputPin);
    Serial.println(iRawValue);

    delay(1000);
  }
}

void loop()
{ 
//  logPositions(); 
//  runTests();

  while (1) {
    TMotorDir eMotorDir;
    byte iSpeed;
  
    int iDiffPct = readSetPositionPct(&rMotor1) - readActualPositionPct(&rMotor1);
    decideDriveSpeedAndDirection(&rMotor1, iDiffPct, &eMotorDir, &iSpeed);
    setMotorSpeed(&rMotor1, eMotorDir, iSpeed);
        
//    iDiffPct = readSetPositionPct(&rMotor2) - readActualPositionPct(&rMotor2);
//    decideDriveSpeedAndDirection(&rMotor2, iDiffPct, &eMotorDir, &iSpeed);
//    setMotorSpeed(&rMotor2, eMotorDir, iSpeed);
        
    delay(1);
  }
}

