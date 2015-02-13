

//***************************************************************
/*
void setup() {
  
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);   // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);   // turn on pullup resistor
  pinMode(brakeA,OUTPUT);
  pinMode(dirA,OUTPUT);
  
  pinMode(encoder2PinA, INPUT); 
  digitalWrite(encoder2PinA, HIGH);   // turn on pullup resistor
  pinMode(encoder2PinB, INPUT); 
  digitalWrite(encoder2PinB, HIGH);   // turn on pullup resistor
  pinMode(brakeB,OUTPUT);
  pinMode(dirB,OUTPUT);
  attachInterrupt(4, doEncoderB, CHANGE);
  attachInterrupt(0, doEncoderA, CHANGE);  // encoder pin on interrupt 0 - pin 2
   // encoder pin on interrupt 2 - pin 21
  
  PIDStart('s'); //turn the speed or position PID on.
  PIDStart('p');
  Serial.begin (9600); //Serial port begins.
  Serial.println("start");   // a personal quirk

}*/
/*
void loop() {
  
  setRobotSpeed(targetSpeedA,targetSpeedB);
  printData();
  //delay(10);
}*/
//****************************************************************
//---------------------------------------------------------------
//Basic motor functions//
void motorForward(int PWM_val)  {
   analogWrite(PWMA, PWM_val);
   digitalWrite(brakeA, LOW);
   digitalWrite(dirA, HIGH);
}

void motorBackward(int PWM_val)  {
   analogWrite(PWMA, PWM_val);
   digitalWrite(brakeA, HIGH);
   digitalWrite(dirA, LOW);
}

void motorStop()  {
   analogWrite(PWMA, 0);
   digitalWrite(brakeA, LOW);
   digitalWrite(dirA, LOW);
}
//--------------------------------------------------------------


//--------------------------------------------------------------
//Function returns position of wheel in cm. 
//Wheel diameter must be in cm.
//if encoderName==(char)0x00 function returns Motor A position
//if encoderName==(char)0x01 function returns Motor B position
double getPosition(char encoderName){
      double scale = 0.000523 * DIAMETER ;      
      if(encoderName==(char)0x00){
          double posA = (double)(getEncoderValue((char)0x00)*scale);
          return posA;
      }
      else if(encoderName==(char)0x01){
          double posB = (double)(getEncoderValue((char)0x01)*scale);
          return posB;
      }
}

//Function returns speed of the wheel in cm/s.
//Uses encoder data
//if encoderName==(char)0x00 function returns Motor A speed
//if encoderName==(char)0x01 function returns Motor B speed
double getSpeed(char encoderName){
      double scale = 0.000523 * DIAMETER ;
      long pos1;
      long pos2;
       pos1 = getEncoderValue(encoderName);
       delay(timeInterval);
       pos2 = getEncoderValue(encoderName);
       return (pos2-pos1)*scale*1000/timeInterval;
}

void calculateSpeed(char encoderName){
      double scale = 0.000523 * DIAMETER ;    
          if(encoderName == (char)0x00 && millis()-lastTimeA>=timeInterval){
                speedA = (encoderAPos-encoderAPosLast)*scale*1000/(millis()-lastTimeA);
                encoderAPosLast = encoderAPos;
                lastTimeA = millis();
                
          }
          else if(encoderName == (char)0x01 && millis()-lastTimeB>=timeInterval){
                speedB = (encoderBPos-encoderBPosLast)*scale*1000/(millis()-lastTimeB);
                encoderBPosLast = encoderBPos;
                lastTimeB = millis();
          }
}
double getSpeedNoDelay(char encoderName){
     if(encoderName == (char)0x00) return speedA;
     else if(encoderName == (char)0x01) return speedB;
}
//--------------------------------------------------------------

//--------------------------------------------------------------
long getEncoderValue(char encoderName){
//Function returns encoder current value
//if encoderName==(char)0x00 function returns Motor A encoder value
//if encoderName==(char)0x01 function returns Motor B encoder value
    if(encoderName==(char)0x00) return encoderAPos;
    else if(encoderName==(char)0x01) return encoderBPos;
}
//---------------------------------------------------------------
void doEncoderA() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
     encoderAPos++;
  } 
  else {
     encoderAPos--;
  }
 }
 
 //*********************doEncoder B **************************
 void doEncoderB() {
   if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
      encoderBPos--;
  } 
  else {
     encoderBPos++;
  }
 }
//----------------------------------------------------------------

//----------------------------------------------------------------
//Function starts the PID.
//if PIDName=='s' starts speed PID (PIDSpeed)
//if PIDName=='p' starts position PID (PIDPos)
void PIDStart(char PIDName){
    if(PIDName == 's'){ 
        PIDSpeedA.SetMode(AUTOMATIC);
        PIDSpeedA.SetOutputLimits(-255,255);
        PIDSpeedA.SetSampleTime(30);
        PIDSpeedB.SetMode(AUTOMATIC);
        PIDSpeedB.SetOutputLimits(-255,255);
        PIDSpeedB.SetSampleTime(30);
    }
    else if(PIDName == 'p'){ 
        PIDPosA.SetMode(AUTOMATIC);
        PIDPosA.SetOutputLimits(-255,255);
        PIDPosA.SetSampleTime(30);
        PIDPosB.SetMode(AUTOMATIC);
        PIDPosB.SetOutputLimits(-255,255);
        PIDPosB.SetSampleTime(30);
    }
}
//------------------------------------------------------------

//-------------------**Speed PID**------------------------------
//Function sets motor speed deseired value
void setMotorSpeed(char encoderName, double targetSpeed){
    if(encoderName==(char)0x00){
        if(targetSpeed>=0) digitalWrite(dirA,HIGH);
        else digitalWrite(dirA,LOW);
        //calculateSpeed((char)0x00);
        InputSpeedA = getSpeed((char)0x00);
        SetPointSpeedA = targetSpeed;
        PIDSpeedA.Compute();
        PWMA_val = constrain(PWMA_val+OutputSpeedA,0,PWMSpeedMaxValue);
        analogWrite(PWMA,int(PWMA_val));
    }
    else if(encoderName==(char)0x01){
        if(targetSpeed>=0) digitalWrite(dirB,HIGH);
        else digitalWrite(dirB,LOW);
        //calculateSpeed((char)0x01);
        InputSpeedB = getSpeed((char)0x01);
        SetPointSpeedB = targetSpeed;
        PIDSpeedB.Compute();
        PWMB_val = constrain(PWMB_val+OutputSpeedB,0,PWMSpeedMaxValue);
        analogWrite(PWMB,int(PWMB_val));
    }  
}
//--------------------------------------------------------------

//--------------------**Position PID**--------------------------
// Sets motor position to desiered value.
void setMotorPosition(char encoderName, double target){
	if(encoderName==(char)0x00){
        InputPosA = getPosition((char)0x00);
        SetPointPosA = target;
        PIDPosA.Compute();
        PWMA_val = constrain(abs(OutputPosA),0,PWMPosMaxValue);
        if(abs(InputPosA-SetPointPosA)<0.75){
            PWMA_val=0;
            OutputPosA = 0;
         }
         if(target>=InputPosA){
            digitalWrite(dirA,HIGH);
            analogWrite(PWMA,int(PWMA_val));
        }
        else if(target<InputPosA){
            digitalWrite(dirA,LOW);
            analogWrite(PWMA,int(PWMA_val));
        }
    }
    else if(encoderName==(char)0x01){
        InputPosB = getPosition((char)0x01);
        SetPointPosB =target;
        PIDPosB.Compute();
        PWMB_val = constrain(abs(OutputPosB),0,PWMPosMaxValue);
        if(abs(InputPosB-SetPointPosB)<0.75){
            PWMB_val=0;
            OutputPosB = 0;
         }
         if(target>=InputPosB){
            digitalWrite(dirB,HIGH);
            analogWrite(PWMB,int(PWMB_val));
        }
        else if(target<InputPosB){
            digitalWrite(dirB,LOW);
            analogWrite(PWMB,int(PWMB_val));
        }
    }
}
//------------------------------------------------------------
//*******************Setting Robot Speed************************
//targetA---->targetSpeedA
//targetB---->targetSpeedB
void setRobotSpeed(double targetA,double targetB){
    setMotorSpeed((char)0x00, targetA);
    setMotorSpeed((char)0x01, targetB);
}
//*******************Setting Robot Position**********************
//targetA---->targetPosA
//targetB---->targetPosB
void setRobotPos(double targetA, double targetB){
    setMotorPosition((char)0x00, targetA);
    setMotorPosition((char)0x01, targetB);
}
//************************************************************


//Function prints sth. in every 1 sec.
void printData(){
  if( count == 100 ){
        //Serial.println(getSpeedNoDelay((char)0x00));
        //Serial.println(getSpeedNoDelay((char)0x01));
       // Serial.println(InputPosA);
        //Serial.println(InputPosB);
        //Serial.println(encoderAPos);
        //Serial.println(encoderBPos);
        count=0;
    }
  count++;  
}

//**************************communication function***************
void getSpeedParam(char motor, int* param){
    
    if(motor==(char)0x00){
      param[0]=KP_S_A;
      param[1]=KI_S_A;
      param[2]=KD_S_A;
    }
    else{
       param[0]=KP_S_B;
       param[1]=KI_S_B;
       param[2]=KD_S_B;
    }
	
}
void getPosParam(char motor, int* param){
    
    if(motor==(char)0x00){
      param[0]=KP_P_A;
      param[1]=KI_P_A;
      param[2]=KD_P_A;
    }
    else{
       param[0]=KP_P_B;
       param[1]=KI_P_B;
       param[2]=KD_P_B;
    }
}
void setSpeedParam(char motor,double p,double i,double d){
    if(motor==(char)0x00){
        KP_S_A=p;
        KI_S_A=i;
        KD_S_A=d;
    }
    else{
         KP_S_B=p;
         KI_S_B=i;
         KD_S_B=d;
    }
}
void setPosParam(char motor,int p,int i,int d){
    if(motor==(char)0x00){
        KP_P_A=p;
        KI_P_A=i;
        KD_P_A=d;
     }
     else{
          KP_P_B=p;
          KI_P_B=i;
          KD_P_B=d;
     }
}
long getMotorCounter(char MotorID){
    if(MotorID==(char)0x00)   
	return encoderAPos;
    else 
	return encoderBPos;
}
void setMotorSpeedValue(char MotorID,double Speed){
    isSpeedPIDActive = true;
    if(MotorID==(char)0x00){
      targetSpeedA=Speed;
    }
    else{
      targetSpeedB=Speed;
    }
}
void setMotorPositionValue(char MotorID,double positionValue){
    isSpeedPIDActive = false;
    if(MotorID==(char)0x00){
      targetPosA=positionValue;
    }
    else{
      targetPosB=positionValue;
    }
}
double getMotorSpeedValue(char MotorID){
    if(MotorID==(char)0x00){
      return getSpeed((char)0x00);
    }
    else{
      return getSpeed((char)0x01);
    }
}
double getMotorPositionValue(char MotorID){
    if(MotorID==(char)0x00){
      return getPosition((char)0x00);
    }
    else{
      return getPosition((char)0x01);
    }
}

void setMotorSpeedValues(double SpeedA,double SpeedB){
      isSpeedPIDActive = true;
      targetSpeedA=SpeedA;
      targetSpeedB=SpeedB;
}
void setMotorPositionValues(double positionA,double positionB){
      isSpeedPIDActive = false;
      targetPosA=positionA;
      targetPosB=positionB;
}
