/*
  ISL_PID_v1.h - Library for PID control for new designed motor cards
  Created by Mehmet Ozgur Turkoglu and Mahmut Demir, February 23, 2015.
*/

#include "Arduino.h"
#include "ISL_PID_v1.h"

long encoderAPos = 0;  //Motor A encoder value
long encoderBPos = 0;  //Motor B encoder value
long encoderAPosLast = 0;  //Motor A encoder value in which speed is calculated for the last time.
long encoderBPosLast = 0;  //Motor B encoder value in which speed is calculated for the last time.

unsigned long lastTimeA = 0; //in ms. time in which motor A speed calculated for the last time.
unsigned long lastTimeB = 0; //in ms. time in which motor B speed calculated for the last time.
	
long speedA = 0;
long speedB = 0;
int timeInterval = 100;//milliseconds


void doEncoderA() {

	//If pinA and pinB are both high or both low, it is spinning
	//forward. If they're different, it's going backward.
	//
	//For more information on speeding up this process, see
	//[Reference/PortManipulation], specifically the PIND register.
	if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
		encoderAPos++;
	} 
	else {
		encoderAPos--;
	}
}


void doEncoderB() {
	if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
		encoderBPos--;
	} 
	else {
		encoderBPos++;
	}
}

void calculateSpeed() {
      double scale = 0.000523 * DIAMETER ;    
      if(millis()-lastTimeA>=timeInterval){
            speedA = (encoderAPos-encoderAPosLast)*scale*1000/(millis()-lastTimeA);
            encoderAPosLast = encoderAPos;
            lastTimeA = millis();
            
      }
      if(millis()-lastTimeB>=timeInterval){
            speedB = (encoderBPos-encoderBPosLast)*scale*1000/(millis()-lastTimeB);
            encoderBPosLast = encoderBPos;
            lastTimeB = millis();
      }
}

IslPID::IslPID()
{
	isSpeedPIDActive = false;
	targetSpeedA = 0;// in cm/s.
	targetSpeedB= 0;
	targetPosA = 0;
	targetPosB = 0;

	PIDSpeedA = new PID(&InputSpeedA, &OutputSpeedA, &SetPointSpeedA,KP_S_A,KI_S_A,KD_S_A, DIRECT);
	PIDPosA = new PID(&InputPosA, &OutputPosA, &SetPointPosA,KP_P_A,KI_P_A,KD_P_A, DIRECT);
	PIDSpeedB = new PID(&InputSpeedB, &OutputSpeedB, &SetPointSpeedB,KP_S_B,KI_S_B,KD_S_B, DIRECT);
	PIDPosB = new PID(&InputPosB, &OutputPosB, &SetPointPosB,KP_P_B,KI_P_B,KD_P_B, DIRECT);

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
	attachInterrupt(2, doEncoderB, CHANGE);
	attachInterrupt(4, doEncoderA, CHANGE);  // encoder pin on interrupt 0 - pin 2
	// encoder pin on interrupt 2 - pin 21

	PIDStart('s'); //turn the speed or position PID on.
	PIDStart('p');

	timer = new Timer;

	timer->every(100, calculateSpeed);
}

//Mid-Level Motor Functions
void IslPID::motorForward(int PWM_val)  {
   analogWrite(PWMA, PWM_val);
   digitalWrite(brakeA, LOW);
   digitalWrite(dirA, HIGH);
}

void IslPID::motorBackward(int PWM_val)  {
   analogWrite(PWMA, PWM_val);
   digitalWrite(brakeA, HIGH);
   digitalWrite(dirA, LOW);
}

void IslPID::motorStop()  {
   analogWrite(PWMA, 0);
   digitalWrite(brakeA, LOW);
   digitalWrite(dirA, LOW);
}


//Function returns position of wheel in cm. 
//Wheel diameter must be in cm.
//if encoderName==(char)0x00 function returns Motor A position
//if encoderName==(char)0x01 function returns Motor B position
long IslPID::getPosition(char encoderName){
      //double scale = 0.000523 * DIAMETER ;      
	  double scale = 1 ; 
      if(encoderName==(char)0x00){
          long posA = (long)(getEncoderValue((char)0x00)*scale);
          return posA;
      }
      else if(encoderName==(char)0x01){
          long posB = (long)(getEncoderValue((char)0x01)*scale);
          return posB;
      }
}



long IslPID::getSpeedNoDelay(char encoderName){
     if(encoderName == (char)0x00) 
		 return speedA;
     else if(encoderName == (char)0x01) 
		 return speedB;
}

//Function returns encoder current value
//if encoderName==(char)0x00 function returns Motor A encoder value
//if encoderName==(char)0x01 function returns Motor B encoder value
long IslPID::getEncoderValue(char encoderName){
    if(encoderName==(char)0x00) 
		return encoderAPos;
    else if(encoderName==(char)0x01) 
		return encoderBPos;
}

//Function starts the PID.
//if PIDName=='s' starts speed PID (PIDSpeed)
//if PIDName=='p' starts position PID (PIDPos)
void IslPID::PIDStart(char PIDName){
    if(PIDName == 's'){ 
        PIDSpeedA->SetMode(AUTOMATIC);
        PIDSpeedA->SetOutputLimits(-255,255);
        PIDSpeedA->SetSampleTime(30);
        PIDSpeedB->SetMode(AUTOMATIC);
        PIDSpeedB->SetOutputLimits(-255,255);
        PIDSpeedB->SetSampleTime(30);
    }
    else if(PIDName == 'p'){ 
        PIDPosA->SetMode(AUTOMATIC);
        PIDPosA->SetOutputLimits(-255,255);
        PIDPosA->SetSampleTime(30);
        PIDPosB->SetMode(AUTOMATIC);
        PIDPosB->SetOutputLimits(-255,255);
        PIDPosB->SetSampleTime(30);
    }
}

//Function sets motor speed desired value
void IslPID::setMotorSpeed(char encoderName, long targetSpeed){
    if(encoderName==(char)0x00){
        if(targetSpeed>=0) digitalWrite(dirA,HIGH);
        else digitalWrite(dirA,LOW);
        //calculateSpeed((char)0x00);
        InputSpeedA = getSpeedNoDelay((char)0x00);
        SetPointSpeedA = targetSpeed;
        PIDSpeedA->Compute();
		PWMA_val = constrain(PWMA_val + OutputSpeedA, -1 * PWMSpeedMaxValue, PWMSpeedMaxValue);

		if (PWMA_val > 0)
		{
			digitalWrite(dirA, HIGH);
			analogWrite(PWMA, int(PWMA_val));
		}
		else
		{
			digitalWrite(dirA, LOW);
			analogWrite(PWMA, -1*int(PWMA_val));
		}

        
    }
    else if(encoderName==(char)0x01){
        if(targetSpeed>=0) digitalWrite(dirB,HIGH);
        else digitalWrite(dirB,LOW);
        //calculateSpeed((char)0x01);
        InputSpeedB = getSpeedNoDelay((char)0x01);
        SetPointSpeedB = targetSpeed;
        PIDSpeedB->Compute();
		PWMB_val = constrain(PWMB_val + OutputSpeedB, -1*PWMSpeedMaxValue, PWMSpeedMaxValue);
        
		if (PWMB_val > 0)
		{
			digitalWrite(dirB, HIGH);
			analogWrite(PWMB, int(PWMB_val));
		}
		else
		{
			digitalWrite(dirB, LOW);
			analogWrite(PWMB, -1 * int(PWMB_val));
		}
    }  
}

// Sets motor position to desired value.
void IslPID::setMotorPosition(char encoderName, long target){
	if(encoderName==(char)0x00){
        InputPosA = getPosition((char)0x00);
        SetPointPosA = target;
        PIDPosA->Compute();
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
        PIDPosB->Compute();
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


//These function must be called repetitevely in order to make PID run..
//Please use speed or position depending on PID control type.

//Setting Robot Speed
//targetA---->targetSpeedA
//targetB---->targetSpeedB
void IslPID::updateSpeedPID(long targetA,long targetB){
    setMotorSpeed((char)0x00, targetA);
    setMotorSpeed((char)0x01, targetB);
}

//Setting Robot Position
//targetA---->targetPosA
//targetB---->targetPosB
void IslPID::updatePosPID(long targetA, long targetB){
    setMotorPosition((char)0x00, targetA);
    setMotorPosition((char)0x01, targetB);
}



// Public Methods used in communication with motor card
void IslPID::getSpeedParam(char motor, int* param){
    
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
void IslPID::getPosParam(char motor, int* param){
    
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
void IslPID::setSpeedParam(char motor,double p,double i,double d){
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
void IslPID::setPosParam(char motor,double p,double i,double d){
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
long IslPID::getMotorCounter(char MotorID){
    if(MotorID==(char)0x00)   
	return encoderAPos;
    else 
	return encoderBPos;
}
void IslPID::setMotorSpeedValue(char MotorID,long Speed){
    isSpeedPIDActive = true;
    if(MotorID==(char)0x00){
      targetSpeedA=Speed;
    }
    else{
      targetSpeedB=Speed;
    }
}
void IslPID::setMotorPositionValue(char MotorID,long positionValue){
    isSpeedPIDActive = false;
    if(MotorID==(char)0x00){
      targetPosA=positionValue;
    }
    else{
      targetPosB=positionValue;
    }
}
long IslPID::getMotorSpeedValue(char MotorID){
    if(MotorID==(char)0x00){
      return getSpeedNoDelay((char)0x00);
    }
    else{
      return getSpeedNoDelay((char)0x01);
    }
}
long IslPID::getMotorPositionValue(char MotorID){
    if(MotorID==(char)0x00){
      return getPosition((char)0x00);
    }
    else{
      return getPosition((char)0x01);
    }
}

void IslPID::setMotorSpeedValues(long SpeedA,long SpeedB){
      isSpeedPIDActive = true;
      targetSpeedA=SpeedA;
      targetSpeedB=SpeedB;
}
void IslPID::setMotorPositionValues(long positionA,long positionB){
      isSpeedPIDActive = false;
      targetPosA=positionA;
      targetPosB=positionB;
}