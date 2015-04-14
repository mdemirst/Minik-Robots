/*
  ISL_PID_v1.h - Library for PID control for new designed motor cards
  Created by Mehmet Ozgur Turkoglu and Mahmut Demir, February 23, 2015.
*/

#ifndef ISL_PID_V1_H
#define ISL_PID_V1_H

#include "Arduino.h"
#include <PID_v1.h> // Official general purpose PID Library for arduino
#include <Timer.h> // Official general purpose PID Library for arduino

//Defines

#define DIAMETER 9  //Wheel diameter is in cm.
#define PWMSpeedMaxValue 255
#define PWMPosMaxValue 175
#define encoder2PinA  21 //Motor A encoder pinA
#define encoder2PinB  20 //Motor A encoder pinB
#define encoder0PinA  18 //Motor B encoder pinA
#define encoder0PinB  19 //Motor B encoder pinB
#define PWMA 3   //Motor A PWM pin
#define brakeA 9  //Motor A brake pin
#define dirA 12   //Motor A direction pin
#define PWMB 11   //Motor B PWM pin
#define brakeB 8  //Motor B brake pin
#define dirB 13   //Motor B direction pin

class IslPID
{
	public: 
		IslPID();
		void getSpeedParam(char motor, int* param);
		void getPosParam(char motor, int* param);
		void setSpeedParam(char motor,double p,double i,double d);
		void setPosParam(char motor,double p,double i,double d);
		long getMotorCounter(char MotorID);
		void setMotorSpeedValue(char MotorID,long Speed);
		void setMotorPositionValue(char MotorID,long positionValue);
		long getMotorSpeedValue(char MotorID);
		long getMotorPositionValue(char MotorID);
		void setMotorSpeedValues(long SpeedA,long SpeedB);
		void setMotorPositionValues(long positionA,long positionB);
		void updateSpeedPID(long targetA, long targetB);
		void updatePosPID(long targetA, long targetB);

	public:	

		PID* PIDSpeedA;
		PID* PIDPosA;
		PID* PIDSpeedB;
		PID* PIDPosB;

		void motorForward(int PWM_val);
		void motorBackward(int PWM_val);
		void motorStop();
		long getPosition(char encoderName);
		long getSpeedNoDelay(char encoderName);
		long getEncoderValue(char encoderName);
		void PIDStart(char PIDName);
		void setMotorSpeed(char encoderName, long targetSpeed);
		void setMotorPosition(char encoderName, long target);

	public:

		boolean isSpeedPIDActive;
		long targetSpeedA;
		long targetSpeedB;
		long targetPosA;
		long targetPosB;
		Timer* timer;

	private:
		double distanceInterval = 1;

		long InputSpeedA;
		long OutputSpeedA;
		long SetPointSpeedA;
		long InputPosA;
		long OutputPosA;
		long SetPointPosA;

		long InputSpeedB;
		long OutputSpeedB;
		long SetPointSpeedB;
		long InputPosB;
		long OutputPosB;
		long SetPointPosB;

		double KP_S_A = 0.1; // Proportional gain speed 10,2,1
		double KI_S_A = 0 ; // Integration gain
		double KD_S_A = 0 ; // Differential gain 
		double KP_P_A = 0; //20; // Proportional gain position
		double KI_P_A = 0; //12; // Integration gain
		double KD_P_A = 0; //10; // Differential gain

		double KP_S_B = 0.1; // Proportional gain speed 10,2,1
		double KI_S_B = 0 ; // Integration gain
		double KD_S_B = 0 ; // Differential gain 
		double KP_P_B = 0;//20 ; // Proportional gain position
		double KI_P_B = 0;//12 ; // Integration gain
		double KD_P_B = 0;//10; // Differential gain




		int PWMA_val = 0; //Initial PWMA value.
		int PWMB_val = 0; //Initial PWMB value.

		int count = 0; //This intger used for DisplayData() function.
};

#endif