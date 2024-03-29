/*****************************************************************************
** PROJECT-NAME: ISL MOTOR CONTROL
** This projected is licensed under the terms of the MIT license
** Copyright (c) 2015, ISL
** All rights reserved.
**
******************************************************************************
** FILENAME:    IslMotorControl.h
** AUTHOR(S):   Mahmut Demir <mahmutdemir@gmail.com>
** DESCRIPTION: This file provides functions for controlling arduino based
* 				motor control card. Also contains some functions to use Minik
* 				robot hardware.
*****************************************************************************/
#ifndef ISL_MOTOR_CONTROL_H
#define ISL_MOTOR_CONTROL_H

#include "Arduino.h"
#include <PID_v1.h> 
#include <Timer.h> 
#include <Encoder.h>
#include <ByteBuffer.h>

#define USE_SERIAL2 0 //Arduino MEGA has 3 serial ports
#define USE_SERIAL3 0 //If you want to use others too, set them true

#define SERIAL_BAUDRATE 19200

#define PIN_CNY70_1 42 //Line follower sensors pins
#define PIN_CNY70_2 40
#define PIN_CNY70_3 38

#define PIN_MOT1_E1 20 //Encoder pins
#define PIN_MOT1_E2 21

#define PIN_MOT2_E1 19 //Encoder pins
#define PIN_MOT2_E2 18

#define PIN_MOT1_PWM   3    //Motor 1 PWM pin
#define PIN_MOT1_BRAKE 9    //Motor 1 brake pin
#define PIN_MOT1_DIR   12   //Motor 1 direction pin
#define PIN_MOT2_PWM   11   //Motor 2 PWM pin
#define PIN_MOT2_BRAKE 8    //Motor 2 brake pin
#define PIN_MOT2_DIR   13   //Motor 2 direction pin

#define PIN_TX2 16
#define PIN_RX2 17

#define PIN_TX3 14
#define PIN_RX3 15

#define PIN_SHARP1 A8 //Distance sensor pins
#define PIN_SHARP2 A9

#define PIN_BTN1 52 //Button pins at control panel
#define PIN_BTN2 50
#define PIN_BTN3 48

#define PIN_LED1 46 //Led pins at control panel
#define PIN_LED2 44

// PWM and Motor control related definitions
#define SPEED_SAMPLE_TIME 5
#define PID_SAMPLE_TIME 30
#define DIST_SENSOR_SAMPLE_TIME 100
#define PER_SEND_TIME 50
#define BUTTON_SEND_TIME 50
#define ADAPTIVE_SPEED_MULTIPLIER 15
#define PWM_MIN 100
#define PWM_MAX 250
#define SPEED_FILTER_CONST 1.0
#define SEC_MS      1000.0
#define SPEED_PID_DIVIDER 1000.0
#define POS_PID_DIVIDER 100.0
#define MIN_PWM_LIM 120
#define PI 3.14159265358979
#define CM_2_COUNTS 106


//Communication related definitions
//Receiver Card IDs
#define REC_CARD1 0x06
#define REC_CARD2 0x07
#define REC_CARD3 0x08

//Protocol constants - header and footer constants
#define PROTOCOL_START_0   0xA0
#define PROTOCOL_START_1   0xA1
#define PROTOCOL_START_2   0xA2
#define PROTOCOL_CHK_POS   3
#define PROTOCOL_DATA_POS  9
#define PROTOCOL_MOTOR_POS 9
#define PROTOCOL_N_FOOTER  3
#define PROTOCOL_STOP      0xAA
#define PROTOCOL_TIMEOUT      0xFF
#define PROTOCOL_PACK_SIZE    25
#define PROTOCOL_CONTROL_SIZE 9

#define CMD_GET_COUNTER   0x00
#define CMD_GET_SPEED     0x01
#define CMD_GET_SPEED_PID 0x02
#define CMD_GET_POS_PID   0x03
#define CMD_GET_TMOD1     0x04
#define CMD_NOT_ASS_1     0x05
#define CMD_NOT_ASS_2     0x06
#define CMD_NOT_ASS_3     0x07
#define CMD_NOT_ASS_4     0x08
#define CMD_NOT_ASS_5     0x09
#define CMD_SET_COUNTER   0x10
#define CMD_SET_SPEED     0x11
#define CMD_SET_SPEED_PID 0x12
#define CMD_SET_POS_PID   0x13
#define CMD_SET_TMOD1     0x14
#define CMD_SET_COUNTERS  0x15
#define CMD_SET_SPEEDS    0x16
#define CMD_GET_DISTS     0x17	//Get sharp distance sensor information in order Right-Left
#define CMD_GET_LINES     0x18	//Get line follower sensor information in order Right-Middle-Left
#define CMD_GET_LED		  0x19
#define CMD_SET_LED		  0x20
#define CMD_GET_BUTTON	  0x21
#define CMD_GET_CONFIG	  0x22
#define CMD_SET_CONFIG	  0x23

#define CONFIG_SEND_SPEED 0
#define CONFIG_SEND_POS   1
#define CONFIG_SEND_FREQ1 2
#define CONFIG_SEND_FREQ2 3
#define CONFIG_SEND_FREQ3 4

#define INIT_SEND_SPEED   0
#define INIT_SEND_POS     1
#define INIT_SEND_FREQ1   1
#define INIT_SEND_FREQ2   1
#define INIT_SEND_FREQ3   1



#define PACKET_RECEIVED   0x00
#define SUCCESS 0xFF
#define FAIL    0x00
#define TIMEOUT_VAL 1000
#define RECV_START_BYTES  0x00
#define RECV_HEADER_BYTES 0x01
#define RECV_DATA_BYTES   0x02


// Initial PID values
#define INIT_KP_S_1 5	// Proportional gain speed
#define INIT_KI_S_1 0 	// Integration gain
#define INIT_KD_S_1 1 	// Differential gain 
#define INIT_KP_P_1 75  // Proportional gain position
#define INIT_KI_P_1 0 	// Integration gain
#define INIT_KD_P_1 20  // Differential gain

#define INIT_KP_S_2 5 	// Proportional gain speed
#define INIT_KI_S_2 0 	// Integration gain
#define INIT_KD_S_2 1 	// Differential gain 
#define INIT_KP_P_2 75  // Proportional gain position
#define INIT_KI_P_2 0 	// Integration gain
#define INIT_KD_P_2 20  // Differential gain


class IslMotorControl
{
  public:
	Encoder *mot1Enc;
    Encoder *mot2Enc;

    long mot1Speed;
    long mot2Speed;

    long mot1CountOld;
    long mot2CountOld;
    long mot1Count;
    long mot2Count;

    long mot1TargetSpeed;
    long mot2TargetSpeed;
    long mot1TargetPos;
    long mot2TargetPos;

    float mot1PWM;
    float mot2PWM;
    
    int distSensorL;
    int distSensorR;
	int distSensorLmm;
	int distSensorRmm;
	
	int lineR;
	int lineM;
	int lineL;
	
	int config;
	int sendSpeedPeriodic; 
	int sendPosPeriodic; 
	int periodicSendFreq;

  public:
    bool   isSpeedPIDActive;
    double KP_S_1;
    double KI_S_1;
    double KD_S_1;
    double KP_P_1;
    double KI_P_1;
    double KD_P_1;

    double KP_S_2;
    double KI_S_2;
    double KD_S_2;
    double KP_P_2;
    double KI_P_2;
    double KD_P_2;

  private:
    unsigned long prevTime;
    unsigned long prevPIDTime;
    unsigned long prevSensorTime;
    unsigned long prevPerSendTime;
    unsigned long prevButtonSendTime;
    PID* PIDSpeed1;
    PID* PIDPos1;
    PID* PIDSpeed2;
    PID* PIDPos2;
    float  mot1PIDPWMOut;
    float  mot2PIDPWMOut;
    long PIDSetSpeed1;
    long PIDSetSpeed2;
    long PIDSetPos1;
    long PIDSetPos2;
    void speedPIDUpdate();
    void posPIDUpdate();

  public:
    void getSpeedParam(char motor, int* param);
    void getPosParam(char motor, int* param);
    void setSpeedParam(char motor, int p, int i, int d);
    void setPosParam(char motor, int p, int i, int d);
    long getMotorCounter(char motor);
    long getMotorSpeed(char motor);
    void setMotorCounter(char motor, long counter);
    void setMotorSpeed(char motor, long speed);
    void setMotorSpeeds(long speed1, long speed2);
    void setMotorCounters(long counter1, long counter2);
	void sendPeriodicCounter();
	void sendMotorSpeeds();
	void sendMotorPositions();
	void sendButtonStatus();
	void readConfig(int newConfig);

  public:
    bool job1ms();
    bool job10ms();
    bool job100ms();
    bool job1000ms();
    void jobDone1ms();
    void jobDone10ms();
    void jobDone100ms();
    void jobDone1000ms();

  public:
    IslMotorControl();
    void toggleYellow();
    void toggleGreen();
    void updateMotors();
    
    Timer* t;

  public:
    void respondPack(unsigned char* pack);
    void setPackHeader(unsigned char* pack, char task, char packetLength, char* receiver);
    char getPackedSender(unsigned char* pack);
    char getPackedLength(unsigned char* pack);
    char getPackedTask(unsigned char* pack);
    boolean checkCardID(unsigned char *pack);
    void getPackedId(unsigned char* pack, char* id);
    unsigned char isPackValid(unsigned char* pack);
    void refreshPack(unsigned char* pack);
    void checkSerialCommand();
    long count2Cm(long count);
    long cm2Count(long cm);

  private:
    double zeroAbsorb(double val);
    char DEVICEID = char(REC_CARD1);
    ByteBuffer sendBuffer;
    unsigned char pack[PROTOCOL_PACK_SIZE];
    char recvState = char(RECV_START_BYTES);
    int nrRecvdBytes = 0;
    long lastRecvTime = 0;
};

#endif
