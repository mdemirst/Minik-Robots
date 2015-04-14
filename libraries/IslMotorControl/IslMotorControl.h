/*
  IslMotorControl.h
  
  Contains methods for controlling 2 motors, sharp sensors, CNY70 sensors, buttons and leds.
  
  Author: Mahmut Demir
  Date: 11.02.2015
  E-Mail: mahmutdemir@gmail.com
  
*/
#ifndef ISL_MOTOR_CONTROL_H
#define ISL_MOTOR_CONTROL_H

#include "Arduino.h"
#include <PID_v1.h> 
#include <Timer.h> 
#include <Encoder.h>
#include <ByteBuffer.h>

#define USE_SERIAL2 1
#define USE_SERIAL3 1

#define SERIAL_BAUDRATE 19200

#define PIN_CNY70_1 42
#define PIN_CNY70_2 40
#define PIN_CNY70_3 38

#define PIN_MOT1_E1 19 //2 //IO 21
#define PIN_MOT1_E2 18 //3// IO 20

#define PIN_MOT2_E1 20 //4 //IO 19
#define PIN_MOT2_E2 21 //5 //IO 18

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

#define PIN_SHARP1 A8
#define PIN_SHARP2 A9

#define PIN_BTN1 52
#define PIN_BTN2 50
#define PIN_BTN3 48

#define PIN_LED1 46
#define PIN_LED2 44

#define SPEED_SAMPLE_TIME 50
#define PID_SAMPLE_TIME 100
#define DIST_SENSOR_SAMPLE_TIME 100
#define PWM_MAX 255

#define WHEEL_DIAMETER 9  //Wheel diameter is in cm.
#define COUNTS_PER_ROTATION 12000
#define COUNTER2CM  (0.0023561944)
#define CM2COUNTER  (424.413181)
#define SEC_MS      1000
#define MOT0_DIR    1
#define MOT1_DIR    -1


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

#define PACKET_RECEIVED   0x00

#define SUCCESS 0xFF
#define FAIL    0x00

#define TIMEOUT_VAL 1000

#define RECV_START_BYTES  0x00
#define RECV_HEADER_BYTES 0x01
#define RECV_DATA_BYTES   0x02

#define PI 3.14159265358979

#define MIN_PWM_LIM 50

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

  public:
    bool   isSpeedPIDActive;
    double KP_S_1 = 0.00005; // Proportional gain speed 10,2,1
    double KI_S_1 = 0.000001; // Integration gain
    double KD_S_1 = 0.000008; // Differential gain 
    double KP_P_1 = 0.03; //20; // Proportional gain position
    double KI_P_1 = 0.001; //12; // Integration gain
    double KD_P_1 = 0.005; //10; // Differential gain

    double KP_S_2 = 0.00005; // Proportional gain speed 10,2,1
    double KI_S_2 = 0.000001; // Integration gain
    double KD_S_2 = 0.000008; // Differential gain 
    double KP_P_2 = 0.03;//20 ; // Proportional gain position
    double KI_P_2 = 0.001;//12 ; // Integration gain
    double KD_P_2 = 0.005;//10; // Differential gain

  private:
    unsigned long prevTime;
    unsigned long prevPIDTime;
    unsigned long prevSensorTime;
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
    void toggleYellow();
    void toggleGreen();
    
    Timer* t;

  public:
    void respondPack(char* pack);
    void setPackHeader(char* pack, char task, char packetLength, char* receiver);
    char getPackedSender(char* pack);
    char getPackedLength(char* pack);
    char getPackedTask(char* pack);
    boolean checkCardID(char *pack);
    void getPackedId(char* pack, char* id);
    char isPackValid(char* pack);
    void refreshPack(char* pack);
    void checkSerialCommand();
    long count2Cm(long count);
    long cm2Count(long cm);

  public:
    //Current card id
    char DEVICEID = char(REC_CARD1);

    ByteBuffer sendBuffer;
    char pack[PROTOCOL_PACK_SIZE];

    char recvState = char(RECV_START_BYTES);
    int nrRecvdBytes = 0;
    long lastRecvTime = 0;

  private:
    double zeroAbsorb(double val);

  public:
    IslMotorControl();
    void updateMotors();
};

#endif