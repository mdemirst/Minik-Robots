#include <ByteBuffer.h>
#include <PID_v1.h>//PID Library
#include "PID.h"

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

//Current card id
char DEVICEID = char(REC_CARD1);  

ByteBuffer sendBuffer;
char pack[PROTOCOL_PACK_SIZE];

char recvState = char(RECV_START_BYTES);
int nrRecvdBytes = 0;
long lastRecvTime = 0;

int ledPin = 13;



void setup() {

   Serial.begin(19200,SERIAL_8N1);
  
   // Initialize the send buffer that we will use to send data
   sendBuffer.init(250);
   
   // Keeps serial command receive state - e.g. waits for start bytes, data bytes etc..
   recvState = char(RECV_START_BYTES);
   
   pinMode(ledPin, OUTPUT);
   
   
   ///////
   
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
}

//Checks if received pack valid
char isPackValid(char* pack) {
  char temp, i;
  int packetLength = (int)pack[5];
  // check start bits
  if (!((pack[0] == char(PROTOCOL_START_0)) && 
        (pack[1] == char(PROTOCOL_START_1)) &&
        (pack[2] == char(PROTOCOL_START_2))))
    return FAIL;  
    
  // check stop bits
  if (pack[packetLength - PROTOCOL_N_FOOTER] != char(PROTOCOL_STOP))
    return FAIL;
    
  // check header
  temp = FAIL;
  for (i = PROTOCOL_CHK_POS; i < PROTOCOL_CONTROL_SIZE; i++)
    temp ^= pack[i];
  if (pack[packetLength - PROTOCOL_N_FOOTER + 1] != temp)
    return FAIL;  
    
  // check footer
  temp = FAIL;
  for (i = PROTOCOL_CHK_POS; i < packetLength - 1; i++)
    temp ^= pack[i];
  if (pack[packetLength - PROTOCOL_N_FOOTER + 2] != temp)
    return FAIL;
    
  return SUCCESS;
}

void getPackedId(char* pack, char* id) {
  id[0] = pack[6];
  id[1] = pack[7];
  id[2] = pack[8];
  return;
}

boolean checkCardID(char *pack) {
  if((pack[6] == DEVICEID) && (pack[7] == 0x00) && (pack[8] == 0x00))
    return true;
  else 
    return false;
}

char getPackedTask(char* pack) {
  return pack[4];
}

char getPackedLength(char* pack) {
  return pack[5];
}

char getPackedSender(char* pack) {
  return pack[3];
}

void setPackHeader(char* pack, char task, char packetLength, char* receiver) {
  pack[3] = 0x04; // Designates sender is slave 
  pack[4] = task;
  pack[5] = packetLength;
  pack[6] = receiver[0];
  pack[7] = receiver[1];
  pack[8] = receiver[2];
}

void refreshPack(char* pack) {
  char temp, i;
  int packLength = (int)pack[5];
  // set start bytes
  pack[0] = PROTOCOL_START_0;
  pack[1] = PROTOCOL_START_1;
  pack[2] = PROTOCOL_START_2;
  pack[packLength - 3] = PROTOCOL_STOP;
  // set header check
  temp = 0x00;
  for (i = PROTOCOL_CHK_POS; i < PROTOCOL_CONTROL_SIZE; i++)
    temp ^= pack[i];
  pack[packLength - 2] = temp;
  // set pack check
  temp = 0x00;
  for (i = PROTOCOL_CHK_POS; i < packLength - 1; i++)
    temp ^= pack[i];
  pack[packLength - 1] = temp;
}

void respondPack(char* pack) {     
  int packetLength = 0;
  char tempArray[3];
  int  tempIntArray[3];
  long temp = 0x00000000;
  
  // check for the embedded command
  if (getPackedLength(pack) == 0x00)
    return;
  
  // control whether the packet is sent to this card
  if (checkCardID(pack) == true) {
    
    // do the task
    // getSpeed command
    if (getPackedTask(pack) == (char)CMD_GET_SPEED ) {
      //get speed
      //temp = (long) getSpeed(pack[PROTOCOL_DATA_POS]);
      temp = (long)getMotorSpeedValue(pack[PROTOCOL_MOTOR_POS]);

      pack[PROTOCOL_DATA_POS + 1] = char(temp >> 8);
      pack[PROTOCOL_DATA_POS + 2] = char((temp));
      packetLength = 3;
    } 
    
    // setSpeed command
    else if (getPackedTask(pack) == (char)CMD_SET_SPEED) {

      // setSpeed
      setMotorSpeedValue( pack[PROTOCOL_MOTOR_POS],
        (double)(((int) pack[PROTOCOL_DATA_POS + 1] << 8) + 
        (double)(pack[PROTOCOL_DATA_POS + 2])));
      packetLength = 1;
    } 
    
    // getMotorCounter command
    else if (getPackedTask(pack) == (char)CMD_GET_COUNTER ) {
      // getMotorCounter
      //temp = (long) getMotorCounter(pack[PROTOCOL_DATA_POS]);
      temp = (long) getMotorPositionValue(pack[PROTOCOL_MOTOR_POS]);
      
      pack[PROTOCOL_DATA_POS + 1] = char(temp >> 24);
      pack[PROTOCOL_DATA_POS + 2] = char(temp >> 16);
      pack[PROTOCOL_DATA_POS + 3] = char(temp >> 8);
      pack[PROTOCOL_DATA_POS + 4] = char(temp);
      packetLength = 5;
    } 
    
    // setMotorCounter command
    else if (getPackedTask(pack) == (char)CMD_SET_COUNTER) {

      // setMotorCounter
      //setCounter( pack[PROTOCOL_DATA_POS], (long)(
      //  ((long) pack[PROTOCOL_DATA_POS + 1] << 24) +
      //  ((long) pack[PROTOCOL_DATA_POS + 2] << 16) + 
      //  ((long) pack[PROTOCOL_DATA_POS + 3] << 8) + 
      //  ((long) pack[PROTOCOL_DATA_POS + 4])));
      
      
      setMotorPositionValue( pack[PROTOCOL_MOTOR_POS], (double)(
        (((long) pack[PROTOCOL_DATA_POS + 1]) << 24) |
        (((long) pack[PROTOCOL_DATA_POS + 2]) << 16) | 
        (((long) pack[PROTOCOL_DATA_POS + 3]) << 8) | 
        ((long) pack[PROTOCOL_DATA_POS + 4])));
      packetLength = 1;
    } 
    
    // get speed PID Coef command
    else if (getPackedTask(pack) == (char)CMD_GET_SPEED_PID ) {
      // getPIDCoef, for speed PID
      //getSpeedPIDCoef(pack[PROTOCOL_DATA_POS], tempArray);
      
      getSpeedParam(pack[PROTOCOL_DATA_POS], tempIntArray);
      
      pack[PROTOCOL_DATA_POS + 1] = (char)tempIntArray[0];
      pack[PROTOCOL_DATA_POS + 2] = (char)tempIntArray[1];
      pack[PROTOCOL_DATA_POS + 3] = (char)tempIntArray[2];
      packetLength = 4;
    } 
    // set speed PID Coef command
    else if (getPackedTask(pack) == (char)CMD_SET_SPEED_PID ) {
      // setPIDCoef, for speed PID
      tempArray[0] = pack[PROTOCOL_DATA_POS + 1];
      tempArray[1] = pack[PROTOCOL_DATA_POS + 2];
      tempArray[2] = pack[PROTOCOL_DATA_POS + 3];
      
      setSpeedParam(pack[PROTOCOL_DATA_POS], tempArray[0], tempArray[1], tempArray[2]);
      //setSpeedPIDCoef(pack[PROTOCOL_DATA_POS], tempArray);
      packetLength = 1;
    } 
    // get position PID Coef command
    else if (getPackedTask(pack) == (char)CMD_GET_POS_PID ) {
      // getPIDCoef, for position PID
      //getPositionPIDCoef(pack[PROTOCOL_DATA_POS], tempArray);
      
      getPosParam(pack[PROTOCOL_DATA_POS], tempIntArray);
      
      pack[PROTOCOL_DATA_POS + 1] = (char)tempIntArray[0];
      pack[PROTOCOL_DATA_POS + 2] = (char)tempIntArray[1];
      pack[PROTOCOL_DATA_POS + 3] = (char)tempIntArray[2];
      packetLength = 4;
    } 
    // set position PID Coef command
    else if (getPackedTask(pack) == (char)CMD_SET_POS_PID ) {
      // setPIDCoef, for position PID
      tempArray[0] = pack[PROTOCOL_DATA_POS + 1];
      tempArray[1] = pack[PROTOCOL_DATA_POS + 2];
      tempArray[2] = pack[PROTOCOL_DATA_POS + 3];
      //setPositionPIDCoef(pack[PROTOCOL_DATA_POS], tempArray);
      
      setPosParam(pack[PROTOCOL_DATA_POS], tempArray[0], tempArray[1], tempArray[2]);
      packetLength = 1;
    }
   
    // getTMOD1, gets the speed PID update rate
    else if (getPackedTask(pack) == (char)CMD_GET_TMOD1 ) {
      // getTMOD1, gets the speed PID update rate
      //pack[PROTOCOL_DATA_POS] = getTMOD1();
      packetLength = 1;     
    } 

    // setTMOD1, sets the speed PID update rate
    else if (getPackedTask(pack) == (char)CMD_SET_TMOD1 ) {
      // setTMOD1, sets the speed PID update rate
      //setTMOD1(pack[PROTOCOL_DATA_POS]);
    } 
    
    // setMotorCounters command
    else if (getPackedTask(pack) == (char)CMD_SET_COUNTERS ) {
      // enable position PID
      //enablePositionPID();
      // setMotorCounter
      //setCounters((long)((
      //  ((long) pack[PROTOCOL_DATA_POS] << 24) +
      //  ((long) pack[PROTOCOL_DATA_POS + 1] << 16) + 
      //  ((long) pack[PROTOCOL_DATA_POS + 2] << 8) + 
      //  ((long) pack[PROTOCOL_DATA_POS + 3]))),
      //  (long)(((long) pack[PROTOCOL_DATA_POS + 4] << 24) +
      //  ((long) pack[PROTOCOL_DATA_POS + 5] << 16) + 
      //  ((long) pack[PROTOCOL_DATA_POS + 6] << 8) + 
      //  ((long) pack[PROTOCOL_DATA_POS + 7])));
      
      setMotorPositionValue( (char)0x00, (double)(
        (((long) pack[PROTOCOL_DATA_POS + 0]) << 24) |
        (((long) pack[PROTOCOL_DATA_POS + 1]) << 16) | 
        (((long) pack[PROTOCOL_DATA_POS + 2]) << 8) | 
        ((long) pack[PROTOCOL_DATA_POS + 3])));
        
      setMotorPositionValue( (char)0x01, (double)(
        (((long) pack[PROTOCOL_DATA_POS + 4]) << 24) |
        (((long) pack[PROTOCOL_DATA_POS + 5]) << 16) | 
        (((long) pack[PROTOCOL_DATA_POS + 6]) << 8) | 
        ((long) pack[PROTOCOL_DATA_POS + 7])));
      
      packetLength = 1;
    }  
    
    // setSpeeds command
    else if (getPackedTask(pack) == (char)CMD_SET_SPEEDS ) {

      // setSpeeds
      //setSpeeds((int)((int)(pack[PROTOCOL_DATA_POS] << 8) + 
      //(int)(pack[PROTOCOL_DATA_POS + 1])),
      //(int)((int)(pack[PROTOCOL_DATA_POS + 2] << 8) + 
      //(int)(pack[PROTOCOL_DATA_POS + 3])));
      
      setMotorSpeedValue( (char)0x00,
        (double)(((int) pack[PROTOCOL_DATA_POS + 0] << 8) + 
        (double)(pack[PROTOCOL_DATA_POS + 1])));
        
      setMotorSpeedValue( (char)0x01,
        (double)(((int) pack[PROTOCOL_DATA_POS + 2] << 8) + 
        (double)(pack[PROTOCOL_DATA_POS + 3])));
        
      packetLength = 1;
    } 
    
    else {
      return;  
    }
    
    // set the header of the pack
    tempArray[0] = DEVICEID; //getPackedSender(pack);    
    tempArray[1] = 0x00;    
    tempArray[2] = 0x00;    
    setPackHeader(pack, getPackedTask(pack), 
      packetLength + PROTOCOL_CONTROL_SIZE 
      + PROTOCOL_N_FOOTER, tempArray);    
    // recalculate the check sums
    refreshPack(pack);
    

       
    // send the pack
    
    //sendBuffer.clear();
    Serial.write(pack,packetLength + PROTOCOL_CONTROL_SIZE + PROTOCOL_N_FOOTER);

  }
}


void serialEvent() {
  


  while( Serial.available() )
  {
        
    //Checks if data connection is timed out. If so, it return to wait for start bytes state
    if(((recvState == char(RECV_HEADER_BYTES)||(recvState == char(RECV_DATA_BYTES))) )&&
       (nrRecvdBytes > 0) &&
       ((millis()-lastRecvTime) > TIMEOUT_VAL))
    {
      recvState = char(RECV_START_BYTES);
      nrRecvdBytes = 0;
    }
    
    //Waits for three start bytes
    if(recvState == char(RECV_START_BYTES))
    {
      pack[0] = pack[1];
      pack[1] = pack[2];
      pack[2] = char(Serial.read());

      if (((pack[0] == char(PROTOCOL_START_0)) && 
           (pack[1] == char(PROTOCOL_START_1)) &&
           (pack[2] == char(PROTOCOL_START_2))))
      {
        

        recvState = char(RECV_HEADER_BYTES); 
        nrRecvdBytes = 0;
      }
    }
    //Waits for header bytes
    else if(recvState == char(RECV_HEADER_BYTES))
    {

      pack[PROTOCOL_CHK_POS + nrRecvdBytes] =char(Serial.read());
      nrRecvdBytes = nrRecvdBytes + 1;
      
        
      if((nrRecvdBytes + PROTOCOL_CHK_POS) == PROTOCOL_DATA_POS)
      {
        
        nrRecvdBytes = 0;
        recvState = char(RECV_DATA_BYTES);
      }
    }
    //Waits for data bytes
    else if(recvState == char(RECV_DATA_BYTES))
    {
      
        
      pack[PROTOCOL_DATA_POS + nrRecvdBytes] = char(Serial.read());
      nrRecvdBytes++;
      
      if(nrRecvdBytes == int(pack[5]) - PROTOCOL_CONTROL_SIZE)
      {
        recvState = char(RECV_START_BYTES);
        nrRecvdBytes = 0;
        
        if(isPackValid(pack) == (char)SUCCESS)
        {
          //Command is taken, respond it
          respondPack(pack);
        }
        else
        {
          /*
          digitalWrite(ledPin,HIGH);
          delay(1000);
          digitalWrite(ledPin,LOW);*/
        }
      }
    }
    
    lastRecvTime = millis();
  }
}

void loop() {
  
  if(isSpeedPIDActive)
    setRobotSpeed(targetSpeedA, targetSpeedB);
  else
    setRobotPos(targetPosA, targetPosB);

}
