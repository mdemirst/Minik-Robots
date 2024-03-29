/////////////////////////////////////////////////////////////
// @Author: Mahmut Demir - mahmutdemir@gmail.com
// @Last update: 13.10.2015
// @Description: ISL Motor Control Card
//               Arduino sources codes
//               Written for Arduino Mega 2560 with motor shield
//               Contains mainly motor position and speed PID control
//               and communication protocol over serial port.
//               Using this library you can control connected two DC
//               motors as well as leds, buttons and distance sensors.
//               This code uses libraries below:
//                 1) IslMotorControl
//                 2) Modified PID_v1
//                 3) TimerMaster
//                 4) Encoder
//                 5) ByteBuffer
//
// 
// @Comments: You can see example usage of functions below in comment.
//            PLASE STRICTLY OBEY FEW RULES:
//              1) DO NOT USE delay(ms) function
//              2) You can use PERIODIC JOBS if you want to call your functions
//                 with delay
//              3) DO NOT ERASE these:
//                 IslMotorControl mot; //Main motor control library
//                 mot.updateMotors();  //Reads values from encoders and runs PID process etc...
//                 mot.jobDone1ms();    //Sets job done flags true
// @Usage:
// --> Pin Functions
// PIN | DEFINE      |   Function 
//
// A8    PIN_SHARP1      Sharp distance sensor analog read pin
// A9    PIN_SHARP2      Sharp distance sensor analog read pin
//
// 52    PIN_BTN1        Button 1 pin on the back panel of robot
// 50    PIN_BTN2        Button 2 pin on the back panel of robot
// 48    PIN_BTN3        Button 3 pin on the back panel of robot
//
// 46    PIN_LED1        Green led on the back panel of robot
// 44    PIN_LED2        Yellow led on the back panel of robot
//
// 42    PIN_CNY70_1     CNY70 1st sensor
// 40    PIN_CNY70_2     CNY70 2nd sensor
// 38    PIN_CNY70_3     CNY70 3rd sensor
// 
// --> Functions:
//   mot.toggleYellow(), mot.toggleGreen(): Toggles states of leds
//   void getSpeedParam(char motor, int* param);
//   void getPosParam(char motor, int* param);
//   void setSpeedParam(char motor, int p, int i, int d);
//   void setPosParam(char motor, int p, int i, int d);
//   long getMotorCounter(char motor);
//   long getMotorSpeed(char motor);
//   void setMotorCounter(char motor, long counter);
//   void setMotorSpeed(char motor, long speed); Speed in counts/sec
//   void setMotorSpeeds(long speed1, long speed2);
//   void setMotorCounters(long counter1, long counter2);
//   long count2Cm(long counts); Get encoder counts in CM 
//                               (You need to set COUNTS_2_CM accordingly
//                                in IslMotorControl.h)
//   long cm2Count(long cm);
//
// --> Variables: You can read state of robot from variables directly.
//  distSensorL, distSensorR: Distance sensor reads
//  mot1Count, mot2Count:     Current counter values;(long) for left and right motor
//  mot1Speed, mot2Speed:     Current speed values(long) for left and right motor
//  lineR, lineM, lineL:      Line following sensor readings
//
// --> Serial Ports: 
//  By default first serial port is set to 19200 bps and don't change it.
//  By modifying IslMotorControl library you can enable or disable 2nd
//    and 3rd serial ports, too. If you you serial ports, you need to
//    INITIALIZE them in the setup() function below!
//  
///////////////////////////////////////////////////////////////////////////////////////////

#include <Encoder.h>
#include <PID_v1.h>
#include "PID_v1.h"
#include "Timer.h"
#include <ByteBuffer.h>
#include "PID_v1.h"
#include <IslMotorControl.h>

IslMotorControl mot;  //Initialize motor control class.
                      //If you like to use functions or variables,
                      //do not forget to call them by prefixing "mot."
                      //Ex: mot.toggleYellow();
                      //    mot.mot2Speed;

boolean showDemo = false;
void setup()
{
  pinMode(10, OUTPUT);
  Serial.begin(19200);  //Don't change it. Standart communication 
                        //protocol with computer uses 19200 bps
}


void loop()
{
  
  mot.updateMotors();  //You need to call this periodically and very fast!.
                       //So, do not use delay() anywhere in the code. It affects
                       //PID and serial port communication. Instead you can use
                       //periodic jobs. It's standard way of it. We can't use threads
                       //here!
  
  if(mot.job1ms() == true)  //This block is guaranteed to be called in every 1ms
  {
    mot.jobDone1ms();
  }
  if(mot.job10ms() == true)  //This block is guaranteed to be called in every 10ms
  {
    mot.jobDone10ms();
    //Example functions
    //mot.setMotorCounters(10000,-10000);
    //mot.setMotorSpeeds(3000,-3000);

//    if(digitalRead(PIN_BTN1)==LOW && showDemo == false)
//    {
//      showDemo = true;
//      while(digitalRead(PIN_BTN1)==LOW){}
//    }
//
//
//    if(digitalRead(PIN_BTN2))
//    {
//      digitalWrite(PIN_LED1,LOW);
//    }
//    else
//    {
//      digitalWrite(PIN_LED1,HIGH);    
//    }
//
//    if(digitalRead(PIN_BTN3))
//    {
//      digitalWrite(PIN_LED2,LOW);
//    }
//    else
//    {
//      digitalWrite(PIN_LED2,HIGH);    
//    }

    
  }
  if(mot.job100ms() == true)  //This block is guaranteed to be called in every 100ms
  {
    //Print counters in CM - this is only used for debugging purpose. Writing dummy data
    //to serial port breaks the communication with computers
//    Serial.print("M1: ");
//    Serial.print(mot.mot1Count);
//    Serial.print(" ");
//    Serial.print("M2: ");
//    Serial.print(mot.mot2Count);
//    Serial.print(" ");
//    Serial.print("DL: ");
//    Serial.print(mot.distSensorL);
//    Serial.print(" ");
//    Serial.print("DR: ");
//    Serial.println(mot.distSensorR);

//    if(showDemo)
//    {
//      mot.setMotorCounters(mot.cm2Count(100),mot.cm2Count(100));
//      showDemo = false;
//    }

    //This function is only used for robotic arm control
    //mot.sendPeriodicCounter();
    mot.jobDone100ms();
  }
  if(mot.job1000ms() == true)  //This block is guaranteed to be called in every 1 sec
  {
    //Serial.println(mot.config);
    //mot.toggleYellow();
    //mot.toggleGreen();
    mot.jobDone1000ms();
  }
  
  
}
