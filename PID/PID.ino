#include <PID_v1.h>//PID Library

#define DIAMETER 9  //Wheel diameter is in cm.
#define PWMSpeedMaxValue 255
#define PWMPosMaxValue 175

#define encoder0PinA  2 //Motor A encoder pinA
#define encoder0PinB  4 //Motor B encoder pinB
#define PWMA 3   //Motor A PWM pin
#define brakeA 9  //Motor A brake pin
#define dirA 12   //Motor A direction pin
double targetSpeed = 50; // in cm/s.
double targetPos = 200;
double distanceInterval = 1;
long encoderAPos = 0;  //Motor A encoder value
long encoderBPos = 0;  //Motor B encoder value
double InputSpeed;
double OutputSpeed;
double SetPointSpeed;
double InputPos;
double OutputPos;
double SetPointPos;
double KP_S = 10 ; // Proportional gain speed
double KI_S = 2 ; // Integration gain
double KD_S = 1 ; // Differential gain 
double KP_P = 20 ; // Proportional gain position
double KI_P = 12 ; // Integration gain
double KD_P = 10; // Differential gain 
PID PIDSpeed(&InputSpeed, &OutputSpeed, &SetPointSpeed,KP_S,KI_S,KD_S, DIRECT);
PID PIDPos(&InputPos, &OutputPos, &SetPointPos,KP_P,KI_P,KD_P, DIRECT);

int PWM_val = 0; //Initial PWM value.
int count = 0; //This intger used for DisplayData() function.

void setup() {
  
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);   // turn on pullup resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);   // turn on pullup resistor
  pinMode(brakeA,OUTPUT);
  pinMode(dirA,OUTPUT);
  
  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  
  PIDStart('s'); //turn the speed or position PID on.
 
  Serial.begin (9600); //Serial port begins.
  Serial.println("start");   // a personal quirk

}

void loop() {
  setMotorSpeed(targetSpeed);
  printData();
  delay(10);
  
  
}
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
//if encoderName=='A' function returns Motor A position
//if encoderName=='B' function returns Motor B position
double getPosition(char encoderName){
      double scale = 0.000523 * DIAMETER ;      
      if(encoderName=='A'){
          double posA = (double)(getEncoderValue('A')*scale);
          return posA;
      }
      else if(encoderName=='B'){
          double posB = (double)(getEncoderValue('B')*scale);
          return posB;
      }
}

//Function returns speed of the wheel in cm/s.
//Uses encoder data
//if encoderName=='A' function returns Motor A speed
//if encoderName=='B' function returns Motor B speed
double getSpeed(char encoderName){
      double scale = 0.000523 * DIAMETER ;
      int timeInterval = 10;//milliseconds
      long pos1;
      long pos2;
       pos1 = getEncoderValue(encoderName);
       delay(timeInterval);
       pos2 = getEncoderValue(encoderName);
       return (pos2-pos1)*scale*1000/timeInterval;
}
//--------------------------------------------------------------

//--------------------------------------------------------------
long getEncoderValue(char encoderName){
//Function returns encoder current value
//if encoderName=='A' function returns Motor A encoder value
//if encoderName=='B' function returns Motor B encoder value
    if(encoderName=='A') return encoderAPos;
    else if(encoderName=='B') return encoderBPos;
}
//---------------------------------------------------------------
void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  double scale = 0.000523 * DIAMETER;
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoderAPos++;
  } 
  else {
    encoderAPos--;
  }
 }
//----------------------------------------------------------------

//----------------------------------------------------------------
//Function starts the PID.
//if PIDName=='s' starts speed PID (PIDSpeed)
//if PIDName=='p' starts position PID (PIDPos)
void PIDStart(char PIDName){
    if(PIDName == 's'){ 
        PIDSpeed.SetMode(AUTOMATIC);
        PIDSpeed.SetOutputLimits(-255,255);
        PIDSpeed.SetSampleTime(30);
    }
    else if(PIDName == 'p'){ 
        PIDPos.SetMode(AUTOMATIC);
        PIDPos.SetOutputLimits(-255,255);
        PIDPos.SetSampleTime(30);
    }
}
//------------------------------------------------------------

//-------------------**Speed PID**------------------------------
//Function sets motor speed deseired value
void setMotorSpeed(double targetSpeed){
    if(targetSpeed>=0) digitalWrite(dirA,HIGH);
    else digitalWrite(dirA,LOW);
    InputSpeed = getSpeed('A');
    SetPointSpeed = targetSpeed;
    PIDSpeed.Compute();
    PWM_val = constrain(PWM_val+OutputSpeed,0,PWMSpeedMaxValue);
    analogWrite(PWMA,int(PWM_val));
}
//--------------------------------------------------------------

//--------------------**Position PID**--------------------------
// Sets motor position to desiered value.
void setMotorPosition(double target){
    InputPos = getPosition('A');
    SetPointPos = target;
    PIDPos.Compute();
    PWM_val = constrain(abs(OutputPos),0,PWMPosMaxValue);
    if(abs(InputPos-SetPointPos)<0.75){
        PWM_val=0;
        OutputPos = 0;
     }
     if(target>=InputPos){
        digitalWrite(dirA,HIGH);
        analogWrite(PWMA,int(PWM_val));
    }
    else if(target<InputPos){
        digitalWrite(dirA,LOW);
        analogWrite(PWMA,int(PWM_val));
    }
}
//------------------------------------------------------------

//------------------------------------------------------------
//This function can be used to adjust motor speed or direction.
int getParam()  {
    char param, cmd;
    if(!Serial.available())    return 0;
    delay(10);                  
    param = Serial.read();    // get parameter byte
    if(!Serial.available())    return 0;
    cmd = Serial.read();     // get command byte
    Serial.flush();
    switch (param) {
       case 'v':           // adjust speed
         if(cmd=='+')  {
             targetSpeed += 5;
           if(targetSpeed>100)   targetSpeed=100;
         }
         if(cmd=='-')    {
           targetSpeed -= 5;
           if(targetSpeed<0)   targetSpeed=0;
         }
         break;
       case 's':       // adjust direction
          if(cmd=='+'){
             digitalWrite(dirA, HIGH);
             digitalWrite(brakeA, LOW);
          }
          if(cmd=='-')   {
             digitalWrite(dirA, LOW);
             digitalWrite(brakeA, LOW);
         }
         break;
       case 'o':              // user should type "oo"
         digitalWrite(dirA, LOW);
         digitalWrite(brakeA, HIGH);
         targetSpeed = 0;
         break;
         default: 
         Serial.println("???");
   }
}
//------------------------------------------------------------

//------------------------------------------------------------
//Function prints sth. every 1 sec.
void printData(){
  if( count == 100 ){
        Serial.println(getSpeed('A'));
        Serial.println(InputSpeed);
        Serial.println(SetPointSpeed);
        Serial.println(OutputSpeed);
        count=0;
    }
  count++;  
}
