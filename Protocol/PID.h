#define DIAMETER 9  //Wheel diameter is in cm.
#define PWMSpeedMaxValue 255
#define PWMPosMaxValue 175
#define encoder0PinA  2 //Motor A encoder pinA
#define encoder0PinB  4 //Motor A encoder pinB
#define encoder2PinA  19 //Motor B encoder pinA
#define encoder2PinB  20 //Motor B encoder pinB
#define PWMA 3   //Motor A PWM pin
#define brakeA 9  //Motor A brake pin
#define dirA 12   //Motor A direction pin
#define PWMB 11   //Motor B PWM pin
#define brakeB 8  //Motor B brake pin
#define dirB 13   //Motor B direction pin

boolean isSpeedPIDActive = false;
double targetSpeedA = 30;// in cm/s.
double targetSpeedB= 30;
double targetPosA = -1000;
double targetPosB = -1000;

double distanceInterval = 1;
long encoderAPos = 0;  //Motor A encoder value
long encoderBPos = 0;  //Motor B encoder value
long encoderAPosLast = 0;  //Motor A encoder value in which speed is calculated for the last time.
long encoderBPosLast = 0;  //Motor B encoder value in which speed is calculated for the last time.

double InputSpeedA;
double OutputSpeedA;
double SetPointSpeedA;
double InputPosA;
double OutputPosA;
double SetPointPosA;

double InputSpeedB;
double OutputSpeedB;
double SetPointSpeedB;
double InputPosB;
double OutputPosB;
double SetPointPosB;

double KP_S_A = 1; // Proportional gain speed 10,2,1
double KI_S_A = 0 ; // Integration gain
double KD_S_A = 0 ; // Differential gain 
double KP_P_A = 20 ; // Proportional gain position
double KI_P_A = 12 ; // Integration gain
double KD_P_A = 10; // Differential gain

double KP_S_B = 1; // Proportional gain speed 10,2,1
double KI_S_B = 0 ; // Integration gain
double KD_S_B = 0 ; // Differential gain 
double KP_P_B = 20 ; // Proportional gain position
double KI_P_B = 12 ; // Integration gain
double KD_P_B = 10; // Differential gain

PID PIDSpeedA(&InputSpeedA, &OutputSpeedA, &SetPointSpeedA,KP_S_A,KI_S_A,KD_S_A, DIRECT);
PID PIDPosA(&InputPosA, &OutputPosA, &SetPointPosA,KP_P_A,KI_P_A,KD_P_A, DIRECT);
PID PIDSpeedB(&InputSpeedB, &OutputSpeedB, &SetPointSpeedB,KP_S_B,KI_S_B,KD_S_B, DIRECT);
PID PIDPosB(&InputPosB, &OutputPosB, &SetPointPosB,KP_P_B,KI_P_B,KD_P_B, DIRECT);

unsigned long lastTimeA = 0; //in ms. time in which motor A speed calculated for the last time.
unsigned long lastTimeB = 0; //in ms. time in which motor B speed calculated for the last time.
int timeInterval = 100;//milliseconds
double speedA = 0;
double speedB = 0;
int PWMA_val = 0; //Initial PWMA value.
int PWMB_val = 0; //Initial PWMB value.

int count = 0; //This intger used for DisplayData() function.
