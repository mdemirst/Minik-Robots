#include "Arduino.h"
#include "IslMotorControl.h"

long JOB_1ms = 0;
long JOB_10ms = 0;
long JOB_100ms = 0;
long JOB_1000ms = 0;

void flagUpdate()
{
  static long flag_count_1ms = 0;
  static long flag_count_10ms = 0;
  static long flag_count_100ms = 0;
  static long flag_count_1000ms = 0;

  flag_count_1ms++;
  flag_count_10ms++;
  flag_count_100ms++;
  flag_count_1000ms++;

  if (flag_count_1ms % 1 == 0)
  {
    flag_count_1ms = 0;
    JOB_1ms = true;
  }
  if (flag_count_10ms % 10 == 0)
  {
    flag_count_10ms = 0;
    JOB_10ms = true;
  }
  if (flag_count_100ms % 100 == 0)
  {
    flag_count_100ms = 0;
    JOB_100ms = true;
  }
  if (flag_count_1000ms % 1000 == 0)
  {
    flag_count_1000ms = 0;
    JOB_1000ms = true;
  }
}

bool IslMotorControl::job1ms(){ return JOB_1ms; }
bool IslMotorControl::job10ms(){ return JOB_10ms; }
bool IslMotorControl::job100ms(){ return JOB_100ms; }
bool IslMotorControl::job1000ms(){ return JOB_1000ms; }
void IslMotorControl::jobDone1ms(){ JOB_1ms = false; }
void IslMotorControl::jobDone10ms(){ JOB_10ms = false; }
void IslMotorControl::jobDone100ms(){ JOB_100ms = false; }
void IslMotorControl::jobDone1000ms(){ JOB_1000ms = false; }

IslMotorControl::IslMotorControl()
{
  pinMode(PIN_CNY70_1, INPUT);
  pinMode(PIN_CNY70_2, INPUT);
  pinMode(PIN_CNY70_3, INPUT);
  
  pinMode(PIN_MOT1_E1, INPUT);
  pinMode(PIN_MOT1_E2, INPUT);
  digitalWrite(PIN_MOT1_E1, HIGH); 
  digitalWrite(PIN_MOT1_E2, HIGH);
  
  pinMode(PIN_MOT2_E1, INPUT);
  pinMode(PIN_MOT2_E2, INPUT);
  digitalWrite(PIN_MOT2_E1, HIGH);
  digitalWrite(PIN_MOT2_E2, HIGH);
  
  if(1 == USE_SERIAL2)
  {
    Serial2.begin(SERIAL_BAUDRATE);
  }
  
  if(1 == USE_SERIAL2)
  {
    Serial3.begin(SERIAL_BAUDRATE);
  }
  
  Serial.begin(SERIAL_BAUDRATE);
  
  pinMode(PIN_SHARP1, INPUT);
  pinMode(PIN_SHARP2, INPUT);
  
  pinMode(PIN_BTN1, INPUT);
  pinMode(PIN_BTN2, INPUT);
  pinMode(PIN_BTN3, INPUT);
  
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  
  mot1Enc = new Encoder(PIN_MOT1_E1, PIN_MOT1_E2);
  mot2Enc = new Encoder(PIN_MOT2_E1, PIN_MOT2_E2);

  mot1Speed = 0;
  mot2Speed = 0;

  mot1CountOld = 0;
  mot2CountOld = 0;
  mot1Count = 0;
  mot2Count = 0;

  mot1PWM = 0;
  mot2PWM = 0;
	
  PIDSpeed1   = new PID(&mot1Speed, &mot1PIDPWMOut, &mot1TargetSpeed, KP_S_1, KI_S_1, KD_S_1, DIRECT);
  PIDPos1     = new PID(&mot1Count, &mot1PIDPWMOut, &mot1TargetPos, KP_P_1, KI_P_1, KD_P_1, DIRECT);
  PIDSpeed2   = new PID(&mot2Speed, &mot2PIDPWMOut, &mot2TargetSpeed, KP_S_2, KI_S_2, KD_S_2, DIRECT);
  PIDPos2     = new PID(&mot2Count, &mot2PIDPWMOut, &mot2TargetPos, KP_P_2, KI_P_2, KD_P_2, DIRECT);
  
  setPosParam((char)0x00, INIT_KP_P_1, INIT_KI_P_1, INIT_KD_P_1);
	setPosParam((char)0x01, INIT_KP_P_2, INIT_KI_P_2, INIT_KD_P_2);
	setSpeedParam((char)0x00, INIT_KP_S_1, INIT_KI_S_1, INIT_KD_S_1);
	setSpeedParam((char)0x01, INIT_KP_S_2, INIT_KI_S_2, INIT_KD_S_2);

  PIDSpeed1->SetMode(AUTOMATIC);
  PIDSpeed1->SetOutputLimits(-255, 255);
  PIDSpeed1->SetSampleTime(PID_SAMPLE_TIME);
  PIDSpeed2->SetMode(AUTOMATIC);
  PIDSpeed2->SetOutputLimits(-255, 255);
  PIDSpeed2->SetSampleTime(PID_SAMPLE_TIME);

  PIDPos1->SetMode(AUTOMATIC);
  PIDPos1->SetOutputLimits(-255, 255);
  PIDPos1->SetSampleTime(PID_SAMPLE_TIME);
  PIDPos2->SetMode(AUTOMATIC);
  PIDPos2->SetOutputLimits(-255, 255);
  PIDPos2->SetSampleTime(PID_SAMPLE_TIME);

  pinMode(PIN_MOT1_BRAKE, OUTPUT);
  pinMode(PIN_MOT1_DIR, OUTPUT);
  pinMode(PIN_MOT2_PWM, OUTPUT);

  pinMode(PIN_MOT2_BRAKE, OUTPUT);
  pinMode(PIN_MOT2_DIR, OUTPUT);
  pinMode(PIN_MOT2_PWM, OUTPUT);

  isSpeedPIDActive = true;

  mot1TargetSpeed = 0;
  mot2TargetSpeed = 0;
  mot1TargetPos = 0;
  mot2TargetPos = 0;

  prevTime = 0;
  prevPIDTime = 0;
  prevSensorTime = 0;

  // Initialize the send buffer that we will use to send data
  sendBuffer.init(250);

  // Keeps serial command receive state - e.g. waits for start bytes, data bytes etc..
  recvState = char(RECV_START_BYTES);

  t = new Timer();

  t->every(1, flagUpdate);
}

void IslMotorControl::sendPeriodicCounter()
{
  int packetLength = 5 + PROTOCOL_CONTROL_SIZE
      + PROTOCOL_N_FOOTER;
  char pack[30];

  pack[0] = PROTOCOL_START_0;
  pack[1] = PROTOCOL_START_1;
  pack[2] = PROTOCOL_START_2;
  pack[3] = 0x04;   //Slave
  pack[4] = CMD_GET_COUNTER;
  pack[5] = 0x11;
  pack[6] = DEVICEID; //receiver
  pack[7] = 0x00;
  pack[8] = 0x00;

 
  long temp = 0x00000000;

  temp = (long)getMotorCounter(0x00);

  pack[PROTOCOL_DATA_POS + 1] = char(temp >> 24);
  pack[PROTOCOL_DATA_POS + 2] = char(temp >> 16);
  pack[PROTOCOL_DATA_POS + 3] = char(temp >> 8);
  pack[PROTOCOL_DATA_POS + 4] = char(temp);
  packetLength = 5;

  // recalculate the check sums
  refreshPack(pack);

  //sendBuffer.clear();
  Serial.write(pack, packetLength + PROTOCOL_CONTROL_SIZE + PROTOCOL_N_FOOTER);
}

double IslMotorControl::zeroAbsorb(double val)
{
  if (val > -0.00001 && val < 0.00001)
  {
    if (val >= 0)
      val = 0.00001;
    else
      val = -0.00001;
  }

  return val;
}

void IslMotorControl::speedPIDUpdate()
{

  PIDSpeed1->Compute();

  mot1PWM = constrain(mot1PWM + mot1PIDPWMOut, -1 * PWM_MAX, PWM_MAX);

  /*
  Serial.print(mot1Speed);
  Serial.print(" ");
  Serial.print(mot1PIDPWMOut);
  Serial.print(" ");
  Serial.print(mot1TargetSpeed);
  */

  if (mot1PWM > 0)
  {
    digitalWrite(PIN_MOT1_DIR, HIGH);
    if (fabs(mot1PWM) < MIN_PWM_LIM)
      analogWrite(PIN_MOT1_PWM, 0);
    else
      analogWrite(PIN_MOT1_PWM, mot1PWM);
  }
  else
  {
    digitalWrite(PIN_MOT1_DIR, LOW);
    if (fabs(mot1PWM) < MIN_PWM_LIM)
      analogWrite(PIN_MOT1_PWM, 0);
    else 
      analogWrite(PIN_MOT1_PWM, -1 * mot1PWM);
  }


  PIDSpeed2->Compute();

  mot2PWM = constrain(mot2PWM + mot2PIDPWMOut, -1 * PWM_MAX, PWM_MAX);
  /*
  Serial.print(" ");
  Serial.print(mot2Speed);
  Serial.print(" ");
  Serial.print(mot2PIDPWMOut);
  Serial.print(" ");
  Serial.println(mot2TargetSpeed);
  */

  if (mot2PWM > 0)
  {
    digitalWrite(PIN_MOT2_DIR, HIGH);
    if (fabs(mot2PWM) < MIN_PWM_LIM)
      analogWrite(PIN_MOT2_PWM, 0);
    else
      analogWrite(PIN_MOT2_PWM, mot2PWM);
  }
  else
  {
    digitalWrite(PIN_MOT2_DIR, LOW);
    if (fabs(mot2PWM) < MIN_PWM_LIM)
      analogWrite(PIN_MOT2_PWM, 0);
    else
      analogWrite(PIN_MOT2_PWM, -1 * mot2PWM);
  }

}

void IslMotorControl::posPIDUpdate()
{

	/* Adaptive speed limit is used for controlling the speed in position control mode
	 * When position control is enabled, speed cannot be controlled and
	 * sometimes speed increases too much and it may not possible to stop motor
	 * immediately when speed is high. Therefore, we limit PWM
	 * so that less torque applied when speed is high.
	 * PWM_out |__
	 *         |  \
	 *         |   \ <--- PWM limit is set inversely prop to speed
	 *         |    \___
	 *         |__________
	 *                   Speed
	 */
	float PWM_max_adaptive_1 = constrain(PWM_MAX - (fabs(mot1Speed) * ADAPTIVE_SPEED_MULTIPLIER),PWM_MIN,PWM_MAX);
	float PWM_max_adaptive_2 = constrain(PWM_MAX - (fabs(mot2Speed) * ADAPTIVE_SPEED_MULTIPLIER),PWM_MIN,PWM_MAX);

    PIDPos1->Compute();
    mot1PWM = constrain(mot1PIDPWMOut, -1 * PWM_max_adaptive_1, PWM_max_adaptive_1);
    
    if (mot1PWM > 0)
    {
      digitalWrite(PIN_MOT1_DIR, HIGH);
      if (fabs(mot1PWM) < MIN_PWM_LIM)
        analogWrite(PIN_MOT1_PWM, 0);
      else
        analogWrite(PIN_MOT1_PWM, mot1PWM);
    }
    else
    {
      digitalWrite(PIN_MOT1_DIR, LOW);
      if (fabs(mot1PWM) < MIN_PWM_LIM)
        analogWrite(PIN_MOT1_PWM, 0);
      else
        analogWrite(PIN_MOT1_PWM, -1 * mot1PWM);
    }


    PIDPos2->Compute();
    mot2PWM = constrain(mot2PIDPWMOut, -1 * PWM_max_adaptive_2, PWM_max_adaptive_2);

    if (mot2PWM > 0)
    {
      digitalWrite(PIN_MOT2_DIR, HIGH);
      if (fabs(mot2PWM) < MIN_PWM_LIM)
        analogWrite(PIN_MOT2_PWM, 0);
      else
        analogWrite(PIN_MOT2_PWM, mot2PWM);
    }
    else
    {
      digitalWrite(PIN_MOT2_DIR, LOW);
      if (fabs(mot2PWM) < MIN_PWM_LIM)
        analogWrite(PIN_MOT2_PWM, 0);
      else
        analogWrite(PIN_MOT2_PWM, -1 * mot2PWM);
    }
}

void IslMotorControl::updateMotors()
{
  t->update();
  checkSerialCommand();

  unsigned long curTime = millis();
  unsigned long timeInterval = curTime - prevTime;
  unsigned long timeIntervalPID = curTime - prevPIDTime;
  unsigned long timeIntervalSensor = curTime - prevSensorTime;

  if (timeInterval > SPEED_SAMPLE_TIME)
  {
    prevTime = curTime;

    mot1Count = mot1Enc->read();
    mot2Count = mot2Enc->read();

    float mot1SpeedTemp = (mot1Count - mot1CountOld) * SPEED_SAMPLE_TIME / ((float)timeInterval);
    float mot2SpeedTemp = (mot2Count - mot2CountOld) * SPEED_SAMPLE_TIME / ((float)timeInterval);

		mot1Speed = (mot1Speed * (SPEED_FILTER_CONST-1) + mot1SpeedTemp ) / SPEED_FILTER_CONST;
		mot2Speed = (mot2Speed * (SPEED_FILTER_CONST-1) + mot2SpeedTemp ) / SPEED_FILTER_CONST;

    mot1CountOld = mot1Count;
    mot2CountOld = mot2Count;
  }
  
  if (timeIntervalSensor > DIST_SENSOR_SAMPLE_TIME)
  {
    prevSensorTime = curTime;

    distSensorL = analogRead(PIN_SHARP1);
    distSensorR = analogRead(PIN_SHARP2);
  }
  

  if (isSpeedPIDActive)
  {
    speedPIDUpdate();
  }
  else
  {
    posPIDUpdate();
  }
}

// Public Methods used in communication with motor card
void IslMotorControl::getSpeedParam(char motor, int* param){

  if (motor == (char)0x00){
    param[0] = KP_S_1*1000.0;
    param[1] = KI_S_1*1000.0;
    param[2] = KD_S_1*1000.0;
  }
  else{
    param[0] = KP_S_2*1000.0;
    param[1] = KI_S_2*1000.0;
    param[2] = KD_S_2*1000.0;
  }

}
void IslMotorControl::getPosParam(char motor, int* param){

  if (motor == (char)0x00){
    param[0] = KP_P_1*1000.0;
    param[1] = KI_P_1*1000.0;
    param[2] = KD_P_1*1000.0;
  }
  else{
    param[0] = KP_P_2*1000.0;
    param[1] = KI_P_2*1000.0;
    param[2] = KD_P_2*1000.0;
  }
}
void IslMotorControl::setSpeedParam(char motor, int p, int i, int d){
  if (motor == (char)0x00){
    KP_S_1 = p/1000.0;
    KI_S_1 = i/1000.0;
    KD_S_1 = d/1000.0;
    PIDSpeed1->SetTunings(KP_S_1, KI_S_1, KD_S_1);
  }
  else{
    KP_S_2 = p/1000.0;
    KI_S_2 = i/1000.0;
    KD_S_2 = d/1000.0;
    PIDSpeed2->SetTunings(KP_S_2, KI_S_2, KD_S_2);
  }
}
void IslMotorControl::setPosParam(char motor, int p, int i, int d){
  if (motor == (char)0x00){
    KP_P_1 = p/1000.0;
    KI_P_1 = i/1000.0;
    KD_P_1 = d/1000.0;
    PIDPos1->SetTunings(KP_P_1, KI_P_1, KD_P_1);
  }
  else{
    KP_P_2 = p/1000.0;
    KI_P_2 = i/1000.0;
    KD_P_2 = d/1000.0;
    PIDPos2->SetTunings(KP_P_2, KI_P_2, KD_P_2);
  }
}


long IslMotorControl::getMotorCounter(char motor){

  if (motor == (char)0x00)
    return mot1Count;
  else
    return mot2Count;
}
void IslMotorControl::setMotorCounter(char motor, long counterVal){

  isSpeedPIDActive = false;
  if (motor == (char)0x00){
    mot1TargetPos = counterVal;
  }
  else{
    mot2TargetPos = counterVal;
  }
}
long IslMotorControl::getMotorSpeed(char motor){
  if (motor == (char)0x00){
    return mot1Speed*SEC_MS / ((float)SPEED_SAMPLE_TIME);
  }
  else{
    return mot2Speed*SEC_MS / ((float)SPEED_SAMPLE_TIME);
  }
}
void IslMotorControl::setMotorSpeed(char motor, long speed){
  isSpeedPIDActive = true;

  speed = speed*SPEED_SAMPLE_TIME / ((float)SEC_MS);

  if (motor == (char)0x00){
    mot1TargetSpeed = speed;
  }
  else{
    mot2TargetSpeed = speed;
  }
}
void IslMotorControl::setMotorSpeeds(long speed1, long speed2){
  isSpeedPIDActive = true;
  mot1TargetSpeed = speed1*SPEED_SAMPLE_TIME / ((float)SEC_MS);
  mot2TargetSpeed = speed2*SPEED_SAMPLE_TIME / ((float)SEC_MS);
}
void IslMotorControl::setMotorCounters(long counter1, long counter2){
  isSpeedPIDActive = false;
  mot1TargetPos = counter1;
  mot2TargetPos = counter2;
}


void IslMotorControl::respondPack(char* pack) {
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
    if (getPackedTask(pack) == (char)CMD_GET_SPEED) {
      temp = (long)getMotorSpeed(pack[PROTOCOL_MOTOR_POS]);

      int tempInt;
      if (temp < 0)
        tempInt = ((int)(temp*-1))*-1;
      else
        tempInt = temp;

      pack[PROTOCOL_DATA_POS + 1] = char((tempInt >> 8) & 0x00FF);
      pack[PROTOCOL_DATA_POS + 2] = char((tempInt) & 0x00FF);
      packetLength = 3;
    }

    // setSpeed command
    else if (getPackedTask(pack) == (char)CMD_SET_SPEED) {
      // setSpeed
      setMotorSpeed(pack[PROTOCOL_MOTOR_POS],
        (int)(((((int)pack[PROTOCOL_DATA_POS + 1]) << 8) & 0xFF00) | ((((int)pack[PROTOCOL_DATA_POS + 2])) & 0x00FF)));

      packetLength = 1;

    }

    // getMotorCounter command
    else if (getPackedTask(pack) == (char)CMD_GET_COUNTER) {
      temp = (long)getMotorCounter(pack[PROTOCOL_MOTOR_POS]);

      pack[PROTOCOL_DATA_POS + 1] = char(temp >> 24);
      pack[PROTOCOL_DATA_POS + 2] = char(temp >> 16);
      pack[PROTOCOL_DATA_POS + 3] = char(temp >> 8);
      pack[PROTOCOL_DATA_POS + 4] = char(temp);
      packetLength = 5;
    }

    // setMotorCounter command
    else if (getPackedTask(pack) == (char)CMD_SET_COUNTER) {
      
      setMotorCounter(pack[PROTOCOL_MOTOR_POS], (long)(
        ((((long)pack[PROTOCOL_DATA_POS + 1]) << 24) & 0xFF000000) |
        ((((long)pack[PROTOCOL_DATA_POS + 2]) << 16) & 0x00FF0000) |
        ((((long)pack[PROTOCOL_DATA_POS + 3]) << 8 ) & 0x0000FF00) |
        ((((long)pack[PROTOCOL_DATA_POS + 4])      ) & 0x000000FF)  ));
		
      packetLength = 1;
    }

    // get speed PID Coef command
    else if (getPackedTask(pack) == (char)CMD_GET_SPEED_PID) {
      getSpeedParam(pack[PROTOCOL_DATA_POS], tempIntArray);

      pack[PROTOCOL_DATA_POS + 1] = (char)tempIntArray[0];
      pack[PROTOCOL_DATA_POS + 2] = (char)tempIntArray[1];
      pack[PROTOCOL_DATA_POS + 3] = (char)tempIntArray[2];
      packetLength = 4;
    }
    // set speed PID Coef command
    else if (getPackedTask(pack) == (char)CMD_SET_SPEED_PID) {
      // setPIDCoef, for speed PID
      tempArray[0] = pack[PROTOCOL_DATA_POS + 1];
      tempArray[1] = pack[PROTOCOL_DATA_POS + 2];
      tempArray[2] = pack[PROTOCOL_DATA_POS + 3];

      setSpeedParam(pack[PROTOCOL_DATA_POS], tempArray[0], tempArray[1], tempArray[2]);
      //setSpeedPIDCoef(pack[PROTOCOL_DATA_POS], tempArray);
      packetLength = 1;
    }
    // get position PID Coef command
    else if (getPackedTask(pack) == (char)CMD_GET_POS_PID) {
      getPosParam(pack[PROTOCOL_DATA_POS], tempIntArray);

      pack[PROTOCOL_DATA_POS + 1] = (char)tempIntArray[0];
      pack[PROTOCOL_DATA_POS + 2] = (char)tempIntArray[1];
      pack[PROTOCOL_DATA_POS + 3] = (char)tempIntArray[2];
      packetLength = 4;
    }
    // set position PID Coef command
    else if (getPackedTask(pack) == (char)CMD_SET_POS_PID) {
      // setPIDCoef, for position PID
      tempArray[0] = pack[PROTOCOL_DATA_POS + 1];
      tempArray[1] = pack[PROTOCOL_DATA_POS + 2];
      tempArray[2] = pack[PROTOCOL_DATA_POS + 3];

      setPosParam(pack[PROTOCOL_DATA_POS], tempArray[0], tempArray[1], tempArray[2]);
      packetLength = 1;
    }

    // setMotorCounters command
    else if (getPackedTask(pack) == (char)CMD_SET_COUNTERS) {
      setMotorCounter((char)0x00, (long)(
        ((((long)pack[PROTOCOL_DATA_POS + 0]) << 24) & 0xFF000000) |
        ((((long)pack[PROTOCOL_DATA_POS + 1]) << 16) & 0x00FF0000) |
        ((((long)pack[PROTOCOL_DATA_POS + 2]) << 8) & 0x0000FF00) |
        ((((long)pack[PROTOCOL_DATA_POS + 3])) & 0x000000FF)));

      setMotorCounter((char)0x01, (long)(
        ((((long)pack[PROTOCOL_DATA_POS + 4]) << 24) & 0xFF000000) |
        ((((long)pack[PROTOCOL_DATA_POS + 5]) << 16) & 0x00FF0000) |
        ((((long)pack[PROTOCOL_DATA_POS + 6]) << 8) & 0x0000FF00) |
        ((((long)pack[PROTOCOL_DATA_POS + 7])) & 0x000000FF)));
        
      packetLength = 1;
    }

    // setSpeeds command
    else if (getPackedTask(pack) == (char)CMD_SET_SPEEDS) {

      setMotorSpeed((char)0x00,
        (int)(((((int)pack[PROTOCOL_DATA_POS + 0]) << 8)&0xFF00) | ((((int)pack[PROTOCOL_DATA_POS + 1]))&0x00FF) ));

      setMotorSpeed((char)0x01,
        (int)(((((int)pack[PROTOCOL_DATA_POS + 2]) << 8) & 0xFF00) | ((((int)pack[PROTOCOL_DATA_POS + 3])) & 0x00FF)));
       
      packetLength = 1;
    }
	
	// getDistances command
    else if (getPackedTask(pack) == (char)CMD_GET_DISTS) {

		temp = (int) distSensorR;

		pack[PROTOCOL_DATA_POS + 1] = char(temp >> 8) & 0xFF;
		pack[PROTOCOL_DATA_POS + 2] = char(temp) & 0xFF;
		
		temp = (int) distSensorL;

		pack[PROTOCOL_DATA_POS + 3] = char(temp >> 8) & 0xFF;
		pack[PROTOCOL_DATA_POS + 4] = char(temp) & 0xFF; 
		
		packetLength = 5;
    }
	
	// getLines command
    else if (getPackedTask(pack) == (char)CMD_GET_LINES) {

		pack[PROTOCOL_DATA_POS + 1] = char(lineR) & 0xFF;
		pack[PROTOCOL_DATA_POS + 2] = char(lineM) & 0xFF;
		pack[PROTOCOL_DATA_POS + 3] = char(lineL) & 0xFF;
				
		packetLength = 4;
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
    Serial.write(pack, packetLength + PROTOCOL_CONTROL_SIZE + PROTOCOL_N_FOOTER);

  }
}

void IslMotorControl::refreshPack(char* pack) {
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


char IslMotorControl::isPackValid(char* pack) {
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

void IslMotorControl::getPackedId(char* pack, char* id) {
  id[0] = pack[6];
  id[1] = pack[7];
  id[2] = pack[8];
  return;
}

boolean IslMotorControl::checkCardID(char *pack) {
  if ((pack[6] == DEVICEID) && (pack[7] == 0x00) && (pack[8] == 0x00))
    return true;
  else
    return false;
}

char IslMotorControl::getPackedTask(char* pack) {
  return pack[4];
}

char IslMotorControl::getPackedLength(char* pack) {
  return pack[5];
}

char IslMotorControl::getPackedSender(char* pack) {
  return pack[3];
}

void IslMotorControl::setPackHeader(char* pack, char task, char packetLength, char* receiver) {
  pack[3] = 0x04; // Designates sender is slave 
  pack[4] = task;
  pack[5] = packetLength;
  pack[6] = receiver[0];
  pack[7] = receiver[1];
  pack[8] = receiver[2];
}

void IslMotorControl::checkSerialCommand()
{
  while (Serial.available())
  {

    //Checks if data connection is timed out. If so, it return to wait for start bytes state
    if (((recvState == char(RECV_HEADER_BYTES) || (recvState == char(RECV_DATA_BYTES)))) &&
      (nrRecvdBytes > 0) &&
      ((millis() - lastRecvTime) > TIMEOUT_VAL))
    {
      recvState = char(RECV_START_BYTES);
      nrRecvdBytes = 0;
    }

    //Waits for three start bytes
    if (recvState == char(RECV_START_BYTES))
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
    else if (recvState == char(RECV_HEADER_BYTES))
    {

      pack[PROTOCOL_CHK_POS + nrRecvdBytes] = char(Serial.read());
      nrRecvdBytes = nrRecvdBytes + 1;


      if ((nrRecvdBytes + PROTOCOL_CHK_POS) == PROTOCOL_DATA_POS)
      {

        nrRecvdBytes = 0;
        recvState = char(RECV_DATA_BYTES);
      }
    }
    //Waits for data bytes
    else if (recvState == char(RECV_DATA_BYTES))
    {


      pack[PROTOCOL_DATA_POS + nrRecvdBytes] = char(Serial.read());
      nrRecvdBytes++;

      if (nrRecvdBytes == int(pack[5]) - PROTOCOL_CONTROL_SIZE)
      {
        recvState = char(RECV_START_BYTES);
        nrRecvdBytes = 0;

        if (isPackValid(pack) == (char)SUCCESS)
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

void IslMotorControl::toggleYellow()
{
  if (digitalRead(PIN_LED1))
  {
    digitalWrite(PIN_LED1, LOW);
  }
  else
  {
    digitalWrite(PIN_LED1, HIGH);
  }
}
void IslMotorControl::toggleGreen()
{
  if (digitalRead(PIN_LED2))
  {
    digitalWrite(PIN_LED2, LOW);
  }
  else
  {
    digitalWrite(PIN_LED2, HIGH);
  }
}
