#include <QueueArray.h>
QueueArray <char> Packet;
char input,sender,task,packetSize,receiver1,receiver2,receiver3,data[100];
boolean trueness=false;
int state=0, xor1=0xA3, xor2, intSize;

void setup(){
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}

void loop(){
  if(!Packet.isEmpty()){
    //Serial.println("giriyor.");
    if(state==5){
      //xorControl();
      conclusion();
      state=0;
    }
    else if(state==4){
      if(Packet.count()>=4){
        getData();
        digitalWrite(13,HIGH);
      }
    }
    else if(state==3){
      if(Packet.count()>=6){
        informations();
      }
    }
    else if(state<=2){
      digitalWrite(13,LOW);
      controller();
    }
    Serial.println(state);
  }
  delay(1000);
}

void serialEvent(){
    int sizeAva = Serial.available();
    //if (sizeAva>0){
    for(int i = 0; i < sizeAva; i++){
      Packet.push(Serial.read()); 
      //trueness=true;    
    }
} 
