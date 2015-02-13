void informations(){
  for(int i=0;i<6;i++){
    input=Packet.pop();
    xor1=xor1^input;
    if(i==0){
      sender=input;
    }
    if(i==1){
      task=input;
    }
    if(i==2){
      packetSize=input;
      intSize=input;
      //Serial.println(intSize);
    }
    if(i==3){
      receiver1=input;
    }
    if(i==4){
      receiver2=input;
    }
    if(i==5){
      receiver3=input;
    }
  }
  state=4;
  delay(1000);
}
