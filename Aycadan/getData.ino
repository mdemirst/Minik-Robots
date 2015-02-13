void getData(){
  //char data[intSize];
  xor2=xor1;
  //for(i=0;i<intSize;i++){
  for(int i=0;i<4;i++){
    input=Packet.pop();
    data[i]=input;
    xor2=xor2^input;
  }
  state=5;
  delay(1000);
}
