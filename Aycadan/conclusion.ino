void conclusion(){
  Serial.println(sender);
  Serial.println(task);
  Serial.println(packetSize);
  Serial.println(receiver1);
  Serial.println(receiver2);
  Serial.println(receiver3);
  for(int i=0;i<4;i++){
    Serial.println(data[i]);
  }
  delay(1000);
}
