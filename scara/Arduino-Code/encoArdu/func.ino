void getDegree(){
  deg[0] = encoder0.getCount();//19200.00;
  deg[1] = encoder1.getCount();//FACTOR_1;
  deg[2] = encoder2.getCount();//FACTOR_2;
  deg[3] = encoder3.getCount();//FACTOR_3;
//  deg[4] = encoder4.getCount();//FACTOR_4;
}

void setEncoder(int value){
  encoder0.setCount(value);
  encoder1.setCount(value);
  encoder2.setCount(value);
  encoder3.setCount(value);
//  encoder4.setCount(value);
}
void print_value(){
  for (int i=0; i<4;i++){
  encoder_msg.encoderPostList[i] = deg[i];
  if(i==3) Serial.println(String(i) + " : " +  String(deg[i]));
  else Serial.print(String(i) + " : " +  String(deg[i]) + " ");
  }

}
