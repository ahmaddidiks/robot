void get_counter(){
  encoder.count[0] = -encoder0.getCount();// * 90 / 2400 ;
  encoder.count[1] = -encoder1.getCount();// * 9.257142857 / 2400;
  encoder.count[2] = -encoder2.getCount();// * 88.76712328/2400;
  encoder.count[3] = encoder3.getCount();// * 53.114754098/2400;
}

void setEncoder(int value){
  encoder0.setCount(value);
  encoder1.setCount(value);
  encoder2.setCount(value);
  encoder3.setCount(value);
}
//void print_value(){
//  for (int i=0; i<4;i++){
////  encoder_msg.encoderPostList[i] = count[i];
//  if(i==3) Serial.println(String(i) + " : " +  String(count[i]));
//  else Serial.print(String(i) + " : " +  String(count[i]) + " ");
//  }
//
//}
