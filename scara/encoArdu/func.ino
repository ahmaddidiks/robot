void getDegree(){
  deg0 = encoder0.getCount()/FACTOR_0;
  deg1 = encoder1.getCount()/FACTOR_1;
  deg2 = encoder2.getCount()/FACTOR_2;
  deg3 = encoder3.getCount()/FACTOR_3;
  deg4 = encoder4.getCount()/FACTOR_4;
}

void setEncoder(int value){
  encoder0.setCount(value);
  encoder1.setCount(value);
  encoder2.setCount(value);
  encoder3.setCount(value);
  encoder4.setCount(value);
}
