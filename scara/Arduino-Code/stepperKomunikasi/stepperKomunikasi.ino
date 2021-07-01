#include <ros.h>
#include <scara/stepper.h>

ros::NodeHandle nh;
scara::stepper stepper_msg;

void stepper_cb(const scara::stepper& stepper_msg){
  if(stepper_msg.enableStepper == false) digitalWrite(2, LOW);
  else digitalWrite(2, HIGH);
  
}

ros::Subscriber<scara::stepper> sub("stepper_test_gui", stepper_cb);

void setup(){
  pinMode(2, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
