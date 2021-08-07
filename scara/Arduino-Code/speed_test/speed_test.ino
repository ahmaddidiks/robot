#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <scara/stepper.h>

#define stepper_num 4
//velocity
const int vel[4] {25, 3, 24, 20};
//float vel[stepper_num];
//PPR
const int  ppr[stepper_num] {720, 7000, 730, 1220};

// defines pins numbers
const int STEP[stepper_num] = {23, 21, 18, 26};
const int DIR[stepper_num]  = {22, 19, 5, 14};
#define enable 25

float _velSet[stepper_num];
long _pulseCount[stepper_num];
long _pulseCountTarget[stepper_num];
long _stepperLastT[stepper_num];
bool _stepperState[stepper_num];
float tetha[stepper_num];
long waktu;

ros::NodeHandle nh;
sensor_msgs::JointState joints;
std_msgs::Bool enableStepper;
scara::stepper fb_msg;
//fb_msg
ros::Publisher fb_pub("stepper_response", &fb_msg);

void enable_stepper_cb(const std_msgs::Bool &enableStepper){
//  if(enableStepper.data == false) digitalWrite(enable, HIGH);
//  else digitalWrite(enable, LOW);
}
void stepper_cb(const sensor_msgs::JointState &joints){
  for (int i=0; i < stepper_num; i++) {
//    vel[i] = joints.velocity[i]; //set stepper velocity from joint state
    if(i==1){
      tetha[i] = joints.position[i] / 0.3 * 360; //mengubah ke derajat dari tinggi 0-0.3 meter
      _pulseCountTarget[i] = -tetha[i] / 360 * ppr[i] * 2;
    }
    else{
      tetha[i] = -joints.position[i] * 57.2958; // mengubah dari radian ke derajat
      _pulseCountTarget[i] = tetha[i] / 360 * ppr[i] * 2;
    }
  }
}

ros::Subscriber<sensor_msgs::JointState> joints_sub("joint_states", stepper_cb);
ros::Subscriber<std_msgs::Bool> enable_stepper_sub("enable_stepper", enable_stepper_cb);

void setup(){
  nh.initNode();
  nh.subscribe(joints_sub);
//  nh.subscribe(enable_stepper_sub);
  nh.advertise(fb_pub);
  
  fb_msg.stepperPostList = (float*)malloc(sizeof(float) * 4);
  fb_msg.stepperPostList_length = 4;

  for (int i=0; i<4;i++){
    pinMode(STEP[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
  }
  pinMode(enable, OUTPUT);
  digitalWrite(enable, LOW); //diactivate stepper
}

void loop(){
  nh.spinOnce();

  for (int i=0; i < stepper_num; i++) {
    stepperControl(i);
  }
  if(millis() - waktu >= 10){
  stepper_fb();
  
  waktu = millis();
  }
}

long getDelayDuration(float degPerSec, int id) {
  if (degPerSec == 0) return 0;
  float pulsePerSec = degPerSec / 360 * ppr[id];
  return (0.5 / pulsePerSec * 1000000);
}

void stepperControl(int id) {
  if (_pulseCount[id] < _pulseCountTarget[id]) _velSet[id] = vel[id];
  else if (_pulseCount[id] > _pulseCountTarget[id]) _velSet[id] = -vel[id];
  else _velSet[id] = 0;

  if ((_velSet[id] != 0) && (micros() - _stepperLastT[id] >= getDelayDuration(abs(_velSet[id]), id))) {
    if (_velSet[id] < 0) {
      digitalWrite(DIR[id], HIGH);
      _pulseCount[id]--;
    }
    else {
      digitalWrite(DIR[id], LOW);
      _pulseCount[id]++;
    }
    digitalWrite(STEP[id], _stepperState[id]);
    _stepperState[id] = !_stepperState[id];
    _stepperLastT[id] = micros();
  }
}

void stepper_fb(){
  fb_msg.stepperPostList[0] = _pulseCount[0] * 360 / ppr[0] / 2;
  fb_msg.stepperPostList[1] = _pulseCount[1] * 360 / ppr[1] / 2;
  fb_msg.stepperPostList[1] = fb_msg.stepperPostList[1] * 0.3 / 360;
  fb_msg.stepperPostList[2] = _pulseCount[2] * 360 / ppr[2] / 2;
  fb_msg.stepperPostList[3] = _pulseCount[3] * 360 / ppr[3] / 2;
  fb_pub.publish(&fb_msg);
}
