#include <ros.h>
#include <scara_like/stepper.h>
#include <Servo.h>
Servo servo;

#define stepper_num 4
//velocity
//const int vel[4] {25, 3, 24, 20};
float vel[stepper_num];
//PPR
const int  ppr[stepper_num] {720, 7000, 730, 1220};

// defines pins numbers
const int STEP[stepper_num] = {23, 21, 18, 26};
const int DIR[stepper_num]  = {22, 19, 5, 14};

#define EN 25

float _velSet[stepper_num];
long _pulseCount[stepper_num];
long _pulseCountTarget[stepper_num];
long _stepperLastT[stepper_num];
bool _stepperState[stepper_num];
float tetha[stepper_num];
long waktu;

ros::NodeHandle nh;
scara_like::stepper msg;

void stepper_cb(const scara_like::stepper &joints){
  if (joints.enable == false) digitalWrite(EN, HIGH);
  else digitalWrite(EN, LOW);
  servo.write(joints.gripper);
  
  for (int i=0; i < stepper_num; i++) {
    vel[i] = joints.speed[i]; //set stepper velocity from joint state
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

ros::Subscriber<scara_like::stepper> joints_sub("real_robot", stepper_cb);

void setup(){
  nh.initNode();
  nh.subscribe(joints_sub);

  for (int i=0; i<4;i++){
    pinMode(STEP[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
  }
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH); //diactivate stepper
  servo.attach(2); //pin servo D2
}

void loop(){
  nh.spinOnce();

  for (int i=0; i < stepper_num; i++) {
    stepperControl(i);
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
