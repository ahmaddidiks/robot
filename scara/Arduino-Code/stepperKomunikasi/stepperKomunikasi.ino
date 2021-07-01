#include <ros.h>
#include <scara/stepper.h>

#define stepper_num 3
#define vel 720
#define ppr 200

// defines pins numbers
const int STEP[5] = {23, 21, 18, 27, 32};
const int DIR[5]  = {22, 19, 5, 14, 33};
#define enable 25

float _velSet[stepper_num];
long _pulseCount[stepper_num];
long _pulseCountTarget[stepper_num];
long _stepperLastT[stepper_num];
bool _stepperState[stepper_num];

ros::NodeHandle nh;
scara::stepper stepper_msg;

void stepper_cb(const scara::stepper& stepper_msg){
  if(stepper_msg.enableStepper == false) digitalWrite(enable, HIGH);
  else digitalWrite(enable, LOW);

  for (int i=0; i < stepper_num; i++) {
    _pulseCountTarget[i] = stepper_msg.stepperPostList[i] / 360 * ppr * 2;
  }
}

ros::Subscriber<scara::stepper> sub("stepper_test_gui", stepper_cb);

void setup(){
  pinMode(2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  for (int i=0; i<5;i++){
    pinMode(STEP[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
  }
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH); //activate stepper
}

void loop(){
  nh.spinOnce();

  for (int i=0; i < stepper_num; i++) {
    stepperControl(i);
  }
}

long getDelayDuration(float degPerSec) {
  if (degPerSec == 0) return 0;
  float pulsePerSec = degPerSec / 360 * ppr;
  return (0.5 / pulsePerSec * 1000000);
}

void stepperControl(int id) {
  if (_pulseCount[id] < _pulseCountTarget[id]) _velSet[id] = vel;
  else if (_pulseCount[id] > _pulseCountTarget[id]) _velSet[id] = -vel;
  else _velSet[id] = 0;

  if ((_velSet[id] != 0) && (micros() - _stepperLastT[id] >= getDelayDuration(abs(_velSet[id])))) {
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
