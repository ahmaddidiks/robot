#include <ros.h>
#include <scara/stepper.h>

#define stepper_num 4
//#define vel 180
const int vel[4] {25, 24, 20, 3};
//#define ppr 360
int  ppr[4] {720, 730, 1220, 7000};
// #define pprLink1 800
// #define pprLink2 811
// #define pprLink3 1357

// defines pins numbers
const int STEP[4] = {23, 21, 18, 26};
const int DIR[4]  = {22, 19, 5, 14};
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
    _pulseCountTarget[i] = stepper_msg.stepperPostList[i] / 360 * ppr[i] * 2;
  }
}

ros::Subscriber<scara::stepper> sub("invers_kinematics_gui", stepper_cb);

void setup(){
  pinMode(2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  for (int i=0; i<4;i++){
    pinMode(STEP[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
  }
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH); //diactivate stepper
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
