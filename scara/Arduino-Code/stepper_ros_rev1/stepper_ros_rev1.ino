#include <ros.h>
#include <scara/stepperResponse.h>
#include <scara/stepperTask.h>

#define stepper_num 4

// defines pins numbers
const int STEP[4] = {23, 21, 18, 26};
const int DIR[4]  = {22, 19, 5, 14};
#define EN 25

#define DELAY_PER_STEP 700

//for storing last value of stepperTask (what the value of stepper's step right now)
int stepper_count[stepper_num] = {0,0,0,0};

//store value from ROS
int stepper_target[stepper_num] = {0,0,0,0};

//stepper direction
int dir_set[stepper_num] = {0,0,0,0};

//last time
long stepper_last[stepper_num] = {0,0,0,0};

//stepper state
bool stepper_state[stepper_num];


ros::NodeHandle nh;
scara::stepperTask stepper_task_msg;
scara::stepperResponse stepper_response_msg;
ros::Publisher pub_stepper_response("stepper_response", &stepper_response_msg);

void stepper_cb(const scara::stepperTask &stepper_task_msg){
  if(stepper_task_msg.enable == false) digitalWrite(EN, HIGH);
  else digitalWrite(EN, LOW);

  for(int id=0; id < stepper_num; id++){
    stepper_target[id] = stepper_task_msg.stepperTaskList[id];
  } 
}
ros::Subscriber<scara::stepperTask> sub("invers_kinematics_gui_rev1", stepper_cb);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_stepper_response);

  stepper_response_msg.stepperResponseList = (int*)malloc(sizeof(int) * 4);
  stepper_response_msg.stepperResponseList_length = 4;
  
  for (int id=0; id<stepper_num;id++){
    pinMode(STEP[id], OUTPUT);
    pinMode(DIR[id], OUTPUT);
  }
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH); //diactivate stepper
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int id=0; id < stepper_num; id++) stepper_control(id);
  
  long last;
  if(millis() - last >= 10){
  pub_stepper_response.publish(&stepper_response_msg);
  nh.spinOnce();
  last = millis();
  }
}

void stepper_control(int id){
  if(stepper_count[id] < stepper_target[id]) dir_set[id] = 1;
  else if(stepper_count[id] > stepper_target[id]) dir_set[id] = -1;
  else dir_set[id] = 0;

  if((dir_set[id] != 0) && (micros() - stepper_last[id] >= DELAY_PER_STEP)){
    if (dir_set[id] < 0) {
      digitalWrite(DIR[id],HIGH);
      stepper_count[id]--;
    }
    else {
      digitalWrite(DIR[id], LOW);
      stepper_count[id]++;
    }
    digitalWrite(STEP[id], stepper_state[id]);
    stepper_state[id] = !stepper_state[id];
    stepper_response_msg.stepperResponseList[id] = stepper_count[id];
    stepper_last[id] = micros();
    
  }
}
