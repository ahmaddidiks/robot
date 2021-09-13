#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <ESP32Encoder.h>
#include <Ticker.h>

#define encoder_ppr 6000
#define wheel_rad 0.045
#define wheel_dist 0.131
#define step_per_rot 1600
#define acc_const 0.05
#define min_speed_tolerance 0.01

#define EncR_A 2
#define EncR_B 4
#define EncL_A 18
#define EncL_B 19

#define en_pin_R 35
#define dir_pin_R 32
#define step_pin_R 33
#define en_pin_L 25
#define dir_pin_L 26
#define step_pin_L 27

#define USE_ROS

ESP32Encoder encoderL;
ESP32Encoder encoderR;
Ticker KinematicTimer;
Ticker ROSTimer;

volatile float angSpeedL, angSpeedR;
volatile float theta=0, pos_x=0, pos_y=0;
float speed_L_target = 0, speed_L_current = 0;
float speed_R_target = 0, speed_R_current = 0;
unsigned long counter_L = 0, counter_R = 0;
unsigned long counter_target_L = 0, counter_target_R = 0;
unsigned long lastT_L = 0, lastT_R = 0;
unsigned long lastT_accControl = 0, counter_accControl = 0;
bool signal_stats_L = LOW, signal_stats_R = LOW;


void cmd_vel_handler(const geometry_msgs::Twist& cmd_vel) {
  inverse_kinematic(cmd_vel.linear.x, cmd_vel.angular.z);
}


#ifdef USE_ROS
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_handler);
geometry_msgs::TransformStamped tf_msg;
tf::TransformBroadcaster TF_broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";
#endif


void setup() {
  setCpuFrequencyMhz(240);
  
  #ifdef USE_ROS
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  TF_broadcaster.init(nh);
  #endif

  ESP32Encoder::useInternalWeakPullResistors=UP;
  encoderL.attachFullQuad(EncL_A, EncL_B);
  encoderR.attachFullQuad(EncR_A, EncR_B);
  
  pinMode(en_pin_R, OUTPUT);
  pinMode(dir_pin_R, OUTPUT);
  pinMode(step_pin_R, OUTPUT);
  pinMode(en_pin_L, OUTPUT);
  pinMode(dir_pin_L, OUTPUT);
  pinMode(step_pin_L, OUTPUT);
  
  digitalWrite(en_pin_L, LOW);
  digitalWrite(en_pin_R, LOW);

  KinematicTimer.attach_ms(10, kinematicHandler);
  ROSTimer.attach_ms(100, ROSHandler);
}


void loop() {
  counter_L = micros() - lastT_L;
  counter_R = micros() - lastT_R;
  counter_accControl = micros() - lastT_accControl;

  if (counter_accControl >= 10000) {
    speed_L_current += (speed_L_target - speed_L_current) * acc_const;
    speed_R_current += (speed_R_target - speed_R_current) * acc_const;

    if (abs(speed_L_current) > min_speed_tolerance) {
      counter_target_L = getCounter(speed_L_current) / 2;
    }
    else {
      counter_target_L = 0;
    }

    if (abs(speed_R_current) > min_speed_tolerance) {
      counter_target_R = getCounter(speed_R_current) / 2;
    }
    else {
      counter_target_R = 0;
    }

    if (speed_L_current > 0) digitalWrite(dir_pin_L, HIGH);
    else digitalWrite(dir_pin_L, LOW);
    if (speed_R_current > 0) digitalWrite(dir_pin_R, HIGH);
    else digitalWrite(dir_pin_R, LOW);

    lastT_accControl = micros();
  }

  if ((counter_target_L != 0 ) && (counter_L >= counter_target_L)) {
    lastT_L = micros();
    signal_stats_L = !signal_stats_L;
    digitalWrite(step_pin_L, signal_stats_L);
  }

  if ((counter_target_R != 0 ) && (counter_R >= counter_target_R)) {
    lastT_R = micros();
    signal_stats_R = !signal_stats_R;
    digitalWrite(step_pin_R, signal_stats_R);
  }
}


void ROSHandler() {
  #ifdef USE_ROS
  tf_msg.header.frame_id = odom;
  tf_msg.child_frame_id = base_link;
  tf_msg.transform.translation.x = pos_x;
  tf_msg.transform.translation.y = pos_y;
  tf_msg.transform.rotation = tf::createQuaternionFromYaw(theta);
  tf_msg.header.stamp = nh.now();
  TF_broadcaster.sendTransform(tf_msg);
  
  nh.spinOnce();
  #endif
}


void kinematicHandler() {
  angSpeedL = (float)encoderL.getCount() * (2*PI) / encoder_ppr;
  encoderL.clearCount();
  angSpeedR = -(float)encoderR.getCount() * (2*PI) / encoder_ppr;
  encoderR.clearCount();

  theta += (wheel_rad/wheel_dist) * (angSpeedR - angSpeedL) / 2;
  pos_x += (wheel_rad/2) * (angSpeedR + angSpeedL) * cos(theta);
  pos_y += (wheel_rad/2) * (angSpeedR + angSpeedL) * sin(theta);
}


void inverse_kinematic(float linear, float angular) {
  speed_L_target = linear / wheel_rad + angular * wheel_dist / wheel_rad;
  speed_R_target = - linear / wheel_rad + angular * wheel_dist / wheel_rad;
}


float getCounter(float wheel_speed) {
  return 2 * PI / abs(wheel_speed) / step_per_rot * 1000000;
}


float getSpeed(float counter) {
  return 2 * PI / counter / 1000000 * step_per_rot;
}


float rad2deg(float rad) {
  return rad * 57.2957795;
}


float deg2rad(float deg) {
  return deg / 57.2957795;
}
