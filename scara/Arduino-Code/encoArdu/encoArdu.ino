#include "ESP32Encoder.h"

#include <ros.h>
#include <scara_like/encoder.h>

ESP32Encoder encoder0;
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

ros::NodeHandle nh;
scara_like::encoder encoder;
ros::Publisher pub_enc("sensor_data", &encoder);

void setup() {

  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder0.attachFullQuad(13, 12);
  encoder1.attachFullQuad(25, 14);
  encoder2.attachFullQuad(32, 33);
  encoder3.attachFullQuad(5, 18);
  setEncoder(0);

  nh.initNode();
  nh.advertise(pub_enc);
  encoder.deg = (float*)malloc(sizeof(float) * 4);
  encoder.deg_length = 4;

}

void loop() {
  get_counter();
  pub_enc.publish(&encoder);
  nh.spinOnce();
  delay(10);
}

void get_counter() {
  encoder.deg[0] = (float) - encoder0.getCount() * 360 / (2400 * 720 / 200); //counter * 360 deg / (ppr encoder * ppr geared/ppr stepper)
  encoder.deg[1] = (float) encoder1.getCount() * 360 / (2400 * 7000 / 200);
  encoder.deg[2] = (float) - encoder3.getCount() * 360 / (2400 * 730 / 200);
  encoder.deg[3] = (float) -encoder2.getCount() * 360 / (2400 * 1220 / 200);
}

void setEncoder(int value) {
  encoder0.setCount(value);
  encoder1.setCount(value);
  encoder2.setCount(value);
  encoder3.setCount(value);
}
