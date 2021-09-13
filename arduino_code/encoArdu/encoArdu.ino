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

  encoder0.setCount(0);
  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder3.setCount(0);
  
  //encoder2.setCount(6400);  //135.34506225585938 / 360 * (1220/200 * 2400) = 6400,000134605
  //encoder3.setCount(-3328); //-109.4136962890625 / 360 * ( 730/200 * 2400) = 3327,999928792

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
  encoder.deg[0] = (float) -encoder0.getCount() * 360 /  10800; //((720/200) * (20/16) * 2400); //counter * 360 deg * (gear step/gear enc) / (ppr encoder * ppr geared/ppr stepper)
  encoder.deg[1] = (float) encoder1.getCount() * 360 / 105000; //((7000/200) * (20/16) * 2400)) * 360; //7000
  encoder.deg[2] = (float) -encoder3.getCount() * 360 / 10950;  // ((730/200) * (20/16) * 2400)) * 360;
  encoder.deg[3] = (float) -encoder2.getCount() * 360 * (90 / 144.4) / 10610; // ((2122 / 600) * (20/16) * 2400)) * 360; //2112 / 660 = 64/20 * 63/ .....
}
