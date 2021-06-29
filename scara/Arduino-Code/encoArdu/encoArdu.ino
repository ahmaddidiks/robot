//#define USE_USBCON

#include "ESP32Encoder.h"
//#include <ArduinoHardware.h>

#include <ros.h>
#include <scara/encoder.h>

ros::NodeHandle nh;
scara::encoder encoder_msg;
ros::Publisher pub_enc("encoder", &encoder_msg);

ESP32Encoder encoder0;
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

float deg[5] = {0,0,0,0,0};

void setup(){
//    Serial.begin(57600);

    ESP32Encoder::useInternalWeakPullResistors=UP;
    encoder0.attachFullQuad(13, 12); 
    encoder1.attachFullQuad(25, 14);
    encoder2.attachFullQuad(32, 33);
    encoder3.attachFullQuad(34, 35);
    encoder4.attachFullQuad(5, 18);

    setEncoder(0);

    nh.initNode();

    nh.advertise(pub_enc);

    encoder_msg.encoderPostList = (float*)malloc(sizeof(float) * 5);

    encoder_msg.encoderPostList_length = 5;
}

void loop(){
//    Serial.println("Encoder count 1 : "+String((int32_t)encoder0.getCount())+" 2 : "+String((int32_t)encoder1.getCount())+" 3 : "+String((int32_t)encoder2.getCount())+" 4 : "+String((int32_t)encoder3.getCount())+" 5: "+String((int32_t)encoder4.getCount()));
getDegree();
for (int i=0; i<5;i++){
  encoder_msg.encoderPostList[i] = deg[i];
//  if(i==4) Serial.println(String(i) + " : " +  String(deg[i]));
//  else Serial.print(String(i) + " : " +  String(deg[i]) + " ");
  }
  pub_enc.publish(&encoder_msg);
  nh.spinOnce();
  delay(100);
    
}
