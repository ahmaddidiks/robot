//#define USE_USBCON

#include "ESP32Encoder.h"
//#include <ArduinoHardware.h>

#include <ros.h>
#include <scara/encoder.h>
//#include <scara/syncEncoder.h>

ESP32Encoder encoder0;
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

ros::NodeHandle nh;

scara::encoder encoder_msg;
//scara::syncEncoder sync_encoder_msg;

ros::Publisher pub_enc("encoder", &encoder_msg);

//void sync_handler(const scara::syncEncoder &sync_encoder_msg){
//  encoder0.setCount(sync_encoder_msg.tetha[0]);
//  encoder1.setCount(sync_encoder_msg.tetha[1]);
//  encoder2.setCount(sync_encoder_msg.tetha[2]);
//  encoder3.setCount(sync_encoder_msg.tetha[3]);
//}
//
//ros::Subscriber<scara::syncEncoder> sub("sync_encoder", sync_handler);

float deg[4] = {0.00,0.00,0.00,0.00};

void setup(){
    Serial.begin(57600);

    ESP32Encoder::useInternalWeakPullResistors=UP;
    encoder0.attachFullQuad(13, 12); 
    encoder1.attachFullQuad(25, 14);
    encoder2.attachFullQuad(32, 33);
    encoder3.attachFullQuad(5, 18);

    setEncoder(0);

    nh.initNode();

    nh.advertise(pub_enc);

    encoder_msg.encoderPostList = (float*)malloc(sizeof(float) * 4);

    encoder_msg.encoderPostList_length = 4;
}

void loop(){
//    Serial.println("Encoder count 1 : "+String((int32_t)encoder0.getCount())+" 2 : "+String((int32_t)encoder1.getCount())+" 3 : "+String((int32_t)encoder2.getCount())+" 4 : "+String((int32_t)encoder3.getCount())+" 5: "+String((int32_t)encoder4.getCount()));
  getDegree();
  print_value();  
  pub_enc.publish(&encoder_msg);
  nh.spinOnce();
  delay(10);
    
}
