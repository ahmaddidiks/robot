#include "ESP32Encoder.h"

float deg0,deg1,deg2,deg3,deg4;

ESP32Encoder encoder0;
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

void setup(){
    Serial.begin(115200);

    ESP32Encoder::useInternalWeakPullResistors=UP;
    encoder0.attachFullQuad(5, 18); 
    encoder1.attachFullQuad(19, 21);
    encoder2.attachFullQuad(13, 12);
    encoder3.attachFullQuad(14, 27);
    encoder4.attachFullQuad(26, 25);

    setEncoder(0);
}

void loop(){
    Serial.println("Encoder count 1 : "+String((int32_t)encoder0.getCount())+" 2 : "+String((int32_t)encoder1.getCount())+" 3 : "+String((int32_t)encoder2.getCount())+" 4 : "+String((int32_t)encoder3.getCount())+" 5: "+String((int32_t)encoder4.getCount()));
    getDegree();
}
