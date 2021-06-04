#pragma once
#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/pcnt.h"

#define MAX_ESP32_ENCODERS PCNT_UNIT_MAX
#define 	_INT16_MAX 32766
#define  	_INT16_MIN -32766

#define PPR 2400

#define GEAR_RATIO_0 8
#define GEAR_RATIO_1 8
#define GEAR_RATIO_2 8
#define GEAR_RATIO_3 8
#define GEAR_RATIO_4 8

#define FACTOR_0 GEAR_RATIO_0*PPR
#define FACTOR_1 GEAR_RATIO_1*PPR
#define FACTOR_2 GEAR_RATIO_2*PPR
#define FACTOR_3 GEAR_RATIO_3*PPR
#define FACTOR_4 GEAR_RATIO_4*PPR

enum encType {
single,
half,
full
};
enum puType {
UP,
DOWN,
NONE
};

void getDegree();
void setEncoder(int value);

class ESP32Encoder {
private:
	void attach(int aPintNumber, int bPinNumber, enum encType et);
	boolean attached=false;


	static  pcnt_isr_handle_t user_isr_handle; //user's ISR service handle
    bool direction;
    bool working;

	static bool attachedInterrupt;
	int64_t getCountRaw();

public:
	ESP32Encoder();
	~ESP32Encoder();
	void attachHalfQuad(int aPintNumber, int bPinNumber);
	void attachFullQuad(int aPintNumber, int bPinNumber);
	void attachSingleEdge(int aPintNumber, int bPinNumber);
	//void attachHalfQuad(int aPintNumber, int bPinNumber);
	int64_t getCount();
	int64_t clearCount();

	boolean isAttached(){return attached;}
	void setCount(int64_t value);
	static ESP32Encoder *encoders[MAX_ESP32_ENCODERS];
	gpio_num_t aPinNumber;
	gpio_num_t bPinNumber;
	pcnt_unit_t unit;
	bool fullQuad=false;
	int countsMode = 2;
	volatile int64_t count=0;
	pcnt_config_t r_enc_config;
	static enum puType useInternalWeakPullResistors;
};

//Added by Sloeber 
#pragma once
