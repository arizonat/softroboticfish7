#define stateControl
#ifdef stateControl


#ifndef STATECONTROLLER_H_
#define STATECONTROLLER_H_

#include "FishController.h"
#include <iostream>
#include "mbed.h"
#include "SerialBase.h"

#define serialDefaultBaudUSB 115200
#define serialDefaultBaud 115200
#define serialDefaultTX p9
#define serialDefaultRX p10

#define INIT 0
#define NORMAL 1
#define OFF 2
#define FLEX 3

class StateController
{
public:
	void initStateMachine(Serial* serialObject /* = NULL */, Serial* usbSerialObject /* = NULL */);
	void runStateMachine();
private:
	Serial* serial;
	Serial* usbSerial;

	//Initialize starting state for fish
	int state = INIT;

	//For simulating boot up time for Init case
	clock_t startTime;
	double timeBeforeStart = 15;
	double secondsPassed;

	//Detecting if Serial Connection still present
	clock_t beatStartTime;
	double secondCounter;
    //double heartrate = 2;  //Seconds between heartbeat messages
	//int missedMessage = 0;
	bool beatFound = 0;
	int maxBeatTime = 2; //Maximum time since last beat detected before state moves to OFF

	uint8_t beatBuffer[20];
	uint32_t beatBufferIndex = 0;

	//For Tail Flex Mode
	bool isActiveFlex = false;
	float flexDegree;

	//To ensure transitioning
	void transitionStateMachine();

	Timer timer2;

	//Access private variables from FishController and SerialController
	friend class FishController;
	friend class SerialController;
};

extern StateController stateController;
#endif
#endif
