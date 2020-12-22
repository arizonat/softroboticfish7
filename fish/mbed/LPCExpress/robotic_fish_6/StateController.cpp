#include "StateController.h"
#include "FishController.h"
#include "SerialControl/SerialController.h"
#include <iostream>

using namespace std;
StateController stateController;
//use if/else to set values for state based on other values
//enum state {Init, Off, Normal, Oscillate, Tail_Freeze}s;
//might need to put everything below into while loop (as well as in a void function)
void StateController::initStateMachine(Serial* serialObject /* = NULL */, Serial* usbSerialObject /* = NULL */)
{
	if(serialObject == NULL)
	{
		serialObject = new Serial(serialDefaultTX, serialDefaultRX);
		serialObject->baud(serialDefaultBaud);
	}
	serial = serialObject;
	// Create usb serial object or use provided one
	if(usbSerialObject == NULL)
	{
		usbSerialObject = new Serial(USBTX, USBRX);
		usbSerialObject->baud(serialDefaultBaudUSB);
	}
	usbSerial = usbSerialObject;

	startTime = clock();
	beatStartTime = clock();
	timer2.start();
}

void StateController::transitionStateMachine()
{
	switch(state)
	{
		case INIT:
		{
			usbSerial->printf("I'm transitioning states now. \n");
			break;
		}

		case NORMAL:
		{
			serialController.stethoscope();
			usbSerial-> printf("heartBeat found \n");
			if(timer2.read() < maxBeatTime && isActiveFlex == false)
			{
				if(serialController.beatFound)
				{
					//run fish controller
					usbSerial->printf("I'm remaining in the Normal State. \n");
					break;
				}
			}

			else
			{
				timer2.stop();

				if(isActiveFlex == true)  //Checks whether to transition to FLEX state. Otherwise go to OFF state.
				{
					state = FLEX;
					usbSerial->printf("I'm transitioning states now. \n");
					break;
				}

				else
				{
					state = OFF;
					usbSerial->printf("I'm transitioning states now. \n");
					fishController.stop();
					break;
					//pumpWithValve.writeToPins(0.0, 0.0);
				}
			}
		}

		case OFF:
		{
			serialController.stethoscope();
			usbSerial-> printf("heartBeat done \n");
			if(serialController.beatFound)
			{
				state = NORMAL;
				usbSerial->printf("I'm transitioning states now. \n");
				fishController.start();
			}
			else
			{
				usbSerial->printf("I'm remaining in the Off State.");
			}
			break;
		}

		case FLEX:
		{
			usbSerial->printf("I'm transitioning states now. \n");
			if(isActiveFlex == false)
			{
				state = NORMAL;
			}
			break;
		}
	}
}

//Assuming that if this keeps getting called, because state is set within cases, always transitions to right case this way
void StateController::runStateMachine()
{
	while(true)
	{
		switch(state)
		{
			case INIT:
			{
				secondsPassed = (clock() - startTime)/CLOCKS_PER_SEC;
				if(secondsPassed >= timeBeforeStart) //beatFound == 1
				{
					usbSerial->printf("I'm in the init state.");
					state = NORMAL;
					fishController.start();
					this->transitionStateMachine();
				}
				else
				{
					usbSerial->printf("Booting Up!");
				}
				break;
			}

			case NORMAL:
			{
				usbSerial->printf("I'm in the Normal state. \n");
				serial->printf("Sending messages over serial to pi \n");

				timer2.reset();

				serialController.heartBeatRun();
				this->transitionStateMachine();

				break;
			}

			case OFF:
			{
				usbSerial->printf("I'm in the off state. \n");
				serial->printf("Cannot detect a serial connection \n");
				this->transitionStateMachine();

				break;
			}

			case FLEX:
			{
				usbSerial->printf("I'm in the flex state.");
				this->transitionStateMachine();
				//flexDegree
					//Probably need to get period and scale based on degree
				//pumpWithValve.writeToPins()
				break;
			}

		}
	}
}

