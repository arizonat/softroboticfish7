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
			usbSerial->printf("I'm transitioning states now.");
			break;
		}

		case NORMAL:
		{
			serialController.heartBeatRun();
			if(timer2.read() < maxBeatTime && isActiveFlex == false)
			{
				if(beatFound == 1)
				{
					//run fish controller
					usbSerial->printf("I'm transitioning states now.");
					break;
				}
			}

			else
			{
				timer2.stop();

				if(isActiveFlex == true)  //Checks whether to transition to FLEX state. Otherwise go to OFF state.
				{
					state = FLEX;
					usbSerial->printf("I'm transitioning states now.");
					break;
				}

				else
				{
					state = OFF;
					usbSerial->printf("I'm transitioning states now.");
					break;
					//pumpWithValve.writeToPins(0.0, 0.0);
				}
			}
		}

		case OFF:
		{
			usbSerial->printf("I'm transitioning states now.");
			serialController.heartBeatRun();
			if(beatFound == 1)
			{
				state = NORMAL;
			}
			break;
		}

		case FLEX:
		{
			usbSerial->printf("I'm transitioning states now.");
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
				usbSerial->printf("I'm in the Normal state.");
				serial->printf("Sending messages over serial to pi");

				timer2.reset();
				//Use SerialController primary buffer to determine command to run
					//Run FishController
					//Or instead run serialController start function, which calls on FishController
						//-Could call it? Or make it so only functionable here
				serialController.start();
					//Will this stop on its own?
				//fishController.start();
				//fishController.stop();

				this->transitionStateMachine();

				break;
			}

			case OFF:
			{
				usbSerial->printf("I'm in the off state.");
				serial->printf("Cannot detect a serial connection");
				this->transitionStateMachine();

				break;
			}

			case FLEX:
			{
				usbSerial->printf("I'm in the flex state.");
				this->transitionStateMachine();
				//flexDegree = 100 //Degree of Flex, from -10 to 10?
					//Probably need to get period and scale based on degree
				//Don't run fish controller while in this mode
				//pumpWithValve.writeToPins()
				break;
			}

		}
	}
}

