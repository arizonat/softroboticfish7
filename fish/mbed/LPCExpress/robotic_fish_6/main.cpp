
// NOTE look at the below h files to define whether that control mode is enabled

#include "mbed.h"
#include "AcousticControl/AcousticController.h" // also need to define acousticControl in ToneDetector.h
#include "SerialControl/SerialController.h"
#include "StateController.h"
#include "ROSControl/ROSController.h"

Serial pc(USBTX, USBRX);

int main()
{
	/* ACOUSTIC CONTROL */
	#ifdef acousticControl
	pc.baud(115200);
	// Initialize the acoustic controller
	acousticController.init(&pc); // if no serial object is provided, it will create one on the USB pins
	// Start the controller
	// NOTE this is a blocking method, and if infiniteLoopAcoustic is defined it will run forever (or until low battery callback or button board reset command)
	// It can be stopped by the acousticController.stop() method, but you have to
	//  control threading here to actually be able to call that
	//  The acoustic controller hasn't been tested with multi-threading though
	acousticController.run();
	#endif

#ifdef noStateControl
//#define noStateControl
	/* SERIAL CONTROL */
	#ifdef serialControl
	pc.baud(115200);
	// Initialize the serial controller
	serialController.init(NULL, &pc);
	// Start the controller
	// NOTE this is a blocking method, and if infiniteLoopSerial is defined it will run forever (or until low battery callback or button board reset command)
	// It can be stopped by the serialController.stop() method, but you have to
	//  control threading here to actually be able to call that
	serialController.run();
	#endif
#endif

	/* STATE CONTROL */
	#ifdef stateControl
	pc.baud(115200);
	stateController.initStateMachine(NULL, &pc);
	stateController.runStateMachine();
	#endif



	/* ROS CONTROL */
	#ifdef rosControl
	pc.baud(115200);
	// Initialize the ROS controller
	rosController.init(NULL, &pc);
	// Start the controller
	// NOTE this is a blocking method, and if infiniteLoopSerial is defined it will run forever (or until low battery callback or button board reset command)
	// It can be stopped by the rosController.stop() method, but you have to
	//  control threading here to actually be able to call that
	rosController.run();
	#endif
}
