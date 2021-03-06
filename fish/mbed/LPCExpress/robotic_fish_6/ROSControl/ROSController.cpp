/*
 * Author: Joseph DelPreto
 */

#include "ROSController.h"

#ifdef rosControl

// The static instance
ROSController rosController;

// Define global methods that call methods of the singleton, to make callbacks easier
// TODO find out how to get a function pointer to a member function of a specific object (so we won't need these methods)
void processROSMessageStatic(const fish_msgs::JoystickState& msg)
{
	rosController.processROSMessage(msg);
}
void lowBatteryCallbackROSStatic()
{
	rosController.lowBatteryCallback();
}

// Initialization
ROSController::ROSController(Serial* serialObject /* = NULL */, Serial* usbSerialObject /* = NULL */):
		subscriber(rosTopicName, &processROSMessageStatic),
		terminated(false)
{
	#ifdef debugLEDsROS
	rosLEDs[0] = new DigitalOut(LED1);
	rosLEDs[1] = new DigitalOut(LED2);
	rosLEDs[2] = new DigitalOut(LED3);
	rosLEDs[3] = new DigitalOut(LED4);
	#endif
	init(serialObject, usbSerialObject);
}

void ROSController::init(Serial* serialObject /* = NULL */, Serial* usbSerialObject /* = NULL */)
{
	// Create serial object or use provided one
	if(serialObject == NULL)
	{
		serialObject = new Serial(rosDefaultTX, rosDefaultRX);
		serialObject->baud(rosDefaultBaud);
	}
	serial = serialObject;
	// Create usb serial object or use provided one
	if(usbSerialObject == NULL)
	{
		usbSerialObject = new Serial(USBTX, USBRX);
		usbSerialObject->baud(rosDefaultBaudUSB);
	}
	usbSerial = usbSerialObject;

	// Will check for low battery at startup and using an interrupt
	lowBatteryVoltageInput = new DigitalIn(lowBatteryVoltagePin);
	lowBatteryVoltageInput->mode(PullUp);
	detectedLowBattery = false;
	lowBatteryTicker.attach(&lowBatteryCallbackROSStatic, 5);

	// ROS setup
	nodeHandle.initNode();
	nodeHandle.subscribe(subscriber);

	// Debug
	#ifdef debugLEDsROS
	rosLEDs[0]->write(1);
	rosLEDs[1]->write(0);
	rosLEDs[2]->write(0);
	rosLEDs[3]->write(0);
	#endif
}

// Process a received ROS message
void ROSController::processROSMessage(const fish_msgs::JoystickState& msg)
{
	#ifdef debugLEDsROS
	rosLEDs[2]->write(1);
	#endif

	// Extract the desired fish state from msg
	// TODO check that this matches your message format
	bool selectButton = 0;
	float pitch = ((msg.depth_ctrl+128) * (rosMaxPitch - rosMinPitch) / 255.0) + rosMinPitch;
	float yaw = ((msg.yaw_ctrl+127) * (rosMaxYaw - rosMinYaw) / 254.0) + rosMinYaw; // 0 MUST map to 0
	float thrust = ((msg.speed_ctrl+128) * (rosMaxThrust - rosMinThrust) / 255.0) + rosMinThrust;
	float frequency = ((msg.freq_ctrl+128) * (rosMaxFrequency- rosMinFrequency) / 255.0) + rosMinFrequency;

	// Apply the new state to the fish
	fishController.setSelectButton(selectButton);
	fishController.setPitch(pitch);
	fishController.setYaw(yaw);
	fishController.setThrust(thrust);
	fishController.setFrequency(frequency);

	#ifdef printStatusROSController
	usbSerial->printf("Start %d\t Pitch %f\t Yaw %f\t Thrust %f\t Freq %.8f\r\n", selectButton, pitch, yaw, thrust, frequency);
	#endif
	#ifdef debugLEDsROS
	rosLEDs[2]->write(0);
	#endif
}

// Stop the controller (will also stop the fish controller)
//
void ROSController::stop()
{
	terminated = true;
}

// Main loop
// This is blocking - will not return until terminated by timeout or by calling stop() in another thread
void ROSController::run()
{

	#ifdef rosControllerControlFish
    // Start the fish controller
    fishController.start();
    #endif

	#ifdef printStatusROSController
	usbSerial->printf("\r\nStarting to listen for ROS commands\r\n");
	#endif

	#ifdef debugLEDsROS
	rosLEDs[0]->write(1);
	rosLEDs[1]->write(1);
	rosLEDs[2]->write(0);
	rosLEDs[3]->write(0);
	#endif

	// Process any incoming ROS messages
	programTimer.reset();
	programTimer.start();
	while(!terminated)
	{
		// Handle any messages that have been received
		nodeHandle.spinOnce();
		// See if we've run for the desired amount of time
		#ifndef infiniteLoopROS
		if(programTimer.read_ms() > runTimeROS)
			stop();
		#endif
	}
	programTimer.stop();
	#ifdef debugLEDsROS
	rosLEDs[0]->write(0);
	rosLEDs[1]->write(0);
	rosLEDs[2]->write(0);
	rosLEDs[3]->write(0);
	#endif

	// TODO stop the ROS node handle / unsubscribe
	// note this is for when we call stop() on the mbed, independent of when the main ROS program terminates on the Pi

	// Stop the fish controller
	#ifdef rosControllerControlFish
    fishController.stop();
    // If battery died, wait a bit for pi to clean up and shutdown and whatnot
    if(lowBatteryVoltageInput == 0)
    {
		wait(90); // Give the Pi time to shutdown
		fishController.setLEDs(255, false);
    }
    #endif

	#ifdef printStatusROSController
	usbSerial->printf("\r\nROS controller done!\r\n");
	#endif
}


void ROSController::lowBatteryCallback()
{
	if(lowBatteryVoltageInput == 0 && detectedLowBattery)
	{
		// Stop the ROS controller
		// This will end the main loop, causing main to terminate
		// Main will also stop the fish controller once this method ends
		stop();
		// Also force the pin low to signal the Pi
		// (should have already been done, but just in case)
		// TODO check that this really forces it low after this method ends and the pin object may be deleted
		DigitalOut simBatteryLow(lowBatteryVoltagePin);
		simBatteryLow = 0;
		#ifdef printStatusROSController
		usbSerial->printf("\r\nLow battery! Shutting down.\r\n");
		wait(0.5); // wait for the message to actually flush
		#endif
	}
	else if(lowBatteryVoltageInput == 0)
	{
		detectedLowBattery = true;
	}
}

#endif // #ifdef rosControl

