/*
 * IMU.h
 *
 *  Created on: Dec 8, 2020
 *      Author: Juan Salazar
 */

#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include "AdaBNO055.h"
#include "mbed.h"

#define PIN_IMU_TX p28
#define PIN_IMU_RX p27

class IMU
{
	public:
		IMU();
		bool start();
		void calibrate(bool includeAccel);
		void readValues(char* data);
		Vector getEuler();
		Vector getGyro();
		Vector getAccels();
		float getAnglefromNorth();
	private:
		bool needData;
		bool goodData;
		float calibrationLevel;
		Vector euler;
		Vector gyro;
		Vector mag;
		Vector acc;
		unsigned char s, g, a, w;
		char calibration[22];
		float heading;
		float declination;
		float angle2TrueNorth;
		BNO055 bno;

};

extern IMU imu; // static instance of IMU

#endif /* IMU_IMU_H_ */
