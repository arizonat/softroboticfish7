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

class IMU
{
	public:
		IMU();
		void start();
		void calibrate(bool includeAccel);
		char* readValues();
	private:
		bool needData;
		Vector euler;
		Vector gyro;
		Vector mag;
		Vector acc;
		unsigned char s, g, a, w;
		char calibration[22];
		float heading;
		float declination;
		float angle2TrueNorth;

};

extern IMU imu; // static instance of IMU

#endif /* IMU_IMU_H_ */
