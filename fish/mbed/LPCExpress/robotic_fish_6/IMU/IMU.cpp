/*
 * IMU.cpp
 *
 *  Created on: Dec 8, 2020
 *      Author: Juan Salazar
 */


#include <IMU/IMU.h>
IMU imu; // static instance of IMU


IMU::IMU():
	bno(PIN_IMU_TX, PIN_IMU_RX)
{
	needData = true;
	goodData = false;
}

bool IMU::start()
{
	bool status = bno.begin(OPERATION_MODE_NDOF);
	return status;
}


void IMU::calibrate(bool includeAccel = false)
{
	while(!goodData) {
	    unsigned char s, g, a, w;
	    bno.getCalibration(&s, &g, &a, &w);
	    for(int i = 0; i < 10; i++) {
	      euler = bno.getVector(VECTOR_EULER);
	      bno.getCalibration(&s, &g, &a, &w);
	      calibrationLevel = (float)(s + g + a + w)/12.0;
	    }
	    gyro = bno.getVector(VECTOR_GYROSCOPE);
	    mag = bno.getVector(VECTOR_MAGNETOMETER);
	    acc = bno.getVector(VECTOR_LINEARACCEL);
	    char calibration[22];
	    goodData = bno.getSensorOffsets(calibration);
	}
}

void IMU::readValues(char* data)
{
	euler = bno.getVector(VECTOR_EULER);
	gyro = bno.getVector(VECTOR_GYROSCOPE);
	mag = bno.getVector(VECTOR_MAGNETOMETER);
	acc = bno.getVector(VECTOR_LINEARACCEL);

    /* Calculate heading in degrees, then angle to true North*/
    heading = atan2(mag[1], mag[0]) * 180/3.1415;
    declination = -6.92; //look up, based on location (latitude and longitude)
    angle2TrueNorth = heading + declination;

	sprintf(data, "%07.2f,%07.2f,%07.2f,%07.2f,%07.2f,%07.2f,%07.2f,%07.2f,%07.2f,%07.2f\n",euler[2], euler[1], euler[0], gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], angle2TrueNorth);
}

Vector IMU::getEuler()
{
	return euler;
}
