/* application_LSM303.cpp
 * This program is used to test the C++ Beaglebone Black/Green LSM303 library 
 * 
 * Github: https://github.com/sedgwickc/RoverDaemons/
 * Changlog
 * Ver    Date       User   Issue #  Change
 * --------------------------------------------------------------------------------
 * 100 25sep2015  cwick              Initial creation. 
 * 101 25sep2016  cwick     1        Add changelog. Add version output. Add
 *                                   orientation output.
 */

#include <iostream>
#include "Adafruit_LSM303.h"
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace rover;

int main() {

	Adafruit_LSM303 LSM303(2);
	if( LSM303.begin() == false)
	{
		std::cout<<"Could not find a valid LSM303!\n"<<endl;
		return 0;
	}

	std::cout<<"LSM303 library version: v"<<LSM303.version<<endl;

	float mag_x = 0.0;
	float mag_y = 0.0;
	float mag_z = 0.0;
	float acc_x = 0.0;
	float acc_y = 0.0;
	float acc_z = 0.0;
    int  millisec = 500;
    struct timespec rec = {0};
    rec.tv_sec = 0;
    rec.tv_nsec = millisec * 1000000L;

	LSM303.getOrientation(&mag_x, &mag_y, &mag_z);
	
	std::cout<<"acc.x,acc.y,acc.z"<<endl;
	for( int i = 0; i < 100; i++ ){
		LSM303.getAcceleration(&acc_x, &acc_y, &acc_z);
		LSM303.getOrientation(&mag_x, &mag_y, &mag_z);
		std::cout<<"Accel: "<<acc_x;
		std::cout<<","<<acc_y;
		std::cout<<","<<acc_z<<"; Orient:";
		std::cout<<mag_x;
		std::cout<<","<<mag_y;
		std::cout<<","<<mag_z<<endl;
    	nanosleep(&rec, (struct timespec *) NULL);
	}
	LSM303.cleanup();
	
	return 0;
}
