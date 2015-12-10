/*
 * EBBGPIO.cpp  Created on: 29 Apr 2014
 * Copyright (c) 2014 Derek Molloy (www.derekmolloy.ie)
 * Made available for the book "Exploring BeagleBone"
 * See: www.exploringbeaglebone.com
 * Licensed under the EUPL V.1.1
 *
 * This Software is provided to You under the terms of the European
 * Union Public License (the "EUPL") version 1.1 as published by the
 * European Union. Any use of this Software, other than as authorized
 * under this License is strictly prohibited (to the extent such use
 * is covered by a right of the copyright holder of this Software).
 *
 * This Software is provided under the License on an "AS IS" basis and
 * without warranties of any kind concerning the Software, including
 * without limitation merchantability, fitness for a particular purpose,
 * absence of defects or errors, accuracy, and non-infringement of
 * intellectual property rights other than copyright. This disclaimer
 * of warranty is an essential part of the License and a condition for
 * the grant of any rights to this Software.
 *
 * For more details, see http://www.derekmolloy.ie/
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

	Adafruit_LSM303 LSM303(1);
	if( LSM303.begin() == false)
	{
		std::cout<<"Could not find a valid LSM303!\n"<<endl;
		return 0;
	}
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
		std::cout<<acc_x;
		std::cout<<","<<acc_y;
		std::cout<<","<<acc_z<<endl;
    	nanosleep(&rec, (struct timespec *) NULL);
	}
	LSM303.cleanup();
	
	return 0;
}
