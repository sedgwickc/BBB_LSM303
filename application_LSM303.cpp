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

	std::cout<<"Getting acceleration and orientation..."<<endl;
	LSM303.getOrientation(&mag_x, &mag_y, &mag_z);
	LSM303.getAcceleration(&acc_x, &acc_y, &acc_z);
	LSM303.cleanup();

	std::cout<<"Acceleration:"<<endl;
	std::cout<<"x: "<<acc_x<<endl;
	std::cout<<"y: "<<acc_y<<endl;
	std::cout<<"z: "<<acc_z<<endl;
	
	std::cout<<"Orientation:"<<endl;
	std::cout<<"x: "<<mag_x<<endl;
	std::cout<<"y: "<<mag_y<<endl;
	std::cout<<"z: "<<mag_z<<endl;

	return 0;
}
