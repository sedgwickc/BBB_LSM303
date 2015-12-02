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
#include "Adafruit_BMP180.h"
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace rover;

int main() {

	Adafruit_BMP180 BMP180(1,0x77);
	if( BMP180.begin() == false)
	{
		std::cout<<"Could not find a valid BMP180!\n"<<std::flush;
		return 0;
	}

	float* temp = (float*)calloc(1,sizeof(float));
	float* pressure = (float*)calloc(1, sizeof(float));
	std::cout<<"Getting Temps"<<endl;
	BMP180.getTemperature(temp);
	BMP180.getPressure(pressure);

	printf("Temp: %f\n", *temp);
	printf("Pressure: %f\n", *pressure);

	return 0;
}
