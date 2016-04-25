/***************************************************************************
  This library is a port of Adafruit's LSM303 library for Arduino to the Beagle
  Bone Black using the MRAA library written by Intel to handle I2C
  communication.

  This port is written and maintained by Charles Sedgwick. 
  This port retains the licence of the software it is based off of which is
  described below.
 ***************************************************************************
  This is a library for the LSM303 pressure sensor

  Designed specifically to work with the Adafruit LSM303 or LSM303 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603
 
  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include "Adafruit_LSM303.h"
#include <math.h>
#include <limits.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace rover;

namespace rover {

#define LSM303_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */

/**
 * Method to combine two 8-bit registers into a single short, which is 16-bits on the BBB. It shifts
 * the MSB 8-bits to the left and then ORs the result with the LSB.
 * @param msb an unsigned character that contains the most significant uint8_t
 * @param lsb an unsigned character that contains the least significant uint8_t
 */
short Adafruit_LSM303::combineRegisters(unsigned char msb, unsigned char lsb){
   //shift the MSB left by 8 bits and OR with LSB
   return ((short)msb<<8)|(short)lsb;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_LSM303 class
*/
/**************************************************************************/
Adafruit_LSM303::Adafruit_LSM303(int I2CBus){
	this->i2c_mag = new mraa::I2c(I2CBus, true);
	this->mag_addr = LSM303_ADDRESS_MAG;
	this->i2c_accel = new mraa::I2c(I2CBus, true);
	this->accel_addr = LSM303_ADDRESS_ACCEL;
	
	this->_lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
	this->_lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
	this->_lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

void Adafruit_LSM303::cleanup(){
	delete(this->i2c_mag);
	delete(this->i2c_accel);
}

 /*****************************************************************************
 @brief Reads accelerometer
 *****************************************************************************/
 void Adafruit_LSM303::readAccelerometer(){
	uint8_t accelDataRaw[NUM_ACCEL_REG];
	this->i2c_accel->address(LSM303_ADDRESS_ACCEL);
 	this->i2c_accel->writeByte(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
 	int ret = this->i2c_accel->read(accelDataRaw, 6);
	if( ret < 0 ){
		cerr<<"Adafruit_LSM303::readAccelerometer(): Error on reading accel data \
			registers"<<endl;
		return;
	}

	// Shift values to create properly formed integer (low uint8_t first)
	// Shift result by 4 since accellerometer data is only return in 12 MSB
	this->_accelData.x = (int16_t)(accelDataRaw[0] | (accelDataRaw[1] << 8)) >> 4;
	this->_accelData.y = (int16_t)(accelDataRaw[2] | (accelDataRaw[3] << 8)) >> 4;
	this->_accelData.z = (int16_t)(accelDataRaw[4] | (accelDataRaw[5] << 8)) >> 4;
}

void Adafruit_LSM303::getAcceleration(float *x, float *y, float *z){

	this->readAccelerometer();
	// sets x,y,z to values in metres/second
	*x = this->_accelData.x * this->_lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
	*y = this->_accelData.y * this->_lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
	*z = this->_accelData.z * this->_lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
}


/**************************************************************************/
/*!
    @brief  Read mag data from LSM303
*/
/**************************************************************************/
void Adafruit_LSM303::readMagnetometerData(){
	// write to LSM303_REGISTER_MAG_OUT_X_H_M
	// request 6 uint8_ts from LSM303_ADDRESS_MAG
	// wait for data to be available

	// read mag data once data is available
	uint8_t xhi = 0;
	uint8_t xlo = 0;
	uint8_t zhi = 0;
	uint8_t zlo = 0;
	uint8_t yhi = 0;
	uint8_t ylo = 0;

	// Shift values to create properly formed integer (low uint8_t first)
	this->_magData.x = (int16_t)(xlo | ((int16_t)xhi << 8));
	this->_magData.y = (int16_t)(ylo | ((int16_t)yhi << 8));
	this->_magData.z = (int16_t)(zlo | ((int16_t)zhi << 8));
}

void Adafruit_LSM303::getOrientation(float *x, float *y, float *z){
	bool readingValid = false;
  
	while(!readingValid)
	{

		uint8_t reg_mg = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_SR_REG_Mg);
		if (!(reg_mg & 0x1)) {
			cout<<"getOrientation(): reg_mg error"<<endl;
			return;
		}

		this->readMagnetometerData();
    	/* Make sure the sensor isn't saturating if auto-ranging is enabled */
    	if (!this->_autoRangeEnabled)
    	{
      	  readingValid = true;
    	}
    	else
    	{
	#ifdef DEBUG
      	  cout<<"mag x: "<<this->_magData.x<<endl;
      	  cout<<"mag y: "<<this->_magData.y<<endl;
      	  cout<<"mag z: "<<this->_magData.z<<endl;
	#endif	  
      	  /* Check if the sensor is saturating or not */
      	  if ( (this->_magData.x >= 2040) | (this->_magData.x <= -2040) | 
           	   (this->_magData.y >= 2040) | (this->_magData.y <= -2040) | 
           	   (this->_magData.z >= 2040) | (this->_magData.z <= -2040) )
      	  {
        	/* Saturating .... increase the range if we can */
        	switch(this->_magGain)
        	{
          	  case LSM303_MAGGAIN_5_6:
            	this->setMagGain(LSM303_MAGGAIN_8_1);
            	readingValid = false;
	#ifdef DEBUG
            	cout<<"Changing range to +/- 8.1"<<endl;
	#endif
            	break;
          	  case LSM303_MAGGAIN_4_7:
            	this->setMagGain(LSM303_MAGGAIN_5_6);
            	readingValid = false;
	#ifdef DEBUG
            	cout<<"Changing range to +/- 5.6"<<endl;
	#endif
            	break;
          	  case LSM303_MAGGAIN_4_0:
            	this->setMagGain(LSM303_MAGGAIN_4_7);
            	readingValid = false;
	#ifdef DEBUG
            	cout<<"Changing range to +/- 4.7"<<endl;
	#endif			
            	break;
          	  case LSM303_MAGGAIN_2_5:
            	this->setMagGain(LSM303_MAGGAIN_4_0);
            	readingValid = false;
	#ifdef DEBUG
            	cout<<"Changing range to +/- 4.0"<<endl;
	#endif			
            	break;
          	  case LSM303_MAGGAIN_1_9:
            	this->setMagGain(LSM303_MAGGAIN_2_5);
            	readingValid = false;
	#ifdef DEBUG
            	cout<<"Changing range to +/- 2.5"<<endl;
	#endif			
            	break;
          	  case LSM303_MAGGAIN_1_3:
            	this->setMagGain(LSM303_MAGGAIN_1_9);
            	readingValid = false;
	#ifdef DEBUG
            	cout<<"Changing range to +/- 1.9"<<endl;
	#endif			
            	break;
          	  default:
            	readingValid = true;
            	break;  
        	}
      	  }
      	  else
      	  {
        	/* All values are withing range */
        	readingValid = true;
      	  }
		}
	}
	*x = this->_magData.x / this->_lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
	*y = this->_magData.y / this->_lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
	*z = this->_magData.z / this->_lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;
}
/**************************************************************************/
/*! 
    @brief  Enables or disables auto-ranging
*/
/**************************************************************************/
void Adafruit_LSM303::enableAutoRange(bool enabled)
{
  this->_autoRangeEnabled = enabled;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void Adafruit_LSM303::setMagGain(lsm303MagGain gain)
{
  this->writeCommand(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t)gain);
  
  this->_magGain = gain;
 
  switch(gain)
  {
    case LSM303_MAGGAIN_1_3:
      this->_lsm303Mag_Gauss_LSB_XY = 1100;
      this->_lsm303Mag_Gauss_LSB_Z  = 980;
      break;
    case LSM303_MAGGAIN_1_9:
      this->_lsm303Mag_Gauss_LSB_XY = 855;
      this->_lsm303Mag_Gauss_LSB_Z  = 760;
      break;
    case LSM303_MAGGAIN_2_5:
      this->_lsm303Mag_Gauss_LSB_XY = 670;
      this->_lsm303Mag_Gauss_LSB_Z  = 600;
      break;
    case LSM303_MAGGAIN_4_0:
      this->_lsm303Mag_Gauss_LSB_XY = 450;
      this->_lsm303Mag_Gauss_LSB_Z  = 400;
      break;
    case LSM303_MAGGAIN_4_7:
      this->_lsm303Mag_Gauss_LSB_XY = 400;
      this->_lsm303Mag_Gauss_LSB_Z  = 355;
      break;
    case LSM303_MAGGAIN_5_6:
      this->_lsm303Mag_Gauss_LSB_XY = 330;
      this->_lsm303Mag_Gauss_LSB_Z  = 295;
      break;
    case LSM303_MAGGAIN_8_1:
      this->_lsm303Mag_Gauss_LSB_XY = 230;
      this->_lsm303Mag_Gauss_LSB_Z  = 205;
      break;
  } 
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's update rate
*/
/**************************************************************************/
void Adafruit_LSM303::setMagRate(lsm303MagRate rate)
{
	uint8_t reg_m = ((uint8_t)rate & 0x07) << 2;
  	this->writeCommand(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, reg_m);
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_LSM303::writeCommand(unsigned int address,unsigned int reg, unsigned char value)
{
	if( address == LSM303_ADDRESS_MAG ){
		this->i2c_mag->address(address);
		this->i2c_mag->writeReg( reg, value );
	}else if( address == LSM303_ADDRESS_ACCEL ){
		this->i2c_accel->address(address);
		this->i2c_accel->writeReg( reg, value );
	}else{
		cout<<"writeCommand(): Invalid Address!"<<endl;
	}
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value (byte) over I2C
*/
/**************************************************************************/
uint8_t Adafruit_LSM303::read8(unsigned int address, unsigned int reg)
{
	if( address == LSM303_ADDRESS_MAG ){
		this->i2c_mag->address(address);
		return (uint8_t)this->i2c_mag->readReg(reg);
	} else if( address == LSM303_ADDRESS_ACCEL ){
		this->i2c_accel->address(address);
		return (uint8_t)this->i2c_accel->readReg(reg);
	}
	cout<<"Adafruit_LSM303::read8(): Invalid sensor passed in."<<endl;
	return 0;
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_LSM303::begin()
{
	// Enable the accelerometer (100Hz)
	this->writeCommand(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
	// FS = 00 (+/- 2g ful scale)
	this->writeCommand(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x08);

	// LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
	// if we are connected or not
	//Consider chaning this as the MSB may change based on select read rate
	// if MSBs = 0x5 then data rate is 100Hz
	uint8_t reg1_a = this->read8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	if (reg1_a != 0x57)
	{
#ifdef DEBUG
	cout<<"begin(): Invalid value found in register CTRL_REG1_A. Aborting..."<<endl;
	cout<<"begin(): Invalid CTRL_REG1_A value: "<<reg1_a<<endl;
#endif
		return false;
	}  
#ifdef DEBUG
	cout<<"begin(): Accelerometer enabled..."<<endl;
	uint8_t control1 = read8(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	cout<<"Power, rate and axes status: "<<std::hex<<(unsigned int)control1<<endl;
#endif
	// Enable the magnetometer
	this->writeCommand(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

	// LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
	// the default value (0b00010000/0x10)
	uint8_t reg1_m = this->read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M);
	if (reg1_m != 0x10)
	{
	return false;
	}
#ifdef DEBUG
	cout<<"begin(): Magnetometer enabled..."<<endl;
#endif

	// Set the gain to a known level
	setMagGain(LSM303_MAGGAIN_1_3);

	return true;
}

Adafruit_LSM303::~Adafruit_LSM303(){}
}; // rover namespace

