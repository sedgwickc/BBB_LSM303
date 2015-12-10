/***************************************************************************
  This library is a port of Adafruit's LSM303 library for Arduino to the Beagle
  Bone Black using Derek Molloy's I2CDevice library in the place of Wire.h to
  communicate over I2C.

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

#include "Adafruit_LSM303.hpp"
#include <math.h>
#include <limits.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

namespace rover {

#define LSM303_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_LSM303 class
*/
/**************************************************************************/
Adafruit_LSM303::Adafruit_LSM303(unsigned int I2CBus){
	this->i2c_lsm303 = new mraa::I2c(I2CBus);
	this->_lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
	this->_lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
	this->_lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

void Adafruit_LSM303::cleanup(){
	delete(this->i2c_lsm303);
}

 /*****************************************************************************
 @brief Reads accelerometer
 *****************************************************************************/
 void Adafruit_LSM303::readAccelerometer(){
 	// read 6 uint8_ts from LSM303_ADDRESS_ACCEL 
	this->i2c_lsm303->address(LSM303_ADDRESS_ACCEL);
 	uint8_t rx_tx_buf[MAX_BUFFER_LENGTH];
 	this->i2c_lsm303->writeByte(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
 	this->i2c_lsm303->read(rx_tx_buf, 6);
#ifdef DEBUG
	uint8_t retval = this->i2c_lsm303->readReg(LSM303_REGISTER_ACCEL_STATUS_REG_A);
	cout<<"Accelerometer Status: "<<std::hex<<(unsigned int)retval<<endl;
#endif
 	// wait until data is available
 	//only read data when STATUS_REG_A signal new data available
    uint8_t xlo = (uint8_t)rx_tx_buf[0];
    uint8_t xhi = (uint8_t)rx_tx_buf[1];
    uint8_t ylo = (uint8_t)rx_tx_buf[2];
    uint8_t yhi = (uint8_t)rx_tx_buf[3];
    uint8_t zlo = (uint8_t)rx_tx_buf[4];
    uint8_t zhi = (uint8_t)rx_tx_buf[5];

#ifdef DEBUG
	for( int i=0; i < 6; i++){
		cout<<"Accel["<<i<<"]: "<<(unsigned int)rx_tx_buf[i]<<endl;
	}
#endif
	// Shift values to create properly formed integer (low uint8_t first)
	// Shift result by 4 since accellerometer data is only return in 12 MSB
	this->_accelData.x = (int16_t)(xlo | (xhi << 8)) >> 4;
	this->_accelData.y = (int16_t)(ylo | (yhi << 8)) >> 4;
	this->_accelData.z = (int16_t)(zlo | (zhi << 8)) >> 4;
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

		uint8_t reg_mg = readRegister(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_SR_REG_Mg);
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
  this->writeRegister(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t)gain);
  
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
  	this->writeRegister(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, reg_m);
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_LSM303::writeRegister(unsigned int address,unsigned int reg, unsigned char value)
{
	this->i2c_lsm303->address(address);
	this->i2c_lsm303->writeReg( reg, (uint8_t)value );
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_LSM303::readRegister(unsigned int address, unsigned int reg)
{
		this->i2c_lsm303->address(address);
		return (uint8_t)this->i2c_lsm303->readReg(reg);
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_LSM303::begin()
{
	// Enable the accelerometer (100Hz)
	this->writeRegister(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
	// FS = 00 (+/- 2g ful scale)
	this->writeRegister(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x08);

	// LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check
	// if we are connected or not
	//Consider chaning this as the MSB may change based on select read rate
	// if MSBs = 0x5 then data rate is 100Hz
	uint8_t reg1_a = this->readRegister(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	if (reg1_a != 0x57)
	{
		return false;
	}  
#ifdef DEBUG
	cout<<"begin(): Accelerometer enabled..."<<endl;
	uint8_t control1 = this->readReg(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	cout<<"Power, rate and axes status: "<<std::hex<<(unsigned int)control1<<endl;
#endif
	// Enable the magnetometer
	this->writeRegister(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);

	// LSM303DLHC has no WHOAMI register so read CRA_REG_M to check
	// the default value (0b00010000/0x10)
	uint8_t reg1_m = this->readRegister(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M);
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

