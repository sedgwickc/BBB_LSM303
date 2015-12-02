/***************************************************************************
  This library is a port of Adafruit's BMP180 library for Arduino to the Beagle
  Bone Black using Derek Molloy's I2CDevice library in the place of Wire.h to
  communicate over I2C.

  This port is written and maintained by Charles Sedgwick. 
  This port retains the licence of the software it is based off of which is
  described below.
 ***************************************************************************
  This is a library for the BMP180 pressure sensor

  Designed specifically to work with the Adafruit BMP180 or BMP180 Breakout 
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603
 
  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include "Adafruit_BMP180.h"
#include <math.h>
#include <limits.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

namespace rover {

Adafruit_BMP180::bmp180_calib_data _bmp180_coeffs;   // Last read accelerometer data will be available here
uint8_t           _bmp180Mode;

#define BMP180_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */


/**************************************************************************/
/*!
    @brief  Compute B5 coefficient used in temperature & pressure calcs.
*/
/**************************************************************************/
int32_t Adafruit_BMP180::computeB5(int32_t ut) {
  int32_t X1 = (ut - (int32_t)_bmp180_coeffs.ac6) * ((int32_t)_bmp180_coeffs.ac5) >> 15;
  int32_t X2 = ((int32_t)_bmp180_coeffs.mc << 11) / (X1+(int32_t)_bmp180_coeffs.md);
  return X1 + X2;
}

/**
 * Method to combine two 8-bit registers into a single short, which is 16-bits on the BBB. It shifts
 * the MSB 8-bits to the left and then ORs the result with the LSB.
 * @param msb an unsigned character that contains the most significant byte
 * @param lsb an unsigned character that contains the least significant byte
 */
short Adafruit_BMP180::combineRegisters(unsigned char msb, unsigned char lsb){
   //shift the MSB left by 8 bits and OR with LSB
   return ((short)msb<<8)|(short)lsb;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
 
/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_BMP180 class
*/
/**************************************************************************/
Adafruit_BMP180::Adafruit_BMP180(unsigned int I2CBus, unsigned int I2CAddress):
	I2CDevice(I2CBus, I2CAddress){   // this member initialisation list calls the parent constructor
	this->I2CAddress = I2CAddress;
	this->I2CBus = I2CBus;
	this->registers = NULL;
	//this->resolution = ADXL345::HIGH;
	//this->writeRegister(POWER_CTL, 0x08);
	//this->updateRegisters();
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_BMP180::writeCommand(unsigned int reg, unsigned char value)
{
	this->writeRegister( reg, value );
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_BMP180::read8(unsigned int reg, uint8_t *value)
{
	*value = this->readRegister(reg);
}

/**************************************************************************/
/*!
    @brief  Reads two 8bit values and combines them
    Consider changing this to use Derek's readRegisters() instead of using two
    reads and shifting. 
*/
/**************************************************************************/
void Adafruit_BMP180::read16(unsigned int reg, uint16_t *value)
{
	uint16_t val = 0;
	unsigned char lsb = 0;
	unsigned char msb = 0;
	
	lsb = this->readRegister(reg);
	msb = this->readRegister(reg+1);
	val = lsb;
	val <<= 8;
	val |= msb;

#ifdef DEBUG
	cout<<"read16(): val = "<<val<<endl;
#endif
	*value = val;
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
    molloy's library doesn't seem to differentiate between signed and unsiged
    check why this class needs two different reads
*/
/**************************************************************************/
void Adafruit_BMP180::readS16(unsigned int reg, int16_t *value)
{
	int16_t val = 0;
	unsigned char lsb = 0;
	unsigned char msb = 0;
	
	lsb = this->readRegister(reg);
	msb = this->readRegister(reg+1);
	val = lsb;
	val <<= 8;
	val |= msb;

#ifdef DEBUG
	cout<<"readS16(): sizeof(val) = "<<sizeof(val)<<endl;
	cout<<"readS16(): val = "<<val<<endl;
#endif
	*value = val;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Adafruit_BMP180::readCoefficients(void)
{
  #if BMP180_USE_DATASHEET_VALS
    _bmp180_coeffs.ac1 = 408;
    _bmp180_coeffs.ac2 = -72;
    _bmp180_coeffs.ac3 = -14383;
    _bmp180_coeffs.ac4 = 32741;
    _bmp180_coeffs.ac5 = 32757;
    _bmp180_coeffs.ac6 = 23153;
    _bmp180_coeffs.b1  = 6190;
    _bmp180_coeffs.b2  = 4;
    _bmp180_coeffs.mb  = -32768;
    _bmp180_coeffs.mc  = -8711;
    _bmp180_coeffs.md  = 2868;
    _bmp180Mode        = 0;
  #else
    this->readS16(BMP180_REGISTER_CAL_AC1, &_bmp180_coeffs.ac1);
    this->readS16(BMP180_REGISTER_CAL_AC2, &_bmp180_coeffs.ac2);
    this->readS16(BMP180_REGISTER_CAL_AC3, &_bmp180_coeffs.ac3);
    this->read16(BMP180_REGISTER_CAL_AC4, &_bmp180_coeffs.ac4);
    this->read16(BMP180_REGISTER_CAL_AC5, &_bmp180_coeffs.ac5);
    this->read16(BMP180_REGISTER_CAL_AC6, &_bmp180_coeffs.ac6);
    this->readS16(BMP180_REGISTER_CAL_B1, &_bmp180_coeffs.b1);
    this->readS16(BMP180_REGISTER_CAL_B2, &_bmp180_coeffs.b2);
    this->readS16(BMP180_REGISTER_CAL_MB, &_bmp180_coeffs.mb);
    this->readS16(BMP180_REGISTER_CAL_MC, &_bmp180_coeffs.mc);
    this->readS16(BMP180_REGISTER_CAL_MD, &_bmp180_coeffs.md);
  #endif

#ifdef DEBUG
	cout<<"Coefficients:"<<endl;
	cout<<"AC1 = "<<_bmp180_coeffs.ac1<<endl;
	cout<<"AC2 = "<<_bmp180_coeffs.ac2<<endl;
	cout<<"AC3 = "<<_bmp180_coeffs.ac3<<endl;
	cout<<"AC4 = "<<_bmp180_coeffs.ac4<<endl;
	cout<<"AC5 = "<<_bmp180_coeffs.ac5<<endl;
	cout<<"AC6 = "<<_bmp180_coeffs.ac6<<endl;
	cout<<"B1 = "<<_bmp180_coeffs.b1<<endl;
	cout<<"B2 = "<<_bmp180_coeffs.b2<<endl;
	cout<<"MB = "<<_bmp180_coeffs.mb<<endl;
	cout<<"MC = "<<_bmp180_coeffs.mc<<endl;
	cout<<"MD = "<<_bmp180_coeffs.md<<endl;
#endif
}

/**************************************************************************/
/*!
 * need to write readtempcmd to BMP185_CONTROL
*/
/**************************************************************************/
void Adafruit_BMP180::readRawTemperature(int32_t *temperature)
{
  #if BMP180_USE_DATASHEET_VALS
    *temperature = 27898;
  #else
    uint16_t t;
    this->writeCommand(BMP180_REGISTER_CONTROL, BMP180_REGISTER_READTEMPCMD);
    int  millisec = 5;
    struct timespec rec = {0};
    rec.tv_sec = 0;
    rec.tv_nsec = millisec * 1000000L;
    nanosleep(&rec, (struct timespec *) NULL);
    //sleep(5);
    this->read16(BMP180_REGISTER_TEMPDATA, &t);
    *temperature = t;
  #endif
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void Adafruit_BMP180::readRawPressure(int32_t *pressure)
{
  #if BMP180_USE_DATASHEET_VALS
    *pressure = 23843;
  #else
    uint8_t  p8;
    uint16_t p16;
    int32_t  p32;

    int  millisec;
    struct timespec rec = {0};
    rec.tv_sec = 0;

    this->writeCommand(BMP180_REGISTER_CONTROL, BMP180_REGISTER_READPRESSURECMD + (_bmp180Mode << 6));
    switch(_bmp180Mode)
    {
      case BMP180_MODE_ULTRALOWPOWER:
        //sleep(5);
        millisec = 5;
    	rec.tv_nsec = millisec * 1000000L;
    	nanosleep(&rec, (struct timespec *) NULL);
        break;
      case BMP180_MODE_STANDARD:
        //sleep(8);
        millisec = 8;
    	rec.tv_nsec = millisec * 1000000L;
    	nanosleep(&rec, (struct timespec *) NULL);
        break;
      case BMP180_MODE_HIGHRES:
        //sleep(14);
        millisec = 14;
    	rec.tv_nsec = millisec * 1000000L;
    	nanosleep(&rec, (struct timespec *) NULL);
        break;
      case BMP180_MODE_ULTRAHIGHRES:
      default:
        //sleep(26);
		millisec = 26;
    	rec.tv_nsec = millisec * 1000000L;
		nanosleep(&rec, (struct timespec *) NULL);
        break;
    }

    this->read16(BMP180_REGISTER_PRESSUREDATA, &p16);
    p32 = (uint32_t)p16 << 8;
    this->read8(BMP180_REGISTER_PRESSUREDATA+2, &p8);
    p32 += p8;
    p32 >>= (8 - _bmp180Mode);

#ifdef DEBUG
	cout<<"readRawPressure(): p32 = "<<p32<<endl;
#endif
    
    *pressure = p32;
  #endif
}
 
/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_BMP180::begin(Adafruit_BMP180::bmp180_mode_t mode)
{
  /* Mode boundary check */
  if ((mode > BMP180_MODE_ULTRAHIGHRES) || (mode < 0))
  {
    mode = BMP180_MODE_ULTRAHIGHRES;
#ifdef DEBUG
	cout<<"begin(): mode = "<<mode<<endl;
#endif
	
  }

  /* Make sure we have the right device */
  uint8_t id;
  this->read8(BMP180_REGISTER_CHIPID, &id);
  if(id != 0x55)
  {	
  	cout<<"BMP180::begin(): wrong device"<<endl;
    return false;
  }

  /* Set the mode indicator */
  _bmp180Mode = mode;

  /* Coefficients need to be read once for calibration*/
  this->readCoefficients();
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
void Adafruit_BMP180::getPressure(float *pressure)
{
  int32_t  ut = 0, up = 0, compp = 0;
  int32_t  x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;

  /* Get the raw pressure and temperature values */
  this->readRawTemperature(&ut);
  this->readRawPressure(&up);
  cout<<"Raw Pressure: "<<up<<endl;

  /* Temperature compensation */
  b5 = computeB5(ut);

  /* Pressure compensation */
  b6 = b5 - 4000;
  x1 = (_bmp180_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (_bmp180_coeffs.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) _bmp180_coeffs.ac1) * 4 + x3) << _bmp180Mode) + 2) >> 2;
  x1 = (_bmp180_coeffs.ac3 * b6) >> 13;
  x2 = (_bmp180_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (_bmp180_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (up - b3) * (50000 >> _bmp180Mode));

  if (b7 < 0x80000000)
  {
    p = (b7 << 1) / b4;
  }
  else
  {
    p = (b7 / b4) << 1;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  compp = p + ((x1 + x2 + 3791) >> 4);

  /* Assign compensated pressure value */
  *pressure = compp;
}

/**************************************************************************/
/*!
    @brief  Reads the temperatures in degrees Celsius
*/
/**************************************************************************/
void Adafruit_BMP180::getTemperature(float *temp)
{
  int32_t UT, X1, X2, B5;     // following ds convention
  float t;

  this->readRawTemperature(&UT);

  #if BMP180_USE_DATASHEET_VALS
    // use datasheet numbers!
    UT = 27898;
    _bmp180_coeffs.ac6 = 23153;
    _bmp180_coeffs.ac5 = 32757;
    _bmp180_coeffs.mc = -8711;
    _bmp180_coeffs.md = 2868;
  #endif
  B5 = computeB5(UT);
  t = (B5+8) >> 4;
  t /= 10;

#ifdef DEBUG
	cout<<"getTemperature(): Calibrated Temp = "<<t<<endl;
#endif

  *temp = t;
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP180::pressureToAltitude(float seaLevel, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).  Note that this
    function just calls the overload of pressureToAltitude which takes
    seaLevel and atmospheric pressure--temperature is ignored.  The original
    implementation of this function was based on calculations from Wikipedia
    which are not accurate at higher altitudes.  To keep compatibility with
    old code this function remains with the same interface, but it calls the
    more accurate calculation.

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
    @param  temp          Temperature in degrees Celsius
*/
/**************************************************************************/
float Adafruit_BMP180::pressureToAltitude(float seaLevel, float atmospheric, float temp)
{
  return this->pressureToAltitude(seaLevel, atmospheric);
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude 
    (in meters), and atmospheric pressure (in hPa).  

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP180::seaLevelForAltitude(float altitude, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  
  return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude 
    (in meters), and atmospheric pressure (in hPa).  Note that this
    function just calls the overload of seaLevelForAltitude which takes
    altitude and atmospheric pressure--temperature is ignored.  The original
    implementation of this function was based on calculations from Wikipedia
    which are not accurate at higher altitudes.  To keep compatibility with
    old code this function remains with the same interface, but it calls the
    more accurate calculation.

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
    @param  temp          Temperature in degrees Celsius
*/
/**************************************************************************/
float Adafruit_BMP180::seaLevelForAltitude(float altitude, float atmospheric, float temp)
{
  return this->seaLevelForAltitude(altitude, atmospheric);
}

Adafruit_BMP180::~Adafruit_BMP180() {}

} // bmp180 namespace
