/*
   MS5x.h - Library for accessing MS5x sensors via I2C
   Copyright (c) 2021 Matthew Bennett

   MS5x is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MS5x is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with MS5x.  If not, see <http://www.gnu.org/licenses/>.

   This code is loosely a fork of the arduino-MS5xxx library by Roman Schmitz
   / a complete overhaul of his code to improve i2c communications, CRC checks,
   provide addition unit options, and more.
*/

#ifndef MS5x_h
#define MS5x_h

#define MS5611

#include "Arduino.h"
#include <Wire.h>

// typical I2C-Address of chip
#define I2C_LOW 0x76
#define I2C_HIGH 0x77

// I2C commands of chip
#define MS5xxx_CMD_RESET	0x1E    // perform reset
#define MS5xxx_CMD_ADC_READ 0x00    // initiate read sequence
#define MS5xxx_CMD_ADC_CONV 0x40    // start conversion
#define MS5xxx_CMD_ADC_D1   0x00    // read ADC 1
#define MS5xxx_CMD_ADC_D2   0x10    // read ADC 2
#define MS5xxx_CMD_ADC_256  0x00    // set ADC oversampling ratio to 256
#define MS5xxx_CMD_ADC_512  0x02    // set ADC oversampling ratio to 512
#define MS5xxx_CMD_ADC_1024 0x04    // set ADC oversampling ratio to 1024
#define MS5xxx_CMD_ADC_2048 0x06    // set ADC oversampling ratio to 2048
#define MS5xxx_CMD_ADC_4096 0x08    // set ADC oversampling ratio to 4096
#define MS5xxx_CMD_PROM_RD  0xA0    // initiate readout of PROM registers

class MS5x
{
  protected:
	// Functions
	void ReadProm(); // Reads factory scaling data, only needs to be read once at startup (or again after sensor reset) 
	
	uint8_t Calc_CRC4(uint16_t nprom[]); // Calculate expected CRC4
	uint8_t Read_CRC4(); // Read CRC4 from sensor
	uint8_t send_cmd(uint8_t aCMD); // Sends a conversion command or read request to sensor
	
	uint32_t read_adc(); // Converts D1 and D2 into scaled values.
    	
	// Variables
	bool hasUpdates = false; // True when pressure and temperature values have updated
	
	int8_t i2caddr; // i2c address of sensor
	int8_t readStep = 0; // Used to handle polling for sensor
	
	uint8_t sampleRate = 0x08; // Oversampling rate, default is 4096.  Set using setSamples(aCMD) where aCMD is value as defined in definition area above (MS5xxx_CMD_ADC_####)
	uint8_t tType = 0; // Temperature units, 0 = C, 1 = F, 2 = K
	uint8_t pType = 0; // Pressure units, 0 = Pa, 1 = mbar, 2 = inch Hg
	
	uint16_t C[8]; // Calibration Coefficents
	
	uint32_t D1=0; // Unscaled pressure reading
	uint32_t D2=0; // Unscaled temperature reading
	uint32_t prevRead = 0; // Time (in millis) of previous read step.
	uint32_t readDelay = 10; // Time in MS to wait between read requests from the device, default is minimum required delay for maximum oversampling
	uint32_t readDelayPrev = 10; // Holds read delay value when preforming Reset.
	
	double P; // converted and scaled pressure reading
	double TEMP; // converted and scaled temperature reading
	
	TwoWire *_Wire;
	
  public:
	// Functions
	MS5x(TwoWire *aWire); // Constructor function
	
	void reset(); // Resets the sensor
	void setI2Caddr(int8_t aAddr); // Sets I2C address
	void setPressHg(); // Sets Pressure reading to Inches Murcury
	void setPressMbar(); // Sets Pressure reading to millbars (default)
	void setPressPa(); // Sets Pressure reading to Pascals
	void setTempC(); // Sets Temperature reading to Celcius (default)
	void setTempF(); // Sets Temperature reading to Fahrenheit
	void setTempK(); // Sets Temperature reading to Kelvin
	void setSamples(uint8_t aCMD);
		
	uint8_t connect(uint8_t aCMD = MS5xxx_CMD_ADC_4096); // Connects to device and sets oversampling ratio.  Default is max oversampling
	uint8_t CRCcodeTest(); // Check validity of CRC, not working currently

	bool checkCRC();
	bool checkUpdates();
	bool Readout(int32_t offset=0); // Converts Temperature and Pressure readings to Celcius and Mbar, offset is used to fix errors in temperature readings where 100 = 1.00 Celcious
	
	double GetPres();
	double GetTemp();
	
	// Variables
};

#endif
