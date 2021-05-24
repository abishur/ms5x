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

   This code is a fork of the arduino-MS5xxx library by Roman Schmitz
*/

#ifndef MS5x_h
#define MS5x_h

#define MS5611 // Comment out this line if you are using the MS5607

#include "Arduino.h"
#include <Wire.h>

// typical I2C-Address of chip
#define I2C_MS5607 0x77
#define I2C_MS5611 0x77

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
	unsigned char send_cmd(uint8_t aCMD); // Sends a conversion command or read request to sensor
	uint32_t read_adc(); // Converts D1 and D2 into scaled values.
	uint8_t Calc_CRC4(uint16_t nprom[]); // Calculate expected CRC4
    	uint8_t Read_CRC4(); // Read CRC4 from sensor
	void ReadProm(); // Reads factory scaling data, only needs to be read once at startup (or again after sensor reset) 
	
	// Variables
	bool hasUpdates = false; // True when pressure and temperature values have updated
	int8_t readStep = 0; // Used to handle polling for sensor
	int8_t i2caddr; // i2c address of sensor
	uint8_t sampleRate = 0x08; // Oversampling rate, default is 4096.  Set using setSamples(aCMD) where aCMD is value as defined in definition area above (MS5xxx_CMD_ADC_####)
	
	uint16_t C[8]; // Calibration Coefficents
	
	uint32_t D1=0, D2=0; // D1 = unconverted pressure reading, D2 = unconverted temperature reading
	uint32_t prevRead = 0; // Time (in millis) of previous read step.
	uint32_t readDelay = 10; // Time in MS to wait between read requests from the device, default is minimum required delay for maximum oversampling
	uint32_t readDelayPrev = 10; // Holds read delay value when preforming Reset.
	
	double P; // converted and scaled pressure reading
	double TEMP; // converted and scaled temperature reading
	
	TwoWire *_Wire;
	
  public:
	// Functions
    	MS5x(TwoWire *aWire); // Constructor function
	
    	void setI2Caddr(int8_t aAddr); // Sets I2C address
    	uint8_t connect(uint8_t aCMD = MS5xxx_CMD_ADC_4096); // Connects to device and sets oversampling ratio.  Default is max oversampling
    
    	bool Readout(int32_t offset=0); // Converts Temperature and Pressure readings to Celcius and Mbar, offset is used to fix errors in temperature readings where 100 = 1.00 Celcious
	
	bool checkCRC();
    	uint8_t CRCcodeTest(); // Check validity of CRC, not working currently
	
	void setSamples(uint8_t aCMD);
	bool checkUpdates();
    
    	double GetTemp();
    	double GetPres();
	
	// Variables
};

#endif
