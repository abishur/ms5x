/*
    AdvancedMS5x.ino - Shows all available function in the MS5X library
    Copyright (c) 2021 Matthew Bennett

    This file is part of arduino-MS5x.

    arduino-MS5x is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    arduino-MS5x is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with arduino-MS5x.  If not, see <http://www.gnu.org/licenses/>.

*/

//**************************************************************************************************************
//!
//! If you are using a MS5607 you need to open the header file and comment out line 24 (#define MS5611)
//! Failing to do this will result in the wrong math calculations being performed for temperature and pressure
//!
//**************************************************************************************************************

#include <Wire.h>
#include <MS5x.h>

MS5x barometer(&Wire);

bool barometerConnected = false;

uint16_t connectionAttemptDelay = 500; // If we can't connect to the sensor, wait this long prior to attempting reconnect

uint32_t prevConnectionAttempt = 0;

void setup() {
	Serial.begin(115200);
	
	/* This will set the I2C address of the sensor, acceptable values are:
	   I2C_LOW		Sets address to 0x76
	   I2C_HIGH		Sets address to 0x77 <- Default
	*/
	barometer.setI2Caddr(I2C_HIGH); 
	
	/* This will set oversampling ratio, acceptable values are:
	   MS5xxx_CMD_ADC_256 
	   MS5xxx_CMD_ADC_512 
	   MS5xxx_CMD_ADC_1024
	   MS5xxx_CMD_ADC_2048
	   MS5xxx_CMD_ADC_4096 <- Default
	*/
	barometer.setSamples(MS5xxx_CMD_ADC_2048);

  //These three lines will set the temperature units returned by GetTemp().

  //barometer.setTempC(); // Uncommenting this line will have GetTemp() return temperature in Celcius (default temperature units)
  barometer.setTempF(); // Uncommenting this line will have GetTemp() return temperature in Fahrenheit 
  //barometer.setTempK(); // Uncommenting this line will have GetTemp() return temperature in Kelvin 
  
  //These three lines will set the pressure units returned by GetPress().
  //barometer.setPressMbar(); // Uncommenting this line will have GetPress() return Pressure in Millibars (default pressure units)
  //barometer.setPressHg(); // Uncommenting this line will have GetPress() return Pressure in Inches Mercury (Inches Hg)
  barometer.setPressPa(); // Uncommenting this line will have GetPress() return Pressure in Pascals
  
	if(barometer.connect()>0) { // barometer.connect starts wire and attempts to connect to sensor
		Serial.println(F("Error connecting..."));
	} else {
		Serial.println(F("Connected to Sensor"));
		barometerConnected = true;
	}
}

void loop() {
	if (!barometerConnected) {
		if (millis() - prevConnectionAttempt >= connectionAttemptDelay) {
		
			// Retry connection attemp
			if(barometer.connect()>0) {
				Serial.println(F("Error connecting..."));
				prevConnectionAttempt = millis();
			} else {
				Serial.println(F("Connected!"));
				barometerConnected = true;
			}
		}
	} else {
		/* In order to not have any delays used in code, checkUpdates cycles through sensor read process
			Step 1: Ask for raw temperature calculation to be performed
			Step 2: Once enough time has passed for calculation ask sensor to send results
			Step 3: Ask for raw pressure calculation to be performed
			Step 4: Once enough time has passed for calculation ask sensor to send results
			At this point checkUpdates returns true, but no new sensor readings will be performed until Readout function is called. */
		if (barometer.checkUpdates()) {
		
			/* I recommend that you perfrom a CRC check just once the first time you get a sensor
				What this does is do a CRC calculation based on the factory sensor callibration coefficents
				and then compares it to the CRC value stored in the sensor.  If they entered the data correctly
				then this function returns true.  Note, this doesn't guarentee that the sensor is calibrated correctly really.
				All it does is confirm they entered the CRC correctly based on the values they burned into sensors PROM.
				But if this comes back false then you definitely shouldn't trust it.
			*/
			bool crcRes = barometer.checkCRC();
			Serial.print(F("Sensor CRC Check Results: "));
			Serial.println(crcRes);
		
			/* This will confirm that the code to check the CRC values are accurate.  Sample PROM values were taken from
			TE-Connectivity tech note AN520.  If this returns anything except 0xB, then I  have a problem with the checkCRC() function.
			*/
			uint8_t crcTest = barometer.CRCcodeTest();
			Serial.println(F("This function should return 0xB."));
			Serial.print(F("CRC code check returned: 0x"));
			Serial.println(crcTest, HEX);
			
			if (barometer.Readout()) { // Updates Temperature and Pressure values for reading.  Returns false if sensor calculations are not finished.
				double temperature = barometer.GetTemp(); // Returns temperature in C
				double pressure = barometer.GetPres(); // Returns pressure in Mbar
				Serial.print(F("The Temperature is: "));
				Serial.println(temperature);
				Serial.print(F("The Pressure is: "));
				Serial.println(pressure);
			}
		}
	}
}