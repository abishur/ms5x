/*
    AdvancedMS5x.ino - Shows how to quickly connect to MS56xx sensors and
	perform a quick CRC check to ensure sensor was calibrated prior to leaving factory.
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

void setup() {
	Serial.begin(115200);
	while(barometer.connect()>0) { // barometer.connect starts wire and attempts to connect to sensor
		Serial.println(F("Error connecting..."));
    delay(500);
	}
		Serial.println(F("Connected to Sensor"));

}

void loop() {
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
