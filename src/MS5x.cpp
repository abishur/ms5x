/*
    MS5x.h - Library for accessing MS5x sensors via I2C
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
    
    This code is loosely a fork of the arduino-MS5xxx library by Roman Schmitz
    / a complete overhaul of his code to improve i2c communications, CRC checks,
    provide addition unit options, and more.

*/

#include "MS5x.h"

//******************************************************** 
//! @brief constructor sets ic2address to default of 0x76
//! 
//! @return N/A
//********************************************************
MS5x::MS5x(TwoWire *aWire) : i2caddr(I2C_HIGH) {
	_Wire=aWire;
}

//******************************************************** 
//! @brief reads factory calibration data, technically only needs to be called once
//! after sensor starts up or resets (or when calculating CRC?)
//! 
//! @return void
//********************************************************
void MS5x::ReadProm() {
	  
	for(uint8_t i=0;i<8;i++) 
	{
	    C[i]=0x0000;
	    send_cmd(MS5xxx_CMD_PROM_RD+2*i);
	    _Wire->requestFrom(i2caddr, 2);

	    unsigned int c = _Wire->read();
	    C[i] = (c << 8);
	    c = _Wire->read();
	    C[i] += c;
	}
	
}

void MS5x::reset() {
	// On device reset clear hasUpdates flag if set and reset device
	readStep = -1;
	hasUpdates = false;
	readDelayPrev = readDelay;
	readDelay = 3; // After a reset wait 3 ms before executing any commands.
	send_cmd(MS5xxx_CMD_RESET);
	prevRead = millis();
	checkUpdates(); //Force checkUpdates to ensure proper order of code execution
}

//******************************************************** 
//! @brief sets I2C address 
//! 
//! @return void
//********************************************************
void MS5x::setI2Caddr(int8_t aAddr) {
	i2caddr=aAddr;
}

//******************************************************** 
//! @brief sets Pressure reading to be in millbars 
//! 
//! @return void
//********************************************************
void MS5x::setPressMbar() {
	if (P != 0 and pType != 1) { // Only convert if a new type is specified and Pressure has a value
		switch (pType) {
			case 0: P *= .01f; break; // Pa to mbar
			case 2: P /= 0.029529983071; break; // inches Hg to mbar
		}
	}
	pType = 0;
}

//******************************************************** 
//! @brief sets Pressure reading to be in Pascalls (default)
//! 
//! @return void
//********************************************************
void MS5x::setPressPa() {
	if (P != 0 and pType != 0) { // Only convert if a new type is specified and Pressure has a value
		switch (pType) {
			case 1: P *= 100.0f; break; // mbar to Pa
			case 2: P *= 3386.3886667; break; // inches Hg to Pa
		}
	}
	pType = 1;
}

//******************************************************** 
//! @brief sets Pressure reading to be in Inches Mecury
//! 
//! @return void
//********************************************************
void MS5x::setPressHg() {
	if (P != 0 and pType != 2) { // Only convert if a new type is specified and Pressure has a value
		switch (pType) {
			case 0: P /= 3386.3886667; break; // Pa to inches Hg
			case 1: P /= 33.863886667; break; // mbar to inches Hg
		}
	}
	pType = 2;
}

//******************************************************** 
//! @brief sets Temperature reading to be in Celcius (default)
//! 
//! @return void
//********************************************************
void MS5x::setTempC() {
	if (TEMP != 0 and tType != 0) { // Only convert if a new type is specified and temp has a value
		switch (tType) { // Convert from existing units to C
			case 1: TEMP = (TEMP - 32.0f) * 5.0f / 9.0f; break; // F to C
			case 2: TEMP -= 273.15f; break; // K to C
		}
	}
	tType = 0;
}

//******************************************************** 
//! @brief sets Temperature reading to be in Fahrenheit
//! 
//! @return void
//********************************************************
void MS5x::setTempF(){
	if (TEMP != 0 and tType != 1) { // Only convert if a new type is specified and temp has a value
		switch (tType) { // Convert from existing units to C
			case 0: TEMP = TEMP * 9.0f/5.0f +32.0f; break; // C to F
			case 2: TEMP = TEMP * 9.0f / 5.0f - 459.67; break; // K to F
		}
	}
	tType = 1;
}

//******************************************************** 
//! @brief sets Temperature reading to be in Kelvin
//! 
//! @return void
//********************************************************
void MS5x::setTempK() {
	if (TEMP != 0 and tType != 2) { // Only convert if a new type is specified and temp has a value
		switch (tType) { // Convert from existing units to C
			case 0: TEMP += 273.15f;; break; // C to K
			case 1: TEMP = (TEMP + 459.67) * 5.0f/9.0f; break; // F to K
		}
	}
	tType = 2;
}


//******************************************************** 
//! @brief Sets the desired oversampling ratio along with minimum required delays rates
//! before conversions can be read from sensor.
//! 
//! @return void
//********************************************************
void MS5x::setSamples(uint8_t samples) {
	switch (samples)
  {
    case MS5xxx_CMD_ADC_256 : 
		readDelay=1;
		sampleRate = 0x00;
		break;
    case MS5xxx_CMD_ADC_512 : 
		readDelay=3;
		sampleRate = 0x02;
		break;
    case MS5xxx_CMD_ADC_1024: 
		readDelay=4;
		sampleRate = 0x02;
		break;
    case MS5xxx_CMD_ADC_2048: 
		readDelay=6;
		sampleRate = 0x06;
		break;
    case MS5xxx_CMD_ADC_4096: 
		readDelay=10;
		sampleRate = 0x08;
		break;
	}
}

//******************************************************** 
//! @brief Calculates CRC based on 16 bits reserved for manufacturer,
//! 6 calibration coefficents, and final CRC address.
//! when called from checkCRC(), it can be used to confirm that sensor had
//! CRC correctly set to calibration data in factory.
//! Please reference TE Connectivity tech note AN520 for more information
//! 
//! @return uint8_t (CRC calculation)
//********************************************************
uint8_t MS5x::Calc_CRC4(uint16_t n_prom[])
{
    uint8_t n_bit;
    int16_t cnt; // Simple counter
    uint16_t n_rem; // CRC remainder
    uint16_t crc_read; // Original value of the CRC

    n_rem = 0x00;

    crc_read = n_prom[7];	// Save original value of CRC
    n_prom[7] = (0xFF00 & (n_prom[7])); // CRC byte is replaced by 0
    for ( cnt = 0; cnt < 16; cnt++ ) // Operation is performed on bytes
    {
		// choose LSB or MSB
        if ( cnt % 2 == 1 ) n_rem ^= (uint8_t)((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (uint8_t)(n_prom[cnt>>1]>>8);

        for (n_bit=8; n_bit>0; n_bit--)
        {
            if (n_rem & (0x8000))
            {
            	n_rem=(n_rem<<1) ^ 0x3000;
            }
            else
            {
                n_rem=(n_rem<<1);
            }
        }
    }
    
    n_rem = (0x000F & (n_rem >> 12)); // final 4-bit remainder is CRC code
	n_prom[7] = crc_read; // restore the CRC_READ to it original place
    return (n_rem ^ 0x00);
}

//******************************************************** 
//! @brief intializes connection to sensor and sets up defauls values
//! 
//! @return byte (i2c status)
//********************************************************
uint8_t MS5x::connect(uint8_t aCMD) {
	prevRead = millis(); // Initialize timer
	setSamples(aCMD); // If a sample ratio has been provided, set sample ratio and delay
	_Wire->begin();
	_Wire->beginTransmission(i2caddr);
	uint8_t ret=_Wire->endTransmission(true);
	reset(); // After starup, reset the device, and read PROM
	return ret;
}

//******************************************************** 
//! @brief Used to make sure the Calc_CRC4 function was working correctly
//! serves no function in actual program beyond showing an example of how CRC check works
//! 
//! @return uint8_t (result of Calc_CRC4 using assumed values for sensor calibration coefficents
//! if my Calc_CRC4 code is correct, this funciton should always return 0xB)
//********************************************************
uint8_t MS5x::CRCcodeTest(){
	uint16_t nprom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500}; //expected output is 0xB
	uint8_t crc = Calc_CRC4(nprom); //expected output is 0xB
	//ReadProm();
	return crc;
}

//******************************************************** 
//! @brief Reads CRC value from sensor
//! 
//! @return uint8_t (Stored CRC value in sensor)
//********************************************************
uint8_t MS5x::Read_CRC4()
{

    unsigned int crc_read = ( 0x000F & ( C[ 7 ] ) );
    return ( crc_read );
}

//******************************************************** 
//! @brief sends command to sensor
//! 
//! @return byte (i2c status)
//********************************************************
uint8_t MS5x::send_cmd(uint8_t aCMD)
{
  _Wire->beginTransmission(i2caddr);
  _Wire->write(aCMD);
  uint8_t ret=_Wire->endTransmission(true);
  return ret;
}

//******************************************************** 
//! @brief Reads unscaled temperature or pressure data from sensor
//! based on previous convserion command sent. 
//! 
//! @return uint32_t (Results of previous calculation.  If
//! data requested prior to calculation finishing, sensor ressponds with 0)
//********************************************************
uint32_t MS5x::read_adc()
{
  unsigned long value=0;
  unsigned long r=0;

  send_cmd(MS5xxx_CMD_ADC_READ);
  _Wire->requestFrom(i2caddr, 3);
  r = _Wire->read();
  value = (r<<16);
  r = _Wire->read();
  value += (r<<8);
  r = _Wire->read();
  value += r;
 
  return value;
}

//******************************************************** 
//! @brief performs calculation of CRC based on factory set calibration coefficents and confirms it is
//! equal to the factory set CRC value.
//! What does this actually do?  It's not like it confirms sensor was calibrated correctly in the factory,
//! just that the CRC was calculated correctly based on whatever values they put into calibration coefficents.
//! If this comes back false, they couldn't even get that level of check right and the sensor definitely shouldn't
//! be trusted.  If it comes back true... it's not like this actually proves they calibrated it correctly.
//! 
//! @return bool (True if 
//********************************************************
bool MS5x::checkCRC()  {
	bool statusRet = false;
	if ((Calc_CRC4(C) == Read_CRC4())) statusRet = true;
	return statusRet;
}


//******************************************************** 
//! @brief Controls checking the sensor for updates.  When conversions are completed
//! based on oversampling settings, function returns true.  No additional conversions will be
//! calculated until Readout() function is called.
//! 
//! @return bool (sensor has finished processing sample calculations)
//********************************************************
bool MS5x::checkUpdates() {
	uint32_t currMillis = millis();
	if (readStep > 4) readStep = 0;
	if ((uint32_t)(currMillis - prevRead) >= readDelay) {
		switch (readStep)
		{
			case -1: // -1 is set when reset() function is called.  Restores read delay settings
				ReadProm(); // Read Prom doesn't need to be called each cycle
				readDelay = readDelayPrev;
				readStep += 1;
				prevRead = currMillis;
				break;
			case 0: // Send command to do Temperature calculations
				send_cmd(MS5xxx_CMD_ADC_CONV+MS5xxx_CMD_ADC_D2+sampleRate);
				D1 = D2 = 0;
				readStep += 1;
				prevRead = currMillis;
				break;
			case 1: // Send command to read Temperature
				D2 = read_adc();
				readStep += 1;
				prevRead = currMillis;
				break;
			case 2: // Send command to do Pressure calculations
				send_cmd(MS5xxx_CMD_ADC_CONV+MS5xxx_CMD_ADC_D1+sampleRate);
				readStep += 1;
				prevRead = currMillis;
				break;
			case 3: // Send command to read Temperature
				D1 = read_adc();
				readStep += 1;
				hasUpdates = true;
				break;
		}			
	}
	return hasUpdates;
}

//******************************************************** 
//! @brief Converts unscaled temperature and pressure readings into scaled values
//! pressure reading is temperature compensated.
//! 
//! @return bool (false if readout was called prior to new values being available)
//********************************************************
bool MS5x::Readout(int32_t offset) {
	if (!hasUpdates) return false;
	
	// While sensor document states these variables should be int32 and int64 unless everything is
	// int64 calculation errors occur.  To ensure compatibility across multiple platforms, I've chosen to
	// make their types as doubles.
	double dT; // Difference in actual vs reference temperature
	double OFF; // Offset at actual temperature
	double SENS; //Sensitivy at actual temperature

	// calculate 1st order pressure and temperature (MS5607 1st order algorithm)
	dT=(float)D2-(float)C[5]*256.0f; // D2 - Tref = D2 - C5 * 2^8
	
	#ifdef MS5611
		OFF=(float)C[2]*65536.0f+(dT*(float)C[4])/128.0f; // OFFt1 + TCO * dT = C2 * 2^16 + (C4 * dT) / 2^7
		SENS=(float)C[1]*32768.0f+(dT*(float)C[3])/256.0f; // SENSt1 + TCS * dT = C1 * 2^15 + (C3*dT) / 2^8
	#else
		OFF=(float)C[2]*131072.0f+(float)dT*C[4]/64.0f; // OFFt1 + TCO * dT = C2 * 2^17 + (C4 * dT) / 2^6
		SENS=(float)C[1]*65536.0f+(float)dT*C[3]/128.0f; // SENSt1 + TCS * dT = C1 * 2^16 + (C3*dT) / 2^7
	#endif
	
	TEMP=(2000.0f+(dT*(float)C[6])/8388608.0f)+(float)offset; // 20C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
	
	 
	// perform higher order corrections
	double T2=0., OFF2=0., SENS2=0.;
	if(TEMP<2000) {
		T2=dT*dT/2147483648.0f; // dT^2 / 2^31
		#ifdef MS5611
			OFF2=5.0f*((TEMP-2000.0f)*(TEMP-2000.0f))/2.0f; // 5 * (TEMP - 2000)^2 / 2
			SENS2=5.0f*((TEMP-2000.0f)*(TEMP-2000.0f))/4.0f; // 5 * (TEMP - 2000)^2 / 4
		#else
			OFF2=61.0f*(TEMP-2000.0f)*(TEMP-2000.0f)/16.0f; // 61 * (TEMP - 2000)^2 / 4
			SENS2=2.0f*(TEMP-2000.0f)*(TEMP-2000.0f); // 2 * (TEMP - 2000)^2
		#endif
	  
	  if(TEMP<-1500.0f) {
		#ifdef MS5611
			OFF2+=7.0f*((TEMP+1500.0f)*(TEMP+1500.0f)); // OFF2 + 7 * (TEMP + 1500)^2
			SENS2+=11.0f*((TEMP+1500.0f)*(TEMP+1500.0f))/2.0f; // SENS2 + 11 * (TEMP + 1500) ^2 /2
		#else
			OFF2+=15.0f*(TEMP+1500.0f)*(TEMP+1500.0f);
			SENS2+=8.0f*(TEMP+1500.0f)*(TEMP+1500.0f);
		#endif
	    
	  }
	}
	  
	TEMP-=T2;
	OFF-=OFF2;
	SENS-=SENS2;
	P=(D1*SENS/2097152.0f-OFF)/32768.0f; // (((D1*SENS)/pow(2,21)-OFF)/pow(2,15))

	TEMP *= 0.01f;
	switch (pType)
	{
		case 0: break; // millibar
		case 1: P *= 100.0f; break; // Pascal
		case 2: P /= 33.863886667; break; // Inches Hg
	}
	P *= 0.01f;	
	
	switch (tType)
	{
		case 0: break; // Celcius
		case 1: TEMP = TEMP * 9.0f/5.0f +32.0f; break; // Fahrenheit
		case 2: TEMP += 273.15f; break; // Kelvin
	}
	
	readStep += 1;
	prevRead = millis();
	hasUpdates = false;
	
	return true;
}

//******************************************************** 
//! @brief Returns sensor pressure, must be updated by calling Readout()
//! 
//! @return double (Sensor's Pressure)
//********************************************************
double MS5x::GetPres() {
	return P;
}

//******************************************************** 
//! @brief Returns sensor temperature, must be updated by calling Readout()
//! 
//! @return double (Sensor's Temperature)
//********************************************************
double MS5x::GetTemp() {
	return TEMP;
}
