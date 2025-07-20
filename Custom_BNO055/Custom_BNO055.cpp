/**
	Custom_BNO055.cpp
	
	For my custom electronics aboard Zenith (tentative name hehe).
	
	Author: Brett Warren
	
*/

#ifndef CUSTOM_BNO055_CPP
#define CUSTOM_BNO055_CPP

#include <Arduino.h>
#include <Wire.h>
#include "Custom_BNO055.h"

// Public functions.

Custom_BNO055::Custom_BNO055(uint8_t addr) {
	memset(_allData, 0, sizeof(_allData);
	_addr = addr;
}

bool Custom_BNO055::begin() {
	// TODO: Figure out preliminary communications to be performed.
	_error = _startingReadAddr = _currentRegisterPage = 0x00;
	delay(450);  // Wait for the IMU's power-up sequence to finish (should take ~400ms, but I decided to add some margin).
	if (readSingleRegister(PAGE_0_REG::CHIP_ID) != 0xA0) {  // This is a dummy read to ensure that communication has been established.
		return false;
	}
	return true;
}

bool Custom_BNO055::begin(OP_MODE mode) {
	if (begin()) {
		setMode(mode);
		return true;
	}
	return false;
}

void Custom_BNO055::setMode(OP_MODE mode) {
	writeSingleRegister(PAGE_0_REG::OPR_MODE, (uint8_t)mode);
}

void Custom_BNO055::setAxisRemap(uint8_t remap, uint8_t sign) {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::setAccelConfig(uint8_t gRange, uint8_t bandwidth, uint8_t opMode) {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::setGyroConfig(uint8_t range, uint8_t bandwidth, uint8_t opMode) {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::setMagConfig(uint8_t rate, uint8_t opMode, uint8_t pwrMode) {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::setAccelUnit(ACCEL_UNIT unit) {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::setAngularRateUnit(ANGULAR_RATE_UNIT unit) {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::setEulerUnit(EULER_UNIT unit) {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::setTempUnit(TEMPERATURE_UNIT unit) {
	// FIXME: Function not properly implemented lol
}

uint8_t* Custom_BNO055::updateMagData() {
	return readMultipleRegisters(PAGE_0_REG::MAG_DATA_X_LSB, _magData, 6);
}

uint8_t* Custom_BNO055::updateGyroData() {
	return readMultipleRegisters(PAGE_0_REG::GYR_DATA_X_LSB, _gyroData, 6);
}

uint8_t* Custom_BNO055::updateAccelData() {
	return readMultipleRegisters(PAGE_0_REG::ACC_DATA_X_LSB, _accelData, 6);
}

uint8_t* Custom_BNO055::updatEulerData() {
	return readMultipleRegisters(PAGE_0_REG::EUL_DATA_X_LSB, _eulerData, 6);
}

uint8_t* Custom_BNO055::updateQuaternionData() {
	return readMultipleRegisters(PAGE_0_REG::QUA_DATA_W_LSB, _quaternionData, 8);
}

uint8_t* Custom_BNO055::updateLinearAccelData() {
	return readMultipleRegisters(PAGE_0_REG::LIA_DATA_X_LSB, _linAccelData, 6);
}

uint8_t* Custom_BNO055::updateGravityVectorData() {
	return readMultipleRegisters(PAGE_0_REG::GRV_DATA_X_LSB, _gravityVectorData, 6);
}

uint8_t* Custom_BNO055::updateTemperatureData() {
	return readSingleRegister(PAGE_0_REG::TEMP);
}

uint8_t* Custom_BNO055::updateAllData() {
	readMultipleRegisters(PAGE_0_REG::ACC_DATA_X_LSB, _allData, 24);  // The Wire library's 32-byte read buffer necessitates splitting the 45-byte read into two function calls.
	readMultipleRegisters(PAGE_0_REG::QUA_DATA_W_LSB, _quaternionData, 21);
	return _allData;
}

void Custom_BNO055::configAccelSMNMInterrupt() {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::configAccelAMInterrupt() {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::configAccelHighGInterrupt() {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::configGyroHRInterrupt() {
	// FIXME: Function not properly implemented lol
}

void Custom_BNO055::configGyroAMInterrupt() {
	// FIXME: Function not properly implemented lol
}

uint8_t Custom_BNO055::getLastSelfTest() {
	return readRegister(PAGE_0_REG::ST_RESULT);
}

void Custom_BNO055::triggerSelfTest() {
	// FIXME: Function not properly implemented lol
}

uint8_t Custom_BNO055::getAccelCalibration() {
	// FIXME: Function not properly implemented lol
	return 0;
}

uint8_t Custom_BNO055::getGyroCalibration() {
	// FIXME: Function not properly implemented lol
	return 0;
}

uint8_t Custom_BNO055::getMagCalibration() {
	// FIXME: Function not properly implemented lol
	return 0;
}

bool Custom_BNO055::isFullyCalibrated() {
	// FIXME: Function not properly implemented lol
	return true;
}


// Private functions.

void Custom_BNO055::setDataUnits() {
	uint8_t result = 0x00;
	result |= (uint8_t)_accelUnit | (uint8_t)_angRateUnit | (uint8_t)_eulerUnit | (uint8_t)_tempUnit | (uint8_t)_fusionOutputFormat;
	writeSingleRegister(PAGE_0_REG::UNIT_SEL, result);
}
		
uint8_t Custom_BNO055::readSingleRegister(PAGE_0_REG reg) {
	uint8_t regNum = (uint8_t)reg;
	if (_currentRegisterPage) {
		_currentRegisterPage = writeSingleRegister(PAGE_ID_REG, 0x00);
	}
	return readSingleRegister(regNum);
}
		
uint8_t Custom_BNO055::readSingleRegister(PAGE_1_REG reg) {
	uint8_t regNum = (uint8_t)reg;
	if (!_currentRegisterPage) {
		_currentRegisterPage = writeSingleRegister(PAGE_ID_REG, 0x01);
	}
	return readSingleRegister(regNum);
}

uint8_t Custom_BNO055::readSingleRegister(uint8_t reg) {
	if (_startingReadAddr != reg) {
		Wire.beginTransmission(_addr);
		Wire.write(reg);  // Update the starting read address if necessary.
		Wire.endTransmission();
		_startingReadAddr = reg;
	}
	unsigned long ctr = millis();
	Wire.requestFrom(_addr, 1);
	while (!Wire.available()) {
		if (millis() > ctr + 5) {  // If the device takes too long to respond, there must be a problem.
			_error |= ERROR_CODE::COMM_ERROR;
			return 0x00;
		}
	}
	return Wire.read();
}

uint8_t Custom_BNO055::writeSingleRegister(PAGE_0_REG reg, uint8_t val) {
	uint8_t regNum = (uint8_t)reg;
	if (_currentRegisterPage) {
		_currentRegisterPage = writeSingleRegister(PAGE_ID_REG, 0x00);
	}
	return writeSingleRegister(regNum, val);
}
		
uint8_t Custom_BNO055::writeSingleRegister(PAGE_1_REG reg, uint8_t val) {
	uint8_t regNum = (uint8_t)reg;
	if (!_currentRegisterPage) {
		_currentRegisterPage = writeSingleRegister(PAGE_ID_REG, 0x01);
	}
	return writeSingleRegister(regNum, val);
}
		
uint8_t Custom_BNO055::writeSingleRegister(uint8_t reg, uint8_t val) {
	_startingReadAddr = reg;
	Wire.beginTransmission(_addr);
	Wire.write(reg);
	Wire.write(val);
	Wire.endTransmission();
	return val;
}

uint8_t* Custom_BNO055::readMultipleRegisters(PAGE_0_REG startReg, uint8_t* dest, int num) {
	uint8_t regNum = (uint8_t)startReg;
	if (_currentRegisterPage) {
		_currentRegisterPage = writeSingleRegister(PAGE_ID_REG, 0x00);
	}
	if (_startingReadAddr != regNum) {
		Wire.beginTransmission(_addr);
		Wire.write(regNum);  // Update the starting read address if necessary.
		Wire.endTransmission();
		_startingReadAddr = regNum;
	}
	
	return readMultipleRegisters(dest, num);
}
		
uint8_t* Custom_BNO055::readMultipleRegisters(PAGE_1_REG startReg, uint8_t* dest, int num) {
	uint8_t regNum = (uint8_t)startReg;
	if (!_currentRegisterPage) {
		_currentRegisterPage = writeSingleRegister(PAGE_ID_REG, 0x01);
	}
	if (_startingReadAddr != regNum) {
		Wire.beginTransmission(_addr);
		Wire.write(regNum);  // Update the starting read address if necessary.
		Wire.endTransmission();
		_startingReadAddr = regNum;
	}
	
	return readMultipleRegisters(dest, num);
}

uint8_t* Custom_BNO055::readMultipleRegisters(uint8_t* dest, int num) {
	const int temp = num;
	int i = 0;
	int result[temp];
	unsigned long ctr = millis();
	Wire.requestFrom(_addr, num);
	while (Wire.available() < num) {
		if (millis() > ctr + 10) {  // If the device takes too long to respond, something bad must have happened.
			_error |= ERROR_CODE::COMM_ERROR;
			return dest;
		}
	}
	while (Wire.available()) {
		result[++i] = Wire.read();
	}
	return memcpy(dest, result, num);
}

#endif