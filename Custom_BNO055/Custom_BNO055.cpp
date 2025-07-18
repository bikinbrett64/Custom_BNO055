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
	
}

void Custom_BNO055::setAxisRemap(uint8_t remap, uint8_t sign) {
	
}

void Custom_BNO055::setAccelConfig(uint8_t gRange, uint8_t bandwidth, uint8_t opMode) {
	
}

void Custom_BNO055::setGyroConfig(uint8_t range, uint8_t bandwidth, uint8_t opMode) {
	
}

void Custom_BNO055::setMagConfig(uint8_t rate, uint8_t opMode, uint8_t pwrMode) {
	
}

void Custom_BNO055::setAccelUnit(ACCEL_UNIT unit) {
	
}

void Custom_BNO055::setAngularRateUnit(ANGULAR_RATE_UNIT unit) {
	
}

void Custom_BNO055::setEulerUnit(EULER_UNIT unit) {
	
}

void Custom_BNO055::setTempUnit(TEMPERATURE_UNIT unit) {
	
}

uint8_t* Custom_BNO055::updateMagData() {
	return _magData;
}

uint8_t* Custom_BNO055::updateGyroData() {
	return _gyroData;
}

uint8_t* Custom_BNO055::updateAccelData() {
	return _accelData;
}

uint8_t* Custom_BNO055::updatEulerData() {
	return _eulerData;
}

uint8_t* Custom_BNO055::updateLinearAccelData() {
	return _linAccelData;
}

uint8_t* Custom_BNO055::updateGravityVectorData() {
	return _gravityVectorData;
}

uint8_t* Custom_BNO055::updateTemperatureData() {
	return _temperatureData;
}

uint8_t* Custom_BNO055::updateAllData() {
	return _allData;
}

void Custom_BNO055::configAccelSMNMInterrupt() {
	
}

void Custom_BNO055::configAccelAMInterrupt() {
	
}

void Custom_BNO055::configAccelHighGInterrupt() {
	
}

void Custom_BNO055::configGyroHRInterrupt() {
	
}

void Custom_BNO055::configGyroAMInterrupt() {
	
}

uint8_t Custom_BNO055::getLastSelfTest() {
	return readRegister(ST_RESULT);
}

void Custom_BNO055::triggerSelfTest() {
	
}

uint8_t Custom_BNO055::getAccelCalibration() {
	return 0;
}

uint8_t Custom_BNO055::getGyroCalibration() {
	return 0;
}

uint8_t Custom_BNO055::getMagCalibration() {
	return 0;
}

bool Custom_BNO055::isFullyCalibrated() {
	return true;
}


// Private functions.

void Custom_BNO055::setDataUnits() {
	uint8_t result = 0x00;
	result |= (uint8_t)_accelUnit | (uint8_t)_angRateUnit | (uint8_t)_eulerUnit | (uint8_t)_tempUnit | (uint8_t)_fusionOutputFormat;
	writeSingleRegister(UNIT_SEL, result);
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
	
	Wire.requestFrom(_addr, 1);
	while (!Wire.available());
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
	if (_startingReadAddr != reg) {
		Wire.beginTransmission(_addr);
		Wire.write(reg);  // Update the starting read address if necessary.
		Wire.endTransmission();
		_startingReadAddr = reg;
	}
	
	return readMultipleRegisters(regNum, dest, num);
}
		
uint8_t* Custom_BNO055::readMultipleRegisters(PAGE_1_REG startReg, uint8_t* dest, int num) {
	uint8_t regNum = (uint8_t)startReg;
	if (!_currentRegisterPage) {
		_currentRegisterPage = writeSingleRegister(PAGE_ID_REG, 0x01);
	}
	if (_startingReadAddr != reg) {
		Wire.beginTransmission(_addr);
		Wire.write(reg);  // Update the starting read address if necessary.
		Wire.endTransmission();
		_startingReadAddr = reg;
	}
	
	return readMultipleRegisters(regNum, dest, num);
}

uint8_t* Custom_BNO055::readMultipleRegisters(uint8_t startReg, uint8_t* dest, int num) {
	const int temp = num;
	int i = 0;
	int result[temp];
	Wire.requestFrom(_addr, num);
	while (Wire.available() < num);
	while (Wire.available()) {
		result[++i] = Wire.read();
	}
	return memcpy(dest, result, num);
}

#endif