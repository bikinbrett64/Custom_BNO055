#include <Arduino.h>
#include <Wire.h>
#include "Custom_BNO055.h"

// Public functions.

Custom_BNO055::Custom_BNO055(uint8_t addr) {
	memset(_allData, 0, 22 * sizeof(int16_t));
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

int16_t* Custom_BNO055::updateMagData() {
	
}

int16_t* Custom_BNO055::updateGyroData() {
	
}

int16_t* Custom_BNO055::updateAccelData() {
	
}

int16_t* Custom_BNO055::updatEulerData() {
	
}

int16_t* Custom_BNO055::updateLinearAccelData() {
	
}

int16_t* Custom_BNO055::updateGravityVectorData() {
	
}

int16_t* Custom_BNO055::updateTemperatureData() {
	
}

int16_t* Custom_BNO055::updateAllData() {
	
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
	return 0;
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
	
}
		
uint8_t Custom_BNO055::readSingleRegister(PAGE_0_REG reg) {
	uint8_t regNum = (uint8_t)reg;
	if (_currentRegisterPage) {
		
	}
	
	
	
}
		
uint8_t Custom_BNO055::readSingleRegister(PAGE_1_REG reg) {
	
}

uint8_t Custom_BNO055::readSingleRegister(uint8_t reg) {
	Wire.beginTransmission(_addr);
	Wire.write(reg);
	Wire.endTransmission();
	
	_startingReadAddr = reg;
	
	Wire.requestFrom(_addr, 1);
	while (!Wire.available());
	return Wire.read();
}