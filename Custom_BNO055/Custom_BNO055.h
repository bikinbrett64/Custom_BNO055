/**
	Custom_BNO055.h
	
	For my custom electronics aboard Zenith (tentative name hehe).
	
	Author: Brett Warren
	
*/

#ifndef CUSTOM_BNO055_H
#define CUSTOM_BNO055_H

#include <Arduino.h>
#include <Wire.h>

/// Register map page 0
enum PAGE_0_REG {
	CHIP_ID = 0x00,
	ACC_ID = 0x01,
	MAG_ID = 0x02,
	GYR_ID = 0x03,
	SW_REV_ID_LSB = 0x04,
	SW_REV_ID_MSB = 0x05,
	BL_REV_ID = 0x06,
	PAGE_ID = 0x07,
	ACC_DATA_X_LSB = 0x08,
	ACC_DATA_X_MSB = 0x09,
	ACC_DATA_Y_LSB = 0x0A,
	ACC_DATA_Y_MSB = 0x0B,
	ACC_DATA_Z_LSB = 0x0C,
	ACC_DATA_Z_MSB = 0x0D,
	MAG_DATA_X_LSB = 0x0E,
	MAG_DATA_X_MSB = 0x0F,
	MAG_DATA_Y_LSB = 0x10,
	MAG_DATA_Y_MSB = 0x11,
	MAG_DATA_Z_LSB = 0x12,
	MAG_DATA_Z_MSB = 0x13,
	GYR_DATA_X_LSB = 0x14,
	GYR_DATA_X_MSB = 0x15,
	GYR_DATA_Y_LSB = 0x16,
	GYR_DATA_Y_MSB = 0x17,
	GYR_DATA_Z_LSB = 0x18,
	GYR_DATA_Z_MSB = 0x19,
	EUL_DATA_X_LSB = 0x1A,
	EUL_DATA_X_MSB = 0x1B,
	EUL_DATA_Y_LSB = 0x1C,
	EUL_DATA_Y_MSB = 0x1D,
	EUL_DATA_Z_LSB = 0x1E,
	EUL_DATA_Z_MSB = 0x1F,
	QUA_DATA_W_LSB = 0x20,
	QUA_DATA_W_MSB = 0x21,
	QUA_DATA_X_LSB = 0x22,
	QUA_DATA_X_MSB = 0x23,
	QUA_DATA_Y_LSB = 0x24,
	QUA_DATA_Y_MSB = 0x25,
	QUA_DATA_Z_LSB = 0x26,
	QUA_DATA_Z_MSB = 0x27,
	LIA_DATA_X_LSB = 0x28,
	LIA_DATA_X_MSB = 0x29,
	LIA_DATA_Y_LSB = 0x2A,
	LIA_DATA_Y_MSB = 0x2B,
	LIA_DATA_Z_LSB = 0x2C,
	LIA_DATA_Z_MSB = 0x2D,
	GRV_DATA_X_LSB = 0x2E,
	GRV_DATA_X_MSB = 0x2F,
	GRV_DATA_Y_LSB = 0x30,
	GRV_DATA_Y_MSB = 0x31,
	GRV_DATA_Z_LSB = 0x32,
	GRV_DATA_Z_MSB = 0x33,
	TEMP = 0x34,
	CALIB_STAT = 0x35,
	ST_RESULT = 0x36,
	INT_STA = 0x37,
	SYS_CLK_STATUS = 0x38,
	SYS_STATUS = 0x39,
	SYS_ERR = 0x3A,
	UNIT_SEL = 0x3B,
	OPR_MODE = 0x3D,
	PWR_MODE = 0x3E,
	SYS_TRIGGER = 0x3F,
	TEMP_SOURCE = 0x40,
	AXIS_MAP_CONFIG = 0x41,
	AXIS_MAP_SIGN = 0x42,
	ACC_OFFSET_X_LSB = 0x55,
	ACC_OFFSET_X_MSB = 0x56,
	ACC_OFFSET_Y_LSB = 0x57,
	ACC_OFFSET_Y_MSB = 0x58,
	ACC_OFFSET_Z_LSB = 0x59,
	ACC_OFFSET_Z_MSB = 0x5A,
	MAG_OFFSET_X_LSB = 0x5B,
	MAG_OFFSET_X_MSB = 0x5C,
	MAG_OFFSET_Y_LSB = 0x5D,
	MAG_OFFSET_Y_MSB = 0x5E,
	MAG_OFFSET_Z_LSB = 0x5F,
	MAG_OFFSET_Z_MSB = 0x60,
	GYR_OFFSET_X_LSB = 0x61,
	GYR_OFFSET_X_MSB = 0x62,
	GYR_OFFSET_Y_LSB = 0x63,
	GYR_OFFSET_Y_MSB = 0x64,
	GYR_OFFSET_Z_LSB = 0x65,
	GYR_OFFSET_Z_MSB = 0x66,
	ACC_RADIUS_LSB = 0x67,
	ACC_RADIUS_MSB = 0x68,
	MAG_RADIUS_LSB = 0x69,
	MAG_RADIUS_MSB = 0x6A
};

enum PAGE_1_REG {
	PAGE_ID = 0x07,
	ACC_CONFIG = 0x08,
	MAG_CONFIG = 0x09,
	GYR_CONFIG_0 = 0x0A,
	GYR_CONFIG_1 = 0x0B,
	ACC_SLEEP_CONFIG = 0x0C,
	GYR_SLEEP_CONFIG = 0x0D,
	INT_MSK = 0x0F,
	INT_EN = 0x10,
	ACC_AM_THRES = 0x11,
	ACC_INT_SETTINGS = 0x12,
	ACC_HG_DURATION = 0x13,
	ACC_HG_THRES = 0x14,
	ACC_NM_THRES = 0x15,
	ACC_NM_SET = 0x16,
	GYR_INT_SETTING = 0x17,
	GYR_HR_X_SET = 0x18,
	GYR_DUR_X = 0x19,
	GYR_HR_Y_SET = 0x1A,
	GYR_DUR_Y = 0x1B,
	GYR_HR_Z_SET = 0x1C,
	GYR_DUR_Z = 0x1D,
	GYR_AM_THRES = 0x1E,
	GYR_AM_SET = 0x1F
};

enum OP_MODE {
	CONFIGMODE = 0x00,
	ACCONLY = 0x01,
	MAGONLY = 0x02,
	GYROONLY = 0x03,
	ACCMAG = 0x04,
	ACCGYRO = 0x05,
	MAGGYRO = 0x06,
	AMG = 0x07,
	IMU = 0x08,
	COMPASS = 0x09,
	M4G = 0x0A,
	NDOF_FMC_OFF = 0x0B,
	NDOF = 0x0C
} opMode = CONFIGMODE;

enum ACCEL_UNIT {
	MS2 = 0x00,
	MG = 0x01
};

enum ANGULAR_RATE_UNIT {
	DPS = 0x00,
	RPS = 0x02
};

enum EULER_UNIT {
	DEGREES = 0x00,
	RADIANS = 0x04
};

enum TEMPERATURE_UNIT {
	DEG_C = 0x00,
	DEG_F = 0x10
};

enum FUSION_OUTPUT_FORMAT {
	WINDOWS = 0x00,
	ANDROID = 0x80
};

class Custom_BNO055 {
	private:
		uint8_t _currentRegisterPage = 0x00;
		
		uint8_t _startingReadAddr = 0x00;
		
		int16_t _magData[3] = {0, 0, 0};
		
		int16_t _gyroData[3] = {0, 0, 0};
		
		int16_t _accelData[3] = {0, 0, 0};
		
		int16_t _eulerData[3] = {0, 0, 0};
		
		// Not going to bother with quaternion data right now.
		
		int16_t _linAccelData[3] = {0, 0, 0};
		
		int16_t _gravityVectorData[3] = {0, 0, 0};
		
		int8_t _temperature = 0;
		
		ACCEL_UNIT _accelUnit = MS2;
		
		ANGULAR_RATE_UNIT _angRateUnit = DPS;
		
		EULER_UNIT _eulerUnit = DEGREES;
		
		TEMPERATURE_UNIT _tempUnit = DEG_C;
		
		FUSION_OUTPUT_FORMAT _fusionOutputFormat = WINDOWS;  // TODO: Verify that this value for the default is correct.
		
		void setDataUnits();
		
		
	public:
		Custom_BNO055(uint8_t addr);
		
		void setMode(OP_MODE mode);
		
		void setAxisRemap(uint8_t remap, uint8_t sign);
		
		void setAccelConfig(uint8_t gRange, uint8_t bandwidth, uint8_t opMode);
		
		void setGyroConfig(uint8_t range, uint8_t bandwidth, uint8_t opMode);
		
		void setMagConfig(uint8_t rate, uint8_t opMode, uint8_t pwrMode);
		
		void setAccelUnit(ACCEL_UNIT unit);
		
		void setAngularRateUnit(ANGULAR_RATE_UNIT unit);
		
		void setEulerUnit(EULER_UNIT unit);
		
		void setTempUnit(TEMPERATURE_UNIT unit);
		
		// Not going to bother with the offsets right now.
		
		// Not going to bother with the radii right now.
		
		// OK, this is where the fun begins.
		
		int16_t* updateMagData();
		
		int16_t* updateGyroData();
		
		int16_t* updateAccelData();
		
		int16_t* updatEulerData();
		
		// Again, not going to bother with quaternion data right now.
		
		int16_t* updateLinearAccelData();
		
		int16_t* updateGravityVectorData();
		
		int16_t* updateTemperatureData();
		
		int16_t* updateAllData();
		
		// And now for the interrupt settings.
		
		void configAccelSMNMInterrupt();  // TODO: Add parameters to this.
		
		void configAccelAMInterrupt();  // TODO: Add parameters to this.
		
		void configAccelHighGInterrupt();  // TODO: Add parameters to this.
		
		void configGyroHRInterrupt();  // TODO: Add parameters to this.
		
		void configGyroAMInterrupt();  // TODO: Add parameters to this.
		
		// Miscellaneous functions.
		
		uint8_t getLastSelfTest();
		
		void triggerSelfTest();
		
		uint8_t getAccelCalibration();
		
		uint8_t getGyroCalibration();
		
		uint8_t getMagCalibration();
		
		bool isFullyCalibrated();
};

#endif