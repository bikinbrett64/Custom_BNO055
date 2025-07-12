/**
	Custom_BNO055.h
	
	For my custom electronics aboard Zenith (tentative name hehe).
	
	Author: Brett Warren
*/

#include <Arduino.h>
#include <Wire.h>

// Register Map Page 0
#define CHIP_ID 0x00
#define ACC_ID 0X01
#define MAG_ID 0X02
#define GYR_ID 0X03
#define SW_REV_ID_LSB 0X04
#define SW_REV_ID_MSB 0X05
#define BL_REV_ID 0X06
#define PAGE_ID 0X07
#define ACC_DATA_X_LSB 0x08
#define ACC_DATA_X_MSB 0x09
#define ACC_DATA_Y_LSB 0x0A
#define ACC_DATA_Y_MSB 0x0B
#define ACC_DATA_Z_LSB 0x0C
#define ACC_DATA_Z_MSB 0x0D
#define MAG_DATA_X_LSB 0x0E
#define MAG_DATA_X_MSB 0x0F
#define MAG_DATA_Y_LSB 0x10
#define MAG_DATA_Y_MSB 0x11
#define MAG_DATA_Z_LSB 0x12
#define MAG_DATA_Z_MSB 0x13
#define GYR_DATA_X_LSB 0x14
#define GYR_DATA_X_MSB 0x15
#define GYR_DATA_Y_LSB 0x16
#define GYR_DATA_Y_MSB 0x17
#define GYR_DATA_Z_LSB 0x18
#define GYR_DATA_Z_MSB 0x19
#define EUL_DATA_X_LSB 0x1A
#define EUL_DATA_X_MSB 0x1B
#define EUL_DATA_Y_LSB 0x1C
#define EUL_DATA_Y_MSB 0x1D
#define EUL_DATA_Z_LSB 0x1E
#define EUL_DATA_Z_MSB 0x1F
#define QUA_DATA_W_LSB 0x20
#define QUA_DATA_W_MSB 0x21
#define QUA_DATA_X_LSB 0x22
#define QUA_DATA_X_MSB 0x23
#define QUA_DATA_Y_LSB 0x24
#define QUA_DATA_Y_MSB 0x25
#define QUA_DATA_Z_LSB 0x26
#define QUA_DATA_Z_MSB 0x27
#define LIA_DATA_X_LSB 0x28
#define LIA_DATA_X_MSB 0x29
#define LIA_DATA_Y_LSB 0x2A
#define LIA_DATA_Y_MSB 0x2B
#define LIA_DATA_Z_LSB 0x2C
#define LIA_DATA_Z_MSB 0x2D
#define GRV_DATA_X_LSB 0x2E
#define GRV_DATA_X_MSB 0x2F
#define GRV_DATA_Y_LSB 0x30
#define GRV_DATA_Y_MSB 0x31
#define GRV_DATA_Z_LSB 0x32
#define GRV_DATA_Z_MSB 0x33
#define TEMP 0x34
#define CALIB_STAT 0x35
#define ST_RESULT 0x36
#define INT_STA 0x37
#define SYS_CLK_STATUS 0x38
#define SYS_STATUS 0x39
#define SYS_ERR 0x3A
#define UNIT_SEL 0x3B
#define OPR_MODE 0x3D
#define PWR_MODE 0x3E
#define SYS_TRIGGER 0x3F
#define TEMP_SOURCE 0x40
#define AXIS_MAP_CONFIG 0x41
#define AXIS_MAP_SIGN 0x42
#define ACC_OFFSET_X_LSB 0x55
#define ACC_OFFSET_X_MSB 0x56
#define ACC_OFFSET_Y_LSB 0x57
#define ACC_OFFSET_Y_MSB 0x58
#define ACC_OFFSET_Z_LSB 0x59
#define ACC_OFFSET_Z_MSB 0x5A
#define MAG_OFFSET_X_LSB 0x5B
#define MAG_OFFSET_X_MSB 0x5C
#define MAG_OFFSET_Y_LSB 0x5D
#define MAG_OFFSET_Y_MSB 0x5E
#define MAG_OFFSET_Z_LSB 0x5F
#define MAG_OFFSET_Z_MSB 0x60
#define GYR_OFFSET_X_LSB 0x61
#define GYR_OFFSET_X_MSB 0x62
#define GYR_OFFSET_Y_LSB 0x63
#define GYR_OFFSET_Y_MSB 0x64
#define GYR_OFFSET_Z_LSB 0x65
#define GYR_OFFSET_Z_MSB 0x66
#define ACC_RADIUS_LSB 0x67
#define ACC_RADIUS_MSB 0x68
#define MAG_RADIUS_LSB 0x69
#define MAG_RADIUS_MSB 0x6A

// Register Map Page 1
#define ACC_CONFIG 0x08
#define MAG_CONFIG 0x09
#define GYR_CONFIG_0 0X0A
#define GYR_CONFIG_1 0x0B
#define ACC_SLEEP_CONFIG 0X0C
#define GYR_SLEEP_CONFIG 0X0D
#define INT_MSK 0x0F
#define INT_EN 0x10
#define ACC_AM_THRES 0x11
#define ACC_INT_SETTINGS 0x12
#define ACC_HG_DURATION 0x13
#define ACC_HG_THRES 0x14
#define ACC_NM_THRES 0x15
#define ACC_NM_SET 0x16
#define GYR_INT_SETTING 0x17
#define GYR_HR_X_SET 0x18
#define GYR_DUR_X 0x19
#define GYR_HR_Y_SET 0x1A
#define GYR_DUR_Y 0x1B
#define GYR_HR_Z_SET 0x1C
#define GYR_DUR_Z 0x1D
#define GYR_AM_THRES 0x1E
#define GYR_AM_SET 0x1F

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
	DPS = 0X00,
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
		
		int _magData[3] = {0, 0, 0};
		
		int _gyrData[3] = {0, 0, 0};
		
		int _accelData[3] = {0, 0, 0};
		
		int _eulData[3] = {0, 0, 0};
		
		// Not going to bother with quaternion data right now.
		
		int _linAccelData[3] = {0, 0, 0};
		
		int _gravityVectorData[3] = {0, 0, 0};
		
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
		
		void 
};