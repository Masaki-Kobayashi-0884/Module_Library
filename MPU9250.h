#define MPU9250_ADDRESS 0x68

#define PWR_MGMT_1 0x6B
#define INT_PIN_CFG 0x37

#define ACCEL_CONFIG 0x1C
#define ACCEL_FS_SEL_2G 0x00
#define ACCEL_FS_SEL_4G 0x08
#define ACCEL_FS_SEL_8G 0x10
#define ACCEL_FS_SEL_16G 0x18

#define GYRO_CONFIG 0x1B
#define GYRO_FS_SEL_250DPS 0x00
#define GYRO_FS_SEL_500DPS 0x08
#define GYRO_FS_SEL_1000DPS 0x10
#define GYRO_FS_SEL_2000DPS 0x18

#define AK8963_ADDRESS 0x0C
#define CNTL1 0x0A
#define CNTL1_MODE_SEL_8HZ 0x12
#define CNTL1_MODE_SEL_100HZ 0x16
#define ST1 0x02
#define ASAX 0x10
#define ASAY 0x11
#define ASAZ 0x12


class MPU9250
{
private:
	float accRange;
	float gyroRange;
	float mAdjx;
	float mAdjy;
	float mAdjz;

public:
	bool I2cInitialize();
	void ReadAccGyr(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
	void ReadGyr(float *x, float *y, float *z);
	void ReadAcc(float *x, float *y, float *z);
	bool ReadMag(float *mx, float *my, float *mz);
};

#include "I2Chandler.h"


/*
#include <Wire.h>

void I2cWriteByte(uint8_t add, uint8_t reg, uint8_t data)
{
	Wire.beginTransmission(add);
	Wire.write(reg);
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t I2cReadByte(uint8_t add, uint8_t reg)
{
	Wire.beginTransmission(add);
	Wire.write(reg);
	Wire.endTransmission(false);
	Wire.requestFrom(add, (uint8_t)1);
	uint8_t data = Wire.read();
	return data;
}

void I2cReadBytes(uint8_t add, uint8_t reg, uint8_t *data, uint8_t count)
{
	Wire.beginTransmission(add);
	Wire.write(reg);
	Wire.endTransmission(false);
	Wire.requestFrom(add, count);
	for (int i = 0; i < count; i++)
	{
		data[i] = Wire.read();
	}
}

void I2cInitializeI2C(){
	Wire.begin();
	Wire.setClock( 400000L );
}
*/

bool::MPU9250::Initialize()
{
	I2cInitialize();
	I2cWriteByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);				 //スリープモードを解除
	I2cWriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_FS_SEL_16G);   //加速度センサの測定レンジの設定
	accRange = 16.0;												 //計算で使用するので，選択したレンジを入力する
	I2cWriteByte(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_SEL_2000DPS); //ジャイロセンサの測定レンジの設定
	gyroRange = 2000.0;												 //計算で使用するので，選択したレンジを入力する

	I2cWriteByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);				 //bypass mode(磁気センサが使用出来るようになる)
	I2cWriteByte(AK8963_ADDRESS, CNTL1, CNTL1_MODE_SEL_100HZ);
	uint8_t asax = I2cReadByte(AK8963_ADDRESS, ASAX);
	uint8_t asay = I2cReadByte(AK8963_ADDRESS, ASAY);
	uint8_t asaz = I2cReadByte(AK8963_ADDRESS, ASAZ);

	mAdjx = ((float)asax - 128.0f) * 0.5f / 128.0f + 1.0f;
	mAdjy = ((float)asay - 128.0f) * 0.5f / 128.0f + 1.0f;
	mAdjz = ((float)asaz - 128.0f) * 0.5f / 128.0f + 1.0f;
	
	mAdjx = 1;
	mAdjy = 1;
	mAdjz = 1;
	return true;
}

void MPU9250::ReadAccGyr(float *ax, float *ay, float *az, float *gx, float *gy, float *gz){
	uint8_t AccGyroTemp[14];
	I2cReadBytes(MPU9250_ADDRESS, 0x3B, AccGyroTemp, 14);
	*ax = (int16_t)(AccGyroTemp[0] << 8 | AccGyroTemp[1]) / 32768.0 * accRange;
	*ay = (int16_t)(AccGyroTemp[2] << 8 | AccGyroTemp[3]) / 32768.0 * accRange;
	*az = (int16_t)(AccGyroTemp[4] << 8 | AccGyroTemp[5]) / 32768.0 * accRange;

	*gx = (int16_t)(AccGyroTemp[8] << 8 | AccGyroTemp[9]) / 32768.0 * gyroRange;
	*gy = (int16_t)(AccGyroTemp[10] << 8 | AccGyroTemp[11]) / 32768.0 * gyroRange;
	*gz = (int16_t)(AccGyroTemp[12] << 8 | AccGyroTemp[13]) / 32768.0 * gyroRange;
}

void MPU9250::ReadAcc(float *x, float *y, float *z){
	uint8_t AccTemp[6];
	I2cReadBytes(MPU9250_ADDRESS, 0x3B, AccTemp, 6);
	*x = (int16_t)(AccTemp[0] << 8 | AccTemp[1]) / 32768.0 * accRange;
	*y = (int16_t)(AccTemp[2] << 8 | AccTemp[3]) / 32768.0 * accRange;
	*z = (int16_t)(AccTemp[4] << 8 | AccTemp[5]) / 32768.0 * accRange;
}

void MPU9250::ReadGyr(float *x, float *y, float *z){
	uint8_t GyroTemp[6];
	I2cReadBytes(MPU9250_ADDRESS, 0x43, GyroTemp, 6);
	*x = (int16_t)(GyroTemp[0] << 8 | GyroTemp[1]) / 32768.0 * gyroRange;
	*y = (int16_t)(GyroTemp[2] << 8 | GyroTemp[3]) / 32768.0 * gyroRange;
	*z = (int16_t)(GyroTemp[4] << 8 | GyroTemp[5]) / 32768.0 * gyroRange;
}

bool MPU9250::ReadMag(float *mx, float *my, float *mz){
	uint8_t ST1Bit;
	ST1Bit = I2cReadByte(AK8963_ADDRESS, ST1);
	if ((ST1Bit & 0x01)){
		uint8_t magneticData[7];
		I2cReadBytes(AK8963_ADDRESS, 0x03, magneticData, 7);
		*mx = ((int16_t)((magneticData[3] << 8) | magneticData[2])) / 32768.0f * 4921.0f * mAdjy;
		*my = ((int16_t)((magneticData[1] << 8) | magneticData[0])) / 32768.0f * 4921.0f * mAdjx;
		*mz = -((int16_t)((magneticData[5] << 8) | magneticData[4])) / 32768.0f * 4921.0f * mAdjz;
		return true;
	}else
		return false;
}