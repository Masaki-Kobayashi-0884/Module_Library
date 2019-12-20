/*

	2019 Oku Dan / oku_dan@yahoo.co.jp

	MPU9250用のI2C版ライブラリです.
	使用する際はデバイスのSDAとSCLをセンサに接続してください.

	1.MPU9250型クラスを作成
	2.Initialize()でセンサ初期化
	3.ReadAccGyr(),ReadMag()で読み出し

	内部DMPを利用してかなり正確なQuaternionを出力してくれる裏ワザ的手法がありますが、かなりの魔境なので追加しません.
	利用したい方は自分で調べてください.
*/

#include "I2CHandler.h"

#define MPU9250_ADDRESS 0x68

#define MPU9250_WHO_AM_I 0x75
#define MPU9250_WHO_AM_I_DEFAULT 0x71
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_INT_PIN_CFG 0x37

#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_FS_SEL_2G 0x00
#define MPU9250_ACCEL_FS_SEL_4G 0x08
#define MPU9250_ACCEL_FS_SEL_8G 0x10
#define MPU9250_ACCEL_FS_SEL_16G 0x18

#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_GYRO_FS_SEL_250DPS 0x00
#define MPU9250_GYRO_FS_SEL_500DPS 0x08
#define MPU9250_GYRO_FS_SEL_1000DPS 0x10
#define MPU9250_GYRO_FS_SEL_2000DPS 0x18

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define AK8963_ADDRESS 0x0C
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL1_MODE_SEL_8HZ 0x12
#define AK8963_CNTL1_MODE_SEL_100HZ 0x16
#define AK8963_ST1 0x02
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

#define AK8963_HXL 0x03
#define AK8963_HXH 0x04
#define AK8963_HYL 0x05
#define AK8963_HYH 0x06
#define AK8963_HZL 0x07
#define AK8963_HZH 0x08


class MPU9250
{
private:
	float accRange;
	float gyroRange;
	float mAdjx;
	float mAdjy;
	float mAdjz;

public:
	bool Initialize();
	bool Initialize(int AccelScale, int GyroScale, int MagnetRate);
	void ReadAccGyr(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
	void ReadAcc(float *ax, float *ay, float *az);
	void ReadGyr(float *gx, float *gy, float *gz);
	bool ReadMag(float *mx, float *my, float *mz);
};


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

void I2cInitialize(){
	Wire.begin();
	Wire.setClock( 400000L );
}
*/

bool MPU9250::Initialize(int AccelScale, int GyroScale, int MagnetRate){
	uint8_t tempRegVal;
	I2cInitialize();
	if(I2cReadByte(MPU9250_ADDRESS,MPU9250_WHO_AM_I) != MPU9250_WHO_AM_I_DEFAULT)return false;

	I2cWriteByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0);				 //スリープモードを解除
	switch (AccelScale)
	{
	case 2:
		tempRegVal = MPU9250_ACCEL_FS_SEL_2G;
		accRange = 2.0;												 //計算で使用するので，選択したレンジを入力する
		break;
	case 4:
		tempRegVal = MPU9250_ACCEL_FS_SEL_4G;
		accRange = 4.0;
		break;
	case 8:
		tempRegVal = MPU9250_ACCEL_FS_SEL_8G;
		accRange = 8.0;
		break;	
	default:
		tempRegVal = MPU9250_ACCEL_FS_SEL_16G;
		accRange = 16.0;
		break;
	}
	I2cWriteByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, tempRegVal);		 //加速度センサの測定レンジの設定

	switch (GyroScale)
	{
	case 250:
		tempRegVal = MPU9250_GYRO_FS_SEL_250DPS;
		gyroRange = 250.0;												 //計算で使用するので，選択したレンジを入力する
		break;
	case 500:
		tempRegVal = MPU9250_GYRO_FS_SEL_500DPS;
		gyroRange = 500.0;
		break;
	case 1000:
		tempRegVal = MPU9250_GYRO_FS_SEL_1000DPS;
		gyroRange = 1000.0;
		break;
	default:
		tempRegVal = MPU9250_GYRO_FS_SEL_2000DPS;
		gyroRange = 2000.0;
		break;
	}
	I2cWriteByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, tempRegVal); //ジャイロセンサの測定レンジの設定

	I2cWriteByte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x02);				 //bypass mode(磁気センサが使用出来るようになる)
	switch (MagnetRate)
	{
	case 8:
		tempRegVal = AK8963_CNTL1_MODE_SEL_8HZ;
		break;
	default:
		tempRegVal = AK8963_CNTL1_MODE_SEL_100HZ;
		break;
	}
	I2cWriteByte(AK8963_ADDRESS, AK8963_CNTL1, tempRegVal);
	uint8_t asax = I2cReadByte(AK8963_ADDRESS, AK8963_ASAX);
	uint8_t asay = I2cReadByte(AK8963_ADDRESS, AK8963_ASAY);
	uint8_t asaz = I2cReadByte(AK8963_ADDRESS, AK8963_ASAZ);

	mAdjx = ((float)asax - 128.0f) * 0.5f / 128.0f + 1.0f;
	mAdjy = ((float)asay - 128.0f) * 0.5f / 128.0f + 1.0f;
	mAdjz = ((float)asaz - 128.0f) * 0.5f / 128.0f + 1.0f;

	return true;
}

bool::MPU9250::Initialize()
{
	Initialize(16, 2000, 100);
}

void MPU9250::ReadAccGyr(float *ax, float *ay, float *az, float *gx, float *gy, float *gz){
	uint8_t AccGyroTemp[14];
	I2cReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, AccGyroTemp, 14);
	*ax = (int16_t)(AccGyroTemp[0] << 8 | AccGyroTemp[1]) / 32768.0 * accRange;
	*ay = (int16_t)(AccGyroTemp[2] << 8 | AccGyroTemp[3]) / 32768.0 * accRange;
	*az = (int16_t)(AccGyroTemp[4] << 8 | AccGyroTemp[5]) / 32768.0 * accRange;

	*gx = (int16_t)(AccGyroTemp[8] << 8 | AccGyroTemp[9]) / 32768.0 * gyroRange;
	*gy = (int16_t)(AccGyroTemp[10] << 8 | AccGyroTemp[11]) / 32768.0 * gyroRange;
	*gz = (int16_t)(AccGyroTemp[12] << 8 | AccGyroTemp[13]) / 32768.0 * gyroRange;
}

void MPU9250::ReadAcc(float *ax, float *ay, float *az){
	uint8_t AccTemp[6];
	I2cReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, AccTemp, 6);
	*ax = (int16_t)(AccTemp[0] << 8 | AccTemp[1]) / 32768.0 * accRange;
	*ay = (int16_t)(AccTemp[2] << 8 | AccTemp[3]) / 32768.0 * accRange;
	*az = (int16_t)(AccTemp[4] << 8 | AccTemp[5]) / 32768.0 * accRange;
}
void MPU9250::ReadGyr(float *gx, float *gy, float *gz){
	
	uint8_t GyroTemp[6];
	I2cReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, GyroTemp, 6);
	*gx = (int16_t)(GyroTemp[0] << 8 | GyroTemp[1]) / 32768.0 * gyroRange;
	*gy = (int16_t)(GyroTemp[2] << 8 | GyroTemp[3]) / 32768.0 * gyroRange;
	*gz = (int16_t)(GyroTemp[4] << 8 | GyroTemp[5]) / 32768.0 * gyroRange;
}

bool MPU9250::ReadMag(float *mx, float *my, float *mz){
	uint8_t ST1Bit;
	ST1Bit = I2cReadByte(AK8963_ADDRESS, AK8963_ST1);
	if ((ST1Bit & 0x01)){
		uint8_t magneticData[7];
		I2cReadBytes(AK8963_ADDRESS, AK8963_HXL, magneticData, 7);
		*mx = ((int16_t)((magneticData[3] << 8) | magneticData[2])) / 32768.0f * 4921.0f * mAdjy;
		*my = ((int16_t)((magneticData[1] << 8) | magneticData[0])) / 32768.0f * 4921.0f * mAdjx;
		*mz = -((int16_t)((magneticData[5] << 8) | magneticData[4])) / 32768.0f * 4921.0f * mAdjz;
		return true;
	}else return false;
}