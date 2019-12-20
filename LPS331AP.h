/*

	2019 Oku Dan / oku_dan@yahoo.co.jp

	STMicroelectronics社製LPS331AP用のI2C版ライブラリです.
	使用する際はデバイスのSDAとSCLをセンサに接続してください.

	1.LPS_331AP型クラスを作成
	2.Initialize()で初期化
	3.Wire.setClock(400000L)でI2Cを高速モードにするとよい
	4.ReadPrs(&prs),ReadTmp(&tmp)で気圧、気温読み出し
	5.4に戻る

	49行目ODRでサンプリングレートを変更できる（デフォルトは25Hz,25Hz)
	このセンサの更新速度にデバイス側で合わせて読みだせばSTATUS_REGのチェックが不要になる

	※なんか気温のキャストが正しくないっぽいので修正してくれ
*/

#include "I2CHandler.h"

#define LPS331AP_ADDR0 0b1011100 //SA0=L(GND)
#define LPS331AP_ADDR1 0b1011101 //SA0=H(VDD)
#define LPS331AP_REF_P_XL 0x08
#define LPS331AP_REF_P_L 0x09
#define LPS331AP_REF_P_H 0x0A
#define LPS331AP_WHO_AM_I 0x0F
#define LPS331AP_WHO_AM_I_DEFAULT 0xBB
#define LPS331AP_RES_CONF 0x10
#define LPS331AP_CTRL_REG1 0x20
#define LPS331AP_CTRL_REG2 0x21
#define LPS331AP_CTRL_REG3 0x22
#define LPS331AP_INTERRUPT_CFG 0x23
#define LPS331AP_INT_SOURCE 0x24
#define LPS331AP_THS_P_L 0x25
#define LPS331AP_THS_P_H 0x26
#define LPS331AP_STATUS_REG 0x27
#define LPS331AP_PRESS_OUT_XL 0x28
#define LPS331AP_PRESS_OUT_L 0x29
#define LPS331AP_PRESS_OUT_H 0x2A
#define LPS331AP_TEMP_OUT_L 0x2B
#define LPS331AP_TEMP_OUT_H 0x2C
#define LPS331AP_AMP_CTRL 0x30
#define LPS331AP_DELTA_PRESS_XL 0x3C
#define LPS331AP_DELTA_PRESS_L 0x3D
#define LPS331AP_DELTA_PRESS_H 0x3E

#define LPS331AP_ODR 0b111
//	(Hz)	pressure	Temperature
//	000  -	One Shot	One Shot
//	001	 -	1			1
//	010  -	7			1
//	011  -	12.5		1
//	101  -	7			7
//	110  -	12.5		12.5
//	111  -	25			25

class LPS_331AP
{
private:
	void RegistersInit();
	uint8_t LPS331AP_ADDR;

public:
	bool Initialize(uint8_t SA0);
	bool Initialize();
	bool ReadPrs(float *prs);
	bool ReadTmp(float *tmp);
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
	byte retVal;
	Wire.beginTransmission(add);
	Wire.write(reg | 0x80);
	Wire.endTransmission(false);
	Wire.requestFrom(add, count);
	for (int i = 0; i < count; i++)
	{
		data[i] = Wire.read();
	}
}

*/

void LPS_331AP::RegistersInit(){
	uint8_t tempRegValue = 0;

	//Power down before change config
	tempRegValue = 0;
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_CTRL_REG1, tempRegValue);

	//Reference pressure
	tempRegValue = 0;
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_REF_P_XL, tempRegValue);
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_REF_P_L, tempRegValue);
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_REF_P_H, tempRegValue);

	//RES_CONF
	//[RFU][AVGT2][AVGT1][AVGT0][AVGP3][AVGP2][AVGP1][AVGP0]
	//RFU - reserved for future
	//AVGT[2:0] - Temperature resolution configuration(Average of AVGT times)
	//	000:1
	//	001:2
	//	  ~
	//	111:128
	//AVGP[3:0] - Pressure resolution configuration(Average of AVGP times)
	//	0000:1
	//	0001:2
	//	  ~
	//	1111:512
	//Register configuration 7Ah not allowed with ODR = 25Hz/25Hz (Register CTRL_REG1).
	//For ORD 25Hz/25Hz the suggested configuration for RES_CONF is 6Ah.
	tempRegValue = 0b01111111;
	if(LPS331AP_ODR == 0b111)tempRegValue = 0x6A; 	//= 0b01101010
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_RES_CONF, tempRegValue);

	//CTRL_REG2
	//[BOOT][RESERVED][RESERVED][RESERVED][RESERVED][SWRESET][AUTO_ZERO][ONE_SHOT]
	//BOOT -  Reboot memory content. Default value: 0
	//SWRESET - Software reset. Default value: 0
	//AUTO_ZERO - Autozero enable. Default value: 0
	//ONE_SHOT -  One shot enable. Default value: 0
	tempRegValue = 0;
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_CTRL_REG2, tempRegValue);

	
	//CTRL_REG3
	//INTERRUPT_CFG
	//INT_SOURCE
	//THS_P_L
	//THS_P_H
	//To set about interrupt.
	tempRegValue = 0;
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_CTRL_REG3, tempRegValue);
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_INTERRUPT_CFG, tempRegValue);
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_INT_SOURCE, tempRegValue);
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_THS_P_L, tempRegValue);
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_THS_P_H, tempRegValue);

	//CTRL_REG1
	//[PD][ODR2][ODR1][ODR0][DIFF_EN][DBDU][DELTA_EN][SIM]
	//PD - PD: power down control(0: power-down mode; 1: active mode)
	//ODR[2:0] - output data rate selection
	//DIFF_EN - Interrupt circuit enable.(0: interrupt generation disabled; 1: interrupt circuit enabled)
	//DBDU - block data update(0: continuous update 1: output registers not updated until MSB and LSB reading)
	//DELTA_EN - delta pressure enable(1: delta pressure registers enabled. 0: disable)
	//SIM -  SPI Serial Interface Mode selection.(0: 4-wire interface; 1: 3-wire interface)
	tempRegValue = 0b10000100;
	tempRegValue |= LPS331AP_ODR << 4;
	I2cWriteByte(LPS331AP_ADDR, LPS331AP_CTRL_REG1, tempRegValue);
	return;
}

bool LPS_331AP::Initialize(uint8_t SA0){
	uint8_t Test = I2cReadByte((SA0 == 1 ? LPS331AP_ADDR1 : LPS331AP_ADDR0), LPS331AP_WHO_AM_I);
	return Test == LPS331AP_WHO_AM_I_DEFAULT;
}

bool LPS_331AP::Initialize(){
	I2cInitialize();
	if(Initialize(0))LPS331AP_ADDR = LPS331AP_ADDR0;
	else if(Initialize(1))LPS331AP_ADDR = LPS331AP_ADDR1;
	else return false;
	RegistersInit();
	return true;
}

bool LPS_331AP::ReadPrs(float *prs){
	uint8_t status;

	//STATUS_REG
	//[0][0][P_OR][T_OR][0][0][P_DA][T_DA]
	//P_OR - Pressure data overrun.Sampling rate is too fast.
	//T_OR - Temperature data overrun.
	//P_DA - Pressure data available.
	//T_DA - Temperature data available.
	status = I2cReadByte(LPS331AP_ADDR, LPS331AP_STATUS_REG);

	if((status & 0b00000010) != 0){
		uint8_t prsTempXL,prsTempL,prsTempH;
		prsTempXL = I2cReadByte(LPS331AP_ADDR, LPS331AP_PRESS_OUT_XL);
		prsTempL = I2cReadByte(LPS331AP_ADDR, LPS331AP_PRESS_OUT_L);
		prsTempH = I2cReadByte(LPS331AP_ADDR, LPS331AP_PRESS_OUT_H);
		*prs = (uint32_t)(prsTempH << 16 | prsTempL << 8 | prsTempXL) / 4096.0f;
		return true;
	}
	else return false;
}

bool LPS_331AP::ReadTmp(float *tmp){
	uint8_t status;
	status = I2cReadByte(LPS331AP_ADDR, LPS331AP_STATUS_REG);
	if((status & 0b00000001) != 0){
		uint8_t tmpTempL,tmpTempH;
		tmpTempL = I2cReadByte(LPS331AP_ADDR, LPS331AP_TEMP_OUT_L);
		tmpTempH = I2cReadByte(LPS331AP_ADDR, LPS331AP_TEMP_OUT_H);
		*tmp =  42.5 + (int16_t)(tmpTempH << 8 | tmpTempL) / 480.0f;
		return true;
	}
	else return false;
}
