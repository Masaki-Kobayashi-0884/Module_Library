#ifndef I2CHANDLER_H
#define I2CHANDLER_H

/*

センサのライブラリ群用のi2cライブラリです。
別のマイコンを利用する場合は関数名、引数を変えないで互換性を保ってください。
新たにセンサのライブラリを作る際にもi2cを利用する際はこのハンドラを通して、特定のマイコン専用にならないようにしてください。

*/

#define ARDUINO	//ここにマイコンを追記していく
//#define MBED

#ifdef ARDUINO	//arduinoのi2c用

#include "Wire.h"

bool initialized = false;

void I2cInitialize(){
	if(!initialized){
		Wire.begin();
		Wire.setClock( 400000L );
		initialized = true;
	}
}

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
#endif

#endif