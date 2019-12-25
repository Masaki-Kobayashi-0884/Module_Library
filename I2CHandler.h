#ifndef I2CHANDLER_H
#define I2CHANDLER_H

/*

センサのライブラリ群用のi2cライブラリです。
別のマイコンを利用する場合は関数名、引数を変えないで互換性を保ってください。
新たにセンサのライブラリを作る際にもi2cを利用する際はこのハンドラを通して、特定のマイコン専用にならないようにしてください。

*/

//#define ARDUINO	//ここにマイコンを追記していく
#define MBED

#ifdef ARDUINO	//arduinoのi2c用

#include "Wire.h"

class I2c
{
public:
	I2c();
	~I2c();
	void WriteByte(uint8_t add, uint8_t reg, uint8_t data);
	uint8_t ReadByte(uint8_t add, uint8_t reg);
	void I2cReadBytes(uint8_t add, uint8_t reg, uint8_t *data, uint8_t count);
private:
	bool initialized = false
};

I2c::I2c(){
	if(!initialized){
		Wire.begin();
		Wire.setClock( 400000L );
		initialized = true;
	}
}

I2c::~I2c(){
}

void I2c::WriteByte(uint8_t add, uint8_t reg, uint8_t data)
{
	Wire.beginTransmission(add);
	Wire.write(reg);
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t I2c::ReadByte(uint8_t add, uint8_t reg)
{
	Wire.beginTransmission(add);
	Wire.write(reg);
	Wire.endTransmission(false);
	Wire.requestFrom(add, (uint8_t)1);
	uint8_t data = Wire.read();
	return data;
}

void I2c::ReadBytes(uint8_t add, uint8_t reg, uint8_t *data, uint8_t count)
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

#ifdef MBED	//mbed用

#include "mbed.h"

#define I2C_DEFAULT_PIN_SDA p9
#define I2C_DEFAULT_PIN_SCL p10

class I2c
{
public:
	I2c(PinName sda, PinName scl);
	~I2c();
	void WriteByte(uint8_t add, uint8_t reg, uint8_t data);
	uint8_t ReadByte(uint8_t add, uint8_t reg);
	void ReadBytes(uint8_t add, uint8_t reg, uint8_t *data, uint8_t count);
private:
	I2C _i2c;
};

I2c::I2c(PinName sda = I2C_DEFAULT_PIN_SDA, PinName scl = I2C_DEFAULT_PIN_SCL) : _i2c(sda, scl)
{
	_i2c.frequency(400 * 1000);
}

I2c::~I2c(){
}

void I2c::WriteByte(uint8_t add, uint8_t reg, uint8_t data)
{
	add = add << 1;
	char data_write[2];
    data_write[0] = reg;
    data_write[1] = data;
    _i2c.write(add, data_write, 2, 0);
}

uint8_t I2c::ReadByte(uint8_t add, uint8_t reg)
{
	add = add << 1;
	char data[1];
    char data_write[1];
    data_write[0] = reg;
    _i2c.write(add, data_write, 1, 1);
    _i2c.read(add, data, 1, 0);
    return data[0];
}

void I2c::ReadBytes(uint8_t add, uint8_t reg, uint8_t *data, uint8_t count)
{
    add = add << 1;
    char buf[count];
    char data_write[1];
    data_write[0] = reg;
    _i2c.write(add, data_write, 1, 1);
    _i2c.read(add, buf, count, 0);
    for (int ii = 0; ii < count; ii++)
    {
        data[ii] = buf[ii];
    }
}

#endif

#endif