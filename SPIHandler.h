#ifndef SPIHANDLER_H
#define SPIHANDLER_H

/*

センサのライブラリ群用のspiライブラリです。
別のマイコンを利用する場合は関数名、引数を変えないで互換性を保ってください。
新たにセンサのライブラリを作る際にもspiを利用する際はこのハンドラを通して、特定のマイコン専用にならないようにしてください。

*/

#define ARDUINO	//ここにマイコンを追記していく
//#define MBED

#ifdef ARDUINO						//arduinoのspi用

#include <SPI.h>
#define SPI_CLOCK 1000000

void SpiInitialize(int pin){
	pinMode(pin, OUTPUT);
}

void SpiWriteByte(uint8_t reg, uint8_t data,int pin)
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(pin, LOW);
    SPI.transfer(reg);
    SPI.transfer(data);
    digitalWrite(pin, HIGH);
	SPI.endTransaction();
}

uint8_t SpiReadByte(uint8_t reg,int pin)
{
	uint8_t data;
	SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(pin, LOW);
	SPI.transfer(reg | 0b1000000);
	data = SPI.transfer(0);
    digitalWrite(pin, HIGH);
	SPI.endTransaction();
	return data;
}

void SpiReadBytes(uint8_t reg, uint8_t *data, uint8_t count)
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(pin, LOW);
	SPI.transfer(reg | 0b1000000);
	SPI.transfer(data, count);
    digitalWrite(pin, HIGH);
	SPI.endTransaction();
}

#endif

#endif