#ifndef SERIAL_H
#define SERIAL_H

/*

--作成中--
mbedのシリアル通信をArduinoに寄せるライブラリ

*/
#define MBED
/*
#define ARDUINO

#ifdef ARDUINO

#include "Wire.h"

#endif
*/
#ifdef MBED

#include　"mbed.h"

class Serial_{
private:
	Serial_();

public:
	void begin(int speed);
	void end();
	int available();
	int read();
	int peek();
	int flush();
	//long print(char data);
	//long println(char data);
	int write(int val);
}

Serial_::Serial_(){
	Serial mserial(USBTX, USBRX);
}

void Serial_::begin(int speed){
	mserial.beud(speed);
}

void Serial_::end(){

}

int Serial_::available(){

}

int Serial_::read(){

}

int Serial_::peek(){

}

int Serial_::flush(){

}

long Serial_::print(char data){
	mserial.printf(data;)
}
/*
long Serial_::println(char data){

}
*/
int Serial_::write(int val){

}
Serial_ Serial;

#endif

#endif