#ifndef SERIAL_H
#define SERIAL_H

/*

シリアル通信用のライブラリ

*/

#define ARDUINO
//#define MBED

#ifdef ARDUINO

#include "Wire.h"

void serialInitialize(int rate)
{
	Serial.begin(rate);
}

void serialEnd(){
	Serial.end();
}

bool serialAvailable(){
	if (Serial.available() > 0){
	return true;
	}
	else{
	return false;
	}
}

char serialReedByte(){
	char buff;
	buff = Serial.reed();
	return buff
}

char serialReedBytes(){
	char buff[BUFF_MAX]={'\0'};
	int counter = 0;
	while (Serial.available()>0){

        char data = Serial.read();
        buff[counter] = data;

        if (data == '\0'){
            //buff[0]～buff[counter-1]までが文字列となってここでうけとれる
            //シリアル送信側で終端文字\0が最後につけられることが前提
            return buff;
        }
        else{
            counter++;
        }
    }
    return buff
}

void serialWriteByte(){


}

#ebdif

#ifdef MBED

#endif

#endif