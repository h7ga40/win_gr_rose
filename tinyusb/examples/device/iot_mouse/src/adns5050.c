/* 
 * MIT License
 *
 * Copyright (c) 2019 Hiroyuki Okada
 *
 * https://github.com/okhiroyuki/ADNS5050
 */
/*
 * Modify to C language from C++/Arduino
 */
#include <stdint.h>
#include "adns5050.h"
#include "bsp/board.h"
#include "iodefine.h"

#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1

static void pinMode(int pin, int mode)
{
	switch(pin){
	case 8:
		if (mode) {
			  PORT3.PODR.BIT.B0 = 0U;
			  PORT3.PMR.BIT.B0  = 0U;
			  PORT3.PDR.BIT.B0  = 1U;
		}
		else {
			  PORT3.PMR.BIT.B0  = 0U;
			  PORT3.PDR.BIT.B0  = 0U;
		}
		break;
	case 9:
		if (mode) {
			  PORT2.PODR.BIT.B6 = 0U;
			  PORT2.PMR.BIT.B6  = 0U;
			  PORT2.PDR.BIT.B6  = 1U;
		}
		else {
			  PORT2.PMR.BIT.B6  = 0U;
			  PORT2.PDR.BIT.B6  = 0U;
		}
		break;
	case 10:
		if (mode) {
			  PORTE.PODR.BIT.B4 = 0U;
			  PORTE.PMR.BIT.B4  = 0U;
			  PORTE.PDR.BIT.B4  = 1U;
		}
		else {
			  PORTE.PMR.BIT.B4  = 0U;
			  PORTE.PDR.BIT.B4  = 0U;
		}
		break;
	case 11:
		if (mode) {
			  PORTE.PODR.BIT.B6 = 0U;
			  PORTE.PMR.BIT.B6  = 0U;
			  PORTE.PDR.BIT.B6  = 1U;
		}
		else {
			  PORTE.PMR.BIT.B6  = 0U;
			  PORTE.PDR.BIT.B6  = 0U;
		}
		break;
	case 12:
		if (mode) {
			  PORTE.PODR.BIT.B7 = 0U;
			  PORTE.PMR.BIT.B7  = 0U;
			  PORTE.PDR.BIT.B7  = 1U;
		}
		else {
			  PORTE.PMR.BIT.B7  = 0U;
			  PORTE.PDR.BIT.B7  = 0U;
		}
		break;
	case 13:
		if (mode) {
			  PORTE.PODR.BIT.B5 = 0U;
			  PORTE.PMR.BIT.B5  = 0U;
			  PORTE.PDR.BIT.B5  = 1U;
		}
		else {
			  PORTE.PMR.BIT.B5  = 0U;
			  PORTE.PDR.BIT.B5  = 0U;
		}
		break;
	case 20:
		if (mode) {
			  PORT5.PODR.BIT.B2 = 0U;
			  PORT5.PMR.BIT.B2  = 0U;
			  PORT5.PDR.BIT.B2  = 1U;
		}
		else {
			  PORT5.PMR.BIT.B2  = 0U;
			  PORT5.PDR.BIT.B2  = 0U;
		}
		break;
	case 21:
		if (mode) {
			  PORT5.PODR.BIT.B0 = 0U;
			  PORT5.PMR.BIT.B0  = 0U;
			  PORT5.PDR.BIT.B0  = 1U;
		}
		else {
			  PORT5.PMR.BIT.B0  = 0U;
			  PORT5.PDR.BIT.B0  = 0U;
		}
		break;
	}
}

static void digitalWrite(int pin, int value)
{
	switch(pin){
	case 8:
		PORT3.PODR.BIT.B0 = value;
		break;
	case 9:
		PORT2.PODR.BIT.B6 = value;
		break;
	case 10:
		PORTE.PODR.BIT.B4 = value;
		break;
	case 11:
		PORTE.PODR.BIT.B6 = value;
		break;
	case 12:
		PORTE.PODR.BIT.B7 = value;
		break;
	case 13:
		PORTE.PODR.BIT.B5 = value;
		break;
	case 20:
		PORT5.PODR.BIT.B2 = value;
		break;
	case 21:
		PORT5.PODR.BIT.B0 = value;
		break;
	}
}

static int digitalRead(int pin)
{
	switch(pin){
	case 8:
		return PORT3.PIDR.BIT.B0;
	case 9:
		return PORT2.PIDR.BIT.B6;
	case 10:
		return PORTE.PIDR.BIT.B4;
	case 11:
		return PORTE.PIDR.BIT.B6;
	case 12:
		return PORTE.PIDR.BIT.B7;
	case 13:
		return PORTE.PIDR.BIT.B5;
	case 20:
		return PORT5.PIDR.BIT.B2;
	case 21:
		return PORT5.PIDR.BIT.B0;
	}
	return 0;
}

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

static void delayMicroseconds(uint32_t ms)
{
	board_delay(ms);
}

static int ConvertToSignedNumber(uint8_t twoscomp);

//Constructor sets the pins used for the mock 'i2c' communication
void adns5050_init(adns5050_t *adns5050, int sdio, int sclk, int ncs)
{
	adns5050->_sdio = sdio;
	adns5050->_sclk = sclk;
	adns5050->_ncs = ncs;
}

//Configures the communication pins for their initial state
void adns5050_begin(adns5050_t *adns5050)
{
	pinMode(adns5050->_sdio, OUTPUT);
	pinMode(adns5050->_sclk, OUTPUT);
	pinMode(adns5050->_ncs, OUTPUT);
}

//Essentially resets communication to the ADNS5050 module
void adns5050_sync(adns5050_t *adns5050)
{
	digitalWrite(adns5050->_ncs, LOW);
	delayMicroseconds(1);
	digitalWrite(adns5050->_ncs, HIGH);
}

//Reads a register from the ADNS5050 sensor. Returns the result to the calling function.
//Example: value = mouse.read(CONFIGURATION_REG);
int adns5050_read(adns5050_t *adns5050, unsigned char addr)
{
  uint8_t temp;
  int n;

  //select the chip
  digitalWrite(adns5050->_ncs, LOW);	//nADNSCS = 0;
  temp = addr;

  //start clock low
  digitalWrite(adns5050->_sclk, LOW); //SCK = 0;

	//set data line for output
  pinMode(adns5050->_sdio, OUTPUT); //DATA_OUT;
  //read 8bit data
  for (n=0; n<8; n++) {
    //delayMicroseconds(2);
    digitalWrite(adns5050->_sclk, LOW);//SCK = 0;
    //delayMicroseconds(2);
    pinMode(adns5050->_sdio, OUTPUT); //DATA_OUT;
    if (temp & 0x80) {
      digitalWrite(adns5050->_sdio, HIGH);//SDOUT = 1;
    }
    else {
      digitalWrite(adns5050->_sdio, LOW);//SDOUT = 0;
    }
    delayMicroseconds(2);
    temp = (temp << 1);
    digitalWrite(adns5050->_sclk, HIGH); //SCK = 1;
  }

  // This is a read, switch to input
  temp = 0;
  pinMode(adns5050->_sdio, INPUT); //DATA_IN;
  //read 8bit data
	for (n=0; n<8; n++) {		// read back the data
    delayMicroseconds(1);
    digitalWrite(adns5050->_sclk, LOW);
    delayMicroseconds(1);
    if(digitalRead(adns5050->_sdio)) {
      temp |= 0x1;
    }
    if( n != 7) temp = (temp << 1); // shift left
    digitalWrite(adns5050->_sclk, HIGH);
  }
  delayMicroseconds(20);
  digitalWrite(adns5050->_ncs, HIGH);// de-select the chip
  return ConvertToSignedNumber(temp);
}

int ConvertToSignedNumber(uint8_t twoscomp){
  int value;

	if (bitRead(twoscomp,7)){
    value = -128 + (twoscomp & 0b01111111 );
  }
  else{
    value = twoscomp;
  }
	return value;
}

//Writes a value to a register on the ADNS2620.
//Example: mouse.write(CONFIGURATION_REG, 0x01);
void adns5050_write(adns5050_t *adns5050, unsigned char addr, unsigned char data)
{
  char temp;
  int n;

  //select the chip
  //nADNSCS = 0;
  digitalWrite(adns5050->_ncs, LOW);

  temp = addr;
  //クロックを開始
  digitalWrite(adns5050->_sclk, LOW);//SCK = 0;					// start clock low
  //SDIOピンを出力にセット
  pinMode(adns5050->_sdio, OUTPUT);//DATA_OUT; // set data line for output
  //8ビットコマンドの送信
  for (n=0; n<8; n++) {
    digitalWrite(adns5050->_sclk, LOW);//SCK = 0;
    pinMode(adns5050->_sdio, OUTPUT);
    delayMicroseconds(1);
    if (temp & 0x80)  //0x80 = 0101 0000
      digitalWrite(adns5050->_sdio, HIGH);//SDOUT = 1;
    else
      digitalWrite(adns5050->_sdio, LOW);//SDOUT = 0;
    temp = (temp << 1);
    digitalWrite(adns5050->_sdio, HIGH);//SCK = 1;
    delayMicroseconds(1);//delayMicroseconds(1);			// short clock pulse
  }
	temp = data;
  for (n=0; n<8; n++) {
    delayMicroseconds(1);
    digitalWrite(adns5050->_sclk, LOW);//SCK = 0;
    delayMicroseconds(1);
    if (temp & 0x80)
      digitalWrite(adns5050->_sdio, HIGH);//SDOUT = 1;
    else
      digitalWrite(adns5050->_sdio, LOW);//SDOUT = 0;
    temp = (temp << 1);
    digitalWrite(adns5050->_sclk, HIGH);//SCK = 1;
    delayMicroseconds(1);			// short clock pulse
  }
  delayMicroseconds(20);
  digitalWrite(adns5050->_ncs, HIGH);//nADNSCS = 1; // de-select the chip

}
