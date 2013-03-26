/*
 *  Copyright (c) 2011-2012, http://atomsoft.wordpress.com
 *  All rights reserved.
 *	
 *  Thanks go out to Proprojects.wordpress.com
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1.- Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *  2.- Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/*********************************************************************
 * By:              Jason Lopez
 * Company:         http://atomsoft.wordpress.com
 * Processor:       PIC18
 * Compiler:        C18 v3.4
 * File Name:       18F27J53_I2C.c
 * Description:     PIC18F27J53 I2C Routines
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rev.     Date        Comment
 * 1.0      xx/xx/xx    Initial version
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Note: ALL ADDRESSES ARE 7 BIT. They are shifted automatically.
 *
 *********************************************************************/

#include "18F27J53_I2C.h"

//#define I2C_BB	//Define for BIT BANGED I2C
//#define I2C_HW	//Define for HARDWARE I2C

#ifdef I2C_BB
void puti2c_bb(char dev, char *data)
{
    dev = (dev<<1);				// Turn 7 bit address into 8 bit WRITE address
	
    i2c_start();				// Initiate a START
    i2c_tx(dev);				// Send the Address

    while(*data)				// Loop data bytes
    {	
        i2c_tx(*data++);		// Send a single byte at a time
    }
    i2c_stop();					// When complete send a STOP

}
void geti2c_bb(char dev, char *data, char len)
{
    dev = (dev<<1) | 0x01;		// Turn 7 bit address into a 8 bit READ address
	
    i2c_start();				// Initiate a START
    i2c_tx(dev);				// Send the address

    while(len)					// Loop through LEN bytes 
    {
        *data++ = i2c_rx(1);	// Load each byte into the data buffer
        len--;					// Decrease LEN
    }
	
    i2c_stop();					// When complete send a STOP

}
void SDA_LOW_bb(void)
{
    SDA = 0; SDA_IN = 0;		// Make SDA output and Set LOW
}
void SCL_LOW_bb(void)
{
    SCL = 0; SCL_IN = 0;		// Make SCL output and Set LOW
}

void i2c_start_bb(void)
{
	// I2C START SEQUENCE
	SDA_HIGH();
	SCL_HIGH();
	SDA_LOW();
	SCL_LOW();
}

void i2c_stop_bb(void)
{
	// I2C STOP SEQUENCE
	SDA_LOW();
	SCL_HIGH();
	SDA_HIGH();
}

char i2c_rx_bb(char ack)
{
	char x, d=0;				// X = Loop var, D = SDA Byte var
	SDA_HIGH();					// Make SDA input (Pulled HIGH)

	for(x=0; x<8; x++) {		// Loop 1 byte (8bits)
		d <<= 1;				// Shift Over SDA Byte var to the LEFT by 1 bit... MSb first

		SCL_HIGH();				// Set SCL HIGH
    
		delay(1);				// Small Delay
		if(SDA_IN)				// D = 0 because of the shift... but if SDA = 1	
			d |= 1;				// D = 1

		SCL_LOW();				// Set SCL LOW
		delay(1);				// Small Delay
	}

	if(ack)						// If ack arg is set then 
	  SDA_LOW();				// Send a ACK Bit
	else
	  SDA_HIGH();				// Send a NACK Bit

	SCL_HIGH();					// Set SCL HIGH (Input)
	delay(10);    				// Med. Delay
	SCL_LOW();					// Set SCL LOW (Output)
	SDA_HIGH();					// Set SDA HIGH
    return d;					// Return the d BYTE (SDA BYTE) 
}

char i2c_tx_bb(unsigned char d)
{
	char x;
	static char b;
	
	for(x=8; x; x--) {
		if(d&0x80)
			SDA_HIGH();
		else
			SDA_LOW();
		SCL_HIGH();
		d <<= 1;
		SCL_LOW();
	}

	SDA_HIGH();
	SCL_HIGH();
	delay(10);
	b = SDA_IN;          // Possible ACK bit
	SCL_LOW();
	return b;
}

void Init_I2C_bb(void) {

	// Processor Specific
    ADCON0 = 0;
    ANCON0 = ANCON1 = 0xFF;

    SDA = SCL_HIGH();
    SCL_IN = SDA_IN = 0;
}

#endif

#ifdef I2C_HW

#endif

// Below Are some SIMPLE delay Routines
// Used for both HW and BB I2C
void delay (unsigned int time)
{
    unsigned int t;
    for(t=0;t<time;t++)
    {
        Nop();
    }
}

void delay2 (unsigned int time)
{
    unsigned int t;
    for(t=0;t<time;t++)
    {
        Delay10KTCYx(1);
    }
}