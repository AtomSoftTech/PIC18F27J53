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
 * File Name:       18F27J53_I2C.h
 * Description:     PIC18F27J53 I2C Routines Header File
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rev.     Date        Comment
 * 1.0      xx/xx/xx    Initial version
 *********************************************************************/

#ifndef __18F27J53_I2C_H
#define __18F27J53_I2C_H

	//#define I2C_BB    //Define for BIT BANGED I2C
	#define I2C_HW    //Define for HARDWARE I2C

	// Main Header for PIC18F27J53
	#include "p18F27j53.h"
	#include "delays.h"
	
	#ifdef I2C_BB
		// I2C Port Definitions (processor specific)
		#define SCL     TRISCbits.TRISC0
		#define SCL_IN  PORTCbits.RC0
		#define SCL_HIGH() (SCL = 1)
		
		#define SDA     TRISCbits.TRISC1
		#define SDA_IN  PORTCbits.RC1
		#define SDA_HIGH() (SDA = 1)
	#elif
		// I2C Port Definitions (processor specific)
		#define SCL     TRISCbits.TRISC3
		#define SCL_IN  PORTCbits.RC3
		#define SCL_HIGH() (SCL = 1)
		
		#define SDA     TRISCbits.TRISC4
		#define SDA_IN  PORTCbits.RC4
		#define SDA_HIGH() (SDA = 1)	
		
		/* SSPCON1 REGISTER */
		#define   SSPENB    			0b00100000  	/* Enable serial port and configures SCK, SDO, SDI*/
		#define   SLAVE_7   			0b00000110     	/* I2C Slave mode, 7-bit address*/
		#define   SLAVE_10  			0b00000111    	/* I2C Slave mode, 10-bit address*/
		#define   MASTER    			0b00001000     	/* I2C Master mode */
		#define   MASTER_FIRMW			0b00001011		//I2C Firmware Controlled Master mode (slave Idle)
		#define   SLAVE_7_STSP_INT 		0b00001110		//I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
		#define   SLAVE_10_STSP_INT 	0b00001111		//I2C Slave mode, 10-bit address with Start and Stop bit interrupts enabled

		/* SSPSTAT REGISTER */
		#define   SLEW_OFF  			0b10000000  	/* Slew rate disabled for 100kHz mode */
		#define   SLEW_ON   			0b00000000  	/* Slew rate enabled for 400kHz mode  */
		
	#endif
	// Functions
	
	void i2c_start(void);
	void i2c_stop(void);
	
	char i2c_tx(unsigned char d);
	
	
	#ifdef I2C_HW
	
		void get_i2c_str(char *data, char len);
		void put_i2c_str(char *data);
		char i2c_rx(void);
		void Init_I2C( unsigned char sync_mode, unsigned char slew )
	#endif

	#ifdef I2C_BB
		void put_i2c_str(char dev, char *data);
		void get_i2c_str(char dev, char *data, char len);
		char i2c_rx(char ack);
		void Init_I2C(void);
		void SDA_LOW(void);
		void SCL_LOW(void);
	#endif

	void delay (unsigned int time);
	void delay2 (unsigned int time);
	
#endif