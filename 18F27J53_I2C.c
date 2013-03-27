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

#ifdef I2C_BB
void put_i2c_str(char dev, char *data)
{
    dev = (dev<<1);                // Turn 7 bit address into 8 bit WRITE address
    
    i2c_start();                   // Initiate a START
    i2c_tx(dev);                   // Send the Address

    while(*data)                   // Loop data bytes
    {    
        i2c_tx(*data++);           // Send a single byte at a time
    }
    i2c_stop();                    // When complete send a STOP

}
void get_i2c_str(char dev, char *data, char len)
{
    dev = (dev<<1) | 0x01;        // Turn 7 bit address into a 8 bit READ address
    
    i2c_start();                  // Initiate a START
    i2c_tx(dev);                  // Send the address

    while(len)                    // Loop through LEN bytes 
    {
        *data++ = i2c_rx(1);      // Load each byte into the data buffer
        len--;                    // Decrease LEN
    }
    
    i2c_stop();                   // When complete send a STOP

}
void SDA_LOW(void)
{
    SDA = 0; SDA_IN = 0;          // Make SDA output and Set LOW
}
void SCL_LOW(void)
{
    SCL = 0; SCL_IN = 0;          // Make SCL output and Set LOW
}

void i2c_start(void)
{
    // I2C START SEQUENCE
    SDA_HIGH();
    SCL_HIGH();
    SDA_LOW();
    SCL_LOW();
}

void i2c_stop(void)
{
    // I2C STOP SEQUENCE
    SDA_LOW();
    SCL_HIGH();
    SDA_HIGH();
}

char i2c_rx(char ack)
{
    char x, d=0;                 // X = Loop var, D = SDA Byte var
    SDA_HIGH();                  // Make SDA input (Pulled HIGH)

    for(x=0; x<8; x++) {         // Loop 1 byte (8bits)
        d <<= 1;                 // Shift Over SDA Byte var to the LEFT by 1 bit... MSb first

        SCL_HIGH();              // Set SCL HIGH
    
        delay(1);                // Small Delay
        if(SDA_IN)               // D = 0 because of the shift... but if SDA = 1    
            d |= 1;              // D = 1

        SCL_LOW();               // Set SCL LOW
        delay(1);                // Small Delay
    }

    if(ack)                      // If ack arg is set then 
      SDA_LOW();                 // Send a ACK Bit
    else
      SDA_HIGH();                // Send a NACK Bit

    SCL_HIGH();                  // Set SCL HIGH (Input)
    delay(10);                   // Med. Delay
    SCL_LOW();                   // Set SCL LOW (Output)
    SDA_HIGH();                  // Set SDA HIGH
    return d;                    // Return the d BYTE (SDA BYTE) 
}

char i2c_tx(unsigned char d)
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
    b = SDA_IN;                  // Possible ACK bit
    SCL_LOW();
    return b;
}

void Init_I2C(void) {

    // Processor Specific
    ADCON0 = 0;
    ANCON0 = ANCON1 = 0xFF;

    SDA = SCL_HIGH();
    SCL_IN = SDA_IN = 0;
}

#endif

#ifdef I2C_HW
	void put_i2c_str(char *data)
	{
		unsigned char temp;  
		while ( *data )                 // transmit data until null character 
		{
		if ( SSP1CON1bits.SSPM3 )      // if Master transmitter then execute the following
		{
		  temp = putcI2C1 ( *data );
		  if (temp ) return ( temp );   	
		  //if ( putcI2C1( *data ) )    // write 1 byte
		  //{
		  //  return ( -3 );             // return with write collision error
		  //}
		  //IdleI2C1();                  // test for idle condition
		  //if ( SSP1CON2bits.ACKSTAT )  // test received ack bit state
		  //{
		  //  return ( -2 );             // bus device responded with  NOT ACK
		  //}                            // terminate putsI2C1() function
		}

		else                           // else Slave transmitter
		{
		  PIR1bits.SSP1IF = 0;         // reset SSP1IF bit
		  SSP1BUF = *data;            // load SSP1BUF with new data
		  SSP1CON1bits.CKP = 1;        // release clock line 
		  while ( !PIR1bits.SSP1IF );  // wait until ninth clock pulse received

		  if ( ( SSP1CON1bits.CKP ) && ( !SSP1STATbits.BF ) )// if R/W=0 and BF=0, NOT ACK was received
		  {
			return ( -2 );             // terminate PutsI2C1() function
		  }
		}

		data ++;                        // increment pointer

		}                                // continue data writes until null character

		return ( 0 );
	}
	void get_i2c_str(char *data, char len)
	{
	    while ( length-- )            // perform getcI2C1() for 'len' number of bytes
		{
		  *data++ = getcI2C1();       // save byte received
		  while ( SSP1CON2bits.RCEN ); // check that receive sequence is over    

		  if ( PIR2bits.BCL1IF )       // test for bus collision
		  {
			return ( -1 );             // return with Bus Collision error 
		  }

		  
		if( ((SSP1CON1&0x0F)==0x08) || ((SSP1CON1&0x0F)==0x0B) )	//master mode only
		{	
		  if ( len )               // test if 'len' bytes have been read
		  {
			SSP1CON2bits.ACKDT = 0;    // set acknowledge bit state for ACK        
			SSP1CON2bits.ACKEN = 1;    // initiate bus acknowledge sequence
			while ( SSP1CON2bits.ACKEN ); // wait until ACK sequence is over 
		  } 
		} 
		  
		}
		return ( 0 );                  // last byte received so don't send ACK      
	}
	
	void i2c_start(void)
	{
		SSP1CON2bits.SEN = 1;            // initiate bus start condition
	}
	
	void i2c_stop(void)
	{
		SSP1CON2bits.PEN = 1;            // initiate bus stop condition
	}
	
	char i2c_rx(void)
	{
		if( ((SSP1CON1&0x0F)==0x08) || ((SSP1CON1&0x0F)==0x0B) )	//master mode only
			SSP1CON2bits.RCEN = 1;           // enable master for 1 byte reception
		
		while ( !SSP1STATbits.BF );      // wait until byte received  

		return ( SSP1BUF );              // return with read byte 
	}
	
	char i2c_tx(unsigned char d)
	{
		SSP1BUF = d;           // write single byte to SSP1BUF
		if ( SSP1CON1bits.WCOL )      // test if write collision occurred
			return ( -1 );              // if WCOL bit is set return negative #
		else
		{
			if( ((SSP1CON1&0x0F)!=0x08) && ((SSP1CON1&0x0F)!=0x0B) )	//slave mode only 
			{
			
			SSP1CON1bits.CKP = 1;        // release clock line 
			while ( !PIR1bits.SSP1IF );  // wait until ninth clock pulse received

			if ( ( !SSP1STATbits.R_W ) && ( !SSP1STATbits.BF ) )// if R/W=0 and BF=0, NOT ACK was received
			{
				return ( -2 );             //Return NACK
			}	
			else
				return(0);				//Return ACK

		}
		else if( ((SSP1CON1&0x0F)==0x08) || ((SSP1CON1&0x0F)==0x0B) )	//master mode only	
		{
			while( SSP1STATbits.BF );   // wait until write cycle is complete      
			IdleI2C1();                  // ensure module is idle
			if ( SSP1CON2bits.ACKSTAT ) // test for ACK condition received
				return ( -2 );				//Return NACK	
			else
				return ( 0 );               //Return ACK
			}
		}
	}	
	
	void Init_I2C( unsigned char sync_mode, unsigned char slew )
	{
		SSP1STAT &= 0x3F;                // power on state 
		SSP1CON1 = 0x00;                 // power on state
		SSP1CON2 = 0x00;                 // power on state
		SSP1CON1 |= sync_mode;           // select serial mode 
		SSP1STAT |= slew;                // slew rate on/off 

		SCL_HIGH = 1;                    // Set SCL1 (PORTC,3) pin to input
		SDA_HIGH = 1;                    // Set SDA1 (PORTC,4) pin to input

		SSP1CON1 |= SSPENB;              // enable synchronous serial port 
	}
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