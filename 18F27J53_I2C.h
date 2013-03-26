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

	// Main Header for PIC18F27J53
	#include "p18F27j53.h"
	#include "delays.h"
	
	// I2C Port Definitions (processor specific)
    #define SCL     TRISCbits.TRISC0
    #define SCL_IN  PORTCbits.RC0
	#define SCL_HIGH() (SCL = 1)
    
	#define SDA     TRISCbits.TRISC1
    #define SDA_IN  PORTCbits.RC1
	#define SDA_HIGH() (SDA = 1)
	
	// Functions
	
#endif