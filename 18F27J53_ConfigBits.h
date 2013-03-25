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
 * File Name:       18F27J53_ConfigBits.h
 * Description:     PIC18F27J53 Configuration Bits
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rev.     Date        Comment
 * 1.0      xx/xx/xx    Initial version
 *********************************************************************/
 
#ifndef __18F27J53_CONFIGBITS_H
#define __18F27J53_CONFIGBITS_H

    #pragma config WDTEN = OFF          // Watchdog Timer:
                                        // OFF            Disabled - Controlled by SWDTEN bit
                                        // ON            Enabled
                                        
    #pragma config PLLDIV = 3           // PLL Prescaler Selection:
                                        // 12            Divide by 12 (48 MHz oscillator input) 
                                        // 10            Divide by 10 (40 MHz oscillator input) 
                                        // 6            Divide by 6 (24 MHz oscillator input) 
                                        // 5            Divide by 5 (20 MHz oscillator input) 
                                        // 4            Divide by 4 (16 MHz oscillator input) 
                                        // 3            Divide by 3 (12 MHz oscillator input) 
                                        // 2            Divide by 2 (8 MHz oscillator input) 
                                        // 1            No prescale (4 MHz oscillator input drives PLL directly)
                                        
    #pragma config CFGPLLEN = ON        // PLL Enable Configuration Bit:
                                        // ON             PLL Enabled
                                        // OFF            PLL Disabled
                                        
    #pragma config STVREN = OFF         // Stack Overflow/Underflow Reset:
                                        // OFF            Disabled
                                        // ON            Enabled
                                        
    #pragma config XINST = OFF          // Extended Instruction Set:
                                        // OFF            Disabled
                                        // ON             Enabled
                                        
    #pragma config CPUDIV = OSC1        // CPU System Clock Postscaler:
                                        // OSC4_PLL6    CPU system clock divide by 6
                                        // OSC3_PLL3    CPU system clock divide by 3
                                        // OSC2_PLL2    CPU system clock divide by 2
                                        // OSC1            No CPU system clock divide 
                                        
    #pragma config CP0 = OFF            // Code Protect:
                                        // ON            Program memory is code-protected
                                        // OFF            Program memory is not code-protected
                                        
    #pragma config OSC = HSPLL          // Oscillator:
                                        // INTOSC         INTOSC 
                                        // INTOSCO        INTOSCO (CLKO-RA6) 
                                        // INTOSCPLL    INTOSCPLL 
                                        // INTOSCPLLO    INTOSCPLLO (CLKO-RA6) 
                                        // HS            HS, USB-HS 
                                        // HSPLL        HS+PLL, USB-HS+PLL 
                                        // EC            EC (CLKO-RA6), USB-EC 
                                        // ECPLL        EC+PLL (CLKO-RA6), USB-EC+PLL
                                        
    #pragma config SOSCSEL = DIG        // T1OSC/SOSC Power Selection Bits:
                                        // RESERVED        Reserved 
                                        // LOW            Low Power T1OSC/SOSC circuit selected 
                                        // DIG            Digital (SCLKI) mode selected 
                                        // HIGH            High Power T1OSC/SOSC circuit selected
                                        
    #pragma config CLKOEC = OFF         // EC Clock Out Enable Bit:
                                        // OFF            CLKO output disabled on the RA6 pin 
                                        // ON            CLKO output enabled on the RA6 pin 
                                        
    #pragma config FCMEN = OFF          // Fail-Safe Clock Monitor:
                                        // OFF            Disabled 
                                        // ON             Enabled
                                        
    #pragma config IESO = OFF           // Internal External Oscillator Switch Over Mode:
                                        // OFF            Disabled
                                        // ON            Enabled
                                        
    //#pragma config WDTPS = 32768      // Watchdog Postscaler:
                                        // 1            1:1
                                        // 2            1:2
                                        // 4            1:4
                                        // 8            1:8
                                        // 16            1:16
                                        // 32            1:32
                                        // 64            1:64
                                        // 128            1:128
                                        // 256            1:256
                                        // 512            1:512
                                        // 1024            1:1024
                                        // 2048            1:2048
                                        // 4096            1:1096
                                        // 8192            1:8192
                                        // 16384        1:16384
                                        // 32768        1:32768
                                        
    #pragma config DSWDTOSC = INTOSCREF // DSWDT Clock Select:
                                        // T1OSCREF        DSWDT uses T1OSC/T1CKI 
                                        // INTOSCREF    DSWDT uses INTRC 
                                        
    #pragma config RTCOSC = INTOSCREF   // RTCC Clock Select:
                                        // INTOSCREF    RTCC uses INTRC 
                                        // T1OSCREF        RTCC uses T1OSC/T1CKI 
                                        
    #pragma config DSBOREN = OFF        // Deep Sleep BOR:
                                        // OFF            Disabled 
                                        // ON            Enabled
                                        
    //#pragma config DSWDTPS = G2       // Deep Sleep Watchdog Postscaler:
                                        // 2            1:2 (2.1 ms) 
                                        // 8            1:8 (8.3 ms) 
                                        // 32            1:32 (33 ms) 
                                        // 128            1:128 (132 ms) 
                                        // 512            1:512 (528 ms) 
                                        // 2048            1:2,048 (2.1 seconds) 
                                        // 8192            1:8,192 (8.5 seconds) 
                                        // K32            1:32,768 (34 seconds) 
                                        // K131            1:131,072 (135 seconds) 
                                        // K524            1:524,288 (9 minutes) 
                                        // M2            1:2,097,152 (36 minutes)
                                        // M8            1:8,388,608 (2.4 hours) 
                                        // M33            1:33,554,432 (9.6 hours) 
                                        // M134            1:134,217,728 (38.5 hours)  
                                        // M536            1:536,870,912 (6.4 days) 
                                        // G2            1:2,147,483,648 (25.7 days)
    #pragma config IOL1WAY = OFF        // IOLOCK One-Way Set Enable bit:
                                        // OFF            The IOLOCK bit (PPSCON<0>) can be set and cleared as needed 
                                        // ON            The IOLOCK bit (PPSCON<0>) can be set once 
                                        
    #pragma config ADCSEL = BIT12       // ADC 10 or 12 Bit Select:
                                        // BIT12        12 - Bit ADC Enabled 
                                        // BIT10        10 - Bit ADC Enabled 
                                        
    #pragma config MSSP7B_EN = MSK7     // MSSP address masking:
                                        // MSK5            5 Bit address masking mode 
                                        // MSK7            7 Bit address masking mode 
                                        
    //#pragma config WPFP = PAGE_0      // Write/Erase Protect Page Start/End Location:
                                        // PAGE_1        Write Protect Program Flash Page 1
                                        // PAGE_2        Write Protect Program Flash Page 2
                                        // PAGE_...        Write Protect Program Flash Page ...
                                        // PAGE_126        Write Protect Program Flash Page 126
                                        // PAGE_127        Write Protect Program Flash Page 127
                                        
    #pragma config WPCFG = OFF          // Write/Erase Protect Configuration Region :
                                        // ON            Configuration Words page erase/write-protected 
                                        // OFF            Configuration Words page not erase/write-protected 
                                        
    #pragma config WPDIS = ON           // Write Protect Disable bit:
                                        // ON             WPFP[5:0], WPEND, and WPCFG bits enabled
                                        // OFF             WPFP[6:0], WPEND, and WPCFG bits ignored 
                                        
    #pragma config WPEND = PAGE_0       // Write/Erase Protect Region Select bit (valid when WPDIS = 0):
                                        // PAGE_0        Page 0 to WPFP<6:0> erase/write-protected 
                                        // PAGE_WPFP    Pages WPFP<6:0> to (Configuration Words page) write/erase protected 
                                        
    #pragma config LS48MHZ = SYS48X8    // Low Speed USB mode with 48 MHz system clock bit:
                                        // SYS24X4        System clock at 24 MHz USB CLKEN divide-by is set to 4
                                        // SYS48X8        System clock at 48 MHz USB CLKEN divide-by is set to 8
 
#endif