/*********************************************************************
 *
 *	Hardware specific definitions
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    None
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Compiler:        Microchip C32 v1.05 or higher
 *					Microchip C30 v3.12 or higher
 *					Microchip C18 v3.30 or higher
 *					HI-TECH PICC-18 PRO 9.63PL2 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2009 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder		10/03/06	Original, copied from Compiler.h
 * Ken Hesky            07/01/08    Added ZG2100-specific features
 ********************************************************************/
#ifndef __HARDWARE_PROFILE_H
#define __HARDWARE_PROFILE_H

#include "GenericTypeDefs.h"
#include "Compiler.h"

// Choose which hardware profile to compile for here.  See 
// the hardware profiles below for meaning of various options.  
//#define EXPLORER_16		// dsPIC33FJ256GP710 PIMs
#define DSP804

// Set configuration fuses (but only once)
#if defined(THIS_IS_STACK_APPLICATION)
	#if defined(__dsPIC33F__)
		// Explorer 16 board
//		_FOSCSEL(FNOSC_PRIPLL)			// PLL enabled
//		_FOSC(OSCIOFNC_OFF & POSCMD_XT)	// XT Osc
//		_FWDT(FWDTEN_OFF)				// Disable Watchdog timer
//			
//    	_FICD (ICS_PGD3)              // ICD Pin Placement Select bits (EMUC/EMUD share PGC3/PGD3)
//
		// JTAG should be disabled as well
	#endif
#endif // Prevent more than one set of config fuse definitions

// Clock frequency value.
// This value is used to calculate Tick Counter value
#if defined(__dsPIC33F__)	
	// dsPIC33F processor
	#define GetSystemClock()		(80000000ul)      // Hz
	#define GetInstructionClock()	(GetSystemClock()/2)
	#define GetPeripheralClock()	GetInstructionClock()
#endif

// Hardware mappings
#if defined(EXPLORER_16)
// Explorer 16 + PIC24FJ128GA010/PIC24HJ256GP610/dsPIC33FJ256GP710/
//				 PIC32MX460F512L/PIC32MX360F512L PIM + 
//               Fast 100Mbps Ethernet PICtail Plus or Ethernet PICtail Plus or ZeroG ZG2100M WiFi PICtail Plus

	#define LED0_TRIS			(TRISAbits.TRISA0)	// Ref D3
	#define LED0_IO				(LATAbits.LATA0)	
	#define LED1_TRIS			(TRISAbits.TRISA1)	// Ref D4
	#define LED1_IO				(LATAbits.LATA1)
	#define LED2_TRIS			(TRISAbits.TRISA2)	// Ref D5
	#define LED2_IO				(LATAbits.LATA2)
	#define LED3_TRIS			(TRISAbits.TRISA3)	// Ref D6
	#define LED3_IO				(LATAbits.LATA3)
	#define LED4_TRIS			(TRISAbits.TRISA4)	// Ref D7
	#define LED4_IO				(LATAbits.LATA4)
	#define LED5_TRIS			(TRISAbits.TRISA5)	// Ref D8
	#define LED5_IO				(LATAbits.LATA5)
	#define LED6_TRIS			(TRISAbits.TRISA6)	// Ref D9
	#define LED6_IO				(LATAbits.LATA6)
	#define LED7_TRIS			(TRISAbits.TRISA7)	// Ref D10	// Note: This is multiplexed with BUTTON1
	#define LED7_IO				(LATAbits.LATA7)
	#define LED_GET()			(*((volatile unsigned char*)(&LATA)))
	#define LED_PUT(a)			(*((volatile unsigned char*)(&LATA)) = (a))


	#define BUTTON0_TRIS		(TRISDbits.TRISD13)	// Ref S4
	#define	BUTTON0_IO			(PORTDbits.RD13)
	#define BUTTON1_TRIS		(TRISAbits.TRISA7)	// Ref S5	// Note: This is multiplexed with LED7
	#define	BUTTON1_IO			(PORTAbits.RA7)
	#define BUTTON2_TRIS		(TRISDbits.TRISD7)	// Ref S6
	#define	BUTTON2_IO			(PORTDbits.RD7)
	#define BUTTON3_TRIS		(TRISDbits.TRISD6)	// Ref S3
	#define	BUTTON3_IO			(PORTDbits.RD6)

	#define UARTTX_TRIS			(TRISFbits.TRISF5)
	#define UARTTX_IO			(PORTFbits.RF5)
	#define UARTRX_TRIS			(TRISFbits.TRISF4)
	#define UARTRX_IO			(PORTFbits.RF4)


	// ENC28J60 I/O pins
	#define ENC_CS_TRIS			(TRISDbits.TRISD14)	// Comment this line out if you are using the ENC424J600/624J600, ZeroG ZG2100, or other network controller.
	#define ENC_CS_IO			(PORTDbits.RD14)
	#define ENC_RST_TRIS		(TRISDbits.TRISD15)	// Not connected by default.  It is okay to leave this pin completely unconnected, in which case this macro should simply be left undefined.
	#define ENC_RST_IO			(PORTDbits.RD15)
	// SPI SCK, SDI, SDO pins are automatically controlled by the 
	// PIC24/dsPIC/PIC32 SPI module 
	#if defined(__C30__)	// PIC24F, PIC24H, dsPIC30, dsPIC33
		#define ENC_SPI_IF			(IFS0bits.SPI1IF)
		#define ENC_SSPBUF			(SPI1BUF)
		#define ENC_SPISTAT			(SPI1STAT)
		#define ENC_SPISTATbits		(SPI1STATbits)
		#define ENC_SPICON1			(SPI1CON1)
		#define ENC_SPICON1bits		(SPI1CON1bits)
		#define ENC_SPICON2			(SPI1CON2)
	#else					// PIC32
		#define ENC_SPI_IF			(IFS0bits.SPI1RXIF)
		#define ENC_SSPBUF			(SPI1BUF)
		#define ENC_SPISTATbits		(SPI1STATbits)
		#define ENC_SPICON1			(SPI1CON)
		#define ENC_SPICON1bits		(SPI1CONbits)
		#define ENC_SPIBRG			(SPI1BRG)
	#endif


	// ENC624J600 Interface Configuration
	// Comment out ENC100_INTERFACE_MODE if you don't have an ENC624J600 or 
	// ENC424J600.  Otherwise, choose the correct setting for the interface you 
	// are using.  Legal values are:
	//  - Commented out: No ENC424J600/624J600 present or used.  All other 
	//                   ENC100_* macros are ignored.
	//	- 0: SPI mode using CS, SCK, SI, and SO pins
	//  - 1: 8-bit demultiplexed PSP Mode 1 with RD and WR pins
	//  - 2: *8-bit demultiplexed PSP Mode 2 with R/Wbar and EN pins
	//  - 3: *16-bit demultiplexed PSP Mode 3 with RD, WRL, and WRH pins
	//  - 4: *16-bit demultiplexed PSP Mode 4 with R/Wbar, B0SEL, and B1SEL pins
	//  - 5: 8-bit multiplexed PSP Mode 5 with RD and WR pins
	//  - 6: *8-bit multiplexed PSP Mode 6 with R/Wbar and EN pins
	//  - 9: 16-bit multiplexed PSP Mode 9 with AL, RD, WRL, and WRH pins
	//  - 10: *16-bit multiplexed PSP Mode 10 with AL, R/Wbar, B0SEL, and B1SEL 
	//        pins
	// *IMPORTANT NOTE: DO NOT USE PSP MODE 2, 4, 6, OR 10 ON EXPLORER 16! 
	// Attempting to do so will cause bus contention with the LCD module which 
	// shares the PMP.  Also, PSP Mode 3 is risky on the Explorer 16 since it 
	// can randomly cause bus contention with the 25LC256 EEPROM.
//	#define ENC100_INTERFACE_MODE			0

	// If using a parallel interface, direct RAM addressing can be used (if all 
	// addresses wires are connected), or a reduced number of pins can be used 
	// for indirect addressing.  If using an SPI interface or PSP Mode 9 or 10 
	// (multiplexed 16-bit modes), which require all address lines to always be 
	// connected, then this option is ignored. Comment out or uncomment this 
	// macro to match your hardware connections.
	#define ENC100_PSP_USE_INDIRECT_RAM_ADDRESSING

	// ENC424J600/624J600 parallel indirect address remapping macro function.
	// This section translates SFR and RAM addresses presented to the 
	// ReadMemory() and WriteMemory() APIs in ENCX24J600.c to the actual 
	// addresses that must be presented on the parallel interface.  This macro 
	// must be modified to match your hardware if you are using an indirect PSP 
	// addressing mode (ENC100_PSP_USE_INDIRECT_RAM_ADDRESSING is defined) and 
	// have some of your address lines tied off to Vdd.  If you are using the 
	// SPI interface, then this section can be ignored or deleted.
	#if (ENC100_INTERFACE_MODE == 1) || (ENC100_INTERFACE_MODE == 2) || (ENC100_INTERFACE_MODE == 5) || (ENC100_INTERFACE_MODE == 6) // 8-bit PSP
		#define ENC100_TRANSLATE_TO_PIN_ADDR(a)		((((a)&0x0100)<<6) | ((a)&0x00FF))
	#elif (ENC100_INTERFACE_MODE == 3) || (ENC100_INTERFACE_MODE == 4) // 16-bit PSP
		#define ENC100_TRANSLATE_TO_PIN_ADDR(a)		(a)
	#endif

	// Auto-crossover pins on Fast 100Mbps Ethernet PICtail/PICtail Plus.  If 
	// your circuit doesn't have such a feature, delete these two defines.
	#define ENC100_MDIX_TRIS				(TRISBbits.TRISB3)
	#define ENC100_MDIX_IO					(LATBbits.LATB3)

	// ENC624J600 I/O control and status pins
	// If a pin is not required for your selected ENC100_INTERFACE_MODE 
	// interface selection (ex: WRH/B1SEL for PSP modes 1, 2, 5, and 6), then 
	// you can ignore, delete, or put anything for the pin definition.  Also, 
	// the INT and POR pins are entirely optional.  If not connected, comment 
	// them out.
	#if defined(__dsPIC33FJ256GP710__) || defined(__PIC24HJ256GP610__)
		#define ENC100_INT_TRIS				(TRISAbits.TRISA13)		// INT signal is optional and currently unused in the Microchip TCP/IP Stack.  Leave this pin disconnected and comment out this pin definition if you don't want it.
		#define ENC100_INT_IO				(PORTAbits.RA13)
	#else
		#define ENC100_INT_TRIS				(TRISEbits.TRISE9)		// INT signal is optional and currently unused in the Microchip TCP/IP Stack.  Leave this pin disconnected and comment out this pin definition if you don't want it.
		#define ENC100_INT_IO				(PORTEbits.RE9)
	#endif
	#if (ENC100_INTERFACE_MODE >= 1)	// Parallel mode
		// PSP control signal pinout
		#define ENC100_CS_TRIS					(TRISAbits.TRISA5)	// CS is optional in PSP mode.  If you are not sharing the parallel bus with another device, tie CS to Vdd and comment out this pin definition.
		#define ENC100_CS_IO					(LATAbits.LATA5)
		#define ENC100_POR_TRIS					(TRISCbits.TRISC1)	// POR signal is optional.  If your application doesn't have a power disconnect feature, comment out this pin definition.
		#define ENC100_POR_IO					(LATCbits.LATC1)
		#define ENC100_SO_WR_B0SEL_EN_TRIS		(TRISDbits.TRISD4)
		#define ENC100_SO_WR_B0SEL_EN_IO		(LATDbits.LATD4)
		#define ENC100_SI_RD_RW_TRIS			(TRISDbits.TRISD5)
		#define ENC100_SI_RD_RW_IO				(LATDbits.LATD5)
		#define ENC100_SCK_AL_TRIS				(TRISBbits.TRISB15)
		#define ENC100_SCK_AL_IO				(LATBbits.LATB15)
	#else	
		// SPI pinout
		#define ENC100_CS_TRIS					(TRISDbits.TRISD14)	// CS is mandatory when using the SPI interface
		#define ENC100_CS_IO					(LATDbits.LATD14)
		#define ENC100_POR_TRIS					(TRISDbits.TRISD15)	// POR signal is optional.  If your application doesn't have a power disconnect feature, comment out this pin definition.
		#define ENC100_POR_IO					(LATDbits.LATD15)
		#define ENC100_SO_WR_B0SEL_EN_TRIS		(TRISFbits.TRISF7)	// SO is ENCX24J600 Serial Out, which needs to connect to the PIC SDI pin for SPI mode
		#define ENC100_SO_WR_B0SEL_EN_IO		(PORTFbits.RF7)
		#define ENC100_SI_RD_RW_TRIS			(TRISFbits.TRISF8)	// SI is ENCX24J600 Serial In, which needs to connect to the PIC SDO pin for SPI mode
		#define ENC100_SI_RD_RW_IO				(LATFbits.LATF8)
		#define ENC100_SCK_AL_TRIS				(TRISFbits.TRISF6)
	#endif

	// ENC624J600 Bit Bang PSP I/O macros and pin configuration for address and 
	// data.  If using the SPI interface (ENC100_INTERFACE_MODE is 0) then this 
	// section is not used and can be ignored or deleted.  If using the PIC PMP
	// hardware module (if present), then ENC100_BIT_BANG_PMP must be commented 
	// out and the remaining definitions will be ignored/can be deleted.  
	// Otherwise, if you are using a parallel interface mode, but do not have a 
	// PMP (or want to interface using different pins), define 
	// ENC100_BIT_BANG_PMP and properly configure the applicable macros.
	//#define ENC100_BIT_BANG_PMP
	#if defined(ENC100_BIT_BANG_PMP)
		#if ENC100_INTERFACE_MODE == 1 || ENC100_INTERFACE_MODE == 2	// Dumultiplexed 8-bit address/data modes
			// SPI2 CANNOT BE ENABLED WHEN ACCESSING THE ENC624J600 FOR THESE TWO MODES AS THE PINS OVERLAP WITH ADDRESS LINES.
			#if defined(ENC100_PSP_USE_INDIRECT_RAM_ADDRESSING)	// Only ENC624J600 address pins A0-A8 connected (A9-A14 tied to Vdd)
				#define ENC100_INIT_PSP_BIT_BANG()	do{((volatile BYTE*)&AD1PCFGH)[1] = 0xFF; ((volatile BYTE*)&AD1PCFGL)[1] |= 0xC0;}while(0)	// Disable AN24-AN31 and AN14-AN15 analog inputs on RE0-RE7 and RB14-RB15 pins (ENCX24J600 AD0-AD7, A1-A0)
				#define ENC100_SET_ADDR_TRIS_OUT()	do{TRISB &= 0x3FFF; TRISG &= 0xFC3F; TRISA &= 0xF9FF; TRISDbits.TRISD11 = 0;}while(0)
				#define ENC100_SET_ADDR_IO(a)		do{WORD _SetMacro = (a); LATBbits.LATB15 = 0; LATBbits.LATB14 = 0; LATG &= 0xFC3F; LATAbits.LATA10 = 0; LATAbits.LATA9 = 0; LATDbits.LATD11 = 0; if(_SetMacro & 0x0001) LATBbits.LATB15 = 1; if(_SetMacro & 0x0002) LATBbits.LATB14 = 1; if(_SetMacro & 0x0004) LATGbits.LATG9 = 1; if(_SetMacro & 0x0008) LATGbits.LATG8 = 1; if(_SetMacro & 0x0010) LATGbits.LATG7 = 1; if(_SetMacro & 0x0020) LATGbits.LATG6 = 1; if(_SetMacro & 0x0040) LATAbits.LATA10 = 1; if(_SetMacro & 0x0080) LATAbits.LATA9 = 1; if(_SetMacro & 0x4000) LATDbits.LATD11 = 1;}while(0)
				#define ENC100_SET_AD_TRIS_IN()		(((volatile BYTE*)&TRISE)[0] = 0xFF)
				#define ENC100_SET_AD_TRIS_OUT()	(((volatile BYTE*)&TRISE)[0] = 0x00)
				#define ENC100_GET_AD_IO()			(((volatile BYTE*)&PORTE)[0])
				#define ENC100_SET_AD_IO(data)		(((volatile BYTE*)&LATE)[0] = (data))
			#else 	// All ENC624J600 address pins A0-A14 connected
				#define ENC100_INIT_PSP_BIT_BANG()	do{((volatile BYTE*)&AD1PCFGH)[1] = 0xFF; ((volatile BYTE*)&AD1PCFGL)[1] |= 0xFC;}while(0)	// Disable AN24-AN31 and AN10-AN15 analog inputs on RE0-RE7 and RB10-RB15 pins (ENCX24J600 AD0-AD7, A1-A0, A13-A10)
				#define ENC100_SET_ADDR_TRIS_OUT()	do{TRISB &= 0x03FF; TRISG &= 0xFC3F; TRISA &= 0xF9FF; TRISF &= 0xFFCF; TRISDbits.TRISD11 = 0;}while(0)
				#define ENC100_SET_ADDR_IO(a)		do{WORD _SetMacro = (a); LATB &= 0x03FF; LATG &= 0xFC3F; LATAbits.LATA10 = 0; LATAbits.LATA9 = 0; LATFbits.LATF5 = 0; LATFbits.LATF4 = 0; LATDbits.LATD11 = 0; if(_SetMacro & 0x0001) LATBbits.LATB15 = 1; if(_SetMacro & 0x0002) LATBbits.LATB14 = 1; if(_SetMacro & 0x0004) LATGbits.LATG9 = 1; if(_SetMacro & 0x0008) LATGbits.LATG8 = 1; if(_SetMacro & 0x0010) LATGbits.LATG7 = 1; if(_SetMacro & 0x0020) LATGbits.LATG6 = 1; if(_SetMacro & 0x0040) LATAbits.LATA10 = 1; if(_SetMacro & 0x0080) LATAbits.LATA9 = 1; if(_SetMacro & 0x0100) LATFbits.LATF5 = 1; if(_SetMacro & 0x0200) LATFbits.LATF4 = 1; if(_SetMacro & 0x0400) LATBbits.LATB13 = 1; if(_SetMacro & 0x0800) LATBbits.LATB12 = 1; if(_SetMacro & 0x1000) LATBbits.LATB11 = 1; if(_SetMacro & 0x2000) LATBbits.LATB10 = 1; if(_SetMacro & 0x4000) LATDbits.LATD11 = 1;}while(0)
				#define ENC100_SET_AD_TRIS_IN()		(((volatile BYTE*)&TRISE)[0] = 0xFF)
				#define ENC100_SET_AD_TRIS_OUT()	(((volatile BYTE*)&TRISE)[0] = 0x00)
				#define ENC100_GET_AD_IO()			(((volatile BYTE*)&PORTE)[0])
				#define ENC100_SET_AD_IO(data)		(((volatile BYTE*)&LATE)[0] = (data))
			#endif
		#elif ENC100_INTERFACE_MODE == 3 || ENC100_INTERFACE_MODE == 4	// Dumultiplexed 16-bit address/data modes
			#if defined(ENC100_PSP_USE_INDIRECT_RAM_ADDRESSING)	// Only ENC624J600 address pins A0-A7 connected (A8-A13 tied to Vdd)
				#define ENC100_INIT_PSP_BIT_BANG()	do{AD1PCFGH = 0xFFFF; AD1PCFGL = 0xFFFF; AD2PCFGL = 0xFFFF;}while(0)
				#define ENC100_SET_ADDR_TRIS_OUT()	do{TRISB &= 0x3FFF; TRISG &= 0xFC3F; TRISA &= 0xF9FF;}while(0)
				#define ENC100_SET_ADDR_IO(a)		do{WORD _SetMacro = (a); LATBbits.LATB15 = 0; LATBbits.LATB14 = 0; LATG &= 0xFC3F; LATAbits.LATA10 = 0; LATAbits.LATA9 = 0; if(_SetMacro & 0x0001) LATBbits.LATB15 = 1; if(_SetMacro & 0x0002) LATBbits.LATB14 = 1; if(_SetMacro & 0x0004) LATGbits.LATG9 = 1; if(_SetMacro & 0x0008) LATGbits.LATG8 = 1; if(_SetMacro & 0x0010) LATGbits.LATG7 = 1; if(_SetMacro & 0x0020) LATGbits.LATG6 = 1; if(_SetMacro & 0x0040) LATAbits.LATA10 = 1; if(_SetMacro & 0x0080) LATAbits.LATA9 = 1;}while(0)
				#define ENC100_WRH_B1SEL_TRIS		ENC100_SO_WR_B0SEL_EN_TRIS
				#define ENC100_WRH_B1SEL_IO			ENC100_SO_WR_B0SEL_EN_IO
				#define ENC100_SET_AD_TRIS_IN()		do{((volatile BYTE*)&TRISE)[0] = 0xFF; TRISG |= 0x0003; TRISF |= 0x0003; TRISD |= 0x30C0;}while(0)
				#define ENC100_SET_AD_TRIS_OUT()	do{((volatile BYTE*)&TRISE)[0] = 0x00; TRISG &= 0xFFFC; TRISF |= 0xFFFC; TRISD |= 0xCF3F;}while(0)
				#define ENC100_GET_AD_IOL()			(((volatile BYTE*)&PORTE)[0])
				#if defined(__ENCX24J600_C)
				    static inline __attribute__((__always_inline__)) BYTE ENC100_GET_AD_IOH(void)
				    {
					    BYTE_VAL ret = {0}; if(PORTGbits.RG0) ret.bits.b0 = 1; if(PORTGbits.RG1) ret.bits.b1 = 1; if(PORTFbits.RF1) ret.bits.b2 = 1; if(PORTFbits.RF0) ret.bits.b3 = 1; if(PORTDbits.RD12) ret.bits.b4 = 1; if(PORTDbits.RD13) ret.bits.b5 = 1; if(PORTDbits.RD6) ret.bits.b6 = 1; if(PORTDbits.RD7) ret.bits.b7 = 1; return ret.Val;
				    }
				#endif
				#define ENC100_GET_AD_IO()			((((volatile BYTE*)&PORTE)[0]) | ENC100_GET_AD_IOH();)
				#define ENC100_SET_AD_IO(data)		do{WORD _SetMacro = (data); ((volatile BYTE*)&LATE)[0] = (BYTE)_SetMacro; LATG &= 0xFFFC; LATF |= 0xFFFC; LATD |= 0xCF3F; if(_SetMacro & 0x0100) LATGbits.LATG0 = 1; if(_SetMacro & 0x0200) LATGbits.LATG1 = 1; if(_SetMacro & 0x0400) LATFbits.LATF1 = 1; if(_SetMacro & 0x0800) LATFbits.LATF0 = 1; if(_SetMacro & 0x1000) LATDbits.LATD12 = 1; if(_SetMacro & 0x2000) LATDbits.LATD13 = 1; if (_SetMacro & 0x4000) LATDbits.LATD6 = 1; if(_SetMacro & 0x8000) LATDbits.LATD7 = 1;}while(0)
			#else 	// All ENC624J600 address pins A0-A13 connected
				#define ENC100_INIT_PSP_BIT_BANG()	do{AD1PCFGH = 0xFFFF; AD1PCFGL = 0xFFFF; AD2PCFGL = 0xFFFF;}while(0)
				#define ENC100_SET_ADDR_TRIS_OUT()	do{TRISE &= 0xFF00; TRISG &= 0x8FF0;}while(0)
				#define ENC100_SET_ADDR_IO(a)		do{WORD _wAddrSetMacro = (a); LATBbits.LATB15 = ((((BYTE*)&_wAddrSetMacro)[0]) & 0x1) == 0x1; LATBbits.LATB14 = ((((BYTE*)&_wAddrSetMacro)[0]) & 0x2) == 0x2; LATGbits.LATG9 = ((((BYTE*)&_wAddrSetMacro)[0]) & 0x4) == 0x4; LATG8bits.LATG8 = ((((BYTE*)&_wAddrSetMacro)[0]) & 0x8) == 0x8; LATGbits.LATG7 = ((((BYTE*)&_wAddrSetMacro)[0]) & 0x10) == 0x10; LATGbits.LATG6 = ((((BYTE*)&_wAddrSetMacro)[0]) & 0x20) == 0x20; LATAbits.LATA10 = ((((BYTE*)&_wAddrSetMacro)[0]) & 0x40) == 0x40; LATAbits.LATA9 = ((((BYTE*)&_wAddrSetMacro)[0]) & 0x80) == 0x80; LATDbits.LATD11 = ((((BYTE*)&_wAddrSetMacro)[1]) & 0x40) == 0x40;}while(0)
				#define ENC100_WRH_B1SEL_TRIS		ENC100_SO_WR_B0SEL_EN_TRIS
				#define ENC100_WRH_B1SEL_IO			ENC100_SO_WR_B0SEL_EN_IO
				#define ENC100_SET_AD_TRIS_IN		do{TRISD = 0xFFFF;}while(0)
				#define ENC100_SET_AD_TRIS_OUT		do{TRISD = 0x0000;}while(0)
				#define ENC100_GET_AD_IO()			(((volatile WORD*)&PORTD)[0])
				#define ENC100_SET_AD_IO(data)		(((volatile WORD*)&LATD)[0] = (data))
			#endif
		#elif ENC100_INTERFACE_MODE == 5 || ENC100_INTERFACE_MODE == 6	// Mutliplexed 8-bit address/data modes
			#if defined(ENC100_PSP_USE_INDIRECT_RAM_ADDRESSING)	// Only ENCX24J600 address pins AD0-AD8 connected (AD9-AD14 tied to Vdd)
				#define ENC100_INIT_PSP_BIT_BANG()	do{((volatile BYTE*)&AD1PCFGH)[1] = 0xFF;}while(0)	// Disable AN24-AN31 analog inputs on RE0-RE7 pins (ENCX24J600 AD0-AD7)
				#define ENC100_SET_AD_TRIS_IN		do{((volatile BYTE*)&TRISE)[0] = 0xFF;}while(0)
				#define ENC100_SET_AD_TRIS_OUT		do{((volatile BYTE*)&TRISE)[0] = 0x00; TRISDbits.TRISD11 = 0;}while(0)
				#define ENC100_GET_AD_IO()			(((volatile BYTE*)&PORTE)[0])
				#define ENC100_SET_AD_IO(data)		do{WORD _wSetMacro = (data); ((volatile BYTE*)&LATE)[0] = (BYTE)_wSetMacro; LATDbits.LATD11 = 0; if(_wSetMacro & 0x4000) LATDbits.LATD11 = 1;}while(0)
				#define ENC100_SET_AD_IOL(data)		(((volatile BYTE*)&LATE)[0] = (BYTE)(data))
			#else 	// All ENCX24J600 address pins AD0-AD14 connected
				// This pinout is bad for doing 8-bit bit-bang operations with all address lines.  The Fast 100Mbps Ethernet PICtail Plus hardware is wired for PMP hardware support, which requires this pinout.  However, if you are designing a custom board, you can simplify these read/write operations dramatically if you wire things more logically by putting all 15 I/O pins, in order, on PORTB or PORTD.  Such a change would enhance performance.
				// UART2 CANNOT BE USED OR ENABLED FOR THESE TWO MODES AS THE PINS OVERLAP WITH ADDRESS LINES.
				#define ENC100_INIT_PSP_BIT_BANG()	do{AD1PCFGL |= 0x3C00; ((volatile BYTE*)&AD1PCFGH)[1] = 0xFF;}while(0)	// Disable AN10-AN13 and AN24-AN31 analog inputs on RB10-RB13 and RE0-RE7 pins (ENCX24J600 AD13-AD10 and AD0-AD7)
				#define ENC100_SET_AD_TRIS_IN		do{((volatile BYTE*)&TRISE)[0] = 0xFF; TRISF |= 0x0030; TRISB |= 0x3C00; TRISD |= 0x0C00;}while(0)
				#define ENC100_SET_AD_TRIS_OUT		do{((volatile BYTE*)&TRISE)[0] = 0x00; TRISF &= 0xFFCF; TRISB &= 0xC3FF; TRISD &= 0xF3FF;}while(0)
				#define ENC100_GET_AD_IO()			(((volatile BYTE*)&PORTE)[0])
				#define ENC100_SET_AD_IO(data)		do{WORD _wDataSetMacro = (data); ((volatile BYTE*)&LATE)[0] = ((BYTE*)&_wDataSetMacro)[0]; LATFbits.LATF5 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x1) == 0x1; LATFbits.LATF4 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x2) == 0x2; LATBbits.LATB13 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x4) == 0x4; LATBbits.LATB12 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x8) == 0x8; LATBbits.LATB11 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x10) == 0x10; LATBbits.LATB10 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x20) == 0x20; LATDbits.LATD10 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x40) == 0x40; LATDbits.LATD11 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x80) == 0x80;}while(0)
				#define ENC100_SET_AD_IOL(data)		(((volatile BYTE*)&LATE)[0] = (BYTE)(data))
			#endif
		#elif ENC100_INTERFACE_MODE == 9 || ENC100_INTERFACE_MODE == 10	// Mutliplexed 16-bit address/data modes
			// All ENC624J600 adddress/data pins AD0-AD15 connected (required for 16-bit data, so there is no differentiation for indirect versus direct addressing mode)
			// This pinout is awful for doing 16-bit bit-bang operations.  The Fast 100Mbps Ethernet PICtail Plus hardware is wired for PMP hardware support, which requires this pinout.  However, if you are designing a custom board, you can simplify these read/write operations dramatically if you wire things more logically by putting all 16 I/O pins, in order, on PORTB or PORTD.  Such a change would enhance performance.
			#define ENC100_INIT_PSP_BIT_BANG()	do{((volatile BYTE*)&AD1PCFGH)[1] = 0xFF;}while(0)	// Disable AN24-AN31 analog inputs on RE0-RE7 pins (ENCX24J600 AD0-AD7)
			#define ENC100_SET_AD_TRIS_IN		do{((volatile BYTE*)&TRISE)[0] = 0xFF; TRISG |= 0x0003; TRISF |= 0x0003; TRISD |= 0x30C0;}while(0)
			#define ENC100_SET_AD_TRIS_OUT		do{((volatile BYTE*)&TRISE)[0] = 0x00; TRISG &= 0xFFFC; TRISF &= 0xFFFC; TRISD &= 0xCF3F;}while(0)
			#define ENC100_GET_AD_IO()			(((volatile BYTE*)&PORTE)[0] | (PORTGbits.RG0<<8) | (PORTGbits.RG1<<9) | (PORTFbits.RF1<<10) | (PORTFbits.RF0<<11) | (PORTDbits.RD12<<12) | (PORTDbits.RD13<<13) | (PORTDbits.RD6<<14) | (PORTDbits.RD7<<15)
			#define ENC100_SET_AD_IO(data)		do{WORD _wDataSetMacro = (data); ((volatile BYTE*)&LATE)[0] = ((BYTE*)&_wDataSetMacro)[0]; LATGbits.LATG0 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x1) == 0x1; LATGbits.LATG1 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x2) == 0x2; LATFbits.LATF1 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x4) == 0x4; LATFbits.LATF0 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x8) == 0x8; LATDbits.LATD12 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x10) == 0x10; LATDbits.LATD13 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x20) == 0x20; LATDbits.LATD6 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x40) == 0x40; LATDbits.LATD7 = ((((BYTE*)&_wDataSetMacro)[1]) & 0x80) == 0x80;}while(0)
		#endif
	#endif

	// ENC624J600 SPI SFR register selection (controls which SPI peripheral to 
	// use on PICs with multiple SPI peripherals).  If a parallel interface is 
	// used (ENC100_INTERFACE_MODE is >= 1), then the SPI is not used and this 
	// section can be ignored or deleted.
	#if defined(__C30__)	// PIC24F, PIC24H, dsPIC30, dsPIC33
		#define ENC100_ISR_ENABLE		(IEC1bits.INT2IE)
		#define ENC100_ISR_FLAG			(IFS1bits.INT2IF)	
		#define ENC100_ISR_POLARITY		(INTCON2bits.INT2EP)
		#define ENC100_ISR_PRIORITY		(IPC7bits.INT2IP)
		#define ENC100_SPI_ENABLE		(ENC100_SPISTATbits.SPIEN)
		#define ENC100_SPI_IF			(IFS0bits.SPI1IF)
		#define ENC100_SSPBUF			(SPI1BUF)
		#define ENC100_SPISTAT			(SPI1STAT)
		#define ENC100_SPISTATbits		(SPI1STATbits)
		#define ENC100_SPICON1			(SPI1CON1)
		#define ENC100_SPICON1bits		(SPI1CON1bits)
		#define ENC100_SPICON2			(SPI1CON2)
	#else					// PIC32MX
		#define ENC100_ISR_ENABLE		(IEC0bits.INT2IE)
		#define ENC100_ISR_FLAG			(IFS0bits.INT2IF)
		#define ENC100_ISR_POLARITY		(INTCONbits.INT2EP)	
		#define ENC100_ISR_PRIORITY		(IPC2bits.INT2IP)	
		#define ENC100_SPI_ENABLE		(ENC100_SPICON1bits.ON)
		#define ENC100_SPI_IF			(IFS0bits.SPI1RXIF)
		#define ENC100_SSPBUF			(SPI1BUF)
		#define ENC100_SPICON1			(SPI1CON)
		#define ENC100_SPISTATbits		(SPI1STATbits)
		#define ENC100_SPICON1bits		(SPI1CONbits)
		#define ENC100_SPIBRG			(SPI1BRG)
	#endif


	// 25LC256 I/O pins
	#define EEPROM_CS_TRIS		(TRISDbits.TRISD12)
	#define EEPROM_CS_IO		(PORTDbits.RD12)
	#define EEPROM_SCK_TRIS		(TRISGbits.TRISG6)
	#define EEPROM_SDI_TRIS		(TRISGbits.TRISG7)
	#define EEPROM_SDO_TRIS		(TRISGbits.TRISG8)
	#if defined(__C30__)	// PIC24F, PIC24H, dsPIC30, dsPIC33
		#define EEPROM_SPI_IF		(IFS2bits.SPI2IF)
		#define EEPROM_SSPBUF		(SPI2BUF)
		#define EEPROM_SPICON1		(SPI2CON1)
		#define EEPROM_SPICON1bits	(SPI2CON1bits)
		#define EEPROM_SPICON2		(SPI2CON2)
		#define EEPROM_SPISTAT		(SPI2STAT)
		#define EEPROM_SPISTATbits	(SPI2STATbits)
	#else					// PIC32
		#define EEPROM_SPI_IF		(IFS1bits.SPI2RXIF)
		#define EEPROM_SSPBUF		(SPI2BUF)
		#define EEPROM_SPICON1		(SPI2CON)
		#define EEPROM_SPICON1bits	(SPI2CONbits)
		#define EEPROM_SPIBRG		(SPI2BRG)
		#define EEPROM_SPISTAT		(SPI2STAT)
		#define EEPROM_SPISTATbits	(SPI2STATbits)
	#endif


	// LCD Module I/O pins.  NOTE: On the Explorer 16, the LCD is wired to the 
	// same PMP lines required to communicate with an ENCX24J600 in parallel 
	// mode.  Since the LCD does not have a chip select wire, if you are using 
	// the ENC424J600/624J600 in parallel mode, the LCD cannot be used.
	#if !defined(ENC100_INTERFACE_MODE) || (ENC100_INTERFACE_MODE == 0)	// SPI only
		#define LCD_DATA_TRIS		(*((volatile BYTE*)&TRISE))
		#define LCD_DATA_IO			(*((volatile BYTE*)&LATE))
		#define LCD_RD_WR_TRIS		(TRISDbits.TRISD5)
		#define LCD_RD_WR_IO		(LATDbits.LATD5)
		#define LCD_RS_TRIS			(TRISBbits.TRISB15)
		#define LCD_RS_IO			(LATBbits.LATB15)
		#define LCD_E_TRIS			(TRISDbits.TRISD4)
		#define LCD_E_IO			(LATDbits.LATD4)
	#endif


//	// Serial Flash/SRAM/UART PICtail Plus attached to SPI2 (middle pin group)
//  // This daughter card is not in production, but if you custom attach an SPI 
//  // RAM or SPI Flash chip to your board, then use these definitions as a 
//  // starting point.
//	#define SPIRAM_CS_TRIS			(TRISGbits.TRISG9)
//	#define SPIRAM_CS_IO			(LATGbits.LATG9)
//	#define SPIRAM_SCK_TRIS			(TRISGbits.TRISG6)
//	#define SPIRAM_SDI_TRIS			(TRISGbits.TRISG7)
//	#define SPIRAM_SDO_TRIS			(TRISGbits.TRISG8)
//	#if defined(__C30__)	// PIC24F, PIC24H, dsPIC30, dsPIC33
//		#define SPIRAM_SPI_IF			(IFS2bits.SPI2IF)
//		#define SPIRAM_SSPBUF			(SPI2BUF)
//		#define SPIRAM_SPICON1			(SPI2CON1)
//		#define SPIRAM_SPICON1bits		(SPI2CON1bits)
//		#define SPIRAM_SPICON2			(SPI2CON2)
//		#define SPIRAM_SPISTAT			(SPI2STAT)
//		#define SPIRAM_SPISTATbits		(SPI2STATbits)
//	#else					// PIC32
//		#define SPIRAM_SPI_IF			(IFS1bits.SPI2RXIF)
//		#define SPIRAM_SSPBUF			(SPI2BUF)
//		#define SPIRAM_SPICON1			(SPI2CON)
//		#define SPIRAM_SPICON1bits		(SPI2CONbits)
//		#define SPIRAM_SPIBRG			(SPI2BRG)
//	#endif
//
//	// NOTE: You must also set the SPI_FLASH_SST/SPI_FLASH_SPANSION, 
//	//       SPI_FLASH_SECTOR_SIZE, and SPI_FLASH_PAGE_SIZE macros in 
//	//       SPIFlash.h to match your particular Flash memory chip!!!
//	#define SPIFLASH_CS_TRIS		(TRISBbits.TRISB8)
//	#define SPIFLASH_CS_IO			(LATBbits.LATB8)
//	#define SPIFLASH_SCK_TRIS		(TRISGbits.TRISG6)
//	#define SPIFLASH_SDI_TRIS		(TRISGbits.TRISG7)
//	#define SPIFLASH_SDI_IO			(PORTGbits.RG7)
//	#define SPIFLASH_SDO_TRIS		(TRISGbits.TRISG8)
//	#if defined(__C30__)	// PIC24F, PIC24H, dsPIC30, dsPIC33
//		#define SPIFLASH_SPI_IF			(IFS2bits.SPI2IF)
//		#define SPIFLASH_SSPBUF			(SPI2BUF)
//		#define SPIFLASH_SPICON1		(SPI2CON1)
//		#define SPIFLASH_SPICON1bits	(SPI2CON1bits)
//		#define SPIFLASH_SPICON2		(SPI2CON2)
//		#define SPIFLASH_SPISTAT		(SPI2STAT)
//		#define SPIFLASH_SPISTATbits	(SPI2STATbits)
//	#else					// PIC32
//		#define SPIFLASH_SPI_IF			(IFS1bits.SPI2RXIF)
//		#define SPIFLASH_SSPBUF			(SPI2BUF)
//		#define SPIFLASH_SPICON1		(SPI2CON)
//		#define SPIFLASH_SPICON1bits	(SPI2CONbits)
//		#define SPIFLASH_SPISTATbits	(SPI2STATbits)
//		#define SPIFLASH_SPIBRG			(SPI2BRG)
//	#endif

    //----------------------------
    // ZeroG ZG2100M WiFi I/O pins
    //----------------------------
	// If you have a ZeroG ZG2100M WiFi PICtail, you must uncomment one of 
	// these two lines to use it.  SPI1 is the top-most slot in the Explorer 16 
	// (closer to the LCD and prototyping area) while SPI2 corresponds to 
	// insertion of the PICtail into the middle of the side edge connector slot.
	//#define ZG2100_IN_SPI1
	//#define ZG2100_IN_SPI2
	
    #if defined( ZG2100_IN_SPI1 ) && !defined(__32MX460F512L__)
        // ZG2100 in SPI1 slot
        #define ZG_CS_TRIS			(TRISBbits.TRISB2)
    	#define ZG_CS_IO			(LATBbits.LATB2)
    	#define ZG_SDI_TRIS			(TRISFbits.TRISF7)
    	#define ZG_SCK_TRIS			(TRISFbits.TRISF6)
    	#define ZG_SDO_TRIS			(TRISFbits.TRISF8)
      	#define ZG_RST_TRIS			(TRISFbits.TRISF0)	
    	#define ZG_RST_IO			(LATFbits.LATF0)  
        #if defined(__dsPIC33FJ256GP710__) || defined(__PIC24HJ256GP610__)
			#define ZG_EINT_TRIS	(TRISAbits.TRISA12)
			#define ZG_EINT_IO		(PORTAbits.RA12)
        #else
			#define ZG_EINT_TRIS	(TRISEbits.TRISE8)  // INT1
			#define ZG_EINT_IO		(PORTEbits.RE8)     
        #endif
    	#define XCEN33_TRIS		    (TRISFbits.TRISF1)  
    	#define	XCEN33_IO			(PORTFbits.RF1)
    	#if defined( __C30__ )
            #define ZG_EINT_EDGE		(INTCON2bits.INT1EP)
        	#define ZG_EINT_IE			(IEC1bits.INT1IE)
        	#define ZG_EINT_IF			(IFS1bits.INT1IF)
    	#elif defined( __PIC32MX__ )
            #define ZG_EINT_EDGE		(INTCONbits.INT1EP)
        	#define ZG_EINT_IE			(IEC0bits.INT1IE)
        	#define ZG_EINT_IF			(IFS0bits.INT1IF)
        	#define ZG_EINT_IE_CLEAR    IEC0CLR
        	#define ZG_EINT_IF_CLEAR    IFS0CLR
        	#define ZG_EINT_IE_SET      IEC0SET
        	#define ZG_EINT_IF_SET      IFS0SET
        	#define ZG_EINT_BIT         0x00000080
        	#define ZG_EINT_IPCSET      IPC1SET
        	#define ZG_EINT_IPCCLR      IPC1CLR
        	#define ZG_EINT_IPC_MASK    0xFF000000
        	#define ZG_EINT_IPC_VALUE   0x0C000000
        #else
            #error Determine ZG2100 external interrupt
        #endif

    	#define ZG_SSPBUF			(SPI1BUF)
    	#define ZG_SPISTAT			(SPI1STAT)
    	#define ZG_SPISTATbits		(SPI1STATbits)
    	#if defined( __C30__ )
        	#define ZG_SPICON1			(SPI1CON1)
        	#define ZG_SPICON1bits		(SPI1CON1bits)
        	#define ZG_SPICON2			(SPI1CON2)
        	#define ZG_SPI_IE			(IEC0bits.SPI1IE)
    //    	#define ZG_SPI_IP			(IPC2bits.SPI1IP)
        	#define ZG_SPI_IF			(IFS0bits.SPI1IF)
        #elif defined( __PIC32MX__ )
        	#define ZG_SPICON1			(SPI1CON)
        	#define ZG_SPICON1bits		(SPI1CONbits)
            #define ZG_SPI_IE_CLEAR     IEC0CLR
            #define ZG_SPI_IF_CLEAR     IFS0CLR
            #define ZG_SPI_INT_BITS     0x03800000
    		#define ZG_SPI_BRG		    (SPI1BRG)
            #define ZG_MAX_SPI_FREQ     (10000000ul)	// Hz
        #else
            #error Determine ZG2100 SPI information
        #endif
        
    #elif defined( ZG2100_IN_SPI2 ) && !defined(__32MX460F512L__)
        // ZG2100 in SPI2 slot
        #define ZG_CS_TRIS			(TRISGbits.TRISG9)
    	#define ZG_CS_IO			(LATGbits.LATG9)
    	#define ZG_SDI_TRIS			(TRISGbits.TRISG7)
    	#define ZG_SCK_TRIS			(TRISGbits.TRISG6)
    	#define ZG_SDO_TRIS			(TRISGbits.TRISG8)
      	#define ZG_RST_TRIS			(TRISGbits.TRISG0)	
    	#define ZG_RST_IO			(LATGbits.LATG0)  
    	#define ZG_EINT_TRIS		(TRISAbits.TRISA14) // INT3 
    	#define ZG_EINT_IO			(PORTAbits.RA14)     
    	#define XCEN33_TRIS		    (TRISGbits.TRISG1)  
    	#define	XCEN33_IO			(PORTGbits.RG1)
    	#if defined( __C30__ )
            #define ZG_EINT_EDGE		(INTCON2bits.INT3EP)
        	#define ZG_EINT_IE			(IEC3bits.INT3IE)
        	#define ZG_EINT_IF			(IFS3bits.INT3IF)
    	#elif defined( __PIC32MX__ )
            #define ZG_EINT_EDGE		(INTCONbits.INT3EP)
        	#define ZG_EINT_IE			(IEC0bits.INT3IE)
        	#define ZG_EINT_IF			(IFS0bits.INT3IF)
        	#define ZG_EINT_IE_CLEAR    IEC0CLR
        	#define ZG_EINT_IF_CLEAR    IFS0CLR
        	#define ZG_EINT_IE_SET      IEC0SET
        	#define ZG_EINT_IF_SET      IFS0SET
        	#define ZG_EINT_BIT         0x00008000
        	#define ZG_EINT_IPCSET      IPC3SET
        	#define ZG_EINT_IPCCLR      IPC3CLR
        	#define ZG_EINT_IPC_MASK    0xFF000000
        	#define ZG_EINT_IPC_VALUE   0x0C000000
        #else
            #error Determine ZG2100 external interrupt
        #endif
        
    	#define ZG_SSPBUF			(SPI2BUF)
    	#define ZG_SPISTAT			(SPI2STAT)
    	#define ZG_SPISTATbits		(SPI2STATbits)
    	#if defined( __C30__ )
        	#define ZG_SPICON1			(SPI2CON1)
        	#define ZG_SPICON1bits		(SPI2CON1bits)
        	#define ZG_SPICON2			(SPI2CON2)
        	#define ZG_SPI_IE			(IEC2bits.SPI2IE)
    //    	#define ZG_SPI_IP			(IPC8bits.SPI2IP)
        	#define ZG_SPI_IF			(IFS2bits.SPI2IF)
        #elif defined( __PIC32MX__ )
        	#define ZG_SPICON1			(SPI2CON)
        	#define ZG_SPICON1bits		(SPI2CONbits)
            #define ZG_SPI_IE_CLEAR     IEC1CLR
            #define ZG_SPI_IF_CLEAR     IFS1CLR
            #define ZG_SPI_INT_BITS     0x000000e0
    		#define ZG_SPI_BRG		    (SPI2BRG)
            #define ZG_MAX_SPI_FREQ     (10000000ul)	// Hz
        #else
            #error Determine ZG2100 SPI information
        #endif

	#elif defined( ZG2100_IN_SPI1 ) && defined(__32MX460F512L__)
        // ZG2100 in SPI1 slot
        #define ZG_CS_TRIS			(TRISDbits.TRISD9)
    	#define ZG_CS_IO			(LATDbits.LATD9)
    	#define ZG_SDI_TRIS			(TRISCbits.TRISC4)
    	#define ZG_SCK_TRIS			(TRISDbits.TRISD10)
    	#define ZG_SDO_TRIS			(TRISDbits.TRISD0)
      	#define ZG_RST_TRIS			(TRISFbits.TRISF0)	
    	#define ZG_RST_IO			(LATFbits.LATF0)  
    	#define ZG_EINT_TRIS		(TRISEbits.TRISE8)  // INT1  
    	#define ZG_EINT_IO			(PORTEbits.RE8)     
    	#define XCEN33_TRIS		    (TRISFbits.TRISF1)  
    	#define	XCEN33_IO			(PORTFbits.RF1)
    	#if defined( __C30__ )
            #define ZG_EINT_EDGE		(INTCON2bits.INT1EP)
        	#define ZG_EINT_IE			(IEC1bits.INT1IE)
        	#define ZG_EINT_IF			(IFS1bits.INT1IF)
    	#elif defined( __PIC32MX__ )
            #define ZG_EINT_EDGE		(INTCONbits.INT1EP)
        	#define ZG_EINT_IE			(IEC0bits.INT1IE)
        	#define ZG_EINT_IF			(IFS0bits.INT1IF)
        	#define ZG_EINT_IE_CLEAR    IEC0CLR
        	#define ZG_EINT_IF_CLEAR    IFS0CLR
        	#define ZG_EINT_IE_SET      IEC0SET
        	#define ZG_EINT_IF_SET      IFS0SET
        	#define ZG_EINT_BIT         0x00000080
        	#define ZG_EINT_IPCSET      IPC1SET
        	#define ZG_EINT_IPCCLR      IPC1CLR
        	#define ZG_EINT_IPC_MASK    0xFF000000
        	#define ZG_EINT_IPC_VALUE   0x0C000000
        #else
            #error Determine ZG2100 external interrupt
        #endif

    	#define ZG_SSPBUF			(SPI1BUF)
    	#define ZG_SPISTAT			(SPI1STAT)
    	#define ZG_SPISTATbits		(SPI1STATbits)
    	#if defined( __C30__ )
        	#define ZG_SPICON1			(SPI1CON1)
        	#define ZG_SPICON1bits		(SPI1CON1bits)
        	#define ZG_SPICON2			(SPI1CON2)
        	#define ZG_SPI_IE			(IEC0bits.SPI1IE)
    //    	#define ZG_SPI_IP			(IPC2bits.SPI1IP)
        	#define ZG_SPI_IF			(IFS0bits.SPI1IF)
        #elif defined( __PIC32MX__ )
        	#define ZG_SPICON1			(SPI1CON)
        	#define ZG_SPICON1bits		(SPI1CONbits)
            #define ZG_SPI_IE_CLEAR     IEC0CLR
            #define ZG_SPI_IF_CLEAR     IFS0CLR
            #define ZG_SPI_INT_BITS     0x03800000
    		#define ZG_SPI_BRG		    (SPI1BRG)
            #define ZG_MAX_SPI_FREQ     (10000000ul)	// Hz
        #else
            #error Determine ZG2100 SPI information
        #endif
    #elif defined( ZG2100_IN_SPI2 ) && defined(__32MX460F512L__)
    	#error "/RST and /CE are on RG2 and RG3 which are multiplexed with USB D+ and D-."
    #endif



#elif defined(DSP804)
// Define your own board hardware profile here

//	#define LED6_TRIS			(TRISAbits.TRISA8)	// Ref D9
//	#define LED6_IO				(LATAbits.LATA8)

	// ENC28J60 I/O pins
	#define ENC_CS_TRIS			(TRISAbits.TRISA9)	// Comment this line out if you are using the ENC424J600/624J600, ZeroG ZG2100, or other network controller.
	#define ENC_CS_IO			(PORTAbits.RA9)
	#define ENC_RST_TRIS		(TRISAbits.TRISA4)	// Not connected by default.  It is okay to leave this pin completely unconnected, in which case this macro should simply be left undefined.
	#define ENC_RST_IO			(PORTAbits.RA4)
	// SPI SCK, SDI, SDO pins are automatically controlled by the 
	// PIC24/dsPIC/PIC32 SPI module 
//	#if defined(__C30__)	// PIC24F, PIC24H, dsPIC30, dsPIC33
		#define ENC_SPI_IF			(IFS0bits.SPI1IF)
		#define ENC_SSPBUF			(SPI1BUF)
		#define ENC_SPISTAT			(SPI1STAT)
		#define ENC_SPISTATbits		(SPI1STATbits)
		#define ENC_SPICON1			(SPI1CON1)
		#define ENC_SPICON1bits		(SPI1CON1bits)
		#define ENC_SPICON2			(SPI1CON2)
//	#endif

#else
	#error "Hardware profile not defined.  See available profiles in HardwareProfile.h and modify or create one."
#endif

	 // PIC24F, PIC24H, dsPIC30, dsPIC33, PIC32
	// Some A/D converter registers on dsPIC30s are named slightly differently 
	// on other procesors, so we need to rename them.
	#if defined(__dsPIC30F__)
		#define ADC1BUF0			ADCBUF0
		#define AD1CHS				ADCHS
		#define	AD1CON1				ADCON1
		#define AD1CON2				ADCON2
		#define AD1CON3				ADCON3
		#define AD1PCFGbits			ADPCFGbits
		#define AD1CSSL				ADCSSL
		#define AD1IF				ADIF
		#define AD1IE				ADIE
		#define _ADC1Interrupt		_ADCInterrupt
	#endif

	// Select which UART the STACK_USE_UART and STACK_USE_UART2TCP_BRIDGE 
	// options will use.  You can change these to U1BRG, U1MODE, etc. if you 
	// want to use the UART1 module instead of UART2.
	#define UBRG					U2BRG
	#define UMODE					U2MODE
	#define USTA					U2STA
	#define BusyUART()				BusyUART2()
	#define CloseUART()				CloseUART2()
	#define ConfigIntUART(a)		ConfigIntUART2(a)
	#define DataRdyUART()			DataRdyUART2()
	#define OpenUART(a,b,c)			OpenUART2(a,b,c)
	#define ReadUART()				ReadUART2()
	#define WriteUART(a)			WriteUART2(a)
	#define getsUART(a,b,c)			getsUART2(a,b,c)
	#if defined(__C32__)
		#define putsUART(a)			putsUART2(a)
	#else
		#define putsUART(a)			putsUART2((unsigned int*)a)
	#endif
	#define getcUART()				getcUART2()
	#define putcUART(a)				do{while(BusyUART()); WriteUART(a); while(BusyUART()); }while(0)
	#define putrsUART(a)			putrsUART2(a)
#endif


