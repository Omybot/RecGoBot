#include <p33FJ128MC804.h>
#include "FonctionsUc.h"


/*void Tempo1mS(unsigned int nbr)
{
	unsigned int i,j;

	for(i=0;i<nbr;i++)
		for(j=0;j<8000;j++);
}*/

/*void Tempo1uS(unsigned int nbr) // Défaut constant de +0.5us
{
	unsigned int i;

	for(i=0;i<nbr;i++)
	{
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
		Nop();	Nop();	Nop();	Nop();
	}
}*/



//void InitUART2(void) 
//{
//	// This is an EXAMPLE, so brutal typing goes into explaining all bit sets
//
//	// The HPC16 board has a DB9 connector wired to UART2, so we will
//	// be configuring this port only
//
//	// configure U2MODE
//	U2MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
//	//U2MODEbits.notimplemented;	// Bit14
//	U2MODEbits.USIDL = 0;	// Bit13 Continue in Idle
//	U2MODEbits.IREN = 0;	// Bit12 No IR translation
//	U2MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
//	//U2MODEbits.notimplemented;	// Bit10
//	U2MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
//	U2MODEbits.WAKE = 0;	// Bit7 No Wake up (since we don't sleep here)
//	U2MODEbits.LPBACK = 0;	// Bit6 No Loop Back
//	U2MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
//	U2MODEbits.URXINV = 0;	// Bit4 IdleState = 1  (for dsPIC)
//	U2MODEbits.BRGH = 1;	// Bit3 4 clocks per bit period
//	U2MODEbits.PDSEL = 0;	// Bits1,2 8bit, No Parity
//	U2MODEbits.STSEL = 0;	// Bit0 One Stop Bit
//	
//	// Load a value into Baud Rate Generator.  Example is for 9600.
//	// See section 19.3.1 of datasheet.
//	//  U2BRG = (Fcy/(16*BaudRate))-1
//	//  U2BRG = (40M/(16*9600))-1
//	//  U2BRG = 260;  	// 40Mhz osc, 9600 Baud (FCR)
//	U2BRG = 522;	// (40MHz/(19100))-1 Ligne rajoutee par Mouly *****
//	  //U2BRG = BRGVAL;  	// 40Mhz osc, 9600 Baud (mode HS,XT PLL)
//
//	// Load all values in for U1STA SFR
//	U2STAbits.UTXISEL1 = 0;	//Bit15 Int when Char is transferred (1/2 config!)
//	U2STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
//	U2STAbits.UTXISEL0 = 0;	//Bit13 Other half of Bit15
//	U2STA &= 0xDFFF;
//	//U2STAbits.notimplemented = 0;	//Bit12
//	U2STAbits.UTXBRK = 0;	//Bit11 Disabled
//	U2STAbits.UTXEN = 0;	//Bit10 TX pins controlled by periph
//	U2STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
//	U2STAbits.TRMT = 0;		//Bit8 *Read Only bit*
//	U2STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
//	U2STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
//	U2STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
//	U2STAbits.PERR = 0;		//Bit3 *Read Only Bit*
//	U2STAbits.FERR = 0;		//Bit2 *Read Only Bit*
//	U2STAbits.OERR = 0;		//Bit1 *Read Only Bit*
//	U2STAbits.URXDA = 0;	//Bit0 *Read Only Bit*
//
//	IPC7 = 0x4400;	// Mid Range Interrupt Priority level, no urgent reason
//
//	IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
//	IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupts
//	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
//	IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts
//
//	U2MODEbits.UARTEN = 1;	// And turn the peripheral on
//
//	U2STAbits.UTXEN = 1;
//	// I think I have the thing working now.
//}



/*float _abs(float value)
{
	if(value<0) return -value;
	else		return  value;
}*/

/*******************************************************************************
Function: UART2*Char( char ch )

Precondition:
    UART2Init must be called prior to calling this routine.

Overview:
    This routine writes a character to the transmit FIFO, and then waits for the
    transmit FIFO to be empty.

Input: Byte to be sent.

Output: None.

*******************************************************************************/
void UART2PutChar( char ch )
{
    U1TXREG = ch;
    while(U1STAbits.TRMT == 0);
}


