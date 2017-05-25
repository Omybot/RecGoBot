#include <p33FJ128MC804.h>
#include "init.h"
#include "Pilotage.h"

unsigned int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int ADC_Results[9],DmaBuffer = 0;

void InitClk(void)
{	
	PLLFBD = 38;				// Multiply by 40 for 160MHz VCO output (8MHz XT oscillator)
	CLKDIV = 0x0000;			// FRC: divide by 2, PLLPOST: divide by 2, PLLPRE: divide by 2
	
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(OSCCON | 0x01);
	while(OSCCONbits.COSC != 0b011);
	while(OSCCONbits.LOCK != 1);
}

void Init_Interrupt_Priority(void)
{
	IPC14bits.QEI1IP = 7;			// Quad Encoder Interrupt
	IPC15bits.DMA5IP  = 6;			// ADC Interrupt
	IPC0bits.T1IP    = 4;			// Timer 1 used by Ethernet (Default value = 2)
	IPC6bits.T4IP    = 3;			// Timer 4 Used by Asser
}

void InitPorts() 
{
	TRISAbits.TRISA0=1; // AN0 - Port I/O
	TRISAbits.TRISA1=1; // AN1 - Port I/O
	TRISAbits.TRISA2=1; // OSC - Oscillateur
	TRISAbits.TRISA3=1; // Port I/O - Port I/O
	TRISAbits.TRISA4=0; // RST - Ethernet RST
	TRISAbits.TRISA7=0; // TLC.LE - Driver leds
	TRISAbits.TRISA8=1; // Port I/O - Port I/O
	TRISAbits.TRISA9=0; // CS - Ethernet CS
	TRISAbits.TRISA10=0; // TLC.!OE - Driver leds
	TRISBbits.TRISB0=1; // AN2 - Port I/O
	TRISBbits.TRISB1=1; // AN3 - Port I/O
	TRISBbits.TRISB2=1; // MUX1 - Ensemble Interrupteurs
	TRISBbits.TRISB3=1; // MUX2 - Ensemble démarrage
	TRISBbits.TRISB4=1; // Port I/O - Port I/O
	TRISBbits.TRISB5=1; // PGD - JTAG 
	TRISBbits.TRISB6=1; // PGC - JTAG 
	TRISBbits.TRISB7=1; // INT - Ethernet INT
	TRISBbits.TRISB8=1; // I²C - SCL - Port I²C
	TRISBbits.TRISB9=1; // I²C - SDA - Port I²C
	TRISBbits.TRISB10=0; // Led RGB - Led RGB
	TRISBbits.TRISB11=0; // TLC.SDO2 - Driver leds
	TRISBbits.TRISB12=0; // Led RGB - Led RGB
	TRISBbits.TRISB13=0; // TLC.SCK2 - Driver leds
	TRISBbits.TRISB14=0; // Led RGB - Led RGB
	TRISBbits.TRISB15=1; // Port I/O - Port I/O
	TRISCbits.TRISC0=1; // MUX3 - Boutons 1 à 4
	TRISCbits.TRISC1=1; // MUX4 - Boutons 5 à 8
	TRISCbits.TRISC2=1; // Mesure tension batterie - 
	TRISCbits.TRISC3=0; // SCK1 - Ethernet MSCK
	TRISCbits.TRISC4=0; // SDO1 - Ethernet MOSI
	TRISCbits.TRISC5=1; // SDI1 - Ethernet MISO
	TRISCbits.TRISC6=0; // Buzzer - Buzzer
	TRISCbits.TRISC7=1; // Port I/O - Port I/O
	TRISCbits.TRISC8=1; // QE1B - Codeur
	TRISCbits.TRISC9=1; // QE1A - Codeur
	
	// Module SPI1 ==> Contrôleur Ethernet (ENC28J60)
	RPOR9bits.RP19R   = 0b00111; // SDO1 		<==> RP19 RC4 // Inversion SDO/SCK par rapport à RecBase
	RPOR10bits.RP20R  = 0b01000; // SCK1 		<==> RP20 RC3
	RPINR20bits.SDI1R = 21     ; // SDI1 		<==> RP21 RC5
	
	// Module SPI0 ==> Contrôleur leds (TLC5925)
	RPOR5bits.RP11R   = 0b01010; // SDO2 		<==> RP11 RB11
	RPOR6bits.RP13R  = 0b01011; // SCK2 		<==> RP13 RB13
	
	TLC_OE=0;
	TLC_LE=0;

}

void Init_Timer4(void)
{
	//--Timer4
	T4CONbits.TON 	= 0;	//Stops the timer
	T4CONbits.TSIDL = 0;
	T4CONbits.TGATE = 0;
	T4CONbits.TCS	= 0;
	T4CONbits.TCKPS = 0b01; //Prescaler set to 1:8
	
	TMR4 = 0; 				//Clear timer register
	PR4  = 5000; 			//Load the period value (Pas) 1/(40e6/8/1250) = 1ms

//	IPC6bits.T4IP = 6; 		//Set Timer4 Interrupt Priority Level
	IFS1bits.T4IF = 0; 		//Clear Timer4 Interrupt Flag
	IEC1bits.T4IE = 1; 		//Enable Timer4 interrupt
	
	T4CONbits.TON = 1;		//Starts the timer
}

void InitPWM(void)
{
	P1TCONbits.PTEN = 1; 		// PWM Time base is On
	P1TPER = 4000 - 1; 			// 10kHz PWM (4000 counts @40MIPS)
	PWM1CON1bits.PEN1L = 0;		// PWM1L1 pin is disabled for PWM output
	PWM1CON1bits.PEN1H = 1;		// PWM1H1 pin is disabled for PWM output
	PWM1CON1bits.PEN2L = 0;		// PWM1L2 pin is disabled for PWM output
	PWM1CON1bits.PEN2H = 1;		// PWM1H2 pin is disabled for PWM output
	PWM1CON1bits.PEN3L = 0;		// PWM1L3 pin is disabled for PWM output
	PWM1CON1bits.PEN3H = 1;		// PWM1H3 pin is disabled for PWM output

	P1DC1 = 0; // Bleu  0		==> PWM 0% (à vérifier)
	P1DC2 = 0; // Vert  4000	==> PWM 50%
	P1DC3 = 0; // Rouge 8000	==> PWM 100%

	P2TCONbits.PTEN = 1; 		// PWM Time base is On
	P2TPER = 30000 - 1; 			// 1.333kHz PWM (30000 counts @40MIPS)
	PWM2CON1bits.PEN1L = 0;		// PWM2L1 pin is disabled for PWM output
	PWM2CON1bits.PEN1H = 1;		// PWM2L2 pin is disabled for PWM output

	P2DC1 = 0;
}

void InitQEI(void)
{
	QEI1CONbits.QEIM  = 0b111;
	QEI1CONbits.SWPAB = 1;
	POS1CNT = 0x0000;
	IFS3bits.QEI1IF = 0;
	IEC3bits.QEI1IE = 1;
}

void InitADC(void)
{
	AD1CON1bits.FORM   = 0;		// Data Output Format: Integer
	AD1CON1bits.SSRC   = 7;		// Sample Clock Source: Conversion autostart
	AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
	AD1CON1bits.AD12B  = 1;		// 12-bit ADC operation

	AD1CON2bits.CSCNA = 1;		// Scan Input Selections for CH0+ during Sample A bit
	AD1CON2bits.CHPS  = 0;		// Converts CH0

	AD1CON3bits.ADRC = 0;		// ADC Clock is derived from Systems Clock
	AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
								// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us	
	
	AD1CON1bits.ADDMABM = 0; 	// DMA buffers are built in scatter/gather mode
	AD1CON2bits.SMPI    = (NUM_CHS2SCAN-1);	// 9 ADC Channel are scanned
	AD1CON4bits.DMABL   = 3;	// Each buffer contains 8 words

	//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
	AD1CSSLbits.CSS0=1;		// Enable AN0 for channel scan
	AD1CSSLbits.CSS1=1;		// Enable AN1 for channel scan
	AD1CSSLbits.CSS2=1;		// Enable AN2 for channel scan
	AD1CSSLbits.CSS3=1;		// Enable AN3 for channel scan
	AD1CSSLbits.CSS4=1;		// Enable AN4 for channel scan
	AD1CSSLbits.CSS5=1;		// Enable AN5 for channel scan
	AD1CSSLbits.CSS6=1;		// Enable AN6 for channel scan
	AD1CSSLbits.CSS7=1;		// Enable AN7 for channel scan
	AD1CSSLbits.CSS8=1;		// Enable AN8 for channel scan
	
 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;
	AD1PCFGLbits.PCFG0 = 0;	// AN0 as Analog Input
	AD1PCFGLbits.PCFG1 = 0;	// AN1 as Analog Input
 	AD1PCFGLbits.PCFG2 = 1;	// AN2 as Digital Input 
	AD1PCFGLbits.PCFG3 = 1;	// AN3 as Digital Input 
	AD1PCFGLbits.PCFG4 = 0;	// AN4 as Analog Input  
	AD1PCFGLbits.PCFG5 = 0;	// AN5 as Analog Input  
	AD1PCFGLbits.PCFG6 = 0;	// AN6 as Analog Input 
	AD1PCFGLbits.PCFG7 = 0;	// AN7 as Analog Input 
	AD1PCFGLbits.PCFG8 = 0;	// AN8 as Analog Input 
	
	IFS0bits.AD1IF   = 0;		// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE   = 0;		// Do Not Enable A/D interrupt 
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter
}

void InitDMA(void)
{
	DMA5CONbits.AMODE = 2;			// Configure DMA for Peripheral indirect mode
	DMA5CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	DMA5PAD=(int)&ADC1BUF0;
	DMA5CNT = (SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;					
	DMA5REQ = 13;					// Select ADC1 as DMA Request source

	DMA5STA = __builtin_dmaoffset(BufferA);		
	DMA5STB = __builtin_dmaoffset(BufferB);

	IFS3bits.DMA5IF = 0; //Clear the DMA interrupt flag bit
	IEC3bits.DMA5IE = 1; //Set the DMA interrupt enable bit
	
	DMA5CONbits.CHEN=1;				// Enable DMA
}

void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void)
{
	static int seuil[2]={2500,2500};
	static int hysteresis[2]={50,50};
	unsigned int telemetre[2];
	static unsigned char i,cpt_secu_courant[2]={0,0};
	ADC_Results[0]=0;
	ADC_Results[1]=0;
	ADC_Results[2]=0;
	ADC_Results[3]=0;
	ADC_Results[4]=0;
	ADC_Results[5]=0;
	ADC_Results[6]=0;
	ADC_Results[7]=0;
	ADC_Results[8]=0;
	if(DmaBuffer == 0)
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferA[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferA[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferA[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferA[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferA[4][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferA[5][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[6] += BufferA[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[7] += BufferA[7][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[8] += BufferA[8][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
		ADC_Results[6] /= SAMP_BUFF_SIZE;
		ADC_Results[7] /= SAMP_BUFF_SIZE;
		ADC_Results[8] /= SAMP_BUFF_SIZE;
	}
	else
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferB[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferB[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferB[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferB[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferB[4][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferB[5][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[6] += BufferB[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[7] += BufferB[7][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[8] += BufferB[8][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
		ADC_Results[6] /= SAMP_BUFF_SIZE;
		ADC_Results[7] /= SAMP_BUFF_SIZE;
		ADC_Results[8] /= SAMP_BUFF_SIZE;
	}

	DmaBuffer ^= 1;

	IFS3bits.DMA5IF = 0;		// Clear the DMA0 Interrupt Flag
}
