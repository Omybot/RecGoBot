#define THIS_IS_STACK_APPLICATION
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/UDPPerformanceTest.h"
#include "Main804.h"
#include "OurFiles/UserUdp.h"
#include "OurFiles/init.h"
#include <stdio.h>
#include <stdlib.h>
#include <uart.h>
#include <math.h>
#include <p33FJ128MC804.h>
#include "OurFiles/Pilotage.h"
#include "OurFiles/Multiplex.h"
// Ce qui est fait
// Configuration Ethernet 
// Initialisation codeur
// Initialisation du module ADC (+DMA)
// Initialisation des modules PWM
// Couche de communication entre la LED RGB + Buzzer et GoBot (manque la 0gestion fréquence)

// Ce qui doit être fait par Nico
// Suppression anciens codes (Asser et Servos en particulier)
// Multiplexage analogique des boutons poussoirs/interrupteurs

// Ce qui doit être fait
// Gestion des LEDs via les deux TLC puis couche de communication vers GoBot
// Gestion du LCD via I²C puis couche de communication vers GoBot
// Couche de communication entre les boutons/interrupteurs et GoBot
// ==> Lancer un match automatiquement avec GoBot


// Bits configuration
_FOSCSEL(FNOSC_FRC)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON)
_FPOR(FPWRT_PWR1)
_FWDT(FWDTEN_OFF)
_FICD(ICS_PGD3 & JTAGEN_OFF)

APP_CONFIG AppConfig;

static void InitAppConfig(void);


unsigned char leds_change;
unsigned int rouge,vert;
long position_codeur;
int tours_codeur;
int inputChanged, inputChannelChanged;
extern unsigned int ADC_Results[9];

void _ISR __attribute__((__no_auto_psv__)) _AddressError(void)
{
    Nop();
	Nop();
}
void _ISR __attribute__((__no_auto_psv__)) _StackError(void)
{
    Nop();
	Nop();
}

int main(void)
{
	int i;
	Trame trame;	

	Trame envoiBouton;
	static BYTE messBouton[4];
	messBouton[0] = UDP_ID;
	messBouton[1] = TRAME_CAPTEUR_ONOFF_RESP;
	envoiBouton.message = messBouton;
	envoiBouton.nbChar = 4;
	
	inputChanged = -1;
	inputChannelChanged = -1;

	InitClk(); 		// Initialisation de l'horloge
	InitPorts(); 	// Initialisation des ports E/S
	
	Init_Timer4();	// Initialisation Timer4
	InitQEI(); 		// Initialisation des entrées en quadrature
	InitPWM();		// Configuration du module PWM 
        
	// Initialize stack-related hardware components that may be 
	// required by the UART configuration routines
    TickInit();
	
	// Initialize Stack and application related NV variables into AppConfig.
	InitAppConfig();

	UDPInit();
    StackInit();	
 	UDPPerformanceTask();
	InitUserUdp();
	Init_Interrupt_Priority();							
	InitADC();
	InitDMA();
	//MultiplexInit(); // A MARCHE PO

	// Init SPI0 (Leds)
	SPI2CON1bits.MODE16 = 1;	// Transmission 16 bits
	SPI2CON1bits.CKE = 1;	// SDO change lors d'un front SCK descendant
	SPI2CON1bits.MSTEN = 1;	// Master mode	
	SPI2CON1bits.SPRE = 0b110;	// Secondary prescaler 2:1
	SPI2CON1bits.PPRE = 0b11; 	// Primary prescaler 1:1

	SPI2STATbits.SPIEN = 1;

	
	
	/*while(1){
	SPI2BUF = 0x0FF0;
	DelayMs(1);
	SPI2BUF = 0xFF00;
	DelayMs(1);
	TLC_LE = 0;
	TLC_LE = 1;
	TLC_LE = 0;
	DelayMs(200);
	
	}*/

	DelayMs(500); 

	while(1)
  	{	
		StackTask();

		// Reception UDP
		trame = ReceptionUserUdp();

		if(trame.nbChar != 0)
		{
			// Réponse UDP
			trame = AnalyseTrame(trame);
			EnvoiUserUdp(trame);
		}

		if(inputChanged != -1)
		{
			messBouton[2] = inputChannelChanged * 4 + inputChanged;
			messBouton[3] = MultiplexGetState(inputChannelChanged, inputChanged);
			EnvoiUserUdp(envoiBouton);
			inputChanged = -1;
		}

        StackApplications();
	}
}

//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata

static void InitAppConfig(void)
{
	AppConfig.Flags.bIsDHCPEnabled = TRUE;
	AppConfig.Flags.bInConfigMode = TRUE;
	memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
//	{
//		_prog_addressT MACAddressAddress;
//		MACAddressAddress.next = 0x157F8;
//		_memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
//	}
	AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
	AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
	AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
	AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
	AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
	AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
	AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;

	// Load the default NetBIOS Host Name
	memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
	FormatNetBIOSName(AppConfig.NetBIOSName);

	#if defined(ZG_CS_TRIS)
		// Load the default SSID Name
		if (sizeof(MY_DEFAULT_SSID_NAME) > sizeof(AppConfig.MySSID))
		{
		    ZGSYS_DRIVER_ASSERT(5, (ROM char *)"AppConfig.MySSID[] too small.\n");
		}
		memcpypgm2ram(AppConfig.MySSID, (ROM void*)MY_DEFAULT_SSID_NAME, sizeof(MY_DEFAULT_SSID_NAME));
	#endif

	#if defined(EEPROM_CS_TRIS)
	{
		BYTE c;
		
	    // When a record is saved, first byte is written as 0x60 to indicate
	    // that a valid record was saved.  Note that older stack versions
		// used 0x57.  This change has been made to so old EEPROM contents
		// will get overwritten.  The AppConfig() structure has been changed,
		// resulting in parameter misalignment if still using old EEPROM
		// contents.
		XEEReadArray(0x0000, &c, 1);
	    if(c == 0x60u)
		    XEEReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
	    else
	        SaveAppConfig();
	}
	#endif
}


void __attribute__ ((interrupt, no_auto_psv)) _T4Interrupt(void) 
{
	static unsigned char etat_leds = 0;
	// Interruption sur base de temps 1ms

	// Cherche un input qui a changé d'état

	int channelNo;
	double measures[4];

	measures[0] = (double)ADC_Results[4]*(double)COEFF_TENSION_ADC;
	measures[1] = (double)ADC_Results[5]*(double)COEFF_TENSION_ADC;
	measures[2] = (double)ADC_Results[6]*(double)COEFF_TENSION_ADC;
	measures[3] = (double)ADC_Results[7]*(double)COEFF_TENSION_ADC;

	channelNo = 0;

	while (inputChanged == -1 && channelNo < 4)
	{
		inputChanged = MultiplexAddMeasure(channelNo , measures[channelNo]);
		inputChannelChanged = channelNo;
		channelNo++;
	}

	switch(etat_leds++)
	{
		case 0:	if(leds_change)
					leds_change=0;
				else
					etat_leds=0;
				break;
		case 1:	SPI2BUF = rouge;
				break;
		case 2:	SPI2BUF = vert;
				break;
		case 3:	TLC_LE = 1;
				break;
		case 4:	TLC_LE = 0;
				break;
		case 5:	leds_change=0;
				etat_leds=0;
				break;
		
	}
	IFS1bits.T4IF = 0;
}

void __attribute__ ((interrupt, no_auto_psv)) _QEI1Interrupt(void) 
{
	QEI1CONbits.QEIM = 0;
	IFS3bits.QEI1IF = 0;
	if((QEI1CONbits.UPDN==1) && POS1CNT < 0x8000)	tours_codeur++; // rollover
	if((QEI1CONbits.UPDN==0) && POS1CNT > 0x8000)	tours_codeur--; // underflow
	QEI1CONbits.QEIM = QEI2CONbits.QEIM;
	position_codeur = (long)POS1CNT + (long)(tours_codeur*0x10000);
}
