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

// Ce qui est fait
// Configuration Ethernet 
// Initialisation codeur
// Initialisation du module ADC (+DMA)
// Initialisation des modules PWM
// Couche de communication entre la LED RGB + Buzzer et GoBot (manque la gestion fréquence)

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


long position_codeur;
int tours_codeur;
unsigned int prd_envoi_position = 100;
unsigned char motor_flag=0,datalogger_blocker=0;
double position_lock;
unsigned int datalogger_counter=0,flag=0,courrier=0,PID_ressource_used;

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
	Trame trame;		

	Trame envoiBouton;
	static BYTE messBouton[4];
	messBouton[0] = MY_ID;
	messBouton[1] = CMD_BOUTONS;
	envoiBouton.message = messBouton;
	envoiBouton.nbChar = 4;
	
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
	//InitUART2();	
	InitADC();
	InitDMA();

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
	// Interruption sur base de temps 1ms
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
