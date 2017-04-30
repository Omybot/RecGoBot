#define THIS_IS_STACK_APPLICATION
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/UDPPerformanceTest.h"
#include "Main804.h"
#include "OurFiles/UserUdp.h"
#include "OurFiles/asser.h"
#include "OurFiles/init.h"
#include <stdio.h>
#include <stdlib.h>
#include <uart.h>
#include <math.h>
#include <p33FJ128MC804.h>
#include "OurFiles/Pilotage.h"
#include "OurFiles/CDS5516.h"
#include "OurFiles/FonctionsUc.h"

#define UART_BUFFER_SIZE	100

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
extern unsigned int ADC_Results[8],cpu_status;
extern double cons_pos[N];
extern double real_pos[N];
extern unsigned char scan;
unsigned char flag_envoi_position;
unsigned int prd_envoi_position = 100;
unsigned char jackAvant = 0;
unsigned char motor_flag=0,datalogger_blocker=0;
double position_lock;
unsigned int datalogger_counter=0,flag=0,courrier=0,PID_ressource_used;
unsigned char flag_envoi=0,flag_blocage=0,flag_calage=0;

//LIDAR
unsigned char Demande_lidar = 0;
unsigned char timeout_lidar=0;
unsigned char nbr_char_to_send=0,Buffer_passerelle_udpuart[250];
unsigned char ptr_write_buffer_uart_rec=0,save_write;
unsigned char ptr_read_buffer_uart_rec=0,save_read;
unsigned char flag_envoi_uart,buffer_envoi_uart[UART_BUFFER_SIZE],ptr_write_buffer_uart;
unsigned char ptr_read_buffer_uart=0,ptr_write_buffer_uart=0;
unsigned char buffer_envoi_uart[UART_BUFFER_SIZE];


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
	unsigned char i;
	unsigned char etatCouleur = 2;
	static DWORD dwLastIP = 0;
	
	Trame trame;		

	Trame Jack;
	static BYTE Presence[2];
	Jack.nbChar = 2;
	Presence[0] = 0xC2;
	Presence[1] = CMD_DEPART_JACK;
	Jack.message = Presence;

	Trame Couleur_Equipe;
	static BYTE Couleur[3];
	Couleur_Equipe.nbChar = 3;
	Couleur[0] = 0xC2;
	Couleur[1] = CMD_REPONSE_COULEUR_EQUIPE;
	Couleur[2] = PORTBbits.RB4;
	Couleur_Equipe.message = Couleur;

	Trame envoiFin;
	static BYTE mess[2];
	mess[0] = 0xC2;
	mess[1] = CMD_FINDEPLACEMENT;
	envoiFin.message = mess;
	envoiFin.nbChar = 2;
		
	Trame envoiBlocage;
	static BYTE messblocage[2];
	messblocage[0] = 0xC2;
	messblocage[1] = 0x13;
	envoiBlocage.message = messblocage;
	envoiBlocage.nbChar = 2;
	
	Trame envoiCalage;
	static BYTE messcalage[2];
	messcalage[0] = 0xC2;
	messcalage[1] = CMD_FINRECALLAGE;
	envoiCalage.message = messcalage;
	envoiCalage.nbChar = 2;

	Trame envoiUART;
	static BYTE messUART[250];
	messUART[0] = 0xC4;
	messUART[1] = CMD_REPONSE_LIDAR;//CMD???;
	messUART[2] = 0xFE;
	envoiUART.message = messUART;
	envoiUART.nbChar = 53;
	
	
	Trame envoiTest;
	static BYTE messTest[19];
	messTest[0] = 0xC4;
	messTest[1] = 0xC4;
	messTest[2] = 7;
	messTest[3] = 'M';
	messTest[4] = 'S';
	messTest[5] = '0';
	messTest[6] = '0';
	messTest[7] = '0';
	messTest[8] = '0';
	messTest[9] = '0';
	messTest[10] = '7';
	messTest[11] = '2';
	messTest[12] = '5';
	messTest[13] = '0';
	messTest[14] = '0';
	messTest[15] = '0';
	messTest[16] = '0';
	messTest[17] = '1';
	messTest[18] = '\n';

	//MS0000072500001
	
	envoiTest.message = messTest;
	envoiTest.nbChar = 19;
	// V V [LF] 0 0 P [LF] 	

	InitClk(); 		// Initialisation de l'horloge
	InitPorts(); 	// Initialisation des ports E/S
	Init_Timer4();	// Initialisation Timer4
	InitQEI(); 		// Initialisation des entrées en quadrature
	InitPWM();		// Configuration du module PWM 
        
	InitProp();
	
	// Initialize stack-related hardware components that may be 
	// required by the UART configuration routines
    TickInit();
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		MPFSInit();
	#endif
	
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
		//Fin Gestion LIDAR	
		if(Demande_lidar)	
		{
			Demande_lidar=0;
			EnvoiUART(envoiTest);
		}
		
		save_write = ptr_write_buffer_uart_rec;
		save_read = ptr_read_buffer_uart_rec;
		if((save_write != save_read))
		{
			if(save_write < save_read)
				nbr_char_to_send = 241 - save_read + save_write;
			else
				nbr_char_to_send = save_write - save_read;
		}
		else
		{
			nbr_char_to_send = 0;
		}	

		if(((nbr_char_to_send > 200) || (nbr_char_to_send !=0 && timeout_lidar > 50)))
		{	
			for(i=0;i<nbr_char_to_send;i++)
			{
				messUART[i+3]=Buffer_passerelle_udpuart[save_read++];
				if(save_read>240)
					save_read=0;
			}
			timeout_lidar=0;
			envoiUART.nbChar = nbr_char_to_send+3;
			EnvoiUserUdp(envoiUART);
			ptr_read_buffer_uart_rec = save_write;
		}			

		if((ptr_write_buffer_uart != ptr_read_buffer_uart) && U2STAbits.TRMT != 0)
		{
			// Gestion envoi trame
			U2TXREG = buffer_envoi_uart[ptr_read_buffer_uart++];
			if(ptr_read_buffer_uart >= UART_BUFFER_SIZE)
				ptr_read_buffer_uart=0;
		}
		//Fin Gestion LIDAR	
	  	if(PORTAbits.RA8 && jackAvant)
	  	{
		  	EnvoiUserUdp (Jack);
		  	jackAvant = 0;
		}
		if(etatCouleur != PORTBbits.RB4)
		{
			Couleur[2] = PORTBbits.RB4;
  			EnvoiUserUdp (Couleur_Equipe);
  			etatCouleur = PORTBbits.RB4;
  		}
		if(flag_envoi) 
		{	
			scan=0;
			EnvoiUserUdp(envoiFin);
			flag_envoi = 0;
		}
		if(flag_blocage)
		{
			EnvoiUserUdp(envoiBlocage);
			flag_blocage = 0;
		}
		if(flag_calage)
		{
			EnvoiUserUdp(envoiCalage);
			flag_calage = 0;
		}
		if(flag_envoi_position)
		{
			EnvoiUserUdp(PilotePositionXYT());
			flag_envoi_position = 0;
		}
		

		StackTask();
		trame = ReceptionUserUdp();
		if(trame.nbChar != 0)
		{
			trame = AnalyseTrame(trame);
			EnvoiUserUdp(trame);
		}
        StackApplications();

		if(dwLastIP != AppConfig.MyIPAddr.Val)
		{
			dwLastIP = AppConfig.MyIPAddr.Val;
			
			#if defined(STACK_USE_UART)
				putrsUART((ROM char*)"\r\nNew IP Address: ");
			#endif

			DisplayIPValue(AppConfig.MyIPAddr);

			#if defined(STACK_USE_UART)
				putrsUART((ROM char*)"\r\n");
			#endif


			#if defined(STACK_USE_ANNOUNCE)
				AnnounceIP();
			#endif
		}
	}
}

// Writes an IP address to the LCD display and the UART as available
void DisplayIPValue(IP_ADDR IPVal)
{
//	printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
    BYTE IPDigit[4];
	BYTE i;
#ifdef USE_LCD
	BYTE j;
	BYTE LCDPos=16;
#endif

	for(i = 0; i < sizeof(IP_ADDR); i++)
	{
	    uitoa((WORD)IPVal.v[i], IPDigit);

		#if defined(STACK_USE_UART)
			putsUART(IPDigit);
		#endif

		#ifdef USE_LCD
			for(j = 0; j < strlen((char*)IPDigit); j++)
			{
				LCDText[LCDPos++] = IPDigit[j];
			}
			if(i == sizeof(IP_ADDR)-1)
				break;
			LCDText[LCDPos++] = '.';
		#else
			if(i == sizeof(IP_ADDR)-1)
				break;
		#endif

		#if defined(STACK_USE_UART)
			while(BusyUART());
			WriteUART('.');
		#endif
	}

	#ifdef USE_LCD
		if(LCDPos < 32u)
			LCDText[LCDPos] = 0;
		LCDUpdate();
	#endif
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


	// SNMP Community String configuration
	#if defined(STACK_USE_SNMP_SERVER)
	{
		BYTE i;
		static ROM char * ROM cReadCommunities[] = SNMP_READ_COMMUNITIES;
		static ROM char * ROM cWriteCommunities[] = SNMP_WRITE_COMMUNITIES;
		ROM char * strCommunity;
		
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			// Get a pointer to the next community string
			strCommunity = cReadCommunities[i];
			if(i >= sizeof(cReadCommunities)/sizeof(cReadCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_READ_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.readCommunity[0]))
				while(1);
			
			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.readCommunity[i], strCommunity);

			// Get a pointer to the next community string
			strCommunity = cWriteCommunities[i];
			if(i >= sizeof(cWriteCommunities)/sizeof(cWriteCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_WRITE_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.writeCommunity[0]))
				while(1);

			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.writeCommunity[i], strCommunity);
		}
	}
	#endif

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
	#elif defined(SPIFLASH_CS_TRIS)
	{
		BYTE c;
		
		SPIFlashReadArray(0x0000, &c, 1);
		if(c == 0x60u)
			SPIFlashReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
		else
			SaveAppConfig();
	}
	#endif
}

#if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
void SaveAppConfig(void)
{
	// Ensure adequate space has been reserved in non-volatile storage to 
	// store the entire AppConfig structure.  If you get stuck in this while(1) 
	// trap, it means you have a design time misconfiguration in TCPIPConfig.h.
	// You must increase MPFS_RESERVE_BLOCK to allocate more space.
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		if(sizeof(AppConfig) > MPFS_RESERVE_BLOCK)
			while(1);
	#endif

	#if defined(EEPROM_CS_TRIS)
	    XEEBeginWrite(0x0000);
	    XEEWrite(0x60);
	    XEEWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #else
	    SPIFlashBeginWrite(0x0000);
	    SPIFlashWrite(0x60);
	    SPIFlashWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #endif
}
#endif

void __attribute__ ((interrupt, no_auto_psv)) _T4Interrupt(void) 
{
	flag = 0;
	courrier = 1;
		
	cpu_status = (TMR4); //Previous value TMR4
	IFS1bits.T4IF = 0;
}

void __attribute__((interrupt,auto_psv)) _U2RXInterrupt(void)
{
	static unsigned etat_rx=0;
	static unsigned int recu,recu_ptr=0;

	IFS1bits.U2RXIF = 0; 		// clear RX interrupt flag
	
	if(U2STAbits.URXDA == 1)
	{
		Buffer_passerelle_udpuart[ptr_write_buffer_uart_rec++] = U2RXREG;
		if(ptr_write_buffer_uart_rec>240)
			ptr_write_buffer_uart_rec=0;
	}
			
}

void __attribute__ ((interrupt, no_auto_psv)) _QEI1Interrupt(void) 
{/* moteur GAUCHE*/
	QEI1CONbits.QEIM = 0;
	IFS3bits.QEI1IF = 0;
	if((QEI1CONbits.UPDN==1) && POS1CNT < 0x8000)	tours_codeur++; // rollover
	if((QEI1CONbits.UPDN==0) && POS1CNT > 0x8000)	tours_codeur--; // underflow
	QEI1CONbits.QEIM = QEI2CONbits.QEIM;
	position_codeur = (long)POS1CNT + (long)(tours_codeur*0x10000);
}
