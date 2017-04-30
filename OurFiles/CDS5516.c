#include <p33FJ128MC804.h>
#include "CDS5516.h"
#include "FonctionsUc.h"

//Contrôle en angle du cds5516 (0 à 1023 soit 0° à 300°)
void CDS5516Pos(double baud,char id, int position)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0,val1 = 0, checksum = 0;
	int i;

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	U2BRG = 522; 
	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x1E; 	
	val = position & 0b0000000011111111;
	val1 = position >> 8;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));

	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(val1);
		UART2PutChar(checksum);
	}
}

//Paramètre de la vitesse du CDS5516 (0 à 1023)
void CDS5516Vit(double baud,char id, unsigned int vitesse)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0,val1 = 0, checksum = 0;
	int i;

//	InitUART2();	//Initialisation de la liaison série 2 (UART2)
//	U2BRG = ((FCY/baud)/4)-1;
	U2BRG = 522;
	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x20;															
	val = vitesse & 0b0000000011111111;
	val1 = vitesse >> 8;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));

	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(val1);
		UART2PutChar(checksum);
	}
}

//Allumer la led du CDS5516
void CDS5516Led(double baud,char id, char led)
{
	char taille = 0, instruction = 0, adresse = 0, checksum = 0;
	int i;
	
	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	//U2BRG = ((FCY/baud)/4)-1;
	//U2BRG = 522;
	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x19; 															//Allumer ou éteindre une led
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + led));
	
	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(led);
		UART2PutChar(checksum);
	}
}


//Chercher le Baudrate du CDS5516
void CDS5516searchBaudrate(char id, double baudratemin, double baudratemax, char led)
{
	char taille = 0, instruction = 0, adresse = 0, checksum = 0;
	int i;
	double baud;

	for(baud = baudratemin;baud <= baudratemax;baud++)
	{	
		//InitUART2();	//Initialisation de la liaison série 2 (UART2)
		U2BRG = ((FCY/baud)/4)-1;			

		taille = 0x04;  															//Nbr de paramètres + 2
		instruction = 0x03;  														//Instruction écriture valeur
		adresse = 0x19; 															//Allumer ou éteindre une led
		checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + led));		

		for(i=0;i<2;i++)
		{
			UART2PutChar(0xFF);
			UART2PutChar(0xFF);
			UART2PutChar(id);
			UART2PutChar(taille);
			UART2PutChar(instruction);
			UART2PutChar(adresse);
			UART2PutChar(led);
			UART2PutChar(checksum);
		}
	}
}

//Reset du CDS5516
void CDS5516reset(char id, double baudratemin, double baudratemax)
{
	char taille = 0, instruction = 0, checksum = 0;
	int i;
	double baud;

	for(baud = baudratemin;baud <= baudratemax;baud++)
	{	
		//InitUART2();	//Initialisation de la liaison série 2 (UART2)
		U2BRG = ((FCY/baud)/4)-1;			

		taille = 0x02;  															//Nbr de paramètres + 2
		instruction = 0x06;  														//Instruction écriture valeur															//Allumer ou éteindre une led
		checksum = (char)(0xFF-(char)(id + taille + instruction));		

		for(i=0;i<2;i++)
		{
			UART2PutChar(0xFF);
			UART2PutChar(0xFF);
			UART2PutChar(id);
			UART2PutChar(taille);
			UART2PutChar(instruction);
			UART2PutChar(checksum);
		}
	}
}

//Activation du couple du CDS5516 (0 ou 1)
void CDS5516enableTorque(double baud, char id, unsigned int enableTorque)
{
	char taille = 0, instruction = 0, adresse = 0, checksum = 0;
	int i;

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	U2BRG = ((FCY/baud)/4)-1;

	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x18;															
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + enableTorque));

	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(enableTorque);
		UART2PutChar(checksum);
	}
}


//Changer le bauderate du CDS5516 
void CDS5516bauderate(double baud,char id, char finalBauderate)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0, checksum = 0;
	int i;

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	U2BRG = ((FCY/baud)/4)-1;

	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x04; 	
	val = finalBauderate;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val));

	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(checksum);
	}
}

//Changer l'ID du CDS5516 
void CDS5516id(double baud,char id, char newID)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0, checksum = 0;
	int i;

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	U2BRG = ((FCY/baud)/4)-1;

	taille = 0x04;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x03; 	
	val = newID;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val));

	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(checksum);
	}
}

//Régler le maximum de couple en EEPROM (0 à 1023)
void CDS5516maxTorque(double baud,char id, int maxTorque)
{
	char taille = 0, instruction = 0, adresse = 0, val = 0,val1 = 0, checksum = 0;
	int i;

	//InitUART2();	//Initialisation de la liaison série 2 (UART2)
	U2BRG = ((FCY/baud)/4)-1;

	taille = 0x05;  															//Nbr de paramètres + 2
	instruction = 0x03;  														//Instruction écriture valeur
	adresse = 0x0E; 	
	val = maxTorque & 0b0000000011111111;
	val1 = maxTorque >> 8;
	checksum = (char)(0xFF-(char)(id + taille + instruction + adresse + val + val1));

	for(i=0;i<2;i++)
	{
		UART2PutChar(0xFF);
		UART2PutChar(0xFF);
		UART2PutChar(id);
		UART2PutChar(taille);
		UART2PutChar(instruction);
		UART2PutChar(adresse);
		UART2PutChar(val);
		UART2PutChar(val1);
		UART2PutChar(checksum);	
	}
}
