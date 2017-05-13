#include "Pilotage.h"
#include <math.h>

extern unsigned int ADC_Results[9];

// ATTENTION /!\ Ces fonctions ne doivent pas être bloquantes

void delay(void)
{
    long i = 10; 
    while(i--);
}
void delayms(void) 
{
	long i = 1600000; //400ms
    while(i--);
}
//Delay seconde
void delays(void) 
{
	long i = 4000000; //seconde
    while(i--);
}

// DEBUG

Trame PiloteDebug0(Trame t)
{
	return t;
}

Trame PiloteDebug1(Trame t)
{
	return t;
}

Trame PiloteDebug2(Trame t)
{
	return t;
}

Trame PiloteDebug3(Trame t)
{
	return t;
}

Trame PiloteDebug4(Trame t)
{
	return t;
}

Trame PiloteDebug5(Trame t)
{
	return t;
}

Trame PiloteDebug6(Trame t)
{
	return t;
}

Trame PiloteDebug7(Trame t)
{
	return t;
}

Trame PiloteDebug8(Trame t)
{
	return t;
}

Trame PiloteDebug9(Trame t)
{
	return t;
}

Trame ReponseEcho()
{
	Trame trame;
	static BYTE tableau[2];
	trame.nbChar = 2;

	tableau[0] = UDP_ID;
	tableau[1] = 0xF5;
	
	trame.message = tableau;
	
	return trame;
}

void PiloteLedRGB(int led, int r, int g, int b)
{
	if(led == LED_RGB_MAIN)
	{
		P1DC1 = b<<4;
		P1DC2 = g<<4;
		P1DC3 = r<<4;
	}	
}

void PiloteLed(int led, int statut)
{
	// TODO
}	

void PiloteBuzzer(int frequency, int volume)
{
	P2DC1 = frequency<<7;
}	

Trame Retour_Valeurs_Analogiques(void)
{
	Trame Etat_Valeurs;
	static BYTE Valeurs[20]; // 20 !!!!!!!!!!! PAS 4 !!!!
	Etat_Valeurs.nbChar = 20;


	Valeurs[0] = UDP_ID;
	Valeurs[1] = CMD_REPONSE_VALEURS_ANALOGIQUES;
	Valeurs[2] = ADC_Results[0] >> 8;
	Valeurs[3] = ADC_Results[0] & 0xFF;
	Valeurs[4] = ADC_Results[1] >> 8;
	Valeurs[5] = ADC_Results[1] & 0xFF;
	Valeurs[6] = ADC_Results[2] >> 8;
	Valeurs[7] = ADC_Results[2] & 0xFF;
	Valeurs[8] = ADC_Results[3] >> 8;
	Valeurs[9] = ADC_Results[3] & 0xFF;
	Valeurs[10] = ADC_Results[4] >> 8;
	Valeurs[11] = ADC_Results[4] & 0xFF;
	Valeurs[12] = ADC_Results[5] >> 8;
	Valeurs[13] = ADC_Results[5] & 0xFF;
	Valeurs[14] = ADC_Results[6] >> 8;
	Valeurs[15] = ADC_Results[6] & 0xFF;
	Valeurs[16] = ADC_Results[7] >> 8;
	Valeurs[17] = ADC_Results[7] & 0xFF;
	Valeurs[18] = ADC_Results[8] >> 8;
	Valeurs[19] = ADC_Results[8] & 0xFF;

	Etat_Valeurs.message = Valeurs;

	return Etat_Valeurs;
}

	
// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	unsigned int param1, param2, param3, param4, i;
	
	retour = t;

	// Les messages ne commencant pas par UDP_ID ne nous sont pas adressés (RecGoBot)
	if(t.message[0] != UDP_ID)
		return t;

	switch(t.message[1])
	{
		case TRAME_DEBUG:
			param1 = t.message[2];							// Numero
			switch(param1)
			{
				case 0:
					retour = PiloteDebug0(t);
					break;
				case 1:
					retour = PiloteDebug1(t);
					break;
				case 2:
					retour = PiloteDebug2(t);
					break;
				case 3:
					retour = PiloteDebug3(t);
					break;
				case 4:
					retour = PiloteDebug4(t);
					break;
				case 5:
					retour = PiloteDebug5(t);
					break;
				case 6:
					retour = PiloteDebug6(t);
					break;
				case 7:
					retour = PiloteDebug7(t);
					break;
				case 8:
					retour = PiloteDebug8(t);
					break;
				case 9:
					retour = PiloteDebug9(t);
					break;
			}
		break;

		case TRAME_TEST_CONNEXION:
			retour = ReponseEcho();
		break;

		case TRAME_RESET:
			Reset();
		break;

		case TRAME_LED_RGB:
			param1 = t.message[2]; 	// N° LED
			param2 = t.message[3];	// R
			param3 = t.message[4];	// G
			param4 = t.message[5];	// B
			PiloteLedRGB(param1, param2, param3, param4);
			break;

		case TRAME_BUZZER:
			param1 = t.message[2] * 256 + t.message[3];	// Frequence
			param2 = t.message[4];						// Volume
			PiloteBuzzer(param1, param2);
			break;
			
		case TRAME_LED:
			param1 = t.message[2]; // N° LED
			param2 = t.message[3]; // Statut (0=Off, 1=Rouge, 2=Orange, 3=Vert)
			PiloteLed(param1, param2);
			break;
		case CMD_DEMANDE_VALEURS_ANALOGIQUES:
			return Retour_Valeurs_Analogiques();
			break;
		
	}
	return retour;
}
