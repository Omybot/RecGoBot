#include "Pilotage.h"
#include "uart2.h"
#include <uart2.h>
#include <math.h>

// ATTENTION /!\ Ces fonctions ne doivent pas être bloquantes

#define UART_BUFFER_SIZE 100

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

	tableau[0] = 0xC2;
	tableau[1] = 0xF5;
	
	trame.message = tableau;
	
	return trame;
}

// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	unsigned int param1, param2, param3, param4, i;
	
	retour = t;

	// Les messages ne commencant pas par MY_ID ne nous sont pas adressés (RecGoBot)
	if(t.message[0] != MY_ID)
		return t;

	switch(t.message[1])
	{
		case CMD_DEBUG:
			param1 = t.message[3];							// Numero
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

		case CMD_ECHO:
			retour = ReponseEcho();
		break;

		case CMD_RESET_CARTE:
			Reset();
		break;

		case CMD_LED_RGB_INTENSITY:
			P1DC1 = t.message[4]<<5;
			P1DC2 = t.message[3]<<5;
			P1DC3 = t.message[2]<<5;
			break;

		case CMD_BUZZER_INTENSITY:
			P2DC1 = t.message[2]<<7;
			break;

		case CMD_LED_RGB_FREQUENCY:
			break;

		case CMD_BUZZER_FREQUENCY:
			break;
		
	}
	return retour;
}
