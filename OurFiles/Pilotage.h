#ifndef __PILOTAGE_H__
#define __PILOTAGE_H__

#include "UserUdp.h"

Trame ReponseEcho(void);

// Constantes des fonctions des actionneurs
#define ON  1
#define OFF 0

#define RISING_EDGE 1
#define FALLING_EDGE 0 


//Delay
void delay(void);
void delays(void);
void delayms(void);
//ASSERVISSEMENT

int PiloteLed(int ledNo, int on);
int PiloteLedRGB(int r, int g, int b);
int PiloteBuzz(int on);

// DEBUG
Trame PiloteDebug0(Trame t);
Trame PiloteDebug1(Trame t);
Trame PiloteDebug2(Trame t);
Trame PiloteDebug3(Trame t);
Trame PiloteDebug4(Trame t);
Trame PiloteDebug5(Trame t);
Trame PiloteDebug6(Trame t);
Trame PiloteDebug7(Trame t);
Trame PiloteDebug8(Trame t);
Trame PiloteDebug9(Trame t);

//Analyse Trame
Trame AnalyseTrame(Trame t);

#endif // __PILOTAGE_H__

#define MY_ID							0xC2

// Diagnostic
#define	CMD_DEBUG						0xEE
#define	CMD_ECHO						0xF0
#define	CMD_RESET_CARTE					0xF1

#define CMD_DEMANDE_LIDAR				0xA0
#define CMD_REPONSE_LIDAR				0xA1

// Périphériques
#define CMD_LEDS_ON_OFF					0x01
#define CMD_BOUTONS						0x02
#define CMD_LED_RGB_INTENSITY			0x03
#define CMD_LED_RGB_FREQUENCY			0x04
#define CMD_BUZZER_INTENSITY			0x05
#define CMD_BUZZER_FREQUENCY			0x06
