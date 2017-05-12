#ifndef __PILOTAGE_H__
#define __PILOTAGE_H__

#include "UserUdp.h"

// Constantes des fonctions des actionneurs
#define ON  1
#define OFF 0

#define RISING_EDGE 1
#define FALLING_EDGE 0 


//Delay
void delay(void);
void delays(void);
void delayms(void);

// Controles
void PiloteLedRGB(int led, int r, int g, int b);

// Diagnostic
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
void PiloteBuzzer(int frequency, int volume);

//Analyse Trame
Trame ReponseEcho(void);
Trame AnalyseTrame(Trame t);

#endif // __PILOTAGE_H__

#define LED_RGB_MAIN 0x00

// Diagnostic
#define TRAME_DEBUG 0xEE
#define TRAME_TEST_CONNEXION 0xF0
#define TRAME_TENSION_BATTERIE 0xF5
#define TRAME_RESET 0xF1
#define TRAME_BUZZER 0xF3

// Capteurs
#define TRAME_DEPART_JACK 0x71
#define TRAME_COULEUR_EQUIPE_ASK 0x72
#define TRAME_COULEUR_EQUIPE_RESP 0x73
#define TRAME_CAPTEUR_ONOFF_ASK 0x74
#define TRAME_CAPTEUR_ONOFF_RESP 0x75
#define TRAME_VALEURS_ANALOGIQUES_ASK 0x76
#define TRAME_VALEURS_ANALOGIQUES_RESP 0x77
#define TRAME_CAPTEUR_COULEUR_ASK 0x52
#define TRAME_CAPTEUR_COULEUR_RESP 0x53
#define TRAME_CODEUR_ASK 0x21
#define TRAME_CODEUR_RESP 0x22

// Actionneurs
#define TRAME_ARMER_JACK 0x70
#define TRAME_PILOTAGE_ONOFF 0x65
#define TRAME_PILOTAGE_VALEURS 0x62
#define TRAME_POSITION_MOTEUR 0x66
#define TRAME_VITESSE_MOTEUR 0x67
#define TRAME_ACCELERATION_MOTEUR 0x68
#define TRAME_SERVOMOTEUR 0x60

// Déplacement
#define TRAME_AVANCE 0x01
#define TRAME_PIVOT 0x03
#define TRAME_VIRAGE 0x04
#define TRAME_STOP 0x05
#define TRAME_RECALLAGE 0x10
#define TRAME_DEPLACEMENT_POLAIRE 0x20
#define TRAME_RECALLAGE_TERMINE 0x11
#define TRAME_DEPLACEMENT_TERMINE 0x12
#define TRAME_BLOCAGE 0x13

// Debug asserv
#define TRAME_POSITION_ASSER_ASK 0x43
#define TRAME_POSITION_ASSER_RESP 0x44
#define TRAME_CONSIGNE_BRUTE_POSITION 0x45
#define TRAME_CHARGE_CPU_PWM_ASK 0x46
#define TRAME_CHARGE_CPU_PWM_RESP 0x47
#define TRAME_INTERVALLE_RETOUR_POSITION 0x48

// Asservissement
#define TRAME_ASSER_POSITION_ASK 0x30
#define TRAME_ASSER_POSITION_RESP 0x31
#define TRAME_ASSER_VITESSE_LIGNE 0x32
#define TRAME_ASSER_ACCELERATION_LIGNE 0x33
#define TRAME_ASSER_VITESSE_PIVOT 0x34
#define TRAME_ASSER_ACCELERATION_PIVOT 0x35
#define TRAME_ASSER_PID_STANDARD 0x36
#define TRAME_ASSER_RESET_POSITION 0x37
#define TRAME_ASSER_PID_CAP 0x38
#define TRAME_ASSER_PID_VITESSE 0x39

// UART
#define TRAME_UART1_ENVOI 0xA0
#define TRAME_UART1_RECEPTION 0xA1
#define TRAME_UART2_ENVOI 0xA2
#define TRAME_UART2_RECEPTION 0xA3
#define TRAME_UART_BAUDRATE 0x61

// Controles
#define TRAME_LCD_DISPLAY 0xB0
#define TRAME_LED_RGB 0xB1
