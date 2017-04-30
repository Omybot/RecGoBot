#include "Pilotage.h"
#include "asser.h"
#include "uart2.h"
#include <uart2.h>
#include <math.h>
#include "CDS5516.h"

// ATTENTION /!\ Ces fonctions ne doivent pas être bloquantes

#define UART_BUFFER_SIZE 100

extern 	unsigned char flag_envoi_uart,buffer_envoi_uart[UART_BUFFER_SIZE],ptr_write_buffer_uart;
extern unsigned char Demande_lidar;
extern unsigned int positions_xy[2][300];
extern unsigned int nbr_points;
extern unsigned char bridage;
extern unsigned int prd_envoi_position;
extern unsigned char jackAvant;
extern unsigned char desactive_interrupt;
extern unsigned int cpt_capteur_vitesse;
extern unsigned int ADC_Results[8];
extern unsigned int position_buffer[6];
extern unsigned char buff_position_ptr,last_send_ptr;
extern long buff_position[N][2]; // gain de place traj. courbe
extern unsigned char buff_status_ptr,last_send_status_ptr;
extern unsigned int buff_status[3][64];
extern long raw_position[2];
extern double erreur[N];
extern double targ_pos[N];
extern double real_pos[N];
extern double cons_pos[N];
extern double pwm_cor[N];
extern double feedforward;
extern unsigned int PID_ressource_used ;
extern double xydistance;
extern double xyangle;
extern double position_lock;
extern double pos_x;
extern double pos_y;
extern double pos_teta;
extern double offset_teta;
extern double erreur_allowed;
extern double kp_cap,ki_cap,kd_cap;
extern double kp_vit,ki_vit,kd_vit;
unsigned char scan;
unsigned int Cpt_Tmr_Periode = 0;

void EnvoiUART(Trame t)
{
	unsigned char i;
	// Copier trame dans buffer circulaire d'envoi
	for(i=0;i<t.nbChar-3;i++)
	{
		buffer_envoi_uart[ptr_write_buffer_uart++]=t.message[i+3];
		if(ptr_write_buffer_uart >= UART_BUFFER_SIZE)
			ptr_write_buffer_uart=0;
	}
}

Trame Retour_Valeurs_Analogiques(void)
{
	Trame Etat_Valeurs;
	static BYTE Valeurs[4];
	Etat_Valeurs.nbChar = 14;
	

	Valeurs[0] = 0xC1;
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
	

	Etat_Valeurs.message = Valeurs;

	return Etat_Valeurs;
}


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



//Fonction qui renvoie sous forme de trame(UDP) la couleur de l'equipe
Trame Couleur_Equipe(void)
{
	Trame Etat_Couleur_Equipe;
	static BYTE Couleur[3];
	Etat_Couleur_Equipe.nbChar = 3;

	Couleur[0] = 0xC1;
	Couleur[1] = CMD_REPONSE_COULEUR_EQUIPE;
	Couleur[2] = PORTBbits.RB4;

	Etat_Couleur_Equipe.message = Couleur;
	
	return Etat_Couleur_Equipe;
}

Trame Presence_Jack(void)
{
	Trame Etat_Jack;
	static BYTE Jack[3];
	Etat_Jack.nbChar = 3;

	Jack[0] = 0xC1;
	Jack[1] = CMD_REPONSE_PRESENCE_JACK;
	Jack[2] = !PORTAbits.RA8;	

	Etat_Jack.message = Jack;

	return Etat_Jack;
}

Trame PiloteGotoXY(int x,int y, unsigned char x_negatif, unsigned char y_negatif)
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[6];
	trame.nbChar = 6;

	GotoXY((double)x,(double)y,0);
	
	tableau[0] = 1;
	tableau[1] = 0x13;
	tableau[2] = (int)xyangle>>8;
	tableau[3] = (int)xyangle&0x00FF;
	tableau[4] = (int)xydistance>>8;
	tableau[5] = (int)xydistance&0x00FF;
	
	trame.message = tableau;
	
	return trame;
	
}

Trame StatusMonitor(void)
{
	Trame trame;
	static BYTE tableau[512];
	unsigned char i,current_send_ptr,nbr_to_send;
	
	tableau[0] = 0xC1; // identifiant trame
	tableau[1] = CMD_REPONSE_BUFF_STATUS;

	

	if(buff_status_ptr > last_send_status_ptr)
		nbr_to_send = buff_status_ptr - last_send_status_ptr;
	else
		nbr_to_send = 64 - last_send_status_ptr + buff_status_ptr;

	//nbr_to_send = (buff_status_ptr - last_send_status_ptr)%64;


	last_send_status_ptr=buff_status_ptr;
	if(nbr_to_send>35) nbr_to_send=35;
	tableau[2] = nbr_to_send;
	trame.nbChar = nbr_to_send*6+4;

	current_send_ptr = last_send_status_ptr + 1;

	for(i=0;i<nbr_to_send;i++)
	{
		tableau[1+2+(i*6)] = buff_status[0][current_send_ptr]>>8; // Status
		tableau[1+3+(i*6)] = buff_status[0][current_send_ptr]&0x00FF;		
		tableau[1+4+(i*6)] = buff_status[1][current_send_ptr]>>8; // PWM gauche
		tableau[1+5+(i*6)] = buff_status[1][current_send_ptr]&0x00FF;
		tableau[1+6+(i*6)] = buff_status[2][current_send_ptr]>>8; // PWM droite
		tableau[1+7+(i*6)] = buff_status[2][current_send_ptr]&0x00FF;
		current_send_ptr = (current_send_ptr + 1)%64;
	}
	
	trame.message = tableau;
	
	return trame;
}


void PilotePIDInit(void)
{
	InitProp();
}


Trame PilotePositionXYT()
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[8];
	trame.nbChar = 8;
	
	tableau[0] = 0xC1;
	tableau[1] = CMD_RETOURPOSITION;
	tableau[2] = (int)(pos_x * 10)>>8;
	tableau[3] = (int)(pos_x * 10)&0x00FF;
	tableau[4] = (int)(pos_y * 10)>>8;
	tableau[5] = (int)(pos_y * 10)&0x00FF;
	tableau[6] = (unsigned int)(pos_teta*36000/(2*PI)+18000)>>8;
	tableau[7] = (unsigned int)(pos_teta*36000/(2*PI)+18000)&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}

Trame PilotePIDRessource()
{
	Trame trame;
	static BYTE tableau[4];
	trame.nbChar = 4;
	
	tableau[0] = 1;
	tableau[1] = 0x66;
	tableau[2] = PID_ressource_used>>8;
	tableau[3] = PID_ressource_used&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}

Trame ReponseEcho()
{
	Trame trame;
	static BYTE tableau[2];
	trame.nbChar = 2;

	tableau[0] = 0xC1;
	tableau[1] = 0xF5;
	
	trame.message = tableau;
	
	return trame;
}


void PilotePIDManual(unsigned int gauche,unsigned int droite)
{
	manual_pid((double)gauche,(double)droite);
}


void PilotePIDFeedforward(unsigned int value)
{
	feedforward = (double)value;
}

Trame PilotePIDErreurs()
{
	Trame trame;
	static BYTE tableau[10];
	trame.nbChar = 10;
	unsigned int data[4];
	
	tableau[0] = 1;
	tableau[1] = 0x48;
	
	data[0] = (unsigned int)fabs(pwm_cor[0]);
	data[1] = (unsigned int)fabs(pwm_cor[1]);
	data[2] = (unsigned int)fabs(real_pos[0]);
	data[3] = (unsigned int)fabs(real_pos[1]);


	tableau[2] = (int)pwm_cor[0]>>8;
	tableau[3] = (int)pwm_cor[0]&0x00FF;
	tableau[4] = (int)pwm_cor[1]>>8;
	tableau[5] = (int)pwm_cor[1]&0x00FF;
	tableau[6] = (int)raw_position[0]>>8;
	tableau[7] = (int)raw_position[0]&0x00FF;
	tableau[8] = (int)raw_position[1]>>8;
	tableau[9] = (int)raw_position[1]&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}
void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd)
{
	double coeffs[N*3]={DEFAULT_KP,DEFAULT_KI,DEFAULT_KD,DEFAULT_KP,DEFAULT_KI,DEFAULT_KD};
	coeffs[0] = (double)new_kp;
	coeffs[1] = (double)new_ki;
	coeffs[2] = (double)new_kd;
	coeffs[3] = (double)new_kp;
	coeffs[4] = (double)new_ki;
	coeffs[5] = (double)new_kd;
	
	set_pid(coeffs);
}

Trame PiloteGetPosition(unsigned char cote)
{
	Trame trame;
	static BYTE tableau[6];
	trame.nbChar = 6;
	double position;	

	tableau[0] = 1;
	tableau[1] = 0x42;
	tableau[2] = cote;
	
	switch(cote)
	{
		case 3:	position = Motors_GetPosition(0);
				break;
		case 4:	position = Motors_GetPosition(1);
				break;
		default:	break;
	}

	tableau[3] = ((unsigned int)fabs(position))>>8;
	tableau[4] = ((unsigned int)fabs(position))&0x00FF;
	if(position<0) 	tableau[5] = 1;
	else			tableau[5] = 0;
	trame.message = tableau;
	
	return trame;
}

Trame PiloteGetRawPosition()
{
	Trame trame;
	static BYTE tableau[15];
	trame.nbChar = 15;
	
	tableau[0] = 1;
	tableau[1] = CMD_DEMANDE_BUFF_POSITION;
	tableau[2] = 0;
	

	tableau[3] = position_buffer[0]>>8;
	tableau[4] = position_buffer[0]&0x00FF;
	tableau[5] = position_buffer[1]>>8;
	tableau[6] = position_buffer[1]&0x00FF;
	tableau[7] = position_buffer[2]>>8;
	tableau[8] = position_buffer[2]&0x00FF;
	tableau[9] = position_buffer[3]>>8;
	tableau[10] = position_buffer[3]&0x00FF;
	tableau[11] = position_buffer[4]>>8;
	tableau[12] = position_buffer[4]&0x00FF;
	tableau[13] = position_buffer[5]>>8;
	tableau[14] = position_buffer[5]&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}

Trame PiloteGetLongPosition()
{
	Trame trame;
	static BYTE tableau[10];
	trame.nbChar = 10;
	
	tableau[0] = 1;
	tableau[1] = 0x44;

	tableau[2] = raw_position[0]>>24;
	tableau[3] = raw_position[0]>>16;
	tableau[4] = raw_position[0]>>8;
	tableau[5] = raw_position[0]&0x00FF;
	
	tableau[6] = raw_position[1]>>24;
	tableau[7] = raw_position[1]>>16;
	tableau[8] = raw_position[1]>>8;
	tableau[9] = raw_position[1]&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}

// A faire : envoie consigne posiiton brut en pas codeur 2 parametres : [sens]8u ; [pascodeur]16u ;

Trame PiloteGetBuffPosition()
{
	Trame trame;
	static BYTE tableau[512];
	unsigned char i,current_send_ptr,nbr_to_send;
	
	tableau[0] = 0xC1; // identifiant trame
	tableau[1] = CMD_REPONSE_BUFF_POSITION;
	nbr_to_send = buff_position_ptr - last_send_ptr;
	last_send_ptr=buff_position_ptr;
	if(nbr_to_send>60) nbr_to_send=60;
	tableau[2] = nbr_to_send;
	trame.nbChar = nbr_to_send*8+4;
	for(i=0;i<nbr_to_send;i++)
	{
		current_send_ptr = buff_position_ptr-nbr_to_send+i;
		tableau[1+2+(i*8)] = buff_position[0][current_send_ptr]>>24;
		tableau[1+3+(i*8)] = buff_position[0][current_send_ptr]>>16;
		tableau[1+4+(i*8)] = buff_position[0][current_send_ptr]>>8;
		tableau[1+5+(i*8)] = buff_position[0][current_send_ptr]&0x00FF;
		
		tableau[1+6+(i*8)] = buff_position[1][current_send_ptr]>>24;
		tableau[1+7+(i*8)] = buff_position[1][current_send_ptr]>>16;
		tableau[1+8+(i*8)] = buff_position[1][current_send_ptr]>>8;
		tableau[1+9+(i*8)] = buff_position[1][current_send_ptr]&0x00FF;
	}
	
	trame.message = tableau;
	
	return trame;
}

int PiloteVitesse(int vitesse)
{
	Motors_SetSpeed(vitesse,MOTEUR_GAUCHE);
	Motors_SetSpeed(vitesse,MOTEUR_DROIT);
	return 1;
}

int PiloteAcceleration(int acceleration)
{
	Motors_SetAcceleration(acceleration,MOTEUR_GAUCHE);
	Motors_SetAcceleration(acceleration,MOTEUR_DROIT);
	return 1;
}

int PiloteAvancer(double distance)
{
	Avance(distance,0);
	return 1;
}

// Recule de la distance spécifiée
// distance : distance à reculer
int PiloteReculer(double distance)
{
	Avance(-distance,0);
	return 1;
}

// Pivote de l'angle spécifié
// angle : angle de rotation (en degrés)
// direction : coté du pivot (Gauche ou Droite)
int PilotePivoter(double angle, Cote direction)
{
	if(direction==Gauche)	Pivot( angle/100.0,0);
	else					Pivot(-angle/100.0,0);
	return 1;
}

// Effectuer un virage
// angle : angle de rotation (en degrés)
// rayon : rayon du virage
// direction : coté du virage (Gauche ou Droite)
int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle)
{
	if(reculer) Virage(direction, rayon, angle/100, 0);
	else	 	Virage(direction, rayon, -angle/100, 0);

	return 1;
}

// Stoppe le robot
// mode : mode de stop (Abrupt, Smooth, Freely)
int PiloteStop(unsigned char stopmode)
{
	/*int distanceRestante;
	Trame envoiReste;
	static BYTE messReste[2];
	messReste[0] = 0xC1;
	messReste[1] = 0x60;
	envoiReste.nbChar = 4;
	*/
	Stop(stopmode);	

	//envoiReste.message = messReste;

	//while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));

	//EnvoiUserUdp(envoiReste);
	
	return 1;
}

// Recallage du robot
// s : sens du recallage (Avant ou Arriere)
int PiloteRecallage(Sens s)
{	
	// TODO : Bah alors, c'est pas encore codé feignasse ?

	return 1;
}

int PiloteOffsetAsserv(double x, double y, double teta)
{
	double toto;
	pos_x = -y;
	pos_y = -x;
	toto = ((teta)/180*PI) / 100.0;
	offset_teta = toto - pos_teta + offset_teta;

	return 1;
}

// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	unsigned int param1, param2, param3, param4, i;
	
	retour = t;

	// Les messages ne commencant pas par 0xC1 ne nous sont pas adressés (RecMove)
	if(t.message[0] != 0xC1)
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
		
		case CMD_AVANCER:
			param1 = t.message[3] * 256 + t.message[4];		// Distance
			if(t.message[2])								// Sens
				PiloteAvancer(param1);
			else
				PiloteReculer(param1);
		break;

		case CMD_GOTOXY:
			param1 = t.message[2] * 256 + t.message[3];		// X
			param2 = t.message[4] * 256 + t.message[5];		// Y
			param3 = t.message[6];							// X positif
			param4 = t.message[7];							// Y positif
			retour = PiloteGotoXY(param1,param2,(unsigned char)param3,(unsigned char)param4);
		break;

		case CMD_RECALLAGE:
			Calage(t.message[2]);							// Sens
		break;

		case CMD_PIVOTER:
			param1 = t.message[3] * 256 + t.message[4];		// Angle
			param2 = t.message[2];							// Sens
			PilotePivoter(param1, (Cote)param2);
		break;

		case CMD_VIRAGE:
			param1 = t.message[4] * 256 + t.message[5];		// Rayon
			param2 = t.message[6] * 256 + t.message[7];		// Angle
			param3 = t.message[3];							// Direction
			param4 = t.message[2];							// Sens
			PiloteVirage((unsigned char)param4,(unsigned char)param3, (double)param1, (double)param2);
		break;

		case CMD_STOP:
			param1 = t.message[2];							// StopMode
			PiloteStop((unsigned char)param1);
		break;

		case CMD_VITESSE_PIVOT:
			param1 = t.message[2] * 256 + t.message[3];		// Vitesse
			Motors_SetSpeed_Pivot(param1);
		break;

		case CMD_VITESSE_LIGNE:
			param1 = t.message[2] * 256 + t.message[3];		// Vitesse
			Motors_SetSpeed_Ligne((double)param1);
		break;

		case CMD_ACCELERATION_PIVOT:
			param1 = t.message[2] * 256 + t.message[3];		// Accélération
			Motors_SetAcceleration_Pivot((double)param1);
		break;

		case CMD_ACCELERATION_LIGNE:
			param1 = t.message[2] * 256 + t.message[3];		// Accélération début
			param2 = t.message[4] * 256 + t.message[5];		// Accélération fin
			Motors_SetAcceleration_Ligne((double)param1, (double)param2);
		break;
			

		case CMD_ENVOI_PID:
			param1 = t.message[2]*256+t.message[3];			// P
			param2 = t.message[4]*256+t.message[5];			// I
			param3 = t.message[6]*256+t.message[7];			// D
			PilotePIDCoeffs(param1,param2,param3);
		break;
		case CMD_ENVOI_PID_CAP:
			kp_cap = ((double)(t.message[2]*256+t.message[3]))*100;			// P
			ki_cap = (double)(t.message[4]*256+t.message[5]);			// I
			kd_cap = ((double)(t.message[6]*256+t.message[7]))*100;			// D
		break;
		case CMD_ENVOI_PID_VITESSE:
			kp_vit = (double)(t.message[2]*256+t.message[3]);			// P
			ki_vit = (double)(t.message[4]*256+t.message[5]);			// I
			kd_vit = (double)(t.message[6]*256+t.message[7]);			// D
		break;

		case CMD_ECHO:
			bridage = t.message[2];
			retour = ReponseEcho();
		break;
			
		case CMD_DEMANDE_COULEUR_EQUIPE:
			// Interrupteur couleur Equipe
			retour = Couleur_Equipe();
			break;
			
		case CMD_OFFSETASSERV:

			param1 = t.message[2]*256+t.message[3];			// X
			param2 = t.message[4]*256+t.message[5];			// Y
			param3 = t.message[6]*256+t.message[7];			// TETA
			PiloteOffsetAsserv((double)param1,(double)param2,(double)param3);

		break;

		case CMD_DEMANDEPOSITION: // Demande POS X Y TETA
			retour = PilotePositionXYT();
		break;

		
		
		case CMD_RESET_CARTE:
			Reset();
		break;
		case CMD_ARME_JACK:
			jackAvant=1;
		break;
		case CMD_DEMANDE_PRESENCE_JACK:
			return Presence_Jack();
		break;
		case CMD_CONSIGNE_POSITION:
			if(t.message[2] == AVANT)
			{
				cons_pos[0] += MM_SCALER * (t.message[3] * 256 + t.message[4]);
				cons_pos[1] += MM_SCALER * (t.message[3] * 256 + t.message[4]);
			}
			else
			{
				cons_pos[0] -= MM_SCALER * (t.message[3] * 256 + t.message[4]);
				cons_pos[1] -= MM_SCALER * (t.message[3] * 256 + t.message[4]);
			}
			break;
		case CMD_DEMANDE_BUFF_POSITION:
			return PiloteGetBuffPosition();
			break;
		case CMD_DEMANDE_BUFF_STATUS:
			return StatusMonitor();
			break;
		case CMD_PRD_ENVOI_POSITION:
			prd_envoi_position = 10*(unsigned int)t.message[2];
			break;
		case CMD_DEMANDE_VALEURS_ANALOGIQUES:
			return Retour_Valeurs_Analogiques();
		break;
		case CMD_DEPLACEMENT_POLAIRE:
		 	nbr_points = t.message[3] * 256 + t.message[4];
			for(i=0;i<nbr_points;i++)
			{
				positions_xy[0][i+1] = t.message[5+i*4] * 256 + t.message[6+i*4]; // X
				positions_xy[1][i+1] = t.message[7+i*4] * 256 + t.message[8+i*4]; // Y
			}
			positions_xy[0][0]=pos_x;
			positions_xy[1][0]=pos_y;
			Deplacement_Polaire();			
		break;
	}
	return retour;
}
