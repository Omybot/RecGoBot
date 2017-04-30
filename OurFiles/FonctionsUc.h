
// Quand il est indiqué XUPDATE, cela signifie qu'il y a une mise a jour 
// a effectuer pour le portage du programme dans la carte finale
// (en gros : pinout, vitesse d'horloge, etc.)

#define RECBUFFER 		50			// Taille du buffer reception UART
#define FCY             37000000	// Nombre d'instructions par secondes (IPS)
#define BAUDRATE        9585		// Debit UART (RS232)
#define BRGVAL          ((FCY/BAUDRATE)/4)-1 // Precalcul pour le baudrate generator
#define N				2			// Nombre de moteurs

#define FALSE			0x00
#define TRUE			0x01
#define NEUF			9

//void Tempo1mS(unsigned int nbr);
//void Tempo1uS(unsigned int nbr);

//float _abs(float value);
void UART2PutChar( char ch );
