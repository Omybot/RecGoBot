#ifndef __CDS5516_H__
#define __CDS5516_H__


//Bauderates CDS5516
#define CDS_BAUD1	9580
#define CDS_BAUD2	19100
#define CDS_BAUD3
#define CDS_BAUD4
#define CDS_BAUD5
#define CDS_BAUD6
#define CDS_BAUD7
#define CDS_BAUD8
#define CDS_BAUD9	1000000

//Contrôle en angle du CDS5516 (0° à 300°)
void CDS5516Pos(double baud,char id, int position);

//Paramètre de la vitesse du CDS5516
void CDS5516Vit(double baud,char id, unsigned int vitesse);

//Allumer la led du CDS5516
void CDS5516Led(double baud,char id, char led);

//Chercher le Baudrate du CDS5516
void CDS5516searchBaudrate(char id, double baudratemin, double baudratemax, char led);

//Reset du CDS5516
void CDS5516reset(char id, double baudratemin, double baudratemax);

//Activation du couple du CDS5516 (0 ou 1)
void CDS5516enableTorque(double baud,char id, unsigned int enableTorque);

//Changer le bauderate du CDS5516
//finalBauderate = 1   pour 1000000
//finalBauderate = 3   pour 500000
//finalBauderate = 4   pour 400000
//finalBauderate = 7   pour 250000
//finalBauderate = 9   pour 200000
//finalBauderate = 16  pour 115200
//finalBauderate = 34  pour 57600
//finalBauderate = 103 pour 19200
//finalBauderate = 207 pour 9600
void CDS5516bauderate(double baud,char id, char finalBauderate);

//Changer l'ID du CDS5516
void CDS5516id(double baud,char id, char newID);

//Régler le maximum de couple en EEPROM (0 à 1023)
void CDS5516maxTorque(double baud,char id, int maxTorque);

#endif // __CDS5516_H__
