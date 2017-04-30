#ifndef __TYPE_H__
#define __TYPE_H__

typedef enum Cote {Droite=3, Gauche=2} Cote;
typedef enum StopMode {Abrupt, Smooth, Freely} StopMode;
typedef enum Sens {Avant, Arriere} Sens;

typedef struct Etape Etape;

struct Etape
{
	int distance;
	Etape* suivante;
};


#endif // __TYPE_H__
