#ifndef __COMMANDES_H__
#define __COMMANDES_H__

#include "Types.h"


int CommandeAvancer(int distance);

int CommandeReculer(int distance);

int CommandePivoter(int angle, char direction);

int CommandeVirage(int angle, int rayon, char direction);

int CommandeStop(char mode);

int CommandeRecallage(char s);

int CommandeAvancerEtapes(int nombreEtapes, Etape etape);

int CommandeValiderEtapes(int numEtape);


#endif // __COMMANDES_H__
