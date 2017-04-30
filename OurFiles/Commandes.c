#include "Commandes.h"
#include "Pilotage.h"

// ATTENTION /!\ Ces fonctions ne doivent pas être bloquantes


int CommandeAvancer(int distance)
{
	float dist = (float)distance / 10.0;
	return PiloteAvancer(dist);
}

int CommandeReculer(int distance)
{
	float dist = (float)distance / 10.0;
	return PiloteReculer(dist);
}

int CommandePivoter(int angle, char direction)
{
	float ang = (float)angle / 10.0;
	Cote cote = (Cote)direction;
	return PilotePivoter(ang, cote);
}

int CommandeVirage(int angle, int rayon, char direction)
{
	float ang = (float)angle / 10.0;
	float ray = (float)rayon / 10.0;
	Cote cote = (Cote)direction;
	return 0;//PiloteVirage(ang, ray, cote);
}

int CommandeStop(char mode)
{
	StopMode stop = (StopMode)mode;
	return PiloteStop(stop);
}

int CommandeRecallage(char s)
{
	Sens sens = (Sens)s;
	return PiloteRecallage(sens);
}

int CommandeAvancerEtapes(int nombreEtapes, Etape etape)
{
	return PiloteAvancerEtapes(nombreEtapes, etape);
}

int CommandeValiderEtapes(int numEtape)
{
	// TODO : Laisser à Christopher
}
