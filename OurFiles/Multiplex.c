#include "Multiplex.h"

void MultiplexInit()
{
	int i, j;

	for (i = 0; i < CHANNELS; i++)
	{
		for (i = 0; i < INPUTS_BY_CHANNEL; j++)
		{
			antiReboundState[i][j] = 0;
			state[i][j] = 0;
		}
	}
}

int MultiplexPow(int val)
{
	int i, j;
	
	j = 1;
	for (i = 0; i < val; i++)
	{
		j *= 2;	
	}
	return j;
}

char MultiplexGetMask(double analogValue)
{
	unsigned char mask = 0;
	char found = 0;

	while (!found && mask < THRESH_COUNT)
	{
		if (analogValue < threshValues[mask])
		{
			found = 1;
		}
		else
		{
			mask += 1;
		}
	}

	return mask;
}


int MultiplexAddMeasure(int channelNo, double analogValue)
{
	char mask;
	int i;
	int inputChanged = -1;

	mask = MultiplexGetMask(analogValue);

	// Comptage jusqu'au seuil anti-rebond, en positif ou en négatif
	for(i = 0; i < INPUTS_BY_CHANNEL; i++)
	{
		if((antiReboundState[channelNo][i] < ANTI_REBOUND_STEPS) && (mask & MultiplexPow(i)))
		{
			antiReboundState[channelNo][i]++;
			if ((antiReboundState[channelNo][i] == ANTI_REBOUND_STEPS) && (state[channelNo][i] == 0))
			{
				// Changement d'état positif, on stoppe la boucle directement
				// Si il doit y avoir un autre changement d'état il sera traité à l'itération suivante
				state[channelNo][i] = 1;
				inputChanged = i;
				break;
			}
		}
		if((antiReboundState[channelNo][i] > 0) && !(mask & MultiplexPow(i)))
		{
			antiReboundState[channelNo][i]--;
			if ((antiReboundState[channelNo][i] == 0) && (state[channelNo][i] == 1))
			{
				// Changement d'état négatif, on stoppe la boucle directement
				// Si il doit y avoir un autre changement d'état il sera traité à l'itération suivante
				state[channelNo][i] = 0;
				inputChanged = i;
				break;
			}
		}
	}

	return inputChanged;
}

char MultiplexGetState(int channelNo, int inputNo)
{
	return state[channelNo][inputNo];
}
