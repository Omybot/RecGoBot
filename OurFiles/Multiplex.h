#ifndef __MULIPLEX_H__
#define __MULIPLEX_H__

#define CHANNELS 4
#define INPUTS_BY_CHANNEL 4
#define THRESH_COUNT INPUTS_BY_CHANNEL * INPUTS_BY_CHANNEL
#define ANTI_REBOUND_STEPS 50

static double threshValues[THRESH_COUNT] = {	1.519, 
												1.577, 
												1.64,
												1.705,
												1.775,
												1.855,
												1.942,
												2.061,
												2.191,
												2.316,
												2.452,
												2.601,
												2.768,
												2.969,
												3.197,
												10.0};

static int antiReboundState[CHANNELS][INPUTS_BY_CHANNEL];
static int state[CHANNELS][INPUTS_BY_CHANNEL];



void MultiplexInit();

char MultiplexGetMask(double analogValue);

int MultiplexAddMeasure(int channelNo, double analogValue);

char MultiplexGetState(int channelNo, int inputNo);

int MultiplexPow(int val);

#endif
