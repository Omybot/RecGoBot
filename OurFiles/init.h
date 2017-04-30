void InitClk(void);
void InitPorts(void);
void InitPWM(void); 
void InitQEI(void);

void Init_Timer2(void);
void Init_Timer5(void);
void Init_Timer4(void);
void InitUART2(void);
void InitADC(void);
void InitDMA(void);
void Init_Interrupt_Priority(void);

#define  MAX_CHNUM	 			7		// Highest Analog input number in Channel Scan
#define  SAMP_BUFF_SIZE	 		8		// Size of the input buffer per analog input
#define  NUM_CHS2SCAN			8		// Number of channels enabled for channel scan

