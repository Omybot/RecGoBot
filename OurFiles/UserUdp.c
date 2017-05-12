/************************************************
*
* UserUdp.c
* Fichier permettant de gérer une communication UDP
* Couche intermédiaire entre le main et le fichier udp.c
*
*
*****************************************************/

#include "UserUdp.h"

BYTE bufferReception[1024];
int nbOctetsRecus;
int accepteClient = 1;

UDP_SOCKET socketReception;
UDP_SOCKET socketEmission;

NODE_INFO infoPc;
NODE_INFO infoMe;

void InitUserUdp()
{
    //Vide le user node du pc
    memclr(&infoPc, sizeof(infoPc));

    //Configure en réception sur le port choisi
    socketReception = UDPOpen(portReception, &infoPc, INVALID_UDP_PORT);
    
    //En cas d'erreur à l'ouverture du socket
    if (socketReception == INVALID_UDP_SOCKET)
	{
        // Afficher message d'erreur
    }
    else
	{
        // Afficher message succès
    }
}

Trame ReceptionUserUdp()
{
	// Variable contenant 1 à 1 tous les octets reçus
	Trame recu;
	recu.nbChar = 0;
	recu.message = (BYTE*)&bufferReception;
	NODE_INFO* pUserNode;

	BYTE c;

	if (UDPIsGetReady(socketReception)) 
	{
		// Socket prêt à reçevoir
        nbOctetsRecus = 0;


        if (accepteClient) 
		{
        	// Si on autorise le changement de client

            // Configure le socket d'émission
			pUserNode = UDPGetNodeInfo();
			infoPc = *pUserNode;
            socketEmission = UDPOpen((WORD)54130, &infoPc, (WORD)portEmission);

			// On garde le premier client et on n'en change plus
			accepteClient = 0;
        }

        // Lit 1 à 1 les octets reçus
        while(UDPGet(&c)) 
		{
            // Stocke les octets reçus dans le buffer
            bufferReception[nbOctetsRecus++] = c;
        }

        // Nettoyage de l'entrée
        UDPDiscard();

		recu.nbChar = nbOctetsRecus;
	}

	return recu;	
}

void EnvoiUserUdp(Trame trame)
{
	if (UDPIsPutReady(socketEmission)) 
	{
        if(trame.nbChar != 0)
		{
			// S'il y a au moins un octet à envoyer, envoi sur le port pc
			UDPPutArray(trame.message, trame.nbChar);

           	// Force l'envoi
           	UDPFlush();
		}
	}
}

void EnvoiStringUdp(const char *string)
{
	unsigned char i;
	Trame envoi;
	static BYTE mess[20];
	
	mess[0] = UDP_ID;
	mess[1] = 0xFF;
	
	for(i=0;string[i]!='\0';i++) 
	{
		mess[2+i] = string[i];
		if((2+i)>19)	break;
	}
	mess[2+i]='\0';
	
	envoi.nbChar = 2+i;
	envoi.message = mess;
	
	EnvoiUserUdp(envoi);
}

/**
 * Fills the given number of bytes in the given array with 0
 *
 * @param dest Pointer to memory area in RAM that has to be set to 0
 * @param size number of consecutive bytes to set to 0
 */
void memclr(void * dest, WORD size) {
    WORD i;
    BYTE * p;
    p = (BYTE *)dest;
    
    for (i = 0; i < size; i++) {
        *p++ = 0;
    }   
}
