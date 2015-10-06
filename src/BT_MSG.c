/*
 * BT_MSG.c
 *
 *  Created on: 2015. okt. 6.
 *      Author: Csabi
 */


#include "BT_MSG.h"


void int2msg(struct BT_MSG * btmsg, int ertek, char* nev)
{
	uint8_t i;
	uint8_t * ptr;

	btmsg->data[0] = 1;
	btmsg->data[1] = 4;

	ptr = &ertek;
	for( i=2 ; i<(sizeof(int)+2) ; i++)
	{
		btmsg->data[i] = *ptr;
		ptr++;
	}

	while (*nev)
	{
		btmsg->data[i] = *nev;
		nev++;
		i++;
	}
	btmsg->data[i] = '\0';
	btmsg->datasize = sizeof(int);
	btmsg->size = i+1;
}


