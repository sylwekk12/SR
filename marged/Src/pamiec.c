/*
 * pamiec.c
 *
 *  Created on: 09.05.2019
 *      Author: sylwekk12
 */
#include "pamiec.h"

uint32_t ostatni_adres = 0x0000;

int zapisz(struct alarm alm)
{
	if(ostatni_adres > MAX_ADRES) return PERROR;
	else {
		BSP_QSPI_Erase_Block(ostatni_adres);
		if(BSP_QSPI_Write(&alm, ostatni_adres, sizeof(alm)) == HAL_ERROR) return PERROR;
		ostatni_adres += BLOK_SIZE;
		return POK;
	}
}

int wczytaj(struct alarm* alm, uint32_t index)
{
	if(index*BLOK_SIZE >= ostatni_adres) return PERROR;
	else {
		if(BSP_QSPI_Read(alm, index*BLOK_SIZE, ALARM_SIZE)==HAL_ERROR) return PERROR;
		return POK;
	}
}






