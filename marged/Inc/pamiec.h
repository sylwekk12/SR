/*
 * pamiec.h
 *
 *  Created on: 09.05.2019
 *      Author: sylwekk12
 */




///pamiec
#ifndef PAMIEC_H_
#define PAMIEC_H_

struct alarm //TODO: wywalic do wlasciwego include'a
{
	int year;
	int month;
	int day;

	int hour;
	int min;
	int sec;

	int type;
};


//bledy
#define PERROR -1
#define POK 0

//wilkosc zmiennych
#define ALARM_SIZE sizeof(struct alarm)
#define BLOK_SIZE 0x1000

#include "stm32l476g_discovery_qspi.h"


int wczytaj(struct alarm* alm, uint32_t index);
//ret true gdy udalo sie zapisac
int zapisz(struct alarm alm);
//ret true gdy udalo sie odczytac dane


#define MAX_ADRES 499*0x1000
#define MAX_WPISOW BLOKI*WPISOW_NA_BLOK

#endif /* PAMIEC_H_ */
