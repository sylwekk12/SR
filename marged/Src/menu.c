/*
 * menu.c
 *
 *  Created on: 28.03.2019
 *      Author: sylwekk12
 */

#include "menu.h"
#include "main.h"
#include "myTime.h"
#include "pamiec.h"

enum OStatus oStatus;
enum Joy_Event joy_event;
RTC_HandleTypeDef hrtc;
SAI_HandleTypeDef hsai_BlockA1;

char* Start = "START ";
char* STime = " USTAW GODZ ";
char* SDate = " USTAW DATE ";
char* SLED = "LED   ";
char* SAUDIO = "AUDIO ";
char* DTime = " WYPISZ GODZ ";
char* SAlarms = " WYSWIETL ALARMY ";

char* tab[26];


char* alarm_to_string(struct alarm* alm)
{
	char tmp[8];
	char table[26];
	strcat(table,"ALARM ");
	itoa(alm->hour,tmp,10);strcat(table,tmp);strcat(table,"-");
	itoa(alm->min, tmp,10);strcat(table,tmp);strcat(table," ");
	itoa(alm->day, tmp,10);strcat(table,tmp);strcat(table, "-");
	itoa(alm->month, tmp,10);strcat(table,tmp);strcat(table, "-");
	itoa(alm->year, tmp,10);strcat(table,tmp);strcat(table, " ");
	strcpy(tab,table);
	return tab;
}

void WyswietlAlarmy() //TODO wczytywanie z flasha i wyswietlanie
{
	char* napis;
	struct alarm alm;
	uint8_t it = 0;

	alm.day = 10;
	alm.hour = 21;
	alm.min = 37;
	alm.year = 2004;
	alm.month = 4;
	alm.sec = 24;
	zapisz(alm);
	if(wczytaj(&alm,it) != POK)
	{
		BSP_LCD_GLASS_ScrollSentence("  BRAK ALARMOW",1,200);
		HAL_Delay(5000);
		__RESET_JOY(joy_event);
		return;
	}
	napis = alarm_to_string(&alm);

	while(joy_event!=fJOY_CENTER&&joy_event!=fJOY_LEFT)
	{

		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			if(wczytaj(&alm,it-1) == POK)
			{
				napis = alarm_to_string(&alm);
				it-=1;
			}
			break;
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			if(wczytaj(&alm,it+1) == POK)
			{
				napis = alarm_to_string(&alm);
				it+=1;
			}
			break;
		default:
			break;
		}
		//wczytaj();
		BSP_LCD_GLASS_ScrollSentence(napis,1,500);
		HAL_Delay(10);
	}
	__RESET_JOY(joy_event);

}

void PetlaMenu()
{
	enum Option opt = OptStart;
	while(joy_event!=fJOY_LEFT)
	{
		switch(joy_event)
		{
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			BSP_LCD_GLASS_Clear();
			opt = ++opt%OPT_LIMIT;
			break;
		case fJOY_UP:
			__RESET_JOY(joy_event);
			BSP_LCD_GLASS_Clear();
			opt = --opt; if(opt>OPT_LIMIT) opt = OptAlarms;
			break;
		case fJOY_RIGHT:
			__RESET_JOY(joy_event);
			break;
		case fJOY_CENTER:
			__RESET_JOY(joy_event);
			Obsluz(opt);
			break;
		default:

			break;
		}

		DrawOpt(opt);
		HAL_Delay(10);

	}
	__RESET_JOY(joy_event);

}

void DrawOpt(enum Option opt)
{
	switch(opt)
	{
	case OptStart:
		BSP_LCD_GLASS_DisplayString(Start);
		break;
	case OptSTime:
		BSP_LCD_GLASS_ScrollSentence(STime,1,100);
		break;
	case OptSDate:
		BSP_LCD_GLASS_ScrollSentence(SDate,1,100);
		break;
	case OptDioda:
		BSP_LCD_GLASS_DisplayString(SLED);
		break;
	case OptAudio:
		BSP_LCD_GLASS_DisplayString(SAUDIO);
		break;
	case OptAlarms:
		BSP_LCD_GLASS_ScrollSentence(SAlarms,1,100);
		break;
	default:
		break;
	}
}

void Obsluz(enum Option opt)
{
	switch(opt)
	{
	case OptStart:
		Start_Acc();
		break;
	case OptSTime:
		SetTime(&hrtc);
		break;
	case OptSDate:
		SetDate(&hrtc);
		break;
	case OptDioda:
		SetLED();
		break;
	case OptAudio:
		SetAUDIO();
		break;
	case OptAlarms:
		WyswietlAlarmy();
		break;
	default:
		break;
	}
}

void Start_Acc()
{
	return;
}

void SetAUDIO()
{
	enum OStatus outst = oStatus;
	uint16_t tabs[16000];


	while(joy_event!=fJOY_CENTER)
	{
		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			if(outst&fAUDIO) outst &= ~fAUDIO;
			else outst |= fAUDIO;
			break;
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			if(outst&fAUDIO) outst &= ~fAUDIO;
			else outst |= fAUDIO;
			break;
		case fJOY_LEFT:
			__RESET_JOY(joy_event);
			return;
			break;
		default:
			break;
		}
		if(outst&fAUDIO) BSP_LCD_GLASS_DisplayString("ON    ");
		else BSP_LCD_GLASS_DisplayString("OFF   ");
	}
	__RESET_JOY(joy_event);

	oStatus = outst;

	return;
}

void SetLED()
{
	enum OStatus outst = oStatus;
	while(joy_event!=fJOY_CENTER)
	{
		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			if(outst&fLED) outst &= ~fLED;
			else outst |= fLED;
			break;
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			if(outst&fLED) outst &= ~fLED;
			else outst |= fLED;
			break;
		case fJOY_LEFT:
			__RESET_JOY(joy_event);
			return;
			break;
		default:
			break;
		}
		if(outst&fLED) BSP_LCD_GLASS_DisplayString("ON    ");
		else BSP_LCD_GLASS_DisplayString("OFF   ");
	}
	__RESET_JOY(joy_event);

	oStatus = outst;

	return;;
}

