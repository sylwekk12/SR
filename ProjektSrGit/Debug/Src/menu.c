/*
 * menu.c
 *
 *  Created on: 28.03.2019
 *      Author: sylwekk12
 */

#include "menu.h"
#include "main.h"
#include "myTime.h"

enum Joy_Event joy_event;
RTC_HandleTypeDef hrtc;

char* Start = "START ";
char* STime = " USTAW GODZ ";
char* SDate = " USTAW DATE ";
char* SLED = "LED   ";
char* SAUDIO = "AUDIO ";
char* DTime = " WYPISZ GODZ ";
char* SAlarms = " WYSWIETL ALARMY ";



void WyswietlAlarmy() //TODO wczytywanie z flasha i wyswietlanie
{
	while(joy_event!=fJOY_CENTER)
	{
		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			break;
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			break;
		default:
			break;
		}
		BSP_LCD_GLASS_ScrollSentence("READING ALARMS",1,300);
		HAL_Delay(10);
	}


}

void PetlaMenu(enum OStatus* oStatus)
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
			Obsluz(opt, oStatus);
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

void Obsluz(enum Option opt, enum OStatus* out)
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
		SetLED(out);
		break;
	case OptAudio:
		SetAUDIO(out);
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

void SetAUDIO(enum OStatus* outs)
{
	enum OStatus outst = *outs;
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

	*outs = outst;

	return;
}

void SetLED(enum OStatus* outs)
{
	enum OStatus outst = *outs;
	while(joy_event!=fJOY_CENTER)
	{
		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			if(outst&fLED) outst &= ~fLED;
			else outst |= fAUDIO;
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

	*outs = outst;

	return;;
}

