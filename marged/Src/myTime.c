/*
 * LCD_interface.c
 *
 *  Created on: 30.03.2019
 *      Author: sylwekk12
 */
#include "myTime.h"


enum Joy_Event joy_event;

/*
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    /* Peripheral clock enable */
//    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 *
  }

}*/

void DrawTime(RTC_HandleTypeDef* dhrtc)
{
	char napis[7] = "      \0";
	RTC_TimeTypeDef sTime; RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(dhrtc,&sTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(dhrtc,&sDate,RTC_FORMAT_BIN);
	BSP_LCD_GLASS_Clear();
	HAL_Delay(10);


	if(sTime.Hours>9) itoa(sTime.Hours,napis,10);
	else {itoa(sTime.Hours,napis+1,10); napis[0] = ' ';}

	if(sTime.Minutes>9) itoa(sTime.Minutes,napis+2,10);
	else {itoa(sTime.Minutes,napis+3,10); napis[2] = ' ';}

	if(sTime.Seconds>9) itoa(sTime.Seconds,napis+4,10);
		else {itoa(sTime.Seconds,napis+5,10); napis[4] = ' ';}

	BSP_LCD_GLASS_DisplayString(napis);



}

void SetTime(RTC_HandleTypeDef* dhrtc)
{
	char* pusty = "      \0";
	char napis[7] = "      \0";
	RTC_TimeTypeDef sTime, sTime2; RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(dhrtc,&sTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(dhrtc,&sDate,RTC_FORMAT_BIN);

	sTime2 = sTime;
	sTime2.DayLightSaving = 0;

	BSP_LCD_GLASS_Clear();
	//GODZINA


	while(joy_event!=fJOY_CENTER){
		strcpy(napis,pusty);
		napis[0] = 'H';

		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			++(sTime2.Hours); if(sTime2.Hours>23) sTime2.Hours = 0;
			break;
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			--(sTime2.Hours); if(sTime2.Hours>23) sTime2.Hours = 23;
			break;
		case fJOY_LEFT:
			__RESET_JOY(joy_event);
			return;
		default:
			break;
		}
		HAL_Delay(10);

		if(sTime2.Hours > 9)
			itoa(sTime2.Hours, napis+4, 10);
		else itoa(sTime2.Hours, napis+5, 10);

		BSP_LCD_GLASS_Clear();
		HAL_Delay(10);
		BSP_LCD_GLASS_DisplayString(napis);

	}
	__RESET_JOY(joy_event);

	//MINUTY

	while(joy_event!=fJOY_CENTER){
		strcpy(napis,pusty);
		napis[0] = 'M'; napis[1] = 'I'; napis[2] = 'N';

		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			++sTime2.Minutes; if(sTime2.Minutes>59) sTime2.Minutes = 0;
			break;
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			--sTime2.Minutes; if(sTime2.Minutes>59) sTime2.Minutes = 59;
			break;
		case fJOY_LEFT:
			__RESET_JOY(joy_event);
			return;
		default:
			break;
		}

		if(sTime2.Minutes>= 10)
			itoa(sTime2.Minutes, napis+4, 10);
		else itoa(sTime2.Minutes, napis+5, 10);

		BSP_LCD_GLASS_Clear();
		HAL_Delay(10);
		BSP_LCD_GLASS_DisplayString(napis);

		HAL_Delay(10);
	}
	__RESET_JOY(joy_event);

	sTime2.Seconds = 0; sTime2.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;

	HAL_RTC_SetTime(dhrtc, &sTime2,RTC_FORMAT_BIN);
}

void SetDate(RTC_HandleTypeDef* dhrtc)
{
	char* pusty = "      \0";
	char napis[7] = "      \0";
	RTC_DateTypeDef sDate2, sDate;
	uint8_t MAX_DAY = 31;
	HAL_RTC_GetDate(dhrtc,&sDate,RTC_FORMAT_BIN);
	sDate2 = sDate;

	BSP_LCD_GLASS_Clear();

	//rok

	while(joy_event!=fJOY_CENTER){
		strcpy(napis,pusty);
		napis[0] = 'Y';
		switch(joy_event)
		{
		case fJOY_UP:
			++sDate2.Year; if(sDate2.Year>99) sDate2.Year = 0;
			__RESET_JOY(joy_event);
			break;
		case fJOY_DOWN:
			--sDate2.Year; if(sDate2.Year>99) sDate2.Year = 99;
			__RESET_JOY(joy_event);
			break;
		case fJOY_LEFT:
			__RESET_JOY(joy_event);
			return;
		default:
			break;
		}
		__RESET_JOY(joy_event);

		HAL_Delay(10);
		napis[2] = '2'; napis[3] = '0';
		if(sDate2.Year > 9)
		itoa(sDate2.Year, napis+4, 10);
		else {itoa(sDate2.Year, napis+5, 10); napis[4] = '0';}
		BSP_LCD_GLASS_Clear();
		HAL_Delay(5);
		BSP_LCD_GLASS_DisplayString(napis);
	}
	__RESET_JOY(joy_event);
	//miesiac


	while(joy_event!=fJOY_CENTER){
		strcpy(napis,pusty);
		napis[0] = 'M';

		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			++sDate2.Month; if(sDate2.Month>11) sDate2.Month = 0;
			break;
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			--sDate2.Month; if(sDate2.Month>11) sDate2.Month = 11;
			break;
		case fJOY_LEFT:
			__RESET_JOY(joy_event);
			return;
		default:
			break;
		}
		HAL_Delay(10);

		if(sDate2.Month > 8)
			itoa(++sDate2.Month, napis+3, 10);
		else itoa(++sDate2.Month, napis+4, 10);
		--sDate2.Month;

		BSP_LCD_GLASS_Clear();
		HAL_Delay(10);
		BSP_LCD_GLASS_DisplayString(napis);

	}
	__RESET_JOY(joy_event);


	//dzien

	if(!((sDate2.Month+1)%2)) --MAX_DAY;
	if((sDate2.Month+1)==2) {
		MAX_DAY = 28;
		if(!(sDate2.Year%4)) ++MAX_DAY;
	}
	--MAX_DAY;


	while(joy_event!=fJOY_CENTER){
		strcpy(napis,pusty);
		napis[0] = 'D';

		switch(joy_event)
		{
		case fJOY_UP:
			__RESET_JOY(joy_event);
			++sDate2.Date; if(sDate2.Date>MAX_DAY) sDate2.Date = 0;
			break;
		case fJOY_DOWN:
			__RESET_JOY(joy_event);
			--sDate2.Date; if(sDate2.Date>MAX_DAY) sDate2.Date = MAX_DAY;
			break;
		case fJOY_LEFT:
			__RESET_JOY(joy_event);
			return;
		default:
				break;
		}

		if(sDate2.Date >= 10)
			itoa(++sDate2.Date, napis+3, 10);
		else itoa(++sDate2.Date, napis+4, 10);
		--sDate2.Date;

		BSP_LCD_GLASS_Clear();
	//	HAL_Delay(10);
		BSP_LCD_GLASS_DisplayString(napis);

		HAL_Delay(10);
	}
	__RESET_JOY(joy_event);


	HAL_RTC_SetDate(dhrtc, &sDate2,RTC_FORMAT_BIN);

}
