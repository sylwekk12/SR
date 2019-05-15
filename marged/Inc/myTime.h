/*
 * LCD_interface.h
 *
 *  Created on: 30.03.2019
 *      Author: sylwekk12
 */

//#include ""


#ifndef MYTIME_H_
#define MYTIME_H_
#include "main.h"
#include "menu.h"
#include <string.h>


void DrawTime(RTC_HandleTypeDef* hrtc);
void SetTime(RTC_HandleTypeDef* hrtc);
void SetDate(RTC_HandleTypeDef* hrtc);

#define __RESET_JOY(vEvent)  (vEvent = fJOY_NONE)


void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc);

#endif /* MYTIME_H_ */
