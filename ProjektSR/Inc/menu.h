/*
 * menu.h
 *
 *  Created on: 28.03.2019
 *      Author: sylwekk12
 */

#ifndef MENU_H_
#define MENU_H_

typedef enum
{
  LCD_BAR_NONE  = 0,
  LCD_BAR_0     = (1 << 0),
  LCD_BAR_1     = (1 << 1),
  LCD_BAR_2     = (1 << 2),
  LCD_BAR_3     = (1 << 3)
} BarId_Typedef;

enum Joy_Event
{
	fJOY_NONE=0,
	fJOY_DOWN=1,
	fJOY_UP=2,
	fJOY_LEFT=3,
	fJOY_RIGHT=4,
	fJOY_CENTER=5
};




#endif /* MENU_H_ */
