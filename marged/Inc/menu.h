/*
 * menu.h
 *
 *  Created on: 28.03.2019
 *      Author: sylwekk12
 */

#ifndef MENU_H_
#define MENU_H_



enum Joy_Event
{
	fJOY_NONE=0,
	fJOY_DOWN=2,
	fJOY_UP=1,
	fJOY_LEFT=3,
	fJOY_RIGHT=4,
	fJOY_CENTER=5
};

enum OStatus
{
	fLED = 1 << 0,
	fAUDIO = 1 << 1
};

#define OPT_LIMIT 6
enum Option
{
	OptStart = 0,
	OptSTime = 1,
	OptSDate = 2,
	OptDioda = 3,
	OptAudio = 4,
	OptAlarms = 5
};

extern enum Joy_Event joy_event;

void WyswietlAlarmy();
void SetOutputs(enum Option opt);
void PetlaMenu();
void DrawOpt(enum Option opt);
void Obsluz(enum Option opt);
void Start_Acc();
void SetLED();
void SetAUDIO();



#endif /* MENU_H_ */
