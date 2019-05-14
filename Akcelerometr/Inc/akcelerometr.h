

#ifndef AKCELEROPETR_H
#define AKCELEROMETR_H

#include "stm32l4xx_hal_spi.h"

void AKC_Init();

void AKC_Pomiar();

extern uint8_t spiTxBufx[2];//tablica danych wysylanych do urzadzenia X
extern uint8_t spiTxBufy[2];//tablica danych wysylanych do urzadzenia Y
extern uint8_t spiTxBufz[2];//tablica danych wysylanych do urzadzenia Z

extern uint8_t spiRxBufx[2];//tablica danych odbieranych z urzadzenia X
extern uint8_t spiRxBufy[2];//tablica danych odbieranych z urzadzenia Y
extern uint8_t spiRxBufz[2];//tablica danych odbieranych z urzadzenia Z



#endif
