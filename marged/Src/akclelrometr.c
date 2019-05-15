/*
 * akclelrometr.c
 *
 *  Created on: 15.05.2019
 *      Author: sylwekk12
 */

#include "akcelerometr.h"

extern SPI_HandleTypeDef hspi2;


uint8_t spiTxBufx[2];//tablica danych wysylanych do urzadzenia X
uint8_t spiTxBufy[2];//tablica danych wysylanych do urzadzenia Y
uint8_t spiTxBufz[2];//tablica danych wysylanych do urzadzenia Z

uint8_t spiRxBufy[2];//tablica danych odbieranych z urzadzenia Y
uint8_t spiRxBufx[2];//tablica danych odbieranych z urzadzenia X
uint8_t spiRxBufz[2];//tablica danych odbieranych z urzadzenia Z

void AKC_Init()
{
	  /* USER CODE BEGIN 2 */


	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);



	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	  spiTxBufx[0] = 0x23;
	  spiTxBufx[1] = 0xFD;
	  HAL_SPI_Transmit(&hspi2,spiTxBufx,2,50);
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	  spiTxBufx[0] = 0x23|0x80;
	  HAL_SPI_Transmit(&hspi2,spiTxBufx,1,50);
	  __HAL_SPI_DISABLE(&hspi2);
	  HAL_SPI_Receive(&hspi2,spiRxBufx,1,50);
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);

	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	    spiTxBufx[0] = 0x20;
	    spiTxBufx[1] = 0x4F;
	    HAL_SPI_Transmit(&hspi2,spiTxBufx,2,50);
	    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
	    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	    spiTxBufx[0] = 0x20|0x80;
	    HAL_SPI_Transmit(&hspi2,spiTxBufx,1,50);
	    __HAL_SPI_DISABLE(&hspi2);
	    HAL_SPI_Receive(&hspi2,spiRxBufx,1,50);
	    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);


	  /* USER CODE END 2 */
}

void AKC_Pomiar()
{
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
    spiTxBufx[0] = 0x29|0x80;
    HAL_SPI_Transmit(&hspi2,spiTxBufx,1,50);
    __HAL_SPI_DISABLE(&hspi2);
    HAL_SPI_Receive(&hspi2,spiRxBufx,1,50);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
    spiTxBufy[0] = 0x2B|0x80;
    HAL_SPI_Transmit(&hspi2,spiTxBufy,1,50);
    __HAL_SPI_DISABLE(&hspi2);
    HAL_SPI_Receive(&hspi2,spiRxBufy,1,50);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
    spiTxBufz[0] = 0x2D|0x80;
    HAL_SPI_Transmit(&hspi2,spiTxBufz,1,50);
    __HAL_SPI_DISABLE(&hspi2);
    HAL_SPI_Receive(&hspi2,spiRxBufz,1,50);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
}
