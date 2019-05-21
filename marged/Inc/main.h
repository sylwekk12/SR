/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l476g_discovery.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SAI1_MCK_Pin GPIO_PIN_2
#define SAI1_MCK_GPIO_Port GPIOE
#define AUDIO_RST_Pin GPIO_PIN_3
#define AUDIO_RST_GPIO_Port GPIOE
#define SAI1_FS_Pin GPIO_PIN_4
#define SAI1_FS_GPIO_Port GPIOE
#define SAI1_SCK_Pin GPIO_PIN_5
#define SAI1_SCK_GPIO_Port GPIOE
#define MAG_CS_Pin GPIO_PIN_0
#define MAG_CS_GPIO_Port GPIOC
#define MAG_DRDY_Pin GPIO_PIN_2
#define MAG_DRDY_GPIO_Port GPIOC
#define JOY_CENTER_Pin GPIO_PIN_0
#define JOY_CENTER_GPIO_Port GPIOA
#define JOY_CENTER_EXTI_IRQn EXTI0_IRQn
#define JOY_LEFT_Pin GPIO_PIN_1
#define JOY_LEFT_GPIO_Port GPIOA
#define JOY_LEFT_EXTI_IRQn EXTI1_IRQn
#define JOY_RIGHT_Pin GPIO_PIN_2
#define JOY_RIGHT_GPIO_Port GPIOA
#define JOY_RIGHT_EXTI_IRQn EXTI2_IRQn
#define JOY_UP_Pin GPIO_PIN_3
#define JOY_UP_GPIO_Port GPIOA
#define JOY_UP_EXTI_IRQn EXTI3_IRQn
#define JOY_DOWN_Pin GPIO_PIN_5
#define JOY_DOWN_GPIO_Port GPIOA
#define JOY_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOB
#define AUODIO_DIN_Pin GPIO_PIN_7
#define AUODIO_DIN_GPIO_Port GPIOE
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOE
#define AOUDIO_CLK_Pin GPIO_PIN_9
#define AOUDIO_CLK_GPIO_Port GPIOE
#define QSPI_CLK_Pin GPIO_PIN_10
#define QSPI_CLK_GPIO_Port GPIOE
#define QSPI_CS_Pin GPIO_PIN_11
#define QSPI_CS_GPIO_Port GPIOE
#define QSPI_D0_Pin GPIO_PIN_12
#define QSPI_D0_GPIO_Port GPIOE
#define QSPI_D1_Pin GPIO_PIN_13
#define QSPI_D1_GPIO_Port GPIOE
#define QSPI_D2_Pin GPIO_PIN_14
#define QSPI_D2_GPIO_Port GPIOE
#define QSPI_D3_Pin GPIO_PIN_15
#define QSPI_D3_GPIO_Port GPIOE
#define MEMS_SCK_Pin GPIO_PIN_1
#define MEMS_SCK_GPIO_Port GPIOD
#define MEMS_MOSI_Pin GPIO_PIN_4
#define MEMS_MOSI_GPIO_Port GPIOD
#define USART_TX_Pin GPIO_PIN_5
#define USART_TX_GPIO_Port GPIOD
#define USART_RX_Pin GPIO_PIN_6
#define USART_RX_GPIO_Port GPIOD
#define XL_CS_Pin GPIO_PIN_0
#define XL_CS_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define AUDIO_SAIx                           SAI1_Block_A
#define AUDIO_SAIx_CLK_ENABLE()              __HAL_RCC_SAI1_CLK_ENABLE()

#define AUDIO_SAIx_FS_GPIO_PORT              GPIOE
#define AUDIO_SAIx_FS_AF                     GPIO_AF13_SAI1
#define AUDIO_SAIx_FS_PIN                    GPIO_PIN_4
#define AUDIO_SAIx_SCK_GPIO_PORT             GPIOE
#define AUDIO_SAIx_SCK_AF                    GPIO_AF13_SAI1
#define AUDIO_SAIx_SCK_PIN                   GPIO_PIN_5
#define AUDIO_SAIx_SD_GPIO_PORT              GPIOE
#define AUDIO_SAIx_SD_AF                     GPIO_AF13_SAI1
#define AUDIO_SAIx_SD_PIN                    GPIO_PIN_6
#define AUDIO_SAIx_MCLK_GPIO_PORT            GPIOE
#define AUDIO_SAIx_MCLK_AF                   GPIO_AF13_SAI1
#define AUDIO_SAIx_MCLK_PIN                  GPIO_PIN_2

#define AUDIO_SAIx_MCLK_ENABLE()             __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_SAIx_SCK_ENABLE()              __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_SAIx_FS_ENABLE()               __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_SAIx_SD_ENABLE()               __HAL_RCC_GPIOE_CLK_ENABLE()
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
