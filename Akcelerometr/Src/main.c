/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "lcd.h"
#include "quadspi.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define LSM303_ACC_ADDRESS (0x1D << 1) // adres akcelerometru: 0011 001x
#define LSM303_ACC_CTRL_REG1_A 0x20 // rejestr ustawien 1

// CTRL_REG1_A = [HR][ODR2][ODR1][ODR0][BDU][ZEN][YEN][XEN]
#define LSM303_ACC_Z_ENABLE 0x07 // 0000 0100
#define LSM303_ACC_100HZ 0x38 // 0011 0000
#define LSM303_ACC_Z_H_A 0x2D // wyzszy bajt danych osi Z

// wypelnienie zmiennej konfiguracyjnej odpowiednimi opcjami
uint8_t Settings = LSM303_ACC_Z_ENABLE | LSM303_ACC_100HZ;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t spiTxBuf[2];//tablica danych wysylanych do urzadzenia
uint8_t spiRxBuf[2];//tablica danych odbieranych z urzadzenia
uint8_t spiTxBuf23[2];//tablica dla CTRL_REG4_A
//uint8_t Data = 0; // Zmienna do bezposredniego odczytu z akcelerometru
//int16_t Zaxis = 0; // Zawiera przeksztalcona forme odczytanych danych

//SPI_HandleTypeDef spi;
/*
void mcp_write_reg(uint8_t addr, uint8_t value)
{
 uint8_t tx_buf[] = { LSM303_ACC_ADDRESS , addr, value };

 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
 HAL_SPI_TransmitReceive(&spi, tx_buf, &Data, 3, HAL_MAX_DELAY);
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//uint8_t to_send;

/*	 spi.Instance = SPI2;
	 spi.Init.Mode = SPI_MODE_MASTER;
	 spi.Init.NSS = SPI_NSS_SOFT;
	 spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 1MHz
	 spi.Init.Direction = SPI_DIRECTION_2LINES;
	 spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	 spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	 spi.Init.DataSize = SPI_DATASIZE_8BIT;
	 spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	 spi.Init.TIMode = SPI_TIMODE_DISABLE;
	 spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	 spi.Init.CRCPolynomial = 7;
	HAL_SPI_Init(&spi);

	__HAL_SPI_ENABLE(&spi);

	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
	 //spi_sendrecv(0x40);
	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);*/
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_LCD_Init();
  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  //HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &Settings, 1, 100);
  //uint8_t adr = 0x2D;


  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
    HAL_Delay(10);
  //Wysylanie danych do ACC
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
    spiTxBuf[0] = 0x20; //wlaczenie rejestru numer 1
    spiTxBuf[1] = 0x11; //wlaczenie na czestotliwosci na ACC i odczyt z osi X
    HAL_SPI_Transmit(&hspi2,spiTxBuf,2,50);
    spiTxBuf23[0] = 0x23;
    spiTxBuf23[1] = 0x01;
    HAL_SPI_Transmit(&hspi2,spiTxBuf23,2,50);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);


   //Odczyt danych z ACC
   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
   spiTxBuf[0] = 0x20|0x80; //wlaczenie odczytywanie, ustawienie Most Significant Bit to High
   HAL_SPI_Transmit(&hspi2,spiTxBuf,1,50);
   HAL_SPI_Receive(&hspi2,spiRxBuf,1,50);
   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*	  HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, (LSM303_ACC_Z_H_A), 1, &Data, 1, 100);
	  Zaxis = Data << 8;*/
    /* USER CODE END WHILE */
	 /* HAL_SPI_TransmitReceive(&spi, &adr, &Data, 3, HAL_MAX_DELAY);
	  Zaxis = Data << 8;*/
	  MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
	  //to_send = 0x8F;
	  //Odczyt danych z ACC


	        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	        spiTxBuf[0] = 0x29|0x80; //wlaczenie odczyt z X na High
	       // HAL_SPI_Transmit(&hspi2,&to_send,1,50);
	        HAL_SPI_Transmit(&hspi2,spiTxBuf,1,50);
	        HAL_SPI_Receive(&hspi2,spiRxBuf,1,50);
	        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);

	        HAL_Delay(300);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
