/*
 * maudio.c
 *
 *  Created on: 21.05.2019
 *      Author: sylwekk12
 */
#include "main.h"
#include "maudio.h"

DMA_HandleTypeDef hSaiDma;

SAI_HandleTypeDef hsai_BlockA1;

AUDIO_DrvTypeDef            *audio_drv;

void Playback_Init(void)
{
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

  /* Configure and enable PLLSAI1 clock to generate 11.294MHz */
  RCC_PeriphCLKInitStruct.PeriphClockSelection    = RCC_PERIPHCLK_SAI1;
  RCC_PeriphCLKInitStruct.PLLSAI1.PLLSAI1Source   = RCC_PLLSOURCE_MSI;
  RCC_PeriphCLKInitStruct.PLLSAI1.PLLSAI1M        = 1;
  RCC_PeriphCLKInitStruct.PLLSAI1.PLLSAI1N        = 24;
  RCC_PeriphCLKInitStruct.PLLSAI1.PLLSAI1P        = 17;
  RCC_PeriphCLKInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  RCC_PeriphCLKInitStruct.Sai1ClockSelection      = RCC_SAI1CLKSOURCE_PLLSAI1;

  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize SAI */
  __HAL_SAI_RESET_HANDLE_STATE(&hsai_BlockA1);

  hsai_BlockA1.Instance = AUDIO_SAIx;

  __HAL_SAI_DISABLE(&hsai_BlockA1);

  hsai_BlockA1.Init.AudioMode      = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.Synchro        = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockA1.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_22K;
  hsai_BlockA1.Init.Protocol       = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.DataSize       = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.Mckdiv         = 0; /* N.U */
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState       = SAI_OUTPUT_NOTRELEASED;

  hsai_BlockA1.FrameInit.FrameLength       = 32;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 16;
  hsai_BlockA1.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber     = 2;
  hsai_BlockA1.SlotInit.SlotActive     = (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1);

  if(HAL_OK != HAL_SAI_Init(&hsai_BlockA1))
  {
    Error_Handler();
  }

  /* Enable SAI to generate clock used by audio driver */
  __HAL_SAI_ENABLE(&hsai_BlockA1);

  /* Initialize audio driver */
  if(CS43L22_ID != cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS))
  {
    Error_Handler();
  }

  audio_drv = &cs43l22_drv;
  audio_drv->Reset(AUDIO_I2C_ADDRESS);
  if(0 != audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 50, AUDIO_FREQUENCY_22K))
  {
    Error_Handler();
  }
}

void HAL_SAI_MspInit2(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  GPIO_Init;

  /* Enable SAI1 clock */
  __HAL_RCC_SAI1_CLK_ENABLE();

  /* Configure GPIOs used for SAI1 */
  AUDIO_SAIx_MCLK_ENABLE();
  AUDIO_SAIx_SCK_ENABLE();
  AUDIO_SAIx_FS_ENABLE();
  AUDIO_SAIx_SD_ENABLE();

  GPIO_Init.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init.Pull      = GPIO_NOPULL;
  GPIO_Init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_Init.Alternate = AUDIO_SAIx_FS_AF;
  GPIO_Init.Pin       = AUDIO_SAIx_FS_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_FS_GPIO_PORT, &GPIO_Init);
  GPIO_Init.Alternate = AUDIO_SAIx_SCK_AF;
  GPIO_Init.Pin       = AUDIO_SAIx_SCK_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_SCK_GPIO_PORT, &GPIO_Init);
  GPIO_Init.Alternate = AUDIO_SAIx_SD_AF;
  GPIO_Init.Pin       = AUDIO_SAIx_SD_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_SD_GPIO_PORT, &GPIO_Init);
  GPIO_Init.Alternate = AUDIO_SAIx_MCLK_AF;
  GPIO_Init.Pin       = AUDIO_SAIx_MCLK_PIN;
  HAL_GPIO_Init(AUDIO_SAIx_MCLK_GPIO_PORT, &GPIO_Init);

  /* Configure DMA used for SAI1 */
  __HAL_RCC_DMA2_CLK_ENABLE();

  if(hsai->Instance == AUDIO_SAIx)
  {

    hSaiDma.Init.Request             = DMA_REQUEST_1;
    hSaiDma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hSaiDma.Init.PeriphInc           = DMA_PINC_DISABLE;
    hSaiDma.Init.MemInc              = DMA_MINC_ENABLE;
    hSaiDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hSaiDma.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hSaiDma.Init.Mode                = DMA_CIRCULAR;
    hSaiDma.Init.Priority            = DMA_PRIORITY_HIGH;

    /* Select the DMA instance to be used for the transfer : DMA2_Channel1 */
    hSaiDma.Instance                 = DMA2_Channel1;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hSaiDma);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hSaiDma);

    /* Configure the DMA channel */
    if (HAL_OK != HAL_DMA_Init(&hSaiDma))
    {
      Error_Handler();
    }
  }

  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0x01, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
}
