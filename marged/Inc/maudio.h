/*
 * maudio.h
 *
 *  Created on: 21.05.2019
 *      Author: sylwekk12
 */

#ifndef MAUDIO_H_
#define MAUDIO_H_
#include "audio.h"
#include "cs43l22.h"
#include "stm32l4xx_hal_sai.h"

#define PLAY_BUFF_SIZE 		 128
#define AUDIO_FILE_ADDRESS   0x08080000
#define AUDIO_FILE_SIZE      (180*1024)
#define PLAY_HEADER          0x2C
#define AUDIO_I2C_ADDRESS                       ((uint16_t) 0x94)

void Playback_Init(void);
void HAL_SAI_MspInit2(SAI_HandleTypeDef *hsai);


#endif /* MAUDIO_H_ */
