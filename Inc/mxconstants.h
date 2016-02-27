/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RESET_CMX7262_Pin GPIO_PIN_13
#define RESET_CMX7262_GPIO_Port GPIOC
#define IRQ_CMX7262_Pin GPIO_PIN_14
#define IRQ_CMX7262_GPIO_Port GPIOC
#define SPI1_CS_AD5601_Pin GPIO_PIN_4
#define SPI1_CS_AD5601_GPIO_Port GPIOA
#define IRQ_CC1120_Pin GPIO_PIN_0
#define IRQ_CC1120_GPIO_Port GPIOB
#define TR_SKY_Pin GPIO_PIN_1
#define TR_SKY_GPIO_Port GPIOB
#define EN_SKY_Pin GPIO_PIN_2
#define EN_SKY_GPIO_Port GPIOB
#define RESET_CC1120_Pin GPIO_PIN_11
#define RESET_CC1120_GPIO_Port GPIOB
#define SPI2_CS_CC1120_Pin GPIO_PIN_12
#define SPI2_CS_CC1120_GPIO_Port GPIOB
#define TNG_Pin GPIO_PIN_8
#define TNG_GPIO_Port GPIOA
#define BYP_SKY_Pin GPIO_PIN_3
#define BYP_SKY_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOB
#define SPI1_CS_CMX7262_Pin GPIO_PIN_7
#define SPI1_CS_CMX7262_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define CC1120_START    HAL_GPIO_WritePin(RESET_CC1120_GPIO_Port, RESET_CC1120_Pin, GPIO_PIN_SET)
#define CC1120_RESET		HAL_GPIO_WritePin(RESET_CC1120_GPIO_Port, RESET_CC1120_Pin, GPIO_PIN_RESET)

#define LED1_ON					HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_OFF				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)

#define LED3_ON					HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_OFF				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
