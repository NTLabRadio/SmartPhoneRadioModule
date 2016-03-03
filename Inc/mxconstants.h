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

// Светодиоды

#define LEDS_OFF()						{LED1_OFF(); LED2_OFF(); LED3_OFF();}

#define LED1_ON()							HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_OFF()						HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)

//Светодиод с инверсией
#define LED2_ON()							HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF()						HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)

#define LED3_ON()							HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_OFF()						HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)


// SKY

#define	SKY_TR_HIGH()					HAL_GPIO_WritePin(TR_SKY_GPIO_Port, TR_SKY_Pin, GPIO_PIN_SET);
#define	SKY_TR_LOW()					HAL_GPIO_WritePin(TR_SKY_GPIO_Port, TR_SKY_Pin, GPIO_PIN_RESET);

#define	SKY_EN_HIGH()					HAL_GPIO_WritePin(EN_SKY_GPIO_Port, EN_SKY_Pin, GPIO_PIN_SET);
#define	SKY_EN_LOW()					HAL_GPIO_WritePin(EN_SKY_GPIO_Port, EN_SKY_Pin, GPIO_PIN_RESET);

#define	SKY_BYP_HIGH()				HAL_GPIO_WritePin(BYP_SKY_GPIO_Port, BYP_SKY_Pin, GPIO_PIN_SET);
#define	SKY_BYP_LOW()					HAL_GPIO_WritePin(BYP_SKY_GPIO_Port, BYP_SKY_Pin, GPIO_PIN_RESET);


// Тангента

#define PTT_PRESSED()					(!HAL_GPIO_ReadPin(TNG_GPIO_Port, TNG_Pin))


// AD5601

#define AD5601_SPI_CS_PIN              	SPI1_CS_AD5601_Pin         	/* PA.04 */
#define AD5601_SPI_CS_GPIO_PORT        	SPI1_CS_AD5601_GPIO_Port   	/* GPIOA */

#define AD5601_CSN_LOW()      HAL_GPIO_WritePin(SPI1_CS_AD5601_GPIO_Port, SPI1_CS_AD5601_Pin, GPIO_PIN_RESET)
#define AD5601_CSN_HIGH()     HAL_GPIO_WritePin(SPI1_CS_AD5601_GPIO_Port, SPI1_CS_AD5601_Pin, GPIO_PIN_SET)


// CMX7262

#define CMX7262_SPI_CS_PIN              	SPI1_CS_CMX7262_Pin        	/* PB.07 */
#define CMX7262_SPI_CS_GPIO_PORT        	SPI1_CS_CMX7262_GPIO_Port  	/* GPIOB */

#define CMX7262_SPI_SCK_PIN               GPIO_PIN_5                  /* PA.05 */
#define CMX7262_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */

#define CMX7262_SPI_MISO_PIN              GPIO_PIN_6                  /* PA.06 */
#define CMX7262_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */

#define CMX7262_SPI_MOSI_PIN              GPIO_PIN_7                  /* PA.07 */
#define CMX7262_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */

#define CMX7262_IRQN_PIN              		IRQ_CMX7262_Pin             /* PC.14 */
#define CMX7262_IRQN_PORT        					IRQ_CMX7262_GPIO_Port       /* GPIOC */

#define CMX7262_START					HAL_GPIO_WritePin(RESET_CMX7262_GPIO_Port, RESET_CMX7262_Pin, GPIO_PIN_SET)
#define CMX7262_RESET					HAL_GPIO_WritePin(RESET_CMX7262_GPIO_Port, RESET_CMX7262_Pin, GPIO_PIN_RESET)

#define CMX7262_CSN_LOW()     HAL_GPIO_WritePin(CMX7262_SPI_CS_GPIO_PORT, CMX7262_SPI_CS_PIN, GPIO_PIN_RESET)
#define CMX7262_CSN_HIGH()    HAL_GPIO_WritePin(CMX7262_SPI_CS_GPIO_PORT, CMX7262_SPI_CS_PIN, GPIO_PIN_SET)


// CC1120

#define CC1120_SPI_CS_PIN              	SPI2_CS_CC1120_Pin         		/* PB.12 */
#define CC1120_SPI_CS_GPIO_PORT        	SPI2_CS_CC1120_GPIO_Port  		/* GPIOB */

#define CC1120_SPI_SCK_PIN               	GPIO_PIN_13                 /* PB.13 */
#define CC1120_SPI_SCK_GPIO_PORT         	GPIOB                       /* GPIOB */

#define CC1120_SPI_MISO_PIN              	GPIO_PIN_14                 /* PB.14 */
#define CC1120_SPI_MISO_GPIO_PORT        	GPIOB                       /* GPIOB */

#define CC1120_SPI_MOSI_PIN              	GPIO_PIN_15                 /* PB.15 */
#define CC1120_SPI_MOSI_GPIO_PORT        	GPIOB                       /* GPIOB */

#define CC1120_IRQN_PIN              			IRQ_CC1120_Pin              /* PB.00 */
#define CC1120_IRQN_PORT        					IRQ_CC1120_GPIO_Port        /* GPIOB */

#define CC1120_START()  			HAL_GPIO_WritePin(RESET_CC1120_GPIO_Port, RESET_CC1120_Pin, GPIO_PIN_SET)
#define CC1120_RESET()				HAL_GPIO_WritePin(RESET_CC1120_GPIO_Port, RESET_CC1120_Pin, GPIO_PIN_RESET)

#define CC1120_CSN_LOW()     	HAL_GPIO_WritePin(CC1120_SPI_CS_GPIO_PORT, CC1120_SPI_CS_PIN, GPIO_PIN_RESET)
#define CC1120_CSN_HIGH()			HAL_GPIO_WritePin(CC1120_SPI_CS_GPIO_PORT, CC1120_SPI_CS_PIN, GPIO_PIN_SET)

#define CC1120_RESET_HIGH()		HAL_GPIO_WritePin(RESET_CC1120_GPIO_Port, RESET_CC1120_Pin, GPIO_PIN_SET);


/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
