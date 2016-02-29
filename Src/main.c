/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "globals.h"
#include "cc1120.h"
#include "cmx7262.h"
#include "mathfuncs.h"
#include "RadioModule.h"
#include "SPIMLogic.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern UART_InitTypeDef DefaultUARTParams;

extern en_UARTstates UARTstate;

//����� ���� ������ �������������� ������ �� ���� ���������� � �������� �����������
RadioModule *pobjRadioModule;

CMX7262_TypeDef  g_CMX7262Struct;
CC1120_TypeDef  g_CC1120Struct;

uint8_t g_flCMX7262_IRQ_CHECKED = FALSE;
uint8_t g_flCC1120_IRQ_CHECKED = FALSE;

#define MAX_SIZE_OF_DATA_FROM_CMX7262 (2048)
uint8_t pDataFromCMX7262[MAX_SIZE_OF_DATA_FROM_CMX7262];
uint16_t nLengthDataFromCMX7262 = 0;

#define MAX_SIZE_OF_DATA_TO_CMX7262 (4096)
uint8_t pDataToCMX7262[MAX_SIZE_OF_DATA_TO_CMX7262];
uint16_t nLengthDataToCMX7262 = 0;

//������������ �������� ������ ������ ����������� ������, ��
#define CMX7262_BUFFER_DURATION_MS (60)

//����� ������� ������ ��������, ������������� ����������� ������ ��� ���������������� ��������
#define NUM_CMX7262_BUFFERS_INITACCUM_FOR_TX	(6)		//60 �� x 6 = 360 ��

//����� ������� ������ �������� � ����� �����������
#define NUM_CMX7262_BUFFERS_IN_RADIOPACK	(3)				//60 �� x 3 = 180 ��

//������ ����������� � ������ �������� ������, ������ ������� ������
#define RADIOPACK_VOICEMODE_SIZE 	(NUM_CMX7262_BUFFERS_IN_RADIOPACK*CMX7262_CODEC_BUFFER_SIZE)

//������ ������������ ����������� � ������ �������� ������, �� ���������� ������� � ���������� � �������
#define RADIOPACK_VOICEMODE_EXTSIZE	(90)

#define MAX_RADIOPACK_SIZE	(128)

//������ ������������ ������ ��� ��������
uint8_t RadioPackForSend[RADIOPACK_VOICEMODE_EXTSIZE];

//������ ��������� �����������
uint8_t RadioPackRcvd[MAX_RADIOPACK_SIZE];


#ifdef DEBUG_CMX7262_CNT_TX_AUDIO_BUF
uint16_t cntCMX7262TxAudioBuf = 0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void RadioModuleInit(void);
void RadioModuleDeInit(void);

void ProcessPTTState(void);

void ProcessRadioState(void);

void CMX7262_TestMode(void);
void ProcessCMX7262State(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	
	// ������������� CS ������������ ��������� � ������� ���������
	CC1120_CSN_HIGH();
	CMX7262_CSN_HIGH();
	
	// Pin Reset CC1120 ������������� � ������� ��������� ��� �������� ���������� � �������� ���������
	CC1120_RESET_HIGH();

	//��������� ����������
	LEDS_OFF;

	//��������� ������� ��� ������ � ����������
	StartPeriphTimers();

	// �������������� ������ �� UART
	UART_InitInterface(&huart1);

	//�������������� ���, ��� ���������� ��� ��������� ������������� ������ SPIM
	SPIMInit();

	#ifdef DEBUG_CHECK_PERIPH_MODULES_ON_STARTUP	//�������� ����������������� ������������ �������
	//������ ���������� ����� CC1120
	CC1120_HardwareReset();
	//��������� �����������������
	CC1120_CheckModule(&hspi2);
	
	//���������� ����� CMX7262
	CMX7262_HardwareReset();
	//��������� �����������������
	CMX7262_CheckModule(&hspi1);
	#endif


	//������ ������������� ����������� ��� ����������� ���������� ��� �������� � �����������
	RadioModuleInit();

	#ifdef TEST_CMX7262
	CMX7262_TestMode();
	#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	#ifdef DEBUG_USE_LEDS
	LED1_ON;
	#endif
	
  while (1)
  {
		//���� �� UART ������� ������
		if(UARTstate==UART_DATA_RX_NEED_TO_PROCESS)
		{
			//������������ ��
			ProcessDataFromExtDev();
			
			//���������, ��� ������ ����������
			UARTstate = UART_IDLE;
		}
		
		//���� ���� ���������� �� CMX7262
		if(g_flCMX7262_IRQ_CHECKED)
		{
			//������������ ����������: ���������, ��� ����� CMX7262                                                                                       
			CMX7262_IRQ(&g_CMX7262Struct);
			//���������� ����, ����� ���������� ��������� ����������
			g_flCMX7262_IRQ_CHECKED = FALSE;
		}
		
		//��������� ��������� ������ CMX7262: ��������/�����/����
		ProcessCMX7262State();
		
		#ifndef TEST_CMX7262
		//������������ ��������
		ProcessPTTState();
		
		ProcessRadioState();
		#endif
	  
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;							// ����� ������: ������������� full duplex
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;									// ������ ������ - 8 ���
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;										// ������������� �� ������� ������
  hspi1.Init.NSS = SPI_NSS_SOFT;														// ����������� CS
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;	// ������������ ������� SPI: 48���/32 = 1.5 ���
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;										// ������� ��� - ������
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;	// CRC �� �����������
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;							// NSS pulse mode ��������
  HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;							// ����� ������: ������������� full duplex
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;									// ������ ������ - 8 ���
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;										// ������������� �� ������� ������
  hspi2.Init.NSS = SPI_NSS_SOFT;														// ����������� CS
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;	// ������������ ������� SPI: 48���/128 = 375 ���
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;										// ������� ��� - ������
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;	// CRC �� �����������
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;							// NSS pulse mode ��������
  HAL_SPI_Init(&hspi2);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
	htim1.Init.Prescaler = (uint16_t) ((SystemCoreClock)/1e5 - 1);	//���� ������� - � 10 ���	
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* ������� TIM2 � TIM3 ��������� ������������ (�������� - 1���) 32-������ ������ ��� ������ ���������
���������� � ���������� �������������� ������� (�������� ����� �� SPI, UART � �.�.) */

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (uint16_t) ((SystemCoreClock) / 1e6) - 1;		//���� ������� - � 1 ���
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

	//������ ������������ ��� master � ��������� ������� �������� ��� ���������� 32-������� �������
	//������� ��������� TIM3. ��������� ��� ���� ������ ������� �� ��������� �������	
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

	//������ ������������ ��� slave � ��������� ������� �������� ��� ���������� 32-������� �������
	//�� ������������ �������� master-������� (TIM2) ���������������� ������� ������� slave-�������
	//���������� ITR1 ������������ ��� ������� ������ ��� ����������������� �������� ��������	
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM15 init function */
void MX_TIM15_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim15.Instance = TIM15;
	htim15.Init.Prescaler = (uint16_t) ((SystemCoreClock)/1e5 - 1);	//���� ������� - � 10 ���	
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim15);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : RESET_CMX7262_Pin */
  GPIO_InitStruct.Pin = RESET_CMX7262_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(RESET_CMX7262_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_CMX7262_Pin */
  GPIO_InitStruct.Pin = IRQ_CMX7262_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_CMX7262_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_AD5601_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_AD5601_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(SPI1_CS_AD5601_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_CC1120_Pin */
  GPIO_InitStruct.Pin = IRQ_CC1120_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_CC1120_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TR_SKY_Pin EN_SKY_Pin BYP_SKY_Pin */
  GPIO_InitStruct.Pin = TR_SKY_Pin|EN_SKY_Pin|BYP_SKY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_CC1120_Pin SPI2_CS_CC1120_Pin LED1_Pin LED2_Pin 
                           LED3_Pin SPI1_CS_CMX7262_Pin */
  GPIO_InitStruct.Pin = RESET_CC1120_Pin|SPI2_CS_CC1120_Pin|LED1_Pin|LED2_Pin 
                          |LED3_Pin|SPI1_CS_CMX7262_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TNG_Pin */
  GPIO_InitStruct.Pin = TNG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TNG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void RadioModuleInit()
{
	//������������� CMX7262: �������� ������ � ������, ��������� ���������
	CMX7262_Init(&g_CMX7262Struct, &hspi1);

	//������� CMX7262 � ����� Idle
	CMX7262_Idle(&g_CMX7262Struct);	
	
	//������������� ��1120
	CC1120_Init(&g_CC1120Struct, &hspi2);
	
	//����� ����, ��� ��� ������������ ���������� ���������� ���������, ������� ������
	//��� ���������� ������ ����������� �����������
	pobjRadioModule = new RadioModule;	
}

void RadioModuleDeInit()
{
	delete pobjRadioModule;
}


//������� ��������� ��������� ��������
//TODO �� ���������� �����������
void ProcessPTTState()
{
	//���� ������ �������
	if(!HAL_GPIO_ReadPin(TNG_GPIO_Port, TNG_Pin))
	{
		//��������� ��������� �����������
		//���� �� ��� ��� �� ��������� � ��������
		if(pobjRadioModule->GetRadioChanState() != RADIOCHAN_STATE_TRANSMIT)
		{
			//��������� � ��������
			pobjRadioModule->SetRadioChanState(RADIOCHAN_STATE_TRANSMIT);
			
			pobjRadioModule->RadioModuleState = RADIOMODULE_STATE_TX_WAITING;
			
			//������� �������� � ����� �����������
			CMX7262_Encode(&g_CMX7262Struct);
			
			nLengthDataFromCMX7262 = 0;
			nLengthDataToCMX7262 = 0;
		}
	}
	else
	{
		//��������� ��������� �����������
		//���� �� ��� ��� �� ��������� � ������
		if( (pobjRadioModule->GetRadioChanState() != RADIOCHAN_STATE_WAIT_RECEIVE) &&
				(pobjRadioModule->GetRadioChanState() != RADIOCHAN_STATE_RECEIVE) )
		{
			//��������� � �����
			pobjRadioModule->SetRadioChanState(RADIOCHAN_STATE_WAIT_RECEIVE);
			
			pobjRadioModule->RadioModuleState = RADIOMODULE_STATE_RX_WAITING;
			
			//������� �������� � ����� �������������
			CMX7262_Decode(&g_CMX7262Struct);
			
			#ifndef TEST_RADIO_IMITATE
			nLengthDataToCMX7262 = 0;
			#endif
			nLengthDataFromCMX7262 = 0;
		}
	}
}

void RadioImitator_TxData(uint8_t* pPackData, uint16_t packSize)
{
	if(nLengthDataToCMX7262 <= MAX_SIZE_OF_DATA_TO_CMX7262-packSize)
	{
		memcpy(&pDataToCMX7262[nLengthDataToCMX7262],pPackData,packSize);
								
		nLengthDataToCMX7262+=packSize;		
	}
	
	g_flCC1120_IRQ_CHECKED = TRUE;
}



void ProcessRadioState()
{
	switch(pobjRadioModule->GetRadioChanState())
	{
		case RADIOCHAN_STATE_TRANSMIT:

			switch(pobjRadioModule->RadioModuleState)
			{		
				case RADIOMODULE_STATE_TX_WAITING:
					//1. ���������, ��� �� ���������� � ������� ����� ������ � �������� - CMX7262_ODA (�������� � ProcessCMX7262State())

					//2. ���� ����, �� �������� ������ � ���������� � ������� �� ��������:	(�������� � ProcessCMX7262State())
					//CMX7262_RxFIFO(&pCmx7262,(uint8_t *)&aPayLoad[0]);

					//3. ���� �������� ���������� �������� ������, ������������� � ����� RADIOMODULE_STATE_TX_RUNNING
					if(nLengthDataFromCMX7262 > NUM_CMX7262_BUFFERS_INITACCUM_FOR_TX * CMX7262_CODEC_BUFFER_SIZE)
					{
						pobjRadioModule->RadioModuleState = RADIOMODULE_STATE_TX_RUNNING;
						g_CC1120Struct.TxState = CC1120_TX_STATE_WAIT;						
					}
						
					break;

				case RADIOMODULE_STATE_TX_RUNNING:
					//0. ���� ���� ���������� � ���, ��� ����� �������
					if(g_flCC1120_IRQ_CHECKED)
					{
						//���������� ���� ����������
						g_flCC1120_IRQ_CHECKED = FALSE;

						g_CC1120Struct.TxState = CC1120_TX_STATE_WAIT;
					}
				
					//1. ���������, ��� �� ���������� � ������� ����� ������ � �������� - CMX7262_ODA	(�������� � ProcessCMX7262State())

					//2. ���� ����, �� �������� ������ � ���������� � ������� �� ��������:	(�������� � ProcessCMX7262State())
					//CMX7262_RxFIFO(&pCmx7262,(uint8_t *)&aPayLoad[0]);

					//3. ���� � ������� �� �������� ���������� ������ ��� ������������ ������ ������ 
					//� ���������� CC1120 ��������, �� �������� �� � ��1120
					if((nLengthDataFromCMX7262 >= RADIOPACK_VOICEMODE_SIZE) &&
						 (g_CC1120Struct.TxState!=CC1120_TX_STATE_ACTIVE))
					{
						//CC1120_TxData(&g_CC1120Struct, pDataFromCMX7262, RADIOPACK_VOICEMODE_SIZE);
						
						//TODO ���������� ������ ��� �������� ������� � ��������� �������
						//1� ���� - �����. ������������� �����������������
						RadioPackForSend[0] = 0;
						//�� ������� ��������� ������� ������
						memcpy(RadioPackForSend+1,pDataFromCMX7262,RADIOPACK_VOICEMODE_SIZE);
						
						#ifdef TEST_RADIO_TX_ONLY_ONES
						memset(RadioPackForSend,0,RADIOPACK_VOICEMODE_EXTSIZE);
						#endif
						
						#ifndef TEST_RADIO_IMITATE
						//���������� ������ �� CC1120
						CC1120_TxData(&g_CC1120Struct, RadioPackForSend, RADIOPACK_VOICEMODE_EXTSIZE);
						#else
						RadioImitator_TxData(RadioPackForSend+1, RADIOPACK_VOICEMODE_SIZE);
						#endif
						
						g_CC1120Struct.TxState = CC1120_TX_STATE_ACTIVE;
						
						memmove(pDataFromCMX7262,pDataFromCMX7262+RADIOPACK_VOICEMODE_SIZE,nLengthDataFromCMX7262-RADIOPACK_VOICEMODE_SIZE);
						nLengthDataFromCMX7262-=RADIOPACK_VOICEMODE_SIZE;
						memset(pDataFromCMX7262+nLengthDataFromCMX7262,0,RADIOPACK_VOICEMODE_SIZE);
					}

					break;
			}

			break;
		case RADIOCHAN_STATE_RECEIVE:
		case RADIOCHAN_STATE_WAIT_RECEIVE:

			switch(pobjRadioModule->RadioModuleState)
			{
				case RADIOMODULE_STATE_RX_WAITING:
					//�������� � ������� ���� �����, ����� �� ����������������� �����������, ��� ��� ����� ������
					#ifndef TEST_RADIO_IMITATE
					memset(pDataToCMX7262,0,MAX_SIZE_OF_DATA_TO_CMX7262);
					#endif
					CMX7262_TxFIFO(&g_CMX7262Struct,(uint8_t *)&pDataToCMX7262[0]);
					CMX7262_TxFIFO(&g_CMX7262Struct,(uint8_t *)&pDataToCMX7262[0]);
				
					//�������� ���� ���������������
					WaitTimeMCS(2*CMX7262_BUFFER_DURATION_MS*1e3);
				
					//������� Rx FIFO
					CC1120_RxFIFOFlush(g_CC1120Struct.hSPI);
					//������� ���������� � ����� ������
					CC1120_Rx(g_CC1120Struct.hSPI);
				
					pobjRadioModule->RadioModuleState = RADIOMODULE_STATE_RX_RUNNING;
					break;
		
				case RADIOMODULE_STATE_RX_RUNNING:
					//1. ���������, ��� �� ���������� flCC1120_IRQ_CHECKED � ���, ��� CC1120 ������ ������
					if(g_flCC1120_IRQ_CHECKED)
					{
						uint16_t nSizeOfRecData = CC1120_RxFIFONumBytes(g_CC1120Struct.hSPI);
					
					//2. ���� ���� ����������, �������� ������ �� CC1120. ���������� �� � ������� ��� ��������, ���� � ��� ���� �����
						if(nLengthDataToCMX7262 <= MAX_SIZE_OF_DATA_TO_CMX7262-nSizeOfRecData)
						{
							CC1120_RxData(&g_CC1120Struct,RadioPackRcvd,&nSizeOfRecData);
							
							#ifndef DEBUG_CMX7262_RECPACK_WO_ADDR
							//�������� ������ ��������� ����������� � ������� ��� �������� � ������ ����, ��� ������ ���� - �����
							//�������� ������ ���������� ������. ��������� ���� ������ �� ����� �������, ������� ������� ���������, �� �������
							memcpy(&pDataToCMX7262[nLengthDataToCMX7262],RadioPackRcvd+1,RADIOPACK_VOICEMODE_SIZE);
							#else
							memcpy(&pDataToCMX7262[nLengthDataToCMX7262],RadioPackRcvd,RADIOPACK_VOICEMODE_SIZE);
							#endif
							
							nLengthDataToCMX7262+=RADIOPACK_VOICEMODE_SIZE;
						}
			
					//3. ���������� ���� ���������� 
						g_flCC1120_IRQ_CHECKED = FALSE;
					}

					//4. ���� � ������� ��� �������� ���� ���������� ������, ���������� �� � CMX7262:	(�������� � ProcessCMX7262State())
					break;
			}
			break;
		default:
			break;
	}
}

void CMX7262_TestMode()
{
	#ifdef TEST_CMX7262_ENCDEC_AUDIO2AUDIO_MODE
	CMX7262_EncodeDecode_Audio(&g_CMX7262Struct);	
	#endif

	#ifdef TEST_CMX7262_AUDIO_TESTMODE
	CMX7262_Test_AudioOut(&g_CMX7262Struct);
	#endif
	
	#ifdef TEST_CMX7262_ENCDEC_AUDIO2CBUS_MODE
	CMX7262_EncodeDecode_Audio2CBUS(&g_CMX7262Struct);
	#endif
	
	#ifdef TEST_CMX7262_ENCDEC_CBUS2AUDIO_MODE
		#ifdef TEST_CMX7262_ENCDEC_CBUS2AUDIO_INTERNAL_SIN
		CMX7262_EncodeDecode_CBUS2Audio(&g_CMX7262Struct);
		
		//��������� ����� ��������� �������� 1���
		FillBufByToneSignal((int16_t*)pDataToCMX7262,CMX7262_AUDIOFRAME_SIZE_SAMPLES,CMX7262_FREQ_SAMPLING,1000);
		CMX7262_TxFIFO_Audio(&g_CMX7262Struct,(uint8_t *)&pDataToCMX7262[0]);
		#endif
	#endif
	
	#ifdef TEST_CMX7262_ENC_MODE
	CMX7262_Encode(&g_CMX7262Struct);
	#endif
}

void ProcessCMX7262State()
{

	if((g_CMX7262Struct.uIRQRequest & CMX7262_ODA) == CMX7262_ODA)
	{
		//���������� ���� CMX7262_ODA
		g_CMX7262Struct.uIRQRequest = g_CMX7262Struct.uIRQRequest & ~CMX7262_ODA;
		
		//------- ������ ������ ----------
		#ifndef TEST_CMX7262	
		//������� �����
			//���� ������ ��� ���� ���� ������������
			if(nLengthDataFromCMX7262 <= MAX_SIZE_OF_DATA_FROM_CMX7262-CMX7262_CODEC_BUFFER_SIZE)
			{
				//�������� �� � CMX7262
				CMX7262_RxFIFO(&g_CMX7262Struct,(uint8_t *)&pDataFromCMX7262[nLengthDataFromCMX7262]);
				nLengthDataFromCMX7262 += CMX7262_CODEC_BUFFER_SIZE;
			}
			#ifdef DEBUG_PRINTF_CMX7262_EXCHANGE
			else
				printf("Bufer pDataFromCMX7262 is Full");
			#endif
		#else
		//�������� ������
			#ifdef TEST_CMX7262_ENC_MODE
			CMX7262_RxFIFO(&pCmx7262,(uint8_t *)&pDataFromCMX7262[0]);
			#endif
			#ifdef TEST_CMX7262_ENCDEC_AUDIO2CBUS_MODE
			CMX7262_RxFIFO_Audio(&g_CMX7262Struct,(uint8_t *)&pDataFromCMX7262[0]);			
			#endif
		#endif
	}

	if((g_CMX7262Struct.uIRQRequest & CMX7262_IDW) == CMX7262_IDW)
	{
		//�������� ������ �� CMX7262
		
		#ifndef TEST_CMX7262
		//������� �����
		#ifndef TEST_RADIO_IMITATE
		if(nLengthDataToCMX7262>=CMX7262_CODEC_BUFFER_SIZE)
		#else
		//� �������� ������ ������������� ���������, �� ������ �� ��������
		//��� ������� �������� ������ �� �������������
		if((nLengthDataToCMX7262>=CMX7262_CODEC_BUFFER_SIZE) && (HAL_GPIO_ReadPin(TNG_GPIO_Port, TNG_Pin)))
		#endif
		{
			CMX7262_TxFIFO(&g_CMX7262Struct,(uint8_t *)&pDataToCMX7262[0]);

			//TODO �������� pDataToCMX7262 �� ��������� �����
			memmove(pDataToCMX7262,pDataToCMX7262+CMX7262_CODEC_BUFFER_SIZE,nLengthDataToCMX7262-CMX7262_CODEC_BUFFER_SIZE);
			nLengthDataToCMX7262-=CMX7262_CODEC_BUFFER_SIZE;
			
			//���������� ���� CMX7262_IDW
			g_CMX7262Struct.uIRQRequest = g_CMX7262Struct.uIRQRequest & ~CMX7262_IDW;

		}
		#ifdef DEBUG_PRINTF_CMX7262_EXCHANGE
		else
			printf("Bufer pDataToCMX7262 is Empty");
		#endif
		
		#else
		//�������� ������
			#ifdef TEST_CMX7262_ENCDEC_CBUS2AUDIO_MODE			
				#ifdef TEST_CMX7262_ENCDEC_CBUS2AUDIO_INTERNAL_SIN
				//��������� ����� ��������� �������� 1���
				FillBufByToneSignal((int16_t*)pDataToCMX7262,CMX7262_AUDIOFRAME_SIZE_SAMPLES,CMX7262_FREQ_SAMPLING,1000);
				#endif
			
				//�������� ����� �� CMX7262
				CMX7262_TxFIFO_Audio(&g_CMX7262Struct,(uint8_t *)&pDataToCMX7262[0]);
				#ifdef DEBUG_CMX7262_CNT_TX_AUDIO_BUF
				cntCMX7262TxAudioBuf++;
				#endif
			
				uint16_t nSizeOfTxBuf = sizeof(uint16_t)*CMX7262_AUDIOFRAME_SIZE_SAMPLES;		
				if(nLengthDataToCMX7262>=nSizeOfTxBuf)
				{
					//TODO �������� pDataToCMX7262 �� ��������� �����
					memmove(pDataToCMX7262,pDataToCMX7262+nSizeOfTxBuf,nLengthDataToCMX7262-nSizeOfTxBuf);
					nLengthDataToCMX7262-=nSizeOfTxBuf;
				}
				#ifdef DEBUG_PRINTF_CMX7262_EXCHANGE
				else
					printf("Bufer pDataToCMX7262 is Empty");
				#endif
			#endif
		#endif
	}

}
		

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
