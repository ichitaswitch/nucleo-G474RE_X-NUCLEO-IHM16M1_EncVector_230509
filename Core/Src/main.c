/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <ctype.h>
#include "define.h"
#include "packetSerial.h"
#include <math.h>
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
uint16_t adc1Buf;
uint16_t adc2Buf[2];
uint8_t encEdgeDetect;
extern float omgDst;
static uint8_t brkErr;
float Tic, vVr;
uint8_t onOff;
static uint8_t rxBuf[256];
static uint16_t rxBtm = 0;
extern float vBus, omgFlt, iD, iQ, vDRef, vQRef;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern void pwmEnable(void);
extern void pwmDisable(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_COMP1_Init();
  MX_COMP4_Init();
  MX_COMP6_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_DAC3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  setbuf( stdout, NULL );
  setbuf( stdin, NULL );
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc1Buf, 1);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2Buf, 2);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_3);
  pwmDisable();
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
  HAL_DAC_Start(&hdac2, DAC1_CHANNEL_1);
  HAL_DAC_Start(&hdac3, DAC1_CHANNEL_2);
  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0xFFF);
  HAL_DAC_SetValue(&hdac2, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0xFFF);
  HAL_DAC_SetValue(&hdac3, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, 0xFFF);
  HAL_UART_Receive_DMA(&huart1, rxBuf, sizeof(rxBuf));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t tick = HAL_GetTick();
  while (1)
  {
    uint16_t rxTop = sizeof(rxBuf) - hlpuart1.hdmarx->Instance->CNDTR;

    if( rxBuf[rxTop] == 0 ){
      uint16_t rxSize = (rxTop-rxBtm)&(sizeof(rxBuf)-1);
      uint8_t packet[8];
      if( rxSize > sizeof(packet) ){
        rxBtm = rxTop;
        continue;
      }
      for( uint16_t i=0; i<rxSize; i++ ){
        packet[i] = rxBuf[rxBtm];
        rxBtm = (rxBtm + 1)&(sizeof(rxBuf)-1);
      }
      decodePacket( packet, rxSize );
      float omgBuf = *(float *)&packet[1] / 60.0f * (_MOTOR_POLE>>1) * _RAD(360.0f);
      if( omgBuf > 233.3f * _RAD(360.0f) ) omgBuf = 233.3f * _RAD(360.0f);
      else if( omgBuf < 0.0f ) omgBuf = 0.0f;
      omgDst = omgBuf;
    }
    
    if( HAL_GetTick() - tick > 1000 ){
      tick = HAL_GetTick();
      float vth = adc2Buf[0] * _REG2VOLT;
      float Rth = vth/(_VCC-vth) * _TH_PDR;
      Tic = _TH_B / ( logf(Rth) - logf(_TH_R0) + (_TH_B/_TH_T0) ) - 273.15f;
      float pD = iD*vDRef;
      float pQ = iQ*vQRef;
      if( pD < 0 ) pD = -pD;
      if( pQ < 0 ) pQ = -pQ;

      uint8_t txBuf[18];
      *(float *)&txBuf[1] = vBus;
      *(float *)&txBuf[5] = Tic;
      *(float *)&txBuf[9] = omgFlt * 60.0f/((_MOTOR_POLE>>1) * _RAD(360.0f));
      *(float *)&txBuf[13] = pD + pQ;
      encodePacket( (uint8_t *)txBuf, sizeof(txBuf) );
      HAL_UART_Transmit(&huart1, (uint8_t *)txBuf, sizeof(txBuf), 100);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  brkErr = 1;
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if( htim == &htim3 )
  {
    encEdgeDetect = 1;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
