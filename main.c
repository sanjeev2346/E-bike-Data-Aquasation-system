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
#include <string.h>
#include <math.h>
#include <stdio.h>
#define BAUD_RATE 9600



ADC_HandleTypeDef hadc1;



UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//uint8_t data[2000];
float raw;
float Iraw;
float Sraw;
float Traw;
float km;
float contemp;
float I;
float b = 3977;
				  	float rto = 1000;
				  							float vcc = 5;
				  							float R = 1000;
				  							float T0 = 15+273.15;
				  							float VRT,VR,RT,ln,TX;

				  							const float SHUNT_CURRENT =100.00;//A
				  							const float SHUNT_VOLTAGE =75.0;// mV
				  							const float CORRECTION_FACTOR = 2.00;


				  							const int ITERATION = 30; //can change (see video)
				  							const float VOLTAGE_REFERENCE = 1200.0;//1.1V
				  							const int BIT_RESOLUTION =12;
				  							float vac;
				  				     		float vc;
				  						    float rphr;
				  						    float rpm;
				  						    int i;
				  						    int j;
				  						    int k;
				  						    int l;
				  						    int m;
				  						    int n;
				  						    int o;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void ADC_Select_CH0(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void ADC_Select_CH1(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void ADC_Select_CH2(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void ADC_Select_CH3(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
uint16_t ADC_VAL[4];
void initUART(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = BAUD_RATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}
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

				  //char msg[10];
				  char uart_buf[100];
				  	int uart_buf_len;
				  	//const int resistorValue = 10000;




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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  ADC_Select_CH0();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  raw = HAL_ADC_GetValue(&hadc1);
	  float vin;
	  	 	  float r1,r2,r3,r4;
	  	 	  vin = (raw*3.3)/4095;

	  	 	  	   	 		 	  	   r1=68;
	  	 	  	   	 		 	  	   r2=3.3;
	  	 	  	   	 		 	  	   vac=((r2+r1)/r2)*vin;
	  	 	  	   	 		 	//data[1] = vac;

	  HAL_ADC_Stop(&hadc1);
	 // HAL_Delay(500);
	  ADC_Select_CH1();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  Iraw = HAL_ADC_GetValue(&hadc1);


	  	 	  	    int sensorValue ;
	  	 	  	    float voltage, current;


	  	 	  	      sensorValue =  Iraw ;

	  	 	  	        voltage = (sensorValue +0.5) * (VOLTAGE_REFERENCE /  (pow(2,BIT_RESOLUTION)-1));
	  	 	  	        current  = voltage * (SHUNT_CURRENT /SHUNT_VOLTAGE )  ;

	  	 	  	   //IGNORE_CURRENT_BELOW
	  	 	  	    I=current;
	  	 	  	//data[2] = I;
	  	 	  	HAL_ADC_Stop(&hadc1);
	  //HAL_Delay(500);
	  ADC_Select_CH2();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  Sraw = HAL_ADC_GetValue(&hadc1);
	  vin =(Sraw*3.3)/4095;
	  	 	  		   	 		 	  	   r3=51;
	  	 	  		   	 		 	  	   r4=4.7;
	  	 	  		   	 		 	  	   vc=((r4+r3)/r4)*vin;
	  	 	  		   	 		 	  	   rpm = (vc/39)*652;
	  	 	  		   	 		 	  	   rphr = rpm*60;
	  	 	  		   	 		 	  	   km=0.001356*rphr;
	  	 	  		   	 		 	//data[3] = km;
	  HAL_ADC_Stop(&hadc1);
	  //HAL_Delay(500);
	  ADC_Select_CH3();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  Traw = HAL_ADC_GetValue(&hadc1);
	  VRT = (3.33 / 1024) *Traw;      //Conversion to voltage
	  	 		  	 	 	  		  		   	 		 	  	 	 	  VR = vcc - VRT;
	  	 		  	 	 	  		  		   	 		 	  	 	 	  RT = VRT / (VR / R);               //Resistance of RT

	  	 		  	 	 	  		  		   	 		 	  	 	 	  ln = log(RT / rto);
	  	 		  	 	 	  		  		   	 		 	  	 	 	  TX = (1 / ((ln / b) + (1 / T0))); //Temperature from thermistor

	  	 		  	 	 	  		  		   	 		 	 contemp = TX - 273.15;
	  	 		  	 	 	  		  		   	//data[4] = contemp;

	  HAL_ADC_Stop(&hadc1);
	 // HAL_Delay(500);
	  if(HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_7))  { //Check if button pressed

	 	  		 i = 1;


	 	  	  }//If pressed Led Switch On
	 	  	  	else{

	 	  	  	 i = 0;

	 	  	  	}
	  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0))  { //Check if button pressed

	  	  		  		 j = 1;


	  	  		  	  }//If pressed Led Switch On
	  	  		  	  	else{

	  	  		  	  	 j = 0;

	  	  		  	  	}
	  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_1))  { //Check if button pressed

		  		  		  	 k = 1;


		  		  		  	  }//If pressed Led Switch On
		  		  		  	  	else{

		  		  		  	 k = 0;
		  		  		  	  	}
	  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_2))  { //Check if button pressed

	  	  		  		 	  	l = 1;


	  	  		  		 	  	  }//If pressed Led Switch On
	  	  		  		 	  	  	else{

	  	  		  		 	  	 l = 0;

	  	  		  		 	  	  	}
	  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_12))  { //Check if button pressed

	  	  		  		 		  	 m = 1;


	  	  		  		 		  	  }//If pressed Led Switch On
	  	  		  		 		  	  	else{

	  	  		  		 		  	 m = 0;

	  	  		  		 		  	  	}
	  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_13))  { //Check if button pressed

	  	  	  		  		 		  	 n = 1;


	  	  	  		  		 		  	  }//If pressed Led Switch On
	  	  	  		  		 		  	  	else{

	  	  	  		  		 		  	 n = 0;

	  	  	  		  		 		  	  	}
	  if(HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_14))  { //Check if button pressed

	  	  	  		  		 		  	 o = 1;


	  	  	  		  		 		  	  }//If pressed Led Switch On
	  	  	  		  		 		  	  	else{

	  	  	  		  		 		  	 o = 0;

	  	  	  		  		 		  	  	}











		  		  		 		// HAL_UART_Transmit (&huart1, data, sizeof (data), 1000);

	 		  	 	 	  		  		   		uart_buf_len = sprintf(uart_buf,"{\"voltage\":%d,\"current\":%d,\"speed\":%d,\"temp\":%d,\"i\":%d,\"j\":%d,\"k\":%d,\"l\":%d,\"m\":%d,\"n\":%d,\"o\":%d}",(int)vac,(int)I,(int) km,(int)contemp,(int)i,(int)j,(int)k,(int)l,(int)m,(int)n,(int)o);

	 		  	 	 	  		  		   		 HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf,uart_buf_len, 1000);
	 		  	 	 	  		  		 HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  /*sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
*/
  /** Configure Regular Channel
  */
 /* sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }*/

  /** Configure Regular Channel
  */
  //sConfig.Channel = ADC_CHANNEL_3;
  //sConfig.Rank = ADC_REGULAR_RANK_3;
  //if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  //{
	  //Error_Handler();
    //}

  /** Configure Regular Channel
  */
  // sConfig.Channel = ADC_CHANNEL_4;
  // sConfig.Rank = ADC_REGULAR_RANK_4;
  // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  // {
	  // Error_Handler();//
 // }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
