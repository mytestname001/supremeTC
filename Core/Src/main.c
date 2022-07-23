/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define INIT          0
#define SEND_TRIG     1
#define WAIT_TRIG_END 2
#define WAIT_ECHO_END 3
#define CALC_DIST     4

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

typedef struct ts_rxType{
	char start;
	unsigned char data;
	char parity;
	char stop;
	char error;
} rxType;
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t curr_state, next_state;
uint8_t trig_pulse_end, echo_pulse_end;
//ultra1
char Uart_Tx_Parity_Mode;
int tim2_cnt;
char tim2_flag;
char tim2_trigger;
char chUartTxData[32];

char Uart_Rx_Parity_Mode;
rxType rxData[32];
int rxByte_cnt;
int rxBit_cnt;
int rxBit_value;
int rxSamp_cnt;
int rxParity_bit;
char Uart_Rx_flag;
char Uart_Rx_active;
char Uart_Rx_Temp;
char Uart_Rx_trigger;
char Uart_Rx_data;
char Uart_Rx_parity;
char Uart_Rx_stop;

int globalCounter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE//use printf in usart2
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

void Uart_Tx_Byte(char TxData, char Parity)
{
	char chExit = 'n';
	char chComp = 0x01;
	int ParityBit_cnt = 0;

	tim2_cnt = 0;
	tim2_flag = 'y';
	tim2_trigger = 'n';

	while(1){
		if(('y' == tim2_trigger) && (tim2_cnt == 1))
		{ // idle
			tim2_trigger = 'n';
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 2))
		{ // start
			tim2_trigger = 'n';
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 3))
		{ // bit 0
			tim2_trigger = 'n';
			if((TxData & chComp) == chComp)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				ParityBit_cnt += 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			chComp <<= 1;
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 4))
		{ // bit 1
			tim2_trigger = 'n';
			if((TxData & chComp) == chComp)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				ParityBit_cnt += 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			chComp <<= 1;
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 5))
		{ // bit 2
			tim2_trigger = 'n';
			if((TxData & chComp) == chComp)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				ParityBit_cnt += 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			chComp <<= 1;
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 6)){ // bit 3
			tim2_trigger = 'n';
			if((TxData & chComp) == chComp)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				ParityBit_cnt += 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			chComp <<= 1;
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 7))
		{ // bit 4
			tim2_trigger = 'n';
			if((TxData & chComp) == chComp)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				ParityBit_cnt += 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			chComp <<= 1;
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 8))
		{ // bit 5
			tim2_trigger = 'n';
			if((TxData & chComp) == chComp)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				ParityBit_cnt += 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			chComp <<= 1;
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 9))
		{ // bit 6
			tim2_trigger = 'n';
			if((TxData & chComp) == chComp)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				ParityBit_cnt += 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			chComp <<= 1;
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 10))
		{ // bit 7
			tim2_trigger = 'n';
			if((TxData & chComp) == chComp)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				ParityBit_cnt += 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
			chComp <<= 1;
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 11))
		{ // [parity : none/even/odd] or [stop]
			tim2_trigger = 'n';
			if(Parity == 'n')
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				chExit = 'y';
			}
			else if(Parity == 'e')
			{
				if((ParityBit_cnt % 2) == 0)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				}
			}
			else if(Parity == 'o')
			{
				if((ParityBit_cnt % 2) == 1)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
				}
				else
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				}
			}
		}
		else if(('y' == tim2_trigger) && (tim2_cnt == 12))
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			chExit = 'y';
		}
		if(('y' == tim2_trigger) && ('y' == chExit))
		{
			tim2_trigger = 'n';
			tim2_flag = 'n';
			break;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim3.Instance)
	{
		if('y' == tim2_flag)
		{
			globalCounter = globalCounter + 1;
			if(globalCounter >= 4)
			{
				tim2_cnt += 1;
				tim2_trigger = 'y';
				globalCounter = 0;
			}
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}
		if('y' == Uart_Rx_flag)
		{
			if('n' == Uart_Rx_active)
			{
				Uart_Rx_Temp <<= 1;
				if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
				{
					Uart_Rx_Temp += 1;
				}
				if((Uart_Rx_Temp & 0x0f) == 0x0c)
				{
					Uart_Rx_active = 'y';
					Uart_Rx_Temp = 0x00;
					rxByte_cnt += 1;
					rxData[rxByte_cnt].start = 'y';
					rxData[rxByte_cnt].data = 0x00;
					rxData[rxByte_cnt].parity = 'n';
					rxData[rxByte_cnt].stop = 'x';
					rxBit_cnt = 0;
					rxSamp_cnt = 0;
					rxParity_bit = 0;
					Uart_Rx_trigger = 'n';
					Uart_Rx_data = 'n';
					Uart_Rx_parity = 'n';
					Uart_Rx_stop = 'n';
				}
			}else{
				Uart_Rx_trigger = 'n';
				rxSamp_cnt += 1;
				if((rxSamp_cnt % 4) == 0){
					Uart_Rx_trigger = 'y';
					if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)){
						rxBit_value = 1;
					}else{
						rxBit_value = 0;
					}
					rxBit_cnt += 1;
					if((1 <= rxBit_cnt) && (rxBit_cnt <= 8)){
						Uart_Rx_data = 'y';
					}else if(9 == rxBit_cnt){
						Uart_Rx_data = 'n';
						if('n' == Uart_Rx_Parity_Mode){
							Uart_Rx_parity = 'n';
							Uart_Rx_stop = 'y';
						}else if('e' == Uart_Rx_Parity_Mode){
							Uart_Rx_parity = 'y';
						}else if('o' == Uart_Rx_Parity_Mode){
							Uart_Rx_parity = 'y';
						}
					}else if(10 == rxBit_cnt){
						Uart_Rx_parity = 'n';
						Uart_Rx_stop = 'y';
					}
				}
				if(('y' == Uart_Rx_trigger) && ('y' == Uart_Rx_data)){
					rxData[rxByte_cnt].data >>= 1;
					if(1 == rxBit_value){
						rxData[rxByte_cnt].data += 0x80;
						rxParity_bit += 1;
					}
				}
				if(('y' == Uart_Rx_trigger) && ('y' == Uart_Rx_parity)){
					if('e' == Uart_Rx_Parity_Mode){
						if(((rxParity_bit % 2) == 0) && (0 == rxBit_value)){
							rxData[rxByte_cnt].parity = 'e';
						}else if(((rxParity_bit % 2) == 1) && (1 == rxBit_value)){
							rxData[rxByte_cnt].parity = 'e';
						}else{
							rxData[rxByte_cnt].parity = 'f';
						}
					}else if('o' == Uart_Rx_Parity_Mode){
						if(((rxParity_bit % 2) == 0) && (1 == rxBit_value)){
							rxData[rxByte_cnt].parity = 'o';

						}else if(((rxParity_bit % 2) == 1) && (0 == rxBit_value)){
							rxData[rxByte_cnt].parity = 'o';
						}else{
							rxData[rxByte_cnt].parity = 'f';
						}
					}
				}
				if(('y' == Uart_Rx_trigger) && ('y' == Uart_Rx_stop)){
					Uart_Rx_active = 'n';
					if(1 == rxBit_value){
						rxData[rxByte_cnt].stop = 'y';
					}else{
						rxData[rxByte_cnt].stop = 'n';
					}
				}
			}
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  int count;
	  int smell;
	  int resultDistance;
	  int resultDistance2;
	  int resultDistance3;

	HAL_StatusTypeDef tim3_it_start;
	HAL_StatusTypeDef tim2_it_start;

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Uart_Rx_Parity_Mode = 'n'; // 'n':none, 'e':even, 'o':odd
  rxByte_cnt = 0;
  rxBit_cnt = 0;
  rxSamp_cnt = 0;
  Uart_Rx_flag = 'n';
  Uart_Rx_active = 'n';
  Uart_Rx_Temp = 0x00;

  Uart_Tx_Parity_Mode = 'n'; // 'n':none, 'e':even, 'o':odd
  tim2_cnt = 0;
  tim2_flag = 'n';
  tim2_trigger = 'n';

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  tim3_it_start = HAL_TIM_Base_Start_IT(&htim3);
  tim2_it_start = HAL_TIM_Base_Start_IT(&htim2);

  if((HAL_OK == tim3_it_start) && (HAL_OK == tim2_it_start))
  {
	  printf("HAL_TIM_Base_Start_IT OK\r\n");
  }
  else
  {
	  printf("HAL_TIM_Base_Start_IT error\r\n");
  }
  chUartTxData[0] = 'a';
  Uart_Rx_flag = 'y';
  Uart_Rx_active = 'n';

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(100);
	  Uart_Tx_Byte(0x55, 'n');
	  if(('n' == Uart_Rx_active) && (1 <= rxByte_cnt))
	  {
		  resultDistance = (rxData[1].data << 8) + rxData[2].data;
		  printf("resultDistance : %d mm\r\n", resultDistance);
		  rxByte_cnt = 0;
	  }
	  HAL_Delay(100);

	  HAL_ADC_Start(&hadc1);

	  if(resultDistance <= 500)//auto_open,close
	  {
		    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 50);//open_motor 2(PB7)
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 200);//open_motor 1(PB6)
			HAL_Delay(3000);
		    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 200);//open_motor 2(PB7)
		    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 50);//open_motor 1(PB6)
		    HAL_Delay(3000);
	  }

	  /*
	  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	  {
		  smell = HAL_ADC_GetValue(&hadc1) / 4;
		  printf("smell = %d\r\n", (int)smell);
		  HAL_Delay(1000);
		  if(smell >= 150)
		  {
			  count++;
		  }
		  if(smell <= 100)
		  {
			  count = 0;
		  }
	  }
	  if(smell >= 150 && count == 10)//motor active and spread!
	  {
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 100);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 200);
		  HAL_Delay(2000);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 200);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 100);
		  HAL_Delay(2000);
		  count = 0;
	  }
	  */


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 640-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, trigger_1_Pin|LD2_Pin|Ultra_Echo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, trigger_2_Pin|trigger_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : echo_1_Pin echo_2_Pin echo_3_Pin IF_Detect_Pin */
  GPIO_InitStruct.Pin = echo_1_Pin|echo_2_Pin|echo_3_Pin|IF_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : trigger_1_Pin LD2_Pin Ultra_Echo_Pin */
  GPIO_InitStruct.Pin = trigger_1_Pin|LD2_Pin|Ultra_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : trigger_2_Pin trigger_3_Pin */
  GPIO_InitStruct.Pin = trigger_2_Pin|trigger_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultra_Trigger_Pin */
  GPIO_InitStruct.Pin = Ultra_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ultra_Trigger_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

