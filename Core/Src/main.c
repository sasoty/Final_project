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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Buttonfunc.h"
#include "liquidcrystal_i2c.h"
#include "Std_Types.h"
#include "../Protocol_Handle/Protocol_Handle.h"
#include "string.h"
#include "stdio.h"
#include "MQTT_Buffer.h"
// add max, calib,height
int height=1;
float calib_height;
float bmi;
int max,calib=0;
bool DISPLAY_FLG = false;
bool WIFI_FLG =false;


int16_t change,prev,pre,first;
uint16_t countbig,countsmall,count_tick= 0;
uint8_t dup =0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const gpin_state pinsta[] =
{
		{GPIO_PIN_RESET,GPIO_PIN_RESET},
		{GPIO_PIN_RESET,GPIO_PIN_SET},
		{GPIO_PIN_SET,GPIO_PIN_RESET},
		{GPIO_PIN_SET,GPIO_PIN_SET}
};
ValueResult Scale[4];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t bfore = 0;
int16_t af =0;
BTON Bt_state;

//---------------------------------------------------------------
void Button_Handle()
{
	char str[10];
	if(Bt_state== NUM_)
	{
		HD44780_SetCursor(7, 1);
		HD44780_PrintStr("   ");
		HD44780_SetCursor(7, 1);
		sprintf(str,"%d",change);
		HD44780_PrintStr(str);

	}
	else if(Bt_state == CALIB_)
	{

		HD44780_SetCursor(6, 1);
		HD44780_PrintStr("   ");
		HD44780_SetCursor(6, 1);
		sprintf(str,"%d",change);
		HD44780_PrintStr(str);
	}
	else if(Bt_state == MAX_)
	{
		HD44780_SetCursor(4, 1);
		HD44780_PrintStr("   ");
		HD44780_SetCursor(4, 1);
		sprintf(str,"%d",change);
		HD44780_PrintStr(str);
	}
}
void Button_Display()
{

	if(Bt_state== NUM_)
	{
		HD44780_SetCursor(0, 1);
		HD44780_PrintStr("Height");

	}
	else if(Bt_state == CALIB_)
	{
		HD44780_SetCursor(0, 1);
		HD44780_PrintStr("Calib");
	}
	else if(Bt_state == MAX_)
	{
		HD44780_SetCursor(0, 1);
		HD44780_PrintStr("Max");
	}
	HD44780_PrintStr("            ");

}
//-------------------------------------------------------------------
void ESP_Com(float ESP_Temp)
{
	Convert Rx_Wifi;
	Rx_Wifi.Fl = ESP_Temp;
	for(int i = 0;i<4;i++)
	{
		MQTT_BuffStuff(Rx_Wifi.Segment[i]);
	}

}
//-----------------------------------------------------------------
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	 bfore = __HAL_TIM_GET_COUNTER(&htim2);
	 af=(int16_t) bfore;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)==0)
		{
		DISPLAY_FLG =false;
		  first = af;
		  Bt_state = bton_buff();
		  Button_Display();
		  dup = 0;
		  countbig = 0;
		  countsmall = 0;
		  HAL_TIM_Base_Start_IT(&htim3);
		  // state ++
		}
	}
	if(GPIO_Pin == GPIO_PIN_5)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0)
		{
			WIFI_FLG = !WIFI_FLG;
		}
	}
}
//----------------------------
uint8_t TxData;
uint8_t ESP_Data;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART6)
	{
		Std_Return ESP_Avail = MQTT_Avail();
		if(ESP_Avail == E_OK)
		{
			ESP_Data = MQTT_BuffDrain();
			HAL_UART_Transmit_IT(&huart6, (uint8_t *)&ESP_Data, 1);
		}
	}
//	else
	if(huart->Instance == USART2)
	{
		Std_Return Tx_Avail =  Tx_RingData_Avail();
		if(Tx_Avail == E_OK)
		{
			TxData = Tx_ReadBuf();
			HAL_UART_Transmit_IT(&huart2,(uint8_t *)&TxData,1);
		}
		else
		{
			Push_Avail();
		}
	}


}
//------------------------------------------------------------------------

uint8_t RxData;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		if(huart->Instance == USART2)
		{
			Rx_BufStuff(RxData);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&RxData,1);
		}

}

//------------------------------------------

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{

	   if(countbig == 50)
	   {
		   change = bton_change(first, af);

		   //LCD Cursor
		  if(countsmall == 20)
		  {
			  Button_Handle();
			  pre = af;
			  if(pre == prev )
			  {
				  dup ++;
			   if(Bt_state== NUM_)
				{
					height = change;

				}
				else if(Bt_state == CALIB_)
				{
					calib = change;
				}
				else if(Bt_state == MAX_)
				{
					max = change;
				}
			  }
			  else
			  {
				dup = 0;
			  }
			  if(dup == 3)
			  {
				countbig = 0;
				countsmall = 0;
				bton_reset();// cho state ve 0
				HD44780_SetCursor(0, 1);
				HD44780_PrintStr("           ");
				DISPLAY_FLG = true;
				HAL_TIM_Base_Stop_IT(&htim3);
			  }
			  countsmall = 0;
			  prev = pre;

		  }
		  countsmall++;
		  countbig = 0;
	   }
		  countbig++;
  }


  if (htim->Instance == TIM4)
    {
		  Error_ModHandle();  // Increment the timer value every 1ms

    }

}

//------------------------------------------------------
void Channel_select(int temp)
{
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, pinsta[temp].spin1);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, pinsta[temp].spin2);
}

//=--------------------------------------------------------------
void VAL_Connect()
{
	   for (int i = 0; i < 4; i++)
	     {
		   Channel_select(i);
	 	  Protocol_Connect(&Scale[i]);
	 	 Rx_ResetBuf();
	     }
}

//------------------------------------------------------------
void VAL_Return()
{
	  for (int i = 0; i < 4; i++)
	   {

		  Channel_select(i);
		  HAL_Delay(10);
		  if(Scale[i].connected)
		  {
//			  char str[2];
			  Protocol_Value(&Scale[i]);
//			  HD44780_SetCursor(0, 1);
//			  sprintf(str,"%d",i);
//			  HD44780_PrintStr(str);
		  }
		  else
		  {
			  Protocol_Connect(&Scale[i]);
		  }
		  Rx_ResetBuf();
	    }
}
float VAL_Sum()
{
	float Sum = 0;
	  for (int i = 0; i < 4; i++)
		   {
			  if(Scale[i].connected)
			  {
				  Sum = Sum + Scale[i].value;
			  }
			  else
			  {

			  }
		    }
	  return Sum;
}
//---------------------------------------------------------
void VAL_Connected_Display()
{
	HD44780_SetCursor(0,1);
	HD44780_PrintStr("CS");
	char str[2];
	for(int i =0;i<4;i++)
	{
		  if(Scale[i].connected)
		  {

			  HD44780_SetCursor(2+i,1);
			  sprintf(str,"%d",i);
			  HD44780_PrintStr(str);
		  }
		  else
		  {

			  HD44780_SetCursor(2+i,1);
			  HD44780_PrintStr(" ");
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  for (int i = 0; i < 4; i++)
     {
	  Scale[i].connected = false;
	  Scale[i].side = true;
     }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  //------------------------------------------------
//  // Lcd_PortType ports[] = { D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port };
//  Lcd_PortType ports[] = { GPIOD, GPIOD, GPIOD, GPIOD };
//  // Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
//  Lcd_PinType pins[] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11};
//  Lcd_HandleTypeDef lcd;
//  // Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);
//  lcd = Lcd_create(ports, pins, GPIOD, GPIO_PIN_12, GPIOD, GPIO_PIN_14, LCD_4_BIT_MODE);
  //-------------------------------------------------

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);
  //----------------------------------------------


  Rx_RingBufferInit();
   Tx_RingBufferInit();
   HAL_UART_Receive_IT(&huart2, (uint8_t *)&RxData, 1);


   VAL_Connect();


  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
  HD44780_Init(2);
  HD44780_Clear();
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("HELLO");
//  Lcd_init(&lcd);
//  Lcd_cursor(&lcd, 0,1);
//    Lcd_string(&lcd, "WEIGHT:");
//    HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);
	  MQTT_Init();
	  VAL_Return();
	  char str[10];
	  HD44780_SetCursor(0,0);
	  HD44780_PrintStr("Weight");
	  HD44780_PrintStr("      ");
	  float val = VAL_Sum() + (float)calib;
	  ESP_Com(val);
//-------------------------------------------------------------

//	  }

//---------------------------------------------------------------
	  HD44780_SetCursor(6,0);
	  HD44780_PrintStr("    ");

	  int Sum_it = (int)(val*100);
	  if(Sum_it <= 20)
	  {
		  HD44780_SetCursor(6,0);
		  HD44780_PrintStr("0");
	  }
	  else
	  {
		  HD44780_SetCursor(6,0);
		  int first_half = Sum_it/100;
		  int last_half = Sum_it%100;
		  HD44780_SetCursor(6,0);
		  sprintf(str,"%d",first_half);
		  HD44780_PrintStr(str);
		  HD44780_PrintStr(".");
		  sprintf(str,"%d",last_half);
		  HD44780_PrintStr(str);
	  }
	  HD44780_SetCursor(12,0);
	  HD44780_PrintStr("Kg");
//-------------------------------------------------------------
	  calib_height = (float)height*height/100;

	  bmi = Sum_it/calib_height;
	  ESP_Com(bmi);
	  int Bmi_it = (int)(bmi*10);
//------------------------------------------------------------------------

	  for(int i = 0 ; i<4;i++)
	  {
		  ESP_Com(Scale[i].value);
	  }
		//need to fix
	  if(WIFI_FLG == 1)
	  {
		  ESP_Data = MQTT_BuffDrain();
		  HAL_UART_Transmit_IT(&huart6, (uint8_t *)&ESP_Data, 1);
	  }


//-----------------------------------------------------------------------------
	  if (DISPLAY_FLG)
	  {
		  VAL_Connected_Display();
		  if(max <= (int)val )
		  {
			  	HD44780_SetCursor(6, 1);
			  	HD44780_PrintStr("OVLOAD");
		  }
		  else
		  {
			  	HD44780_SetCursor(6, 1);
			  	HD44780_PrintStr("      ");
	  			HD44780_SetCursor(6, 1);
	  			HD44780_PrintStr("BMI");
	  			HD44780_SetCursor(10, 1);
	  			int first_bmi = Bmi_it/10;
	  			int last_bmi = Bmi_it%10;
				sprintf(str,"%d",first_bmi);
				HD44780_PrintStr(str);
				HD44780_PrintStr(".");
				sprintf(str,"%d",last_bmi);
				HD44780_PrintStr(str);
		  }

	  }



//	  float calib_height = (float)height/100;
//



//
//			HD44780_SetCursor(9, 1);
//			HD44780_PrintStr("BMI");
//			sprintf(str,"%d",bmi);
//			  int first_bmi = Sum_it/100;
//			  int last_bmi = Sum_it%100;
//			  HD44780_SetCursor(12,1);


//			HD44780_PrintStr(str);



//
//	  sprintf(str,"%2.f", Scale[0].value);
//	HD44780_PrintStr("12 kg");
//	HAL_Delay(500);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 720-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
