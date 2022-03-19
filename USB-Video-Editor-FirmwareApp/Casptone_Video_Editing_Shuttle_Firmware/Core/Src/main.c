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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_BUTTONS 8
#define MODIFIER_INDEX
#define KEYCODE_1_INDEX
#define PACKET_SIZE 8
// Button Port Defines
#define GPIO_BUTTON_1_PIN GPIO_PIN_9
#define GPIO_BUTTON_1_PORT GPIOA
#define GPIO_BUTTON_2_PIN GPIO_PIN_8
#define GPIO_BUTTON_2_PORT GPIOA
#define GPIO_BUTTON_3_PIN GPIO_PIN_15
#define GPIO_BUTTON_3_PORT GPIOB
#define GPIO_BUTTON_4_PIN GPIO_PIN_14
#define GPIO_BUTTON_4_PORT GPIOB
#define GPIO_BUTTON_5_PIN GPIO_PIN_13
#define GPIO_BUTTON_5_PORT GPIOB
#define SET_ALL_BUTTONS 0xFFFF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

TSC_HandleTypeDef htsc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */


extern USBD_HandleTypeDef hUsbDeviceFS;



typedef enum  {false, true} bool;
typedef enum {active, inactive, serviced} state;

typedef enum {noEvent, errorRollOver, postFail, errorUndefined, letter_a, letter_b, letter_c,
	letter_d, letter_e, letter_f, letter_g, letter_h, letter_i, letter_j, letter_k, letter_l,
	letter_m, letter_n, letter_o, letter_p, letter_q, letter_r, letter_s, letter_t, letter_u,
	letter_v, letter_w, letter_x, letter_y, letter_z, number_1, number_2, number_3, number_4,
	number_5, number_6, number_7, number_8, number_9, number_0, command_Enter, command_Escape,
	command_Delete, command_Tab, command_Space, command_Minus, command_Equal, command_LeftKey = 0x50, command_RightKey = 0x4F,
	command_Spacebar = 0x2C} key; // More to Add!

typedef enum {leftCtrl = 1, leftShift = 2, leftAlt = 4, leftGUI = 8, rightCtrl = 16,
	rightShift = 32, rightAlt = 64,rightGUI = 128
} modifier;
typedef struct
{
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode_1;
	uint8_t keycode_2;
	uint8_t keycode_3;
	uint8_t keycode_4;
	uint8_t keycode_5;
	uint8_t keycode_6;

} packetStruct;

typedef union buttonMapping
{
	uint8_t packetBuffer[PACKET_SIZE];
	packetStruct packet;
} buttonMapping; // mapping for a single button

typedef struct buttonStates
{
	uint16_t buttonState_1 : 2;
	uint16_t buttonState_2 : 2;
	uint16_t buttonState_3 : 2;
	uint16_t buttonState_4 : 2;
	uint16_t buttonState_5 : 2;
	uint16_t buttonState_6 : 2;
	uint16_t buttonState_7 : 2;
	uint16_t buttonState_8 : 2;
} buttonStates;
typedef struct GlobalState
{
	buttonMapping buttonMappings[NUM_BUTTONS];
	buttonStates buttonPressed;
} GlobalState; // holds state


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_TIM2_Init(void);
//static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

void PollUserInput(void);
void InitShuttle(void);
void WriteOutputToPC(USBD_HandleTypeDef*);
void WriteButtonState(state val, int buttonIndex);
state ReadButtonState(int buttonIndex);

/* USER CODE BEGIN PV */
bool System_Init(void);
GlobalState globalState;
uint8_t testbuf[8] = {0};
packetStruct keyboardhid = {0};
// Encoder Test Variables
int counter = 0;
int16_t count =0; // defined to deal with underflow of case of counter going below 0
int16_t position = 0; //position of encoder
int16_t old_position = 0;
extern USBD_HandleTypeDef hUsbDeviceFS;
//uint8_t HID_buffer[8] = {0};
/* USER CODE END PV */


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	counter = __HAL_TIM_GET_COUNTER(htim);
	count = (int16_t)counter;
	position = count/4;

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TSC_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  InitShuttle(); // Initiailize data structures
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  PollUserInput();
	  WriteOutputToPC(&hUsbDeviceFS);


		 if (position > old_position) {
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 1); //red LED
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
			 old_position = position;
			 HAL_Delay(50);


			 keyboardhid.keycode_1 = 0x4F; // right arrow
		 } else if (position < old_position) {
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1); // blue LED
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
			 old_position = position;
			 HAL_Delay(50);

			 keyboardhid.keycode_1 = 0x50; // left arrow


		 } else if (position == old_position) {
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, 0);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
			 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1); // yellow LED

		 }

		  USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));
		  HAL_Delay(50);
		  keyboardhid.keycode_1 = 0x00; // release key
		  USBD_HID_SendReport(&hUsbDeviceFS, &keyboardhid, sizeof(keyboardhid));



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */
  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
//                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
//                           LD4_Pin LD5_Pin */
//  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
//                          |LD4_Pin|LD5_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
//  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//}

void InitShuttle(void)
{
	WriteButtonState(inactive, SET_ALL_BUTTONS); // set all button states to false
	globalState.buttonMappings[0].packet.modifier = 0;
	globalState.buttonMappings[0].packet.keycode_1 = command_Spacebar;
	globalState.buttonMappings[1].packet.modifier = leftCtrl;
	globalState.buttonMappings[1].packet.keycode_1 = letter_t;
	globalState.buttonMappings[2].packet.modifier = 0;
	globalState.buttonMappings[2].packet.keycode_1 = letter_f;
	globalState.buttonMappings[3].packet.modifier = 0;
	globalState.buttonMappings[3].packet.keycode_1 = letter_g;
	globalState.buttonMappings[4].packet.modifier = 0;
	globalState.buttonMappings[4].packet.keycode_1 = letter_h;
}

void PollUserInput(void) // poll for User Input
{
	 	 if(HAL_GPIO_ReadPin(GPIO_BUTTON_1_PORT, GPIO_BUTTON_1_PIN))
		  { if (ReadButtonState(0) == inactive)
			  globalState.buttonPressed.buttonState_1 = active;
		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_1 = inactive;
				uint8_t HID_buffer[8] = {0};
			  USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, PACKET_SIZE); // Send a null packet
		  }

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_2_PORT, GPIO_BUTTON_2_PIN))
		  {	if (ReadButtonState(1) == inactive)
			  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			  globalState.buttonPressed.buttonState_2 = active;
		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_2 = inactive;
				uint8_t HID_buffer[8] = {0};
			  USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, PACKET_SIZE); // Send a null packet
		  }


		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_3_PORT, GPIO_BUTTON_3_PIN))
		  {
			  if (ReadButtonState(2) == inactive)
			  globalState.buttonPressed.buttonState_3 = active;
		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_3 = inactive;
				uint8_t HID_buffer[8] = {0};
			  USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, PACKET_SIZE); // Send a null packet
		  }

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_4_PORT, GPIO_BUTTON_4_PIN))
		  {
			  if (ReadButtonState(3) == inactive)
			  globalState.buttonPressed.buttonState_4 = active;		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_4 = inactive;
				uint8_t HID_buffer[8] = {0};
	   		USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, PACKET_SIZE); // Send a null packet
			  			}

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_5_PORT, GPIO_BUTTON_5_PIN))
		  {
			  if (ReadButtonState(4) == inactive)
			  globalState.buttonPressed.buttonState_5 = active;		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_5 = inactive;
				uint8_t HID_buffer[8] = {0};
			  USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, PACKET_SIZE); // Send a null packet}
}
				/*HAL_Delay(100);
				USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, PACKET_SIZE);
				HAL_Delay(100);
				USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, PACKET_SIZE);
				USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, PACKET_SIZE);*/
}

void WriteOutputToPC(USBD_HandleTypeDef* hUsbDeviceFS)
{
	for (int i = 0 ; i<NUM_BUTTONS; i++)
	{
		if (ReadButtonState(i) == active)
		{
			USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[i].packetBuffer, PACKET_SIZE);
			HAL_Delay(100);
			WriteButtonState(serviced, i);
		    USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[i].packetBuffer, PACKET_SIZE);
			HAL_Delay(100);
			//USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[i].packetBuffer, PACKET_SIZE);
			//USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[i].packetBuffer, PACKET_SIZE);
		}
	}
}

void WriteButtonState(state val, int buttonIndex)
{
	switch (buttonIndex)
	{
	case 0:
		globalState.buttonPressed.buttonState_1 = val;
		break;
	case 1:
		globalState.buttonPressed.buttonState_2 = val;
		break;
	case 2:
		globalState.buttonPressed.buttonState_3 = val;
		break;
	case 3:
		globalState.buttonPressed.buttonState_4 = val;
		break;
	case 4:
		globalState.buttonPressed.buttonState_5 = val;
		break;
	case 5:
		globalState.buttonPressed.buttonState_6 = val;
		break;
	case 6:
		globalState.buttonPressed.buttonState_7 = val;
		break;
	case 7:
		globalState.buttonPressed.buttonState_8 = val;
		break;
	case SET_ALL_BUTTONS:
		for (int i = 0 ; i < NUM_BUTTONS ; i++)
		{
			WriteButtonState(val, i);
		}
		break;
	}

	return;
}

state ReadButtonState(int buttonIndex)
{
	state val;
	switch (buttonIndex)
	{
	case 0:
		val = globalState.buttonPressed.buttonState_1;
		break;
	case 1:
		val = globalState.buttonPressed.buttonState_2;
		break;
	case 2:
		val = globalState.buttonPressed.buttonState_3;
		break;
	case 3:
		val = globalState.buttonPressed.buttonState_4;
		break;
	case 4:
		val = globalState.buttonPressed.buttonState_5;
		break;
	case 5:
		val = globalState.buttonPressed.buttonState_6;
		break;
	case 6:
		val = globalState.buttonPressed.buttonState_7;
		break;
	case 7:
		val = globalState.buttonPressed.buttonState_8;
		break;
	}
	return val;
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

