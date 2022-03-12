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

typedef enum  {false, true} bool;
typedef enum {active, inactive, serviced} state;

typedef enum {noEvent, errorRollOver, postFail, errorUndefined, letter_a, letter_b, letter_c,
	letter_d, letter_e, letter_f, letter_g, letter_h, letter_i, letter_j, letter_k, letter_l,
	letter_m, letter_n, letter_o, letter_p, letter_q, letter_r, letter_s, letter_t, letter_u,
	letter_v, letter_w, letter_x, letter_y, letter_z, number_1, number_2, number_3, number_4,
	number_5, number_6, number_7, number_8, number_9, number_0, command_Enter, command_Escape,
	command_Delete, command_Tab, command_Space, command_Minus, command_Equal, command_Comma = 0x50, command_Period = 0x4F} key; // More to Add!

typedef enum {leftCtrl, leftShift = 2, leftAlt = 4, leftGUI = 8, rightCtrl = 16,
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
/* USER CODE END PTD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool System_Init(void);
GlobalState globalState;
int counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PollUserInput(void);
void InitShuttle(void);
void WriteOutputToPC(USBD_HandleTypeDef*);
void WriteButtonState(state val, int buttonIndex);
state ReadButtonState(int buttonIndex);
/* USER CODE BEGIN PFP */

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
  System_Init(); // Initialize all variables;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  InitShuttle(); // Initiailize data structures
  /* USER CODE BEGIN 2 */
 extern USBD_HandleTypeDef hUsbDeviceFS;
 uint8_t HID_buffer[8] = {0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HID_buffer[0] = 2; // left shift
	  //HID_buffer[2] = 7;
	  //USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, 8);

	  //HAL_Delay(20);
	  //HID_buffer[0] = 0; // left shift
	  //HID_buffer[2] = 0;
	  //USBD_HID_SendReport(&hUsbDeviceFS, HID_buffer, 8);

	 // HAL_Delay(20);

	 PollUserInput();
	 WriteOutputToPC(&hUsbDeviceFS);




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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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

/* USER CODE BEGIN 4 */
bool			// Returns whether system was initialized successfully
System_Init
(void)
{
	return false;
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

void InitShuttle(void)
{
	WriteButtonState(inactive, SET_ALL_BUTTONS); // set all button states to false
	globalState.buttonMappings[0].packet.modifier = leftShift;
	globalState.buttonMappings[0].packet.keycode_1 = command_Comma;
	globalState.buttonMappings[1].packet.modifier = leftShift;
	globalState.buttonMappings[1].packet.keycode_1 = letter_e;
	globalState.buttonMappings[2].packet.modifier = leftShift;
	globalState.buttonMappings[2].packet.keycode_1 = letter_f;
	globalState.buttonMappings[3].packet.modifier = leftShift;
	globalState.buttonMappings[3].packet.keycode_1 = letter_g;
	globalState.buttonMappings[4].packet.modifier = leftShift;
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
		  }

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_2_PORT, GPIO_BUTTON_2_PIN))
		  {	if (ReadButtonState(1) == inactive)
			  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			  globalState.buttonPressed.buttonState_2 = inactive;
		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_2 = inactive;
		  }


		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_3_PORT, GPIO_BUTTON_3_PIN))
		  {
			  if (ReadButtonState(2) == inactive)
			  globalState.buttonPressed.buttonState_3 = active;
		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_3 = inactive;
		  }

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_4_PORT, GPIO_BUTTON_4_PIN))
		  {
			  if (ReadButtonState(3) == inactive)
			  globalState.buttonPressed.buttonState_4 = active;		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_4 = inactive;		  }

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_5_PORT, GPIO_BUTTON_5_PIN))
		  {
			  if (ReadButtonState(4) == inactive)
			  globalState.buttonPressed.buttonState_5 = active;		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_5 = inactive;		  }
}

void WriteOutputToPC(USBD_HandleTypeDef* hUsbDeviceFS)
{
	for (int i = 0 ; i<NUM_BUTTONS; i++)
	{
		if (ReadButtonState(i) == active)
		{
			USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[i].packetBuffer, PACKET_SIZE);
			HAL_Delay(30);
			WriteButtonState(serviced, i);
			uint8_t HID_buffer[8] = {0};
			USBD_HID_SendReport(hUsbDeviceFS, HID_buffer, PACKET_SIZE); // Send a null packet
			HAL_Delay(200);
			counter++;
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

