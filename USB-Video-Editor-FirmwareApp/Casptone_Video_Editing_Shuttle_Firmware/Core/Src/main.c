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


/* USER CODE END PD */

typedef enum  {false, true} bool;

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
	uint16_t buttonState_1 : 1;
	uint16_t buttonState_2 : 1;
	uint16_t buttonState_3 : 1;
	uint16_t buttonState_4 : 1;
	uint16_t buttonState_5 : 1;
	uint16_t buttonState_6 : 1;
	uint16_t buttonState_7 : 1;
	uint16_t buttonState_8 : 1;
	uint16_t Spare : 8; // In case we need to expand and use more buttons
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PollUserInput(void);
void InitShuttle(void);
void WriteOutputToPC(USBD_HandleTypeDef*);
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
	globalState.buttonPressed.buttonState_1 = false;
	globalState.buttonPressed.buttonState_2 = false;
	globalState.buttonPressed.buttonState_3 = false;
	globalState.buttonPressed.buttonState_4 = false;
	globalState.buttonPressed.buttonState_5 = false;
	globalState.buttonPressed.buttonState_6 = false;
	globalState.buttonPressed.buttonState_7 = false;
	globalState.buttonPressed.buttonState_8 = false;
	globalState.buttonPressed.Spare = false;
	globalState.buttonMappings[0].packet.modifier = 2;
	globalState.buttonMappings[0].packet.keycode_1 = 7;
	globalState.buttonMappings[1].packet.modifier = 2;
	globalState.buttonMappings[1].packet.keycode_1 = 8;
	globalState.buttonMappings[2].packet.modifier = 2;
	globalState.buttonMappings[2].packet.keycode_1 = 9;
	globalState.buttonMappings[3].packet.modifier = 2;
	globalState.buttonMappings[3].packet.keycode_1 = 0x0A;
	globalState.buttonMappings[4].packet.modifier = 2;
	globalState.buttonMappings[4].packet.keycode_1 = 0x0B;


}
void PollUserInput(void) // poll for User Input
{
	 if(HAL_GPIO_ReadPin(GPIO_BUTTON_1_PORT, GPIO_BUTTON_1_PIN))
		  {
			  globalState.buttonPressed.buttonState_1 = true;
		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_1 = false;

		  }

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_2_PORT, GPIO_BUTTON_2_PIN))
		  {
			  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			  globalState.buttonPressed.buttonState_2 = true;
		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_2 = false;
		  }


		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_3_PORT, GPIO_BUTTON_3_PIN))
		  {
			  globalState.buttonPressed.buttonState_3 = true;
		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_3 = false;
		  }

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_4_PORT, GPIO_BUTTON_4_PIN))
		  {
			  globalState.buttonPressed.buttonState_4 = true;		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_4 = false;		  }

		  if(HAL_GPIO_ReadPin(GPIO_BUTTON_5_PORT, GPIO_BUTTON_5_PIN))
		  {
			  globalState.buttonPressed.buttonState_5 = true;		  }
		  else
		  {
			  globalState.buttonPressed.buttonState_5 = false;		  }
}

void WriteOutputToPC(USBD_HandleTypeDef* hUsbDeviceFS)
{
	if (globalState.buttonPressed.buttonState_1 == true)
	{
		USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[0].packetBuffer, PACKET_SIZE);
	}
	else if (globalState.buttonPressed.buttonState_2 == true)
	{
		USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[1].packetBuffer, PACKET_SIZE);
	}
	else if (globalState.buttonPressed.buttonState_3 == true)
	{
		USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[2].packetBuffer, PACKET_SIZE);
	}
	else if (globalState.buttonPressed.buttonState_4 == true)
	{
		USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[3].packetBuffer, PACKET_SIZE);
	}
	else if (globalState.buttonPressed.buttonState_5 == true)
	{
		USBD_HID_SendReport(hUsbDeviceFS, globalState.buttonMappings[4].packetBuffer, PACKET_SIZE);
	}
	else
	{
		uint8_t HID_buffer[8] = {0};
		USBD_HID_SendReport(hUsbDeviceFS, HID_buffer, PACKET_SIZE);
	}
	globalState.buttonPressed.buttonState_1 = false;
	globalState.buttonPressed.buttonState_2 = false;
	globalState.buttonPressed.buttonState_3 = false;
	globalState.buttonPressed.buttonState_4 = false;
	globalState.buttonPressed.buttonState_5 = false;
	globalState.buttonPressed.buttonState_6 = false;
	globalState.buttonPressed.buttonState_7 = false;
	globalState.buttonPressed.buttonState_8 = false;

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

