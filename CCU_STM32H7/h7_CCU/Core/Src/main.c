/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADDR_24LCxx_Write 0xA0
#define ADDR_24LCxx_Read  0xA1
#define BufferSize        0x100

#define BUTTON_GPIO_Port  GPIOD   // For example, if D9 is on GPIOD pin 9
#define BUTTON_Pin        GPIO_PIN_9

#define LED_GPIO_Port     GPIOD   // For example, if D10 is on GPIOD pin 10
#define LED_Pin           GPIO_PIN_10

uint32_t x_axis = 0;
uint32_t y_axis = 0;
char uartMsg[50];
uint8_t receivedData[20];
uint8_t rxByte2;  // Used for USART2 RX interrupt

// New variables for main-based processing of received data:
volatile uint8_t rxFlag = 0;    // Flag set when a new byte is received
volatile uint8_t rxData = 0;    // Holds the received byte

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t I2C_Buffer_Write[BufferSize], ReadBuffer[BufferSize];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
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
  HAL_Init();
  
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  
  /* Configure the system clock */
  SystemClock_Config();
  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
  
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
  // Start interrupt-based reception on USART2 (1 byte at a time)
  if (HAL_UART_Receive_IT(&huart2, &rxByte2, 1) != HAL_OK)
  {
      Error_Handler();
  }
  
  // Turn LED ON by default
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */
  
  // Initialize button state (assume pull-up, so default is SET)
  GPIO_PinState lastButtonState = GPIO_PIN_SET;
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Optional: transmit test data via USART1 */
    HAL_UART_Transmit(&huart1, (uint8_t*)"666\n", 4, HAL_MAX_DELAY);
    
    HAL_Delay(100);
    
    // Read ADC1 for x_axis (PF11)
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      x_axis = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    
    // Read ADC2 for y_axis (PF13)
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
    {
      y_axis = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);
    
    // Check joystick direction and transmit message via USART2
    if (x_axis <= 3000 && y_axis <= 3000)
    {
        sprintf(uartMsg, "moving up left\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    else if (x_axis <= 3000 && y_axis >= 55000)
    {
        sprintf(uartMsg, "moving up right\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    else if (x_axis >= 55000 && y_axis <= 3000)
    {
        sprintf(uartMsg, "moving down left\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    else if (x_axis >= 55000 && y_axis >= 55000)
    {
        sprintf(uartMsg, "moving down right\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    else if (x_axis <= 3000)  // Only vertical (up) if not diagonal
    {
        sprintf(uartMsg, "moving up\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    else if (x_axis >= 55000) // Only vertical (down) if not diagonal
    {
        sprintf(uartMsg, "moving down\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    else if (y_axis <= 3000)  // Only horizontal (left) if not diagonal
    {
        sprintf(uartMsg, "moving left\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    else if (y_axis >= 55000) // Only horizontal (right) if not diagonal
    {
        sprintf(uartMsg, "moving right\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    else
    {
        sprintf(uartMsg, "not moving\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
    }
    
    HAL_Delay(100);
    
    // Check the button state and toggle LED if pressed
    GPIO_PinState currentButtonState = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
    if (currentButtonState == GPIO_PIN_RESET && lastButtonState == GPIO_PIN_SET)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(50); // simple debounce delay
    }
    lastButtonState = currentButtonState;
    
    // --- New feature: Process received USART2 data in main ---
    if (rxFlag == 1)
    {
      // Format the received byte and transmit it via USART2
			sprintf(uartMsg, "Received: 0x%02X\r\n", rxData);
			HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), HAL_MAX_DELAY);
      // Clear the flag so that we only print once per received byte
      rxFlag = 0;
    }
  /* USER CODE END WHILE */
  
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Callback for UART RX complete (called by HAL when 1 byte is received)
  * @param huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
  {
    // Immediately echo the single character
    HAL_UART_Transmit(&huart2, &rxByte2, 1, HAL_MAX_DELAY);

    // Restart reception
    HAL_UART_Receive_IT(&huart2, &rxByte2, 1);
}
	}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
  /* Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  
  /* Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  
  /* Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  /* Initializes the peripherals clock */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    // Stay here in case of error
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief Reports the name of the source file and the source line number
  *        where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  // User can add his own implementation to report the file name and line number,
  // ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
}
#endif /* USE_FULL_ASSERT */
