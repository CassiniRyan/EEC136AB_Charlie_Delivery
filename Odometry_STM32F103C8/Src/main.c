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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "kalman_filter.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern volatile uint32_t systick_counter;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// IMU Slave address
#define IMU_ADDR 0x6A
#define MAG_ADDR 0x1E
#define ACC_ADDR 0x19

// Configuration registers (unchanged from your code)
#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define OUTX_L_A 0x28
#define OUTX_H_A 0x29
#define OUTY_L_A 0x2A
#define OUTY_H_A 0x2B
#define OUTZ_L_A 0x2C
#define OUTZ_H_A 0x2D

#define OUT_TEMP_L_A 0x0D
#define OUT_TEMP_H_A 0x0C
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define OUT_X_L_REG_M 0x68
#define OUT_X_H_REG_M 0x69
#define OUT_Y_L_REG_M 0x6A
#define OUT_Y_H_REG_M 0x6B
#define OUT_Z_L_REG_M 0x6C
#define OUT_Z_H_REG_M 0x6D

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_SYSTICK_Callback(void)
{
  static uint32_t counter = 0;
  counter++;
  if (counter >= 1000)
  {
    counter = 0;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  systick_counter++;  // Increment global systick counter
}

float Converter(uint8_t xr_l, uint8_t xr_h, uint8_t mode)
{
  float x, param;
  uint16_t xr;

  if (mode == 0)
    param = 60.0f; // Temperature
  else if (mode == 1)
    param = 19.6f; // Accelerometer
  else if (mode == 2)
    param = 250.0f; // Gyroscope
  else if (mode == 3)
    param = 50.0f; // Magnetometer

  xr = ((int16_t)xr_h << 8) | xr_l;

  if (xr > 32768)
  {
    x = (xr - 65536) * param / 32768.0f;
  }
  else
  {
    x = xr * param / 32768.0f;
  }
  return x;
}

void I2C_Write_Register(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
  HAL_I2C_Mem_Write(hi2c, devAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef I2C_Read_Register(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
  return HAL_I2C_Mem_Read(hi2c, devAddr << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

void Init_Register(I2C_HandleTypeDef *hi2c)
{
  uint8_t PIN_CTRL[] = {0x02, 0x7F};
  uint8_t CTRL1_XL[] = {0x10, 0x60};
  uint8_t CTRL2_G[] = {0x11, 0x60};
  uint8_t CTRL4_C[] = {0x13, 0x00};
  uint8_t CTRL6_C[] = {0x15, 0x00};
  uint8_t CTRL7_G[] = {0x16, 0x00};

  uint8_t TEMP_CFG_REG_A[] = {0x1F, 0xC0};
  uint8_t CTRL_REG1_A[] = {0x20, 0x77};
  uint8_t CTRL_REG2_A[] = {0x21, 0x00};
  uint8_t CTRL_REG3_A[] = {0x22, 0x00};
  uint8_t CTRL_REG4_A[] = {0x23, 0x80};
  uint8_t CTRL_REG_A_M[] = {0x60, 0x9C};
  uint8_t CFG_REG_B_M[] = {0x61, 0x02};
  uint8_t CFG_REG_C_M[] = {0x62, 0x10};

  I2C_Write_Register(hi2c, IMU_ADDR, CTRL1_XL[0], CTRL1_XL[1]);
  I2C_Write_Register(hi2c, IMU_ADDR, CTRL2_G[0], CTRL2_G[1]);
  I2C_Write_Register(hi2c, IMU_ADDR, CTRL4_C[0], CTRL4_C[1]);
  I2C_Write_Register(hi2c, IMU_ADDR, CTRL6_C[0], CTRL6_C[1]);
  I2C_Write_Register(hi2c, IMU_ADDR, CTRL7_G[0], CTRL7_G[1]);

  I2C_Write_Register(hi2c, ACC_ADDR, TEMP_CFG_REG_A[0], TEMP_CFG_REG_A[1]);
  I2C_Write_Register(hi2c, ACC_ADDR, CTRL_REG1_A[0], CTRL_REG1_A[1]);
  I2C_Write_Register(hi2c, ACC_ADDR, CTRL_REG2_A[0], CTRL_REG2_A[1]);
  I2C_Write_Register(hi2c, ACC_ADDR, CTRL_REG3_A[0], CTRL_REG3_A[1]);
  I2C_Write_Register(hi2c, ACC_ADDR, CTRL_REG4_A[0], CTRL_REG4_A[1]);
  I2C_Write_Register(hi2c, ACC_ADDR, CTRL_REG_A_M[0], CTRL_REG_A_M[1]);

  I2C_Write_Register(hi2c, MAG_ADDR, CTRL_REG_A_M[0], CTRL_REG_A_M[1]);
  I2C_Write_Register(hi2c, MAG_ADDR, CFG_REG_B_M[0], CFG_REG_B_M[1]);
  I2C_Write_Register(hi2c, MAG_ADDR, CFG_REG_C_M[0], CFG_REG_C_M[1]);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  KalmanFilter kf1;
  kalman_init(&kf1, 10);

  // Sensor data buffers
  uint8_t TEMP_IMU1[] = {0x00, 0x00};
  uint8_t GYR_P_IMU1[] = {0x00, 0x00};
  uint8_t GYR_R_IMU1[] = {0x00, 0x00};
  uint8_t GYR_Y_IMU1[] = {0x00, 0x00};
  uint8_t ACC_X_IMU1[] = {0x00, 0x00};
  uint8_t ACC_Y_IMU1[] = {0x00, 0x00};
  uint8_t ACC_Z_IMU1[] = {0x00, 0x00};
  uint8_t TEMP_ECP1[] = {0x00, 0x00};
  uint8_t ACC_X_ECP1[] = {0x00, 0x00};
  uint8_t ACC_Y_ECP1[] = {0x00, 0x00};
  uint8_t ACC_Z_ECP1[] = {0x00, 0x00};
  uint8_t MAG_X_ECP1[] = {0x00, 0x00};
  uint8_t MAG_Y_ECP1[] = {0x00, 0x00};
  uint8_t MAG_Z_ECP1[] = {0x00, 0x00};

  float temp_imu1 = 0.0f, gyr_r_imu1 = 0.0f, gyr_p_imu1 = 0.0f, gyr_y_imu1 = 0.0f;
  float acc_x_imu1 = 0.0f, acc_y_imu1 = 0.0f, acc_z_imu1 = 0.0f;
  float temp_ecp1 = 0.0f, acc_x_ecp1 = 0.0f, acc_y_ecp1 = 0.0f, acc_z_ecp1 = 0.0f;
  float mag_x_ecp1 = 0.0f, mag_y_ecp1 = 0.0f, mag_z_ecp1 = 0.0f;

  uint8_t TEMP_IMU2[] = {0x00, 0x00};
  uint8_t GYR_P_IMU2[] = {0x00, 0x00};
  uint8_t GYR_R_IMU2[] = {0x00, 0x00};
  uint8_t GYR_Y_IMU2[] = {0x00, 0x00};
  uint8_t ACC_X_IMU2[] = {0x00, 0x00};
  uint8_t ACC_Y_IMU2[] = {0x00, 0x00};
  uint8_t ACC_Z_IMU2[] = {0x00, 0x00};
  uint8_t TEMP_ECP2[] = {0x00, 0x00};
  uint8_t ACC_X_ECP2[] = {0x00, 0x00};
  uint8_t ACC_Y_ECP2[] = {0x00, 0x00};
  uint8_t ACC_Z_ECP2[] = {0x00, 0x00};
  uint8_t MAG_X_ECP2[] = {0x00, 0x00};
  uint8_t MAG_Y_ECP2[] = {0x00, 0x00};
  uint8_t MAG_Z_ECP2[] = {0x00, 0x00};

  float temp_imu2 = 0.0f, gyr_r_imu2 = 0.0f, gyr_p_imu2 = 0.0f, gyr_y_imu2 = 0.0f;
  float acc_x_imu2 = 0.0f, acc_y_imu2 = 0.0f, acc_z_imu2 = 0.0f;
  float temp_ecp2 = 0.0f, acc_x_ecp2 = 0.0f, acc_y_ecp2 = 0.0f, acc_z_ecp2 = 0.0f;
  float mag_x_ecp2 = 0.0f, mag_y_ecp2 = 0.0f, mag_z_ecp2 = 0.0f;

  char buffer[1024];
  HAL_StatusTypeDef I2C_Status;

  Init_Register(&hi2c1); // Sensor Set 1 on I2C1
  Init_Register(&hi2c2); // Sensor Set 2 on I2C2

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
