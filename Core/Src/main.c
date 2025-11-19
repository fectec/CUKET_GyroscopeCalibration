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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define FLASH_WRITE_ENABLE  0x06
#define FLASH_PAGE_PROGRAM  0x02
#define FLASH_READ_DATA     0x03

#define FLASH_TARGET_ADDRESS 0x000000

#define L3G4200D_ADDR       (0x69 << 1)
#define L3G4200D_WHO_AM_I   0x0F
#define L3G4200D_CTRL_REG1  0x20
#define L3G4200D_CTRL_REG4  0x23
#define L3G4200D_OUT_X_L    0x28
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Flash_ReadStatus(void) {
    uint8_t cmd = 0x05;
    uint8_t status;

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, &status, 1, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    return status;
}

void Flash_WaitForWriteEnd(void) {
    while (Flash_ReadStatus() & 0x01);
}

void Flash_WriteEnable(void) {
    uint8_t cmd = FLASH_WRITE_ENABLE;

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    HAL_Delay(1);
}

void Flash_ClearWriteProtect(void) {
    Flash_WriteEnable();
    uint8_t cmd[] = { 0x01, 0x00, 0x00 };

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    Flash_WaitForWriteEnd();
}

void Flash_Erase4K(uint32_t address) {

    Flash_WriteEnable();

    uint8_t cmd[4] = { 0x20,

                      (address >> 16) & 0xFF,

                      (address >> 8) & 0xFF,

                      address & 0xFF };

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    Flash_WaitForWriteEnd();
}



uint8_t Flash_ReadByte(uint32_t address) {

    uint8_t cmd[4];

    uint8_t recv;

    cmd[0] = FLASH_READ_DATA;

    cmd[1] = (address >> 16) & 0xFF;

    cmd[2] = (address >> 8) & 0xFF;

    cmd[3] = address & 0xFF;

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);

    HAL_SPI_Receive(&hspi2, &recv, 1, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    return recv;
}

void Send_String(char* str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void Flash_ReadID(void) {
    uint8_t cmd = 0x9F;

    uint8_t id[3];

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(&hspi2, id, 3, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    char msg[64];

    sprintf(msg, "JEDEC ID: %02X %02X %02X\r\n", id[0], id[1], id[2]);

    Send_String(msg);
}

void Test_WriteEnable_Status(void) {

    Flash_WriteEnable();

    HAL_Delay(1);

    uint8_t status = Flash_ReadStatus();

    char msg[64];

    sprintf(msg, "After WREN, Status Reg: 0x%02X\r\n", status);

    Send_String(msg);
}

void Flash_WriteByte(uint32_t address, uint8_t data) {
    Flash_WriteEnable();

    uint8_t cmd[4] = {

            0x02,

            (address >> 16) & 0xFF,

            (address >> 8) & 0xFF,

            address & 0xFF

        };

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);

    HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    Flash_WaitForWriteEnd();
}

uint8_t I2C_ReadByte(uint8_t reg)

{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

void I2C_WriteByte(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, L3G4200D_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

void UART_Print(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void L3G4200D_Init(void)
{
    uint8_t who_am_i = I2C_ReadByte(L3G4200D_WHO_AM_I);

    if (who_am_i != 0xD3) {
        UART_Print("L3G4200D not found! WHO_AM_I failed.\r\n");
        Error_Handler();
    }
    else
    {
        UART_Print("L3G4200D detected successfully.\r\n");
    }

    I2C_WriteByte(L3G4200D_CTRL_REG1, 0x0F);  // Power ON, XYZ enable, 100Hz
    I2C_WriteByte(L3G4200D_CTRL_REG4, 0x00);  // ±250 dps
}

void L3G4200D_ReadGyro(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR, L3G4200D_OUT_X_L | 0x80, I2C_MEMADD_SIZE_8BIT, buf, 6, 100);

    *x = (int16_t)(buf[1] << 8 | buf[0]);
    *y = (int16_t)(buf[3] << 8 | buf[2]);
    *z = (int16_t)(buf[5] << 8 | buf[4]);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  UART_Print("System Initialized\r\n");

  L3G4200D_Init();

  int16_t gyro_x, gyro_y, gyro_z;

  char msg[64];

  Send_String("Test\r\n");

  // Check ID:20 BA 21
  Flash_ReadID();

  // Check status
  uint8_t status = Flash_ReadStatus();
  char buf[64];

  sprintf(buf, "Status Reg: 0x%02X\r\n", status);
  Send_String(buf);

  // Test
  HAL_Delay(100);

  Test_WriteEnable_Status();

  Send_String("\r\n");

  Send_String("read and write \r\n");

  Send_String("Enter a number (0-9):\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	uint8_t rx;

	HAL_UART_Receive(&huart2, &rx, 1, HAL_MAX_DELAY);

	if (rx >= '0' && rx <= '9') {

	  char echo[32];
	  sprintf(echo, "You entered: %c\r\n", rx);
	  Send_String(echo);

	  // Write

	  uint8_t val = rx - '0';
	  Flash_Erase4K(FLASH_TARGET_ADDRESS);
	  Send_String("Sector Erased!\r\n");
	  Flash_WriteByte(FLASH_TARGET_ADDRESS, val);
	  HAL_Delay(10);

	  // Read

	  uint8_t read_val = Flash_ReadByte(FLASH_TARGET_ADDRESS);
	  char msg2[64];
	  sprintf(msg2, "Read: %d\r\n", read_val);
	  Send_String(msg2);

	  L3G4200D_ReadGyro(&gyro_x, &gyro_y, &gyro_z);

	  snprintf(msg, sizeof(msg), "X:%d Y:%d Z:%d\r\n", gyro_x, gyro_y, gyro_z);

	  UART_Print(msg);

	  HAL_Delay(500);
	}
	else
	{
	  Send_String("Invalid input. Enter 0-9 only.\r\n");
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
