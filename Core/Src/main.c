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

// Flash Memory
#define FLASH_CS_LOW()   				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()  				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define FLASH_JEDEC_ID_LEN  			3
#define FLASH_PAGE_SIZE                 256

#define FLASH_CMD_READ_STATUS_REGISTER	0x05
#define FLASH_CMD_WRITE_ENABLE		    0x06
#define FLASH_CMD_WRITE_STATUS_REG      0x01
#define FLASH_CMD_SECTOR_ERASE_4K       0x20
#define FLASH_CMD_READ_DATA     		0x03
#define FLASH_CMD_READ_ID     		    0x9F
#define FLASH_CMD_PAGE_PROGRAM          0x02

// Gyroscope
#define L3G4200D_ADDR       			(0x69 << 1)
#define L3G4200D_WHO_AM_I   			0x0F
#define L3G4200D_CTRL_REG1  			0x20
#define L3G4200D_CTRL_REG4  			0x23
#define L3G4200D_OUT_X_L    			0x28

// Test Configuration
#define LOG_DURATION_MS                 60000   // 60s cycle
#define SAMPLE_PERIOD_MS                10      // 10ms = 100Hz
#define TEST_HEADER_MARKER              0xAA55  // Number for data start

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
// Global Test Variables
uint32_t current_flash_addr = 0x000000;
uint8_t  test_cycle_id = 1;
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

// UART Helper
void UART_Print(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// Flash Memory Driver Functions

/**
  * @brief  Reads the 8-bit Status Register to determine device state.
  * @note   Implements "Command Set READ REGISTER Operations - READ STATUS REGISTER (05h)".
  * @retval uint8_t: The current value of the Status Register.
  */
uint8_t Flash_ReadStatus(void) {
    uint8_t cmd = FLASH_CMD_READ_STATUS_REGISTER;
    uint8_t status;

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, &status, 1, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    return status;
}

/**
  * @brief  Polls Status Register Bit 0 (Write in progress) to wait for internal operations to complete.
  * @note   Blocks execution while the Flash is BUSY (Bit 0 = 1).
  * Waits for WRITE, PROGRAM, or ERASE cycles to finish before returning (Ready = 0).
  */
void Flash_WaitForWriteEnd(void) {
    while (Flash_ReadStatus() & 0x01);
}

/**
  * @brief  Sets the Write Enable Latch (WEL) bit in the Status Register.
  * @note   Implements "Command Set WRITE Operations - WRITE ENABLE (06h)".
  * @retval None
  */
void Flash_WriteEnable(void) {
    uint8_t cmd = FLASH_CMD_WRITE_ENABLE;

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    HAL_Delay(1);
}

/**
  * @brief  Clears Block Protection bits (BP) in the Status Register.
  * @note   Implements "Command Set WRITE REGISTER Operations - WRITE STATUS REGISTER (01h)".
  * Transaction is 8-bit Command + 8-bit Data.
  * Writing 0x00 clears bits 7:2 (BP bits), disabling software protection.
  * @retval None
  */
void Flash_ClearWriteProtect(void) {
    Flash_WriteEnable();

    // Command 0x01 + 1 Data Byte (0x00)
    uint8_t cmd[] = { FLASH_CMD_WRITE_STATUS_REG, 0x00 };

    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    Flash_WaitForWriteEnd();
}

/**
  * @brief  Erases a 4KB Subsector (Sets all bits to 1 / 0xFF).
  * @note   Implements "Command Set ERASE Operations - SUBSECTOR ERASE (20h)".
  * - An ERASE operation changes bits from 0 to 1.
  * - Any address within the subsector is valid for entry.
  * - The Write Enable Latch (WEL) is automatically cleared to 0
  * when the operation completes, regardless of success.
  * @param  address: Any 24-bit address inside the target 4KB sector.
  */
void Flash_Erase4K(uint32_t address) {

    // Before any ERASE command WRITE ENABLE command must be executed
    Flash_WriteEnable();

    uint8_t cmd[] = {
        FLASH_CMD_SECTOR_ERASE_4K,
        (address >> 16) & 0xFF,     // Address High Byte
        (address >> 8) & 0xFF,      // Address Mid Byte
        address & 0xFF              // Address Low Byte
    };

    // S# is driven LOW and held LOW until the eighth bit of the last data byte
    FLASH_CS_LOW();

    HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), HAL_MAX_DELAY);

    // After which [S#] must be driven HIGH
    // If S# is not driven HIGH, the command is not executed
    FLASH_CS_HIGH();

    // When the operation is in progress the write in progress bit is set to 1
    Flash_WaitForWriteEnd();
}

/**
  * @brief  Reads a single byte from a specific address.
  * @note   Implements "Command Set READ MEMORY Operations - READ (03h)".
  * - Supports 3-byte addressing (A[23:0]).
  * @param  address: 24-bit Flash address.
  * @retval uint8_t: The data byte read from memory.
  */
uint8_t Flash_ReadByte(uint32_t address) {
    uint8_t recv_data = 0;

    // Command 0x03 + 3 Bytes of Address
    uint8_t cmd[] = {
        FLASH_CMD_READ_DATA,        // 0x03
        (address >> 16) & 0xFF,     // Address High
        (address >> 8) & 0xFF,      // Address Mid
        address & 0xFF              // Address Low
    };

    // To initiate a command, S# is driven LOW
    FLASH_CS_LOW();

    // Command code is input, followed by input of the address bytes
    HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), HAL_MAX_DELAY);

    // The device will output data from the selected address
    HAL_SPI_Receive(&hspi2, &recv_data, 1, HAL_MAX_DELAY);

    // The operation is terminated by driving S# HIGH
    FLASH_CS_HIGH();

    return recv_data;
}

/**
  * @brief  Reads the 3-byte JEDEC Device ID.
  * @note   Implements "Command Set READ ID Operations - READ ID (9E/9Fh)".
  * - Bytes returned: [1: Manufacturer] [2: Memory Type] [3: Capacity]
  * - WARNING: If an ERASE or PROGRAM cycle is in progress, this command
  * is NOT decoded and the command cycle in progress is not affected.
  * (The chip will likely return garbage or 0xFF if busy).
  */
void Flash_ReadID(void) {
    uint8_t cmd = FLASH_CMD_READ_ID;

    uint8_t id[FLASH_JEDEC_ID_LEN];

    FLASH_CS_LOW();

    // Send Command 0x9F
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);

    // Receive 3 Bytes of ID Data
    HAL_SPI_Receive(&hspi2, id, sizeof(id), HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    char msg[64];

    sprintf(msg, "Flash ID: %02X %02X %02X\r\n", id[0], id[1], id[2]);

    UART_Print(msg);
}

/**
  * @brief  Debug function to verify SPI communication and Write Enable logic.
  * @note   Performs a "Write Enable" (06h) followed by a "Read Status" (05h).
  * Purpose:
  * 1. Confirms the chip is listening to commands.
  * 2. Verifies that Bit 1 (WEL - Write Enable Latch) successfully latches to '1'.
  * * Expected Output: Status Register should have Bit 1 set (e.g., 0x02).
  * If Output is 0x00, SPI communication is likely failing or the chip is hardware-locked.
  */
void Test_WriteEnable_Status(void) {

    Flash_WriteEnable();

    HAL_Delay(1);

    uint8_t status = Flash_ReadStatus();

    char msg[64];

    sprintf(msg, "After WREN, Status Reg: 0x%02X\r\n", status);

    UART_Print(msg);
}

/**
  * @brief  Writes a buffer of data to a specific address.
  * @note   Implements "Command Set PROGRAM Operations - PAGE PROGRAM (02h)".
  * Optimization: Writes multiple bytes in one transaction to save time.
  * WARNING: Cannot cross Page Boundaries (256-byte blocks).
  * Ensure (address % 256) + len <= 256.
  * @param  address: 24-bit Flash address.
  * @param  data: Pointer to data buffer.
  * @param  len: Number of bytes to write.
  */
void Flash_WriteBuffer(uint32_t address, uint8_t *data, uint16_t len) {
    Flash_WriteEnable();

    uint8_t cmd[] = {
        FLASH_CMD_PAGE_PROGRAM, // 0x02
        (address >> 16) & 0xFF,
        (address >> 8) & 0xFF,
        address & 0xFF
    };

    FLASH_CS_LOW();

    // 1. Send Command + Address
    HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), HAL_MAX_DELAY);

    // 2. Send the Data Buffer
    HAL_SPI_Transmit(&hspi2, data, len, HAL_MAX_DELAY);

    FLASH_CS_HIGH();

    // 3. Wait ONCE for the whole buffer to be written
    Flash_WaitForWriteEnd();
}

void Flash_WriteByte(uint32_t address, uint8_t data) {
    Flash_WriteBuffer(address, &data, 1);
}

// Gyroscope Driver Functions

/**
  * @brief  Reads a single byte from a specific L3G4200D register.
  * @param  reg: The register address to read from.
  * @retval uint8_t: The value read from the register.
  */
uint8_t L3G4200D_ReadByte(uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

/**
  * @brief  Writes a single byte to a specific L3G4200D register.
  * @param  reg: The register address to write to.
  * @param  value: The data to write.
  */
void L3G4200D_WriteByte(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, L3G4200D_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

/**
  * @brief  Initializes the L3G4200D Gyroscope.
  */
void L3G4200D_Init(void)
{
    // 1. Check Device ID
    uint8_t who_am_i = L3G4200D_ReadByte(L3G4200D_WHO_AM_I);

    if (who_am_i != 0xD3) {
        UART_Print("ERR: Gyro not found!\r\n");
        Error_Handler();
    }
    else
    {
        UART_Print("Gyro Detected.\r\n");
    }

    // 2. Configure Power and Axes (CTRL_REG1)
    // Register Address: 0x20
    // We write 0x0F (Binary 0000 1111) which corresponds to:
    // Bits 7-6 (DR/BW): 00 -> ODR 100Hz, Cut-Off 12.5Hz
    // Bit 3 (PD): 1 -> Normal Mode
    // Bits 2-0 (Zen, Yen, Xen): 111 -> All axes enabled
    L3G4200D_WriteByte(L3G4200D_CTRL_REG1, 0x0F);

    // 3. Configure Full Scale (CTRL_REG4)
    // Register Address: 0x23
    // We write 0x00 (Binary 0000 0000) which corresponds to:
    // Bit 7 (BDU): 0 -> Continuous Update
    // Bit 6 (BLE): 0 -> Data LSB @ lower address (Little Endian)
    // Bits 5-4 (FS): 00 -> 250 dps
    L3G4200D_WriteByte(L3G4200D_CTRL_REG4, 0x00);
}

/**
  * @brief  Reads X, Y, and Z axis raw data from the gyroscope.
  * @note   Uses I2C "Multiple Byte Read" via address auto-increment.
  * If the MSb of the SUB field is 1, the SUB (register address) is automatically incremented.
  */
void L3G4200D_ReadGyro(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];

    // Read 6 bytes starting from OUT_X_L (0x28).
    // Logic: 0x28 (Register) | 0x80 (Auto-Increment Bit)
    // This allows reading X_L, X_H, Y_L, Y_H, Z_L, Z_H in one burst.
    HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR, L3G4200D_OUT_X_L | 0x80, I2C_MEMADD_SIZE_8BIT, buf, 6, 100);

    // Reassemble the data.
    // (BLE bit): Default is 0 (Little Endian).
    // Therefore, Low Byte is at Lower Address (index 0, 2, 4).
    *x = (int16_t)(buf[1] << 8 | buf[0]);
    *y = (int16_t)(buf[3] << 8 | buf[2]);
    *z = (int16_t)(buf[5] << 8 | buf[4]);
}

// Chamber Test Logic

void Perform_Bulk_Erase(void) {
    UART_Print("CMD: Erasing 512KB (Wait ~5s)...\r\n");
    HAL_GPIO_WritePin(EXT_LED_GPIO_Port, EXT_LED_Pin, GPIO_PIN_SET); // LED ON

    // Erase enough for ~10 cycles of 60s @ 100Hz (~360KB needed)
    // Erasing 0x000000 to 0x080000 (512KB)
    for (uint32_t addr = 0; addr < 0x080000; addr += 4096) {
        Flash_Erase4K(addr);
    }

    current_flash_addr = 0x000000;
    test_cycle_id = 1;

    HAL_GPIO_WritePin(EXT_LED_GPIO_Port, EXT_LED_Pin, GPIO_PIN_RESET); // LED OFF
    UART_Print("DONE: Flash Erased.\r\n");
}

void Retrieve_Data(void) {
    UART_Print("--- START DATA ---\r\n");

    uint32_t ptr = 0;
    // Limit search to 512KB to prevent reading forever
    const uint32_t SEARCH_LIMIT = 0x080000;
    char msg[64];

    while (ptr < SEARCH_LIMIT) {

        // --- 1. CRITICAL: Reader Page Boundary Logic ---
        // We must mirror the Writer's logic. If we are close to the end of a page,
        // we know the Writer skipped these bytes. We must skip them too.
        // We check for 6 bytes space because that is the size of a Data Packet.
        if ((ptr % FLASH_PAGE_SIZE) > (FLASH_PAGE_SIZE - 6)) {
            // Calculate how many bytes to skip to get to the next page
            ptr += (FLASH_PAGE_SIZE - (ptr % FLASH_PAGE_SIZE));
            continue; // Force loop to restart at the new aligned address
        }

        // --- 2. Read Data ---
        uint8_t m1 = Flash_ReadByte(ptr);
        uint8_t m2 = Flash_ReadByte(ptr + 1);
        uint16_t marker = (m1 << 8) | m2;

        // CHECK A: Is it the Header (0xAA55)?
        if (marker == TEST_HEADER_MARKER) {
            uint8_t cycle = Flash_ReadByte(ptr + 2);
            sprintf(msg, "\r\nCYCLE_ID:%d\r\n", cycle);
            UART_Print(msg);
            ptr += 4; // Header is 4 bytes
        }
        // CHECK B: Is it Empty Flash (0xFFFF)?
        // Since we handled boundary skipping above, finding FF FF here
        // usually means we genuinely hit the end of the recorded data.
        else if (m1 == 0xFF && m2 == 0xFF) {
            // Double check byte 3 just to be sure it's not a fluke data value
            if (Flash_ReadByte(ptr + 2) == 0xFF) {
                UART_Print("--- END DATA (Found Empty Space) ---\r\n");
                return; // STOP READING
            } else {
                // Rare edge case: 0xFFFF was actual gyro data?
                // Highly unlikely for gyro data, but valid.
                // Treat as data below.
            }
        }

        // CHECK C: Assume it is Data
        // Note: We don't use 'else' here to catch the rare "FFFF data" case
        // if we wanted to be 100% strict, but for now, let's use the standard flow.
        if (marker != TEST_HEADER_MARKER && !(m1 == 0xFF && m2 == 0xFF)) {
            uint8_t d[6];
            for(int i=0; i<6; i++) d[i] = Flash_ReadByte(ptr + i);

            int16_t x = (int16_t)(d[1] << 8 | d[0]);
            int16_t y = (int16_t)(d[3] << 8 | d[2]);
            int16_t z = (int16_t)(d[5] << 8 | d[4]);

            sprintf(msg, "%d,%d,%d\r\n", x, y, z);
            UART_Print(msg);
            ptr += 6;
        }
    }
    UART_Print("--- END DATA (Limit Reached) ---\r\n");
}

void Run_Logging_Cycle(void) {
	char msg[64];
	sprintf(msg, "LOG: Starting %ds Cycle...\r\n", LOG_DURATION_MS / 1000);
	UART_Print(msg);

    HAL_GPIO_WritePin(EXT_LED_GPIO_Port, EXT_LED_Pin, GPIO_PIN_SET);	// LED Solid ON

    // 1. Write Header
    uint8_t header[4];
    header[0] = (TEST_HEADER_MARKER >> 8) & 0xFF;
    header[1] = (TEST_HEADER_MARKER) & 0xFF;
    header[2] = test_cycle_id;
    header[3] = 0x00;

    // Check page alignment for Header (unlikely to cross, but good practice)
    if ((current_flash_addr % FLASH_PAGE_SIZE) > (FLASH_PAGE_SIZE - 4)) {
        current_flash_addr += (FLASH_PAGE_SIZE - (current_flash_addr % FLASH_PAGE_SIZE));
    }
    Flash_WriteBuffer(current_flash_addr, header, 4);
    current_flash_addr += 4;

    // 2. Logging Loop
    uint32_t start_tick = HAL_GetTick();
    int16_t gx, gy, gz;
    uint8_t data_row[6];

    while ((HAL_GetTick() - start_tick) < LOG_DURATION_MS) {
        uint32_t sample_tick = HAL_GetTick();

        // A. Read Sensor
        L3G4200D_ReadGyro(&gx, &gy, &gz);

        data_row[0] = gx & 0xFF;
        data_row[1] = (gx >> 8) & 0xFF;
        data_row[2] = gy & 0xFF;
        data_row[3] = (gy >> 8) & 0xFF;
        data_row[4] = gz & 0xFF;
        data_row[5] = (gz >> 8) & 0xFF;

        // B. Check Page Boundary
        // If current offset + 6 bytes > 256, move to next page
        if ((current_flash_addr % FLASH_PAGE_SIZE) > (FLASH_PAGE_SIZE - 6)) {
             // Fill remaining bytes with 0xFF (implicit by skipping) or explicit dummy?
             // Skipping is fine, retrieving logic handles 0xFF skipping.
             current_flash_addr += (FLASH_PAGE_SIZE - (current_flash_addr % FLASH_PAGE_SIZE));
        }

        // C. Write Flash
        Flash_WriteBuffer(current_flash_addr, data_row, 6);
        current_flash_addr += 6;

        // D. Wait for 10ms (100Hz)
        while ((HAL_GetTick() - sample_tick) < SAMPLE_PERIOD_MS);
    }

    test_cycle_id++;
    HAL_GPIO_WritePin(EXT_LED_GPIO_Port, EXT_LED_Pin, GPIO_PIN_RESET); // LED OFF
    UART_Print("LOG: Cycle Complete.\r\n");
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
  HAL_Delay(5000);				// User experience delay
  UART_Print("\r\n=== MO-2 GYRO CHAMBER SYSTEM ===\r\n");

  L3G4200D_Init();
  Flash_ReadID();				// Verify SPI Flash
  Flash_ClearWriteProtect();	// Ensure we can write

  UART_Print("IDLE: 'e'=Erase, 'r'=Retrieve, BTN=Log\r\n");

  uint8_t uart_rx;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// 1. Check UART Commands
	if (HAL_UART_Receive(&huart2, &uart_rx, 1, 10) == HAL_OK) {
		if (uart_rx == 'e') Perform_Bulk_Erase();
		if (uart_rx == 'r') Retrieve_Data();
	}

	// 2. Check Button (Active Low)
	if (HAL_GPIO_ReadPin(EXT_PUSH_BUTTON_GPIO_Port, EXT_PUSH_BUTTON_Pin) == GPIO_PIN_RESET) {
	  HAL_Delay(50);	// Debounce
	  if (HAL_GPIO_ReadPin(EXT_PUSH_BUTTON_GPIO_Port, EXT_PUSH_BUTTON_Pin) == GPIO_PIN_RESET) {
		  Run_Logging_Cycle();
		  // Wait for release
		  while(HAL_GPIO_ReadPin(EXT_PUSH_BUTTON_GPIO_Port, EXT_PUSH_BUTTON_Pin) == GPIO_PIN_RESET);
	  }
	}

	// 3. IDLE Blink
	HAL_GPIO_TogglePin(EXT_LED_GPIO_Port, EXT_LED_Pin);
	HAL_Delay(500);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin|EXT_LED_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : EXT_LED_Pin */
  GPIO_InitStruct.Pin = EXT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(EXT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_PUSH_BUTTON_Pin */
  GPIO_InitStruct.Pin = EXT_PUSH_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EXT_PUSH_BUTTON_GPIO_Port, &GPIO_InitStruct);

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
