/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (FLASH + L3G4200D + LOG)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_WAIT_PLACEMENT = 0,   // 20 s para ponerla en la mesa
    STATE_MEASURE_POS,          // log giro wi+
    STATE_WAIT_REVERSE,         // tiempo para invertir giro
    STATE_MEASURE_NEG,          // log giro wi-
    STATE_IDLE                  // espera comando UART
} system_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- FLASH ---- */
#define FLASH_CS_LOW()        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define FLASH_CS_HIGH()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define FLASH_WRITE_ENABLE    0x06
#define FLASH_PAGE_PROGRAM    0x02
#define FLASH_READ_DATA       0x03
#define FLASH_READ_STATUS     0x05
#define FLASH_WRITE_STATUS    0x01
#define FLASH_SECTOR_ERASE_4K 0x20

#define FLASH_TARGET_ADDRESS  0x000000  // sector initialization

/* LED to read State - PA15 - (EXT_LED_Pin) */
#define EXT_LED_ON()      HAL_GPIO_WritePin(EXT_LED_GPIO_Port, EXT_LED_Pin, GPIO_PIN_SET)
#define EXT_LED_OFF()     HAL_GPIO_WritePin(EXT_LED_GPIO_Port, EXT_LED_Pin, GPIO_PIN_RESET)
#define EXT_LED_TOGGLE()  HAL_GPIO_TogglePin(EXT_LED_GPIO_Port, EXT_LED_Pin)

/* Blinking period */
#define LOG_LED_BLINK_MS  200U   // 200 ms (5 Hz approx.)


/* ---- L3G4200D ---- */
#define L3G4200D_ADDR        (0x69 << 1)
#define L3G4200D_WHO_AM_I    0x0F
#define L3G4200D_CTRL_REG1   0x20
#define L3G4200D_CTRL_REG4   0x23
#define L3G4200D_OUT_X_L     0x28

/* ---- Time defines ---- */
#define START_DELAY_MS         20000U   // 20 s to put the gyroscope on the table
#define MEASURE_POS_WINDOW_MS  30000U   // 30 s in wi+
#define MEASURE_NEG_WINDOW_MS  30000U   // 30 s in wi-
#define REVERSE_DELAY_MS       20000U   // 20 s to invert the direction
#define SAMPLE_PERIOD_MS       10U      // 100 Hz
#define MAX_SAMPLES            6002    // approximate maximum of samples with around 1 minute

// Rotary table's Nominal velocity in rad/s
#define OMEGA_REF_RAD_S   (5.0f * PI_F / 180.0f)   // Dummy value until we go to lab

/* Gyroscope conversions */
#define PI_F                 3.14159265f
static const float g_lsb2dps = 0.00875f;   // ±250 dps → 8.75 mdps/LSB

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
/* Gyroscope's bias */
static int16_t bias_x = 0, bias_y = 0, bias_z = 0;

/* Global state for system */
static system_state_t g_state = STATE_WAIT_PLACEMENT;

/* Timers */
static uint32_t t_start_wait = 0;
static uint32_t t_start_measure = 0;
static uint32_t t_last_sample = 0;
static uint32_t t_last_blink = 0;

/* Signals where wi+ ends and where wi- starts*/
static uint16_t split_index = 0;

/* Time to start inverting the direction */
static uint32_t t_start_reverse = 0;

/* Logging in Flash */
static uint32_t flash_write_addr = FLASH_TARGET_ADDRESS;
static uint16_t sample_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* ---- UART ---- */
void UART_Print(char *msg);

/* ---- FLASH ---- */
uint8_t Flash_ReadStatus(void);
void Flash_WaitForWriteEnd(void);
void Flash_WriteEnable(void);
void Flash_ClearWriteProtect(void);
void Flash_Erase4K(uint32_t address);
uint8_t Flash_ReadByte(uint32_t address);
void  Flash_WriteByte(uint32_t address, uint8_t data);
void  Flash_ReadID(void);

/* ---- I2C / L3G4200D ---- */
uint8_t I2C_ReadByte(uint8_t reg);
void    I2C_WriteByte(uint8_t reg, uint8_t value);
void    L3G4200D_Init(void);
void    L3G4200D_ReadGyro(int16_t *x, int16_t *y, int16_t *z);
static  void Gyro_Calibrate(uint16_t n);

/* ---- Logging ---- */
static void Log_Sample_ToFlash(int16_t gx, int16_t gy, int16_t gz);
static void Send_AllData_FromFlash(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ======================= FLASH ======================= */
uint8_t Flash_ReadStatus(void)
{
    uint8_t cmd = FLASH_READ_STATUS;
    uint8_t status;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, &status, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    return status;
}

void Flash_WaitForWriteEnd(void)
{
    while (Flash_ReadStatus() & 0x01) {
        /* bit0 = WIP (Write In Progress) */
    }
}

void Flash_WriteEnable(void)
{
    uint8_t cmd = FLASH_WRITE_ENABLE;

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    HAL_Delay(1);
}

void Flash_ClearWriteProtect(void)
{
    Flash_WriteEnable();
    uint8_t cmd[3] = { FLASH_WRITE_STATUS, 0x00, 0x00 };

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    Flash_WaitForWriteEnd();
}

void Flash_Erase4K(uint32_t address)
{
    Flash_WriteEnable();

    uint8_t cmd[4] = {
        FLASH_SECTOR_ERASE_4K,
        (uint8_t)((address >> 16) & 0xFF),
        (uint8_t)((address >> 8)  & 0xFF),
        (uint8_t)( address        & 0xFF)
    };

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    Flash_WaitForWriteEnd();
}

uint8_t Flash_ReadByte(uint32_t address)
{
    uint8_t cmd[4];
    uint8_t recv;

    cmd[0] = FLASH_READ_DATA;
    cmd[1] = (uint8_t)((address >> 16) & 0xFF);
    cmd[2] = (uint8_t)((address >> 8)  & 0xFF);
    cmd[3] = (uint8_t)( address        & 0xFF);

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, &recv, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    return recv;
}

void Flash_WriteByte(uint32_t address, uint8_t data)
{

    Flash_WriteEnable();

    uint8_t cmd[4] = {
        FLASH_PAGE_PROGRAM,
        (uint8_t)((address >> 16) & 0xFF),
        (uint8_t)((address >> 8)  & 0xFF),
        (uint8_t)( address        & 0xFF)
    };

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    Flash_WaitForWriteEnd();
}

void Flash_ReadID(void)
{
    uint8_t cmd = 0x9F;
    uint8_t id[3];

    FLASH_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, id, 3, HAL_MAX_DELAY);
    FLASH_CS_HIGH();

    char msg[64];
    sprintf(msg, "JEDEC ID: %02X %02X %02X\r\n", id[0], id[1], id[2]);
    UART_Print(msg);
}

/* ======================= UART ======================= */
void UART_Print(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

/* ======================= I2C / L3G4200D ======================= */
uint8_t I2C_ReadByte(uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR, reg,
                     I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    return data;
}

void I2C_WriteByte(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, L3G4200D_ADDR, reg,
                      I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

void L3G4200D_Init(void)
{
    uint8_t who = 0;

    /* Try until gyroscope is found */
    do {
        who = I2C_ReadByte(L3G4200D_WHO_AM_I);

        if (who != 0xD3) {
            UART_Print("L3G4200D not found! Retrying in 1 s...\r\n");

            /* Indicate error with LED. In this case, since it is OFF at the start,
             * the LED will be turned ON */
            EXT_LED_TOGGLE();

            HAL_Delay(1000);
        }

    } while (who != 0xD3);

    UART_Print("L3G4200D detected successfully.\r\n");

    /* CTRL_REG1: 0x3F -> ODR=200Hz, BW=50Hz, PD=1, X/Y/Z enable */
    I2C_WriteByte(L3G4200D_CTRL_REG1, 0x3F);

    /* CTRL_REG4: BDU=1, FS=±250 dps */
    I2C_WriteByte(L3G4200D_CTRL_REG4, 0x80);
}

void L3G4200D_ReadGyro(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
    HAL_I2C_Mem_Read(&hi2c1, L3G4200D_ADDR,
                     L3G4200D_OUT_X_L | 0x80,
                     I2C_MEMADD_SIZE_8BIT, buf, 6, 100);

    *x = (int16_t)((buf[1] << 8) | buf[0]);
    *y = (int16_t)((buf[3] << 8) | buf[2]);
    *z = (int16_t)((buf[5] << 8) | buf[4]);
}

static void Gyro_Calibrate(uint16_t n)
{
    int32_t sx = 0, sy = 0, sz = 0;
    int16_t x, y, z;

    UART_Print("Calibrating gyro bias, keep still...\r\n");

    for (uint16_t i = 0; i < n; i++) {
        L3G4200D_ReadGyro(&x, &y, &z);
        sx += x; sy += y; sz += z;
        HAL_Delay(2);
    }

    bias_x = (int16_t)(sx / (int32_t)n);
    bias_y = (int16_t)(sy / (int32_t)n);
    bias_z = (int16_t)(sz / (int32_t)n);

    UART_Print("Gyro bias calibrated.\r\n");
}

/* ======================= Logging and sending ======================= */

static void Log_Sample_ToFlash(int16_t gx, int16_t gy, int16_t gz)
{
    if (sample_count >= MAX_SAMPLES) return;  // safety measure

    // If the pointer has already reached the sector's end, then use the next sector
    if ((flash_write_addr & 0x0FFF) >= (4096 - 6)) {
        uint32_t next_sector = (flash_write_addr & ~0x0FFF) + 0x1000;
        Flash_Erase4K(next_sector);
        flash_write_addr = next_sector;
    }

    /* Save raw X, Y, Z values, 6 bytes per sample */
    Flash_WriteByte(flash_write_addr++, (uint8_t)(gx & 0xFF));
    Flash_WriteByte(flash_write_addr++, (uint8_t)((gx >> 8) & 0xFF));

    Flash_WriteByte(flash_write_addr++, (uint8_t)(gy & 0xFF));
    Flash_WriteByte(flash_write_addr++, (uint8_t)((gy >> 8) & 0xFF));

    Flash_WriteByte(flash_write_addr++, (uint8_t)(gz & 0xFF));
    Flash_WriteByte(flash_write_addr++, (uint8_t)((gz >> 8) & 0xFF));

    sample_count++;
}

/*Read all logged data and send it through UART, text format*/
static void Send_AllData_FromFlash(void)
{
    char line[160];
    uint32_t addr = FLASH_TARGET_ADDRESS;

    // ----- Accums for average of wi+ and wi- -----
    float sum_pos_x = 0.0f, sum_pos_y = 0.0f, sum_pos_z = 0.0f;
    float sum_neg_x = 0.0f, sum_neg_y = 0.0f, sum_neg_z = 0.0f;
    uint16_t cnt_pos = 0, cnt_neg = 0;

    // header for CSV
    UART_Print("dir,wx_rad_s,wy_rad_s,wz_rad_s,omega_rad_s\r\n");

    for (uint16_t i = 0; i < sample_count; i++) {

        uint8_t b0 = Flash_ReadByte(addr++);
        uint8_t b1 = Flash_ReadByte(addr++);
        uint8_t b2 = Flash_ReadByte(addr++);
        uint8_t b3 = Flash_ReadByte(addr++);
        uint8_t b4 = Flash_ReadByte(addr++);
        uint8_t b5 = Flash_ReadByte(addr++);

        int16_t gx_raw = (int16_t)((b1 << 8) | b0);
        int16_t gy_raw = (int16_t)((b3 << 8) | b2);
        int16_t gz_raw = (int16_t)((b5 << 8) | b4);

        // Conversion to rad/s
        float gx_dps = (gx_raw - bias_x) * g_lsb2dps;
        float gy_dps = (gy_raw - bias_y) * g_lsb2dps;
        float gz_dps = (gz_raw - bias_z) * g_lsb2dps;

        float gx = gx_dps * (PI_F / 180.0f);
        float gy = gy_dps * (PI_F / 180.0f);
        float gz = gz_dps * (PI_F / 180.0f);
        float omega = sqrtf(gx*gx + gy*gy + gz*gz);

        // dir = +1 for wi+ (first window), -1 for wi- (second window)
        int8_t dir = (i < split_index) ? 1 : -1;

        // Accum for averages of wi+ and wi-
        if (dir == 1) {
            sum_pos_x += gx;
            sum_pos_y += gy;
            sum_pos_z += gz;
            cnt_pos++;
        } else {
            sum_neg_x += gx;
            sum_neg_y += gy;
            sum_neg_z += gz;
            cnt_neg++;
        }

        snprintf(line, sizeof(line),
                 "%d,%.6f,%.6f,%.6f,%.6f\r\n",
                 dir, gx, gy, gz, omega);

        UART_Print(line);
    }
    UART_Print("End of data.\r\n");

    if (cnt_pos == 0 || cnt_neg == 0) {
        UART_Print("Not enough samples in wi+ / wi- windows to compute calibration.\r\n");
        return;
    }

    //Averages of wi+ and wi- per axis
    float wx_pos = sum_pos_x / (float)cnt_pos;
    float wy_pos = sum_pos_y / (float)cnt_pos;
    float wz_pos = sum_pos_z / (float)cnt_pos;

    float wx_neg = sum_neg_x / (float)cnt_neg;
    float wy_neg = sum_neg_y / (float)cnt_neg;
    float wz_neg = sum_neg_z / (float)cnt_neg;

    // Bias estimation: b_i = (wi+ - wi-) / 2
    float bx = (wx_pos - wx_neg) / 2.0f;
    float by = (wy_pos - wy_neg) / 2.0f;
    float bz = (wz_pos - wz_neg) / 2.0f;

    // Scale factor error:
    // s_i = (wi- - wi+ - 2*omega_ref) / (2*omega_ref)
    float sx = (wx_neg - wx_pos - 2.0f * OMEGA_REF_RAD_S) / (2.0f * OMEGA_REF_RAD_S);
    float sy = (wy_neg - wy_pos - 2.0f * OMEGA_REF_RAD_S) / (2.0f * OMEGA_REF_RAD_S);
    float sz = (wz_neg - wz_pos - 2.0f * OMEGA_REF_RAD_S) / (2.0f * OMEGA_REF_RAD_S);

    // Calibration summary
    UART_Print("---- Calibration summary (per axis) ----\r\n");

    snprintf(line, sizeof(line),
             "wx+: %.6f rad/s, wx-: %.6f rad/s, bx: %.6f rad/s, sx: %.6f\r\n",
             wx_pos, wx_neg, bx, sx);
    UART_Print(line);

    snprintf(line, sizeof(line),
             "wy+: %.6f rad/s, wy-: %.6f rad/s, by: %.6f rad/s, sy: %.6f\r\n",
             wy_pos, wy_neg, by, sy);
    UART_Print(line);

    snprintf(line, sizeof(line),
             "wz+: %.6f rad/s, wz-: %.6f rad/s, bz: %.6f rad/s, sz: %.6f\r\n",
             wz_pos, wz_neg, bz, sz);
    UART_Print(line);

    UART_Print("----------------------------------------\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  /*Gyroscope initialization + bias */
  L3G4200D_Init();
  Gyro_Calibrate(200);

  EXT_LED_ON();  // LED ON

  /* USER CODE BEGIN 2 */

  UART_Print("System boot.\r\n");
  /*Initialize FLASH and clean logging*/
  Flash_ClearWriteProtect();
  Flash_ReadID();
  Flash_Erase4K(FLASH_TARGET_ADDRESS);
  flash_write_addr = FLASH_TARGET_ADDRESS;
  sample_count     = 0;
  split_index = 0;

  UART_Print("Waiting 20 s to start logging...\r\n");

  g_state       = STATE_WAIT_PLACEMENT;
  t_start_wait  = HAL_GetTick();
  t_start_measure = 0;
  t_last_sample = 0;
  t_last_blink = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      switch (g_state)
      {
      case STATE_WAIT_PLACEMENT:
          /* 20s to place the gyroscope on the table */
          if ((now - t_start_wait) >= START_DELAY_MS) {
              UART_Print("Starting gyro logging (POS direction, wi+). Set rate table to +omega.\r\n");
              g_state         = STATE_MEASURE_POS;
              t_start_measure = now;
              t_last_sample   = now;
              t_last_blink    = now;
              /* LED starts blinking */
          }
          break;

      case STATE_MEASURE_POS:
          /* logging while MEASURE_POS_WINDOW_MS */
          if ((now - t_start_measure) >= MEASURE_POS_WINDOW_MS) {
              UART_Print("First logging window (wi+) finished.\r\n");
              /* Index is saved when wi+ is over */
              split_index = sample_count;

              UART_Print("Reverse rotation direction on the rate table (to wi-) in 20 s...\r\n");
              g_state        = STATE_WAIT_REVERSE;
              t_start_reverse = now;
              EXT_LED_ON();  // LED will remain ON
          } else {
              /* logging data */
              if ((now - t_last_sample) >= SAMPLE_PERIOD_MS) {
                  t_last_sample = now;
                  int16_t gx, gy, gz;
                  L3G4200D_ReadGyro(&gx, &gy, &gz);
                  Log_Sample_ToFlash(gx, gy, gz);
              }

              /* LED blinking */
              if ((now - t_last_blink) >= LOG_LED_BLINK_MS) {
                  t_last_blink = now;
                  EXT_LED_TOGGLE();
              }
          }
          break;

      case STATE_WAIT_REVERSE:
          if ((now - t_start_reverse) >= REVERSE_DELAY_MS) {
              UART_Print("Starting gyro logging (NEG direction, wi-).\r\n");
              g_state         = STATE_MEASURE_NEG;
              t_start_measure = now;
              t_last_sample   = now;
              t_last_blink    = now;
          }
          break;

      case STATE_MEASURE_NEG:
          /* logging while MEASURE_NEG_WINDOW_MS */
          if ((now - t_start_measure) >= MEASURE_NEG_WINDOW_MS) {
              UART_Print("Second logging window (wi-) finished. Going to IDLE.\r\n");
              g_state = STATE_IDLE;
              EXT_LED_ON(); // LED ON
          } else {
              if ((now - t_last_sample) >= SAMPLE_PERIOD_MS) {
                  t_last_sample = now;
                  int16_t gx, gy, gz;
                  L3G4200D_ReadGyro(&gx, &gy, &gz);
                  Log_Sample_ToFlash(gx, gy, gz);
              }

              if ((now - t_last_blink) >= LOG_LED_BLINK_MS) {
                  t_last_blink = now;
                  EXT_LED_TOGGLE();
              }
          }
          break;

      case STATE_IDLE:
      {
          uint8_t rx;
          if (HAL_UART_Receive(&huart2, &rx, 1, 10) == HAL_OK) {
              if (rx == 'D' || rx == 'd') {
                  Send_AllData_FromFlash();
              } else {
                  UART_Print("Unknown cmd. Send 'D' to download data.\r\n");
              }
          }
      }
          break;

      default:
          g_state = STATE_IDLE;
          break;
      }

  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */


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
