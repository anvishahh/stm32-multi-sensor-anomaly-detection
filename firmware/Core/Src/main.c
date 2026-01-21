/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

/* Thresholds / constants -----------------------------------------------------*/
#define DOUBLE_PRESS_INTERVAL_MS 300U

#define TEMP_THRESHOLD        35.0f
#define PRESSURE_THRESHOLD  1300.0f
#define HUMIDITY_THRESHOLD    98.0f
#define ACCEL_THRESHOLD        9.9f
#define GYRO_THRESHOLD      1200.0f
#define MAGNETO_THRESHOLD      2.5f

#define BUZZER_PORT GPIOD
#define BUZZER_PIN  GPIO_PIN_14

#define GHOST_FAR_THRESHOLD     1.0f
#define GHOST_MEDIUM_THRESHOLD  3.0f
#define GHOST_CLOSE_THRESHOLD   5.0f

/* HT16K33 LED matrix ---------------------------------------------------------*/
#define HT16K33_ADDRESS          0x70
#define HT16K33_CMD_SYSTEM_SETUP 0x20
#define HT16K33_CMD_DISPLAY_SETUP 0x80
#define HT16K33_CMD_BRIGHTNESS   0xE0

#define I2C_PORT     GPIOB
#define I2C_SCL_PIN  GPIO_PIN_8
#define I2C_SDA_PIN  GPIO_PIN_9
#define I2C_INSTANCE I2C1

/* External / retarget --------------------------------------------------------*/
extern void initialise_monitor_handles(void);

/* Private function prototypes ------------------------------------------------*/
static void printAcc(void);
static void printTemp(void);
static void printHumid(void);
static void printPressure(void);
static void printGyro(void);
static void printMag(void);

static void UART1_Init(void);
static void Button_Init(void);
static void MX_GPIO_Init(void);

static void thresholdsChecker(void);
static void SysTick_Init(void);

static float magnetometerCalc(void);
static void LEDBlinkingChange(float mag_magnitude);

static void Buzzer_Init(void);
static void buzzerOn(void);
static void buzzerOff(void);

static void I2C_Init(void);
static void HT16K33_Init(void);
static void heartDisplay(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* Globals --------------------------------------------------------------------*/
volatile uint32_t last_button_press_time = 0;
volatile uint8_t  button_press_count     = 0;
volatile uint8_t  current_mode           = 0;
volatile uint8_t  active                 = 1;
volatile uint8_t  ghost_captured_count   = 0;

uint32_t led_toggle_interval = 1000;
uint32_t last_toggle_time    = 0;

UART_HandleTypeDef huart1;
I2C_HandleTypeDef  hi2c1;

/* Heart pattern for 8x8 matrix */
static uint8_t heart_pattern[8] = {
    0b00011000,
    0b00111100,
    0b01111110,
    0b11111111,
    0b11111111,
    0b01111110,
    0b00111100,
    0b00011000
};

/* Functions ------------------------------------------------------------------*/
static void SysTick_Init(void)
{
    uint32_t sysClock = HAL_RCC_GetHCLKFreq();
    SysTick_Config(sysClock / 1000U); /* 1ms tick */
}

static void Buzzer_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = BUZZER_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);
}

static void buzzerOn(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

static void buzzerOff(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

int main(void)
{
    initialise_monitor_handles();

    HAL_Init();
    SysTick_Init();

    UART1_Init();
    Button_Init();
    MX_GPIO_Init();
    Buzzer_Init();

    I2C_Init();
    HT16K33_Init();

    BSP_ACCELERO_Init();
    BSP_TSENSOR_Init();
    BSP_HSENSOR_Init();
    BSP_PSENSOR_Init();
    BSP_GYRO_Init();
    BSP_MAGNETO_Init();

    uint32_t sensor_pivot_time = uwTick;

    while (1)
    {
        if (current_mode == 0) /* Normal mode: print telemetry */
        {
            if ((uwTick - sensor_pivot_time) >= 1000U)
            {
                printAcc();
                printTemp();
                printHumid();
                printPressure();
                printGyro();
                printMag();
                sensor_pivot_time = uwTick;
            }
        }
        else if (current_mode == 1) /* Detection mode */
        {
            float mag_magnitude = magnetometerCalc();
            LEDBlinkingChange(mag_magnitude);

            if (mag_magnitude >= GHOST_CLOSE_THRESHOLD)
            {
                /* Optional: show heart on matrix */
                /* heartDisplay(); */
                buzzerOn();
            }
            else
            {
                buzzerOff();
            }

            if ((uwTick - last_toggle_time) >= led_toggle_interval)
            {
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
                last_toggle_time = uwTick;
            }

            if ((uwTick - sensor_pivot_time) >= 1000U)
            {
                thresholdsChecker();
                sensor_pivot_time = uwTick;
            }
        }
    }
}

/* Telemetry helpers ----------------------------------------------------------*/
static void printAcc(void)
{
    int16_t accel_data_i16[3] = {0};
    BSP_ACCELERO_AccGetXYZ(accel_data_i16);

    float ax = (float)accel_data_i16[0] * (9.8f / 1000.0f);
    float ay = (float)accel_data_i16[1] * (9.8f / 1000.0f);
    float az = (float)accel_data_i16[2] * (9.8f / 1000.0f);

    float acc_magnitude = sqrtf(ax * ax + ay * ay + az * az);

    char buffer[100];
    int len = snprintf(buffer, sizeof(buffer), "A: %.2f m/s^2\r\n", acc_magnitude);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}

static void printGyro(void)
{
    float gyro_data[3] = {0};
    BSP_GYRO_GetXYZ(gyro_data);

    float gx = gyro_data[0];
    float gy = gyro_data[1];
    float gz = gyro_data[2];

    float gyro_magnitude = sqrtf(gx * gx + gy * gy + gz * gz) / 1000.0f;

    char buffer[100];
    int len = snprintf(buffer, sizeof(buffer), "G: %.2f deg/s\r\n", gyro_magnitude);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}

static void printMag(void)
{
    int16_t mag_data_i16[3] = {0};
    BSP_MAGNETO_GetXYZ(mag_data_i16);

    float mx = (float)mag_data_i16[0];
    float my = (float)mag_data_i16[1];
    float mz = (float)mag_data_i16[2];

    float mag_magnitude = sqrtf(mx * mx + my * my + mz * mz) / 1000.0f;

    char buffer[100];
    int len = snprintf(buffer, sizeof(buffer), "M: %.2f uT\r\n", mag_magnitude);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}

static void printTemp(void)
{
    float temp_data = BSP_TSENSOR_ReadTemp();

    char buffer[50];
    int len = snprintf(buffer, sizeof(buffer), "T: %.2f C\r\n", temp_data);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}

static void printHumid(void)
{
    float humid_data = BSP_HSENSOR_ReadHumidity();

    char buffer[50];
    int len = snprintf(buffer, sizeof(buffer), "H: %.2f %%\r\n", humid_data);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}

static void printPressure(void)
{
    float pressure_data = BSP_PSENSOR_ReadPressure();

    char buffer[50];
    int len = snprintf(buffer, sizeof(buffer), "P: %.2f hPa\r\n", pressure_data);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
}

/* UART / GPIO / Button -------------------------------------------------------*/
static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin       = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 115200;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while (1) {;}
    }
}

static void Button_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_14;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* Threshold checker ----------------------------------------------------------*/
static void thresholdsChecker(void)
{
    char buffer[140];
    int len = 0;

    float temperature = BSP_TSENSOR_ReadTemp();
    if (temperature > TEMP_THRESHOLD)
    {
        len = snprintf(buffer, sizeof(buffer),
                       "Temperature anomaly detected! Latest: %.2f C\r\n", temperature);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
    }

    float pressure = BSP_PSENSOR_ReadPressure();
    if (pressure > PRESSURE_THRESHOLD)
    {
        len = snprintf(buffer, sizeof(buffer),
                       "Pressure anomaly detected! Latest: %.2f hPa\r\n", pressure);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
    }

    float humidity = BSP_HSENSOR_ReadHumidity();
    if (humidity > HUMIDITY_THRESHOLD)
    {
        len = snprintf(buffer, sizeof(buffer),
                       "Humidity anomaly detected! Latest: %.2f %%\r\n", humidity);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
    }

    int16_t accel_data[3] = {0};
    BSP_ACCELERO_AccGetXYZ(accel_data);
    float ax = (float)accel_data[0] * (9.8f / 1000.0f);
    float ay = (float)accel_data[1] * (9.8f / 1000.0f);
    float az = (float)accel_data[2] * (9.8f / 1000.0f);
    float accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);

    if (accel_magnitude > ACCEL_THRESHOLD)
    {
        len = snprintf(buffer, sizeof(buffer),
                       "Acceleration anomaly detected! Latest: %.2f m/s^2\r\n", accel_magnitude);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
    }

    float gyro_data[3] = {0};
    BSP_GYRO_GetXYZ(gyro_data);
    float gx = gyro_data[0], gy = gyro_data[1], gz = gyro_data[2];
    float gyro_magnitude = sqrtf(gx * gx + gy * gy + gz * gz);

    if (gyro_magnitude > GYRO_THRESHOLD)
    {
        len = snprintf(buffer, sizeof(buffer),
                       "Gyro anomaly detected! Latest: %.2f\r\n", gyro_magnitude);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
    }

    int16_t mag_data_i16[3] = {0};
    BSP_MAGNETO_GetXYZ(mag_data_i16);

    float mx = (float)mag_data_i16[0];
    float my = (float)mag_data_i16[1];
    float mz = (float)mag_data_i16[2];
    float mag_magnitude = sqrtf(mx * mx + my * my + mz * mz) / 1000.0f;

    if (mag_magnitude > MAGNETO_THRESHOLD)
    {
        len = snprintf(buffer, sizeof(buffer),
                       "Magnetic anomaly detected! Latest: %.2f uT\r\n", mag_magnitude);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)len, HAL_MAX_DELAY);
    }
}

/* Button interrupt handler ---------------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_13)
        return;

    uint32_t current_time = HAL_GetTick();

    if ((current_time - last_button_press_time) < DOUBLE_PRESS_INTERVAL_MS)
    {
        button_press_count++;
    }
    else
    {
        button_press_count = 1;
    }

    last_button_press_time = current_time;

    /* Double press: toggle mode */
    if (button_press_count == 2)
    {
        current_mode = !current_mode;

        if (current_mode == 0)
        {
            char msg[120];
            int len = snprintf(msg, sizeof(msg),
                               "Returning to Normal Mode...\r\nTotal events captured: %d\r\n",
                               ghost_captured_count);
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, HAL_MAX_DELAY);
            ghost_captured_count = 0;
        }
        else
        {
            const char msg[] = "Entering Detection Mode...\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1), HAL_MAX_DELAY);
        }

        button_press_count = 0;
    }
    /* Single press in detection mode: toggle active + capture event */
    else if (button_press_count == 1 && current_mode == 1)
    {
        if (active)
        {
            active = 0;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            ghost_captured_count++;

            const char msg[] = "Event captured!\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)(sizeof(msg) - 1), HAL_MAX_DELAY);
        }
        else
        {
            active = 1;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        }
    }
}

/* I2C + HT16K33 --------------------------------------------------------------*/
static void I2C_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);

    hi2c1.Instance             = I2C_INSTANCE;
    hi2c1.Init.Timing          = 0x00303D5B; /* ~400kHz on STM32L4 (as you had) */
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    (void)HAL_I2C_Init(&hi2c1);
}

static void HT16K33_Init(void)
{
    uint8_t init_cmds[] = {
        (uint8_t)(HT16K33_CMD_SYSTEM_SETUP | 0x01),   /* oscillator on */
        (uint8_t)(HT16K33_CMD_DISPLAY_SETUP | 0x01),  /* display on */
        (uint8_t)(HT16K33_CMD_BRIGHTNESS | 0x0F)      /* max brightness */
    };

    for (uint32_t i = 0; i < (sizeof(init_cmds) / sizeof(init_cmds[0])); i++)
    {
        HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HT16K33_ADDRESS << 1),
                                &init_cmds[i], 1, HAL_MAX_DELAY);
    }
}

static void heartDisplay(void)
{
    uint8_t data[17];
    data[0] = 0x00;

    for (uint32_t i = 0; i < 8; i++)
    {
        data[1 + i * 2] = heart_pattern[i];
        data[2 + i * 2] = 0x00;
    }

    HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(HT16K33_ADDRESS << 1),
                            data, 17, HAL_MAX_DELAY);
}

/* Magnetometer helpers -------------------------------------------------------*/
static float magnetometerCalc(void)
{
    int16_t mag_data[3] = {0};
    BSP_MAGNETO_GetXYZ(mag_data);

    float mx = (float)mag_data[0];
    float my = (float)mag_data[1];
    float mz = (float)mag_data[2];

    return sqrtf(mx * mx + my * my + mz * mz) / 1000.0f;
}

static void LEDBlinkingChange(float mag_magnitude)
{
    if (mag_magnitude >= GHOST_CLOSE_THRESHOLD)
    {
        led_toggle_interval = 100;
    }
    else if (mag_magnitude >= GHOST_MEDIUM_THRESHOLD)
    {
        led_toggle_interval = 250;
    }
    else if (mag_magnitude >= GHOST_FAR_THRESHOLD)
    {
        led_toggle_interval = 500;
    }
    else
    {
        led_toggle_interval = 1000;
    }
}
