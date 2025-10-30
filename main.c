  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "stdio.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "string.h"
#include "../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include"ssd1306.h"
#include"ssd1306_tests.h"
#include"ssd1306_fonts.h"


/* --- Peripheral Handles --- */
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
I2C_HandleTypeDef hi2c1;


/* --- Role Configuration --- */
#define ROLE_PLAYER      1
#define ROLE_ENFORCER    2
uint8_t role = ROLE_PLAYER;   // Change manually for now


/* --- Thresholds and Timing Game 2--- */
#define MAG_THRESHOLD     400.0f      // Example proximity threshold
#define PROXIMITY_THRESHOLD 100.0f
#define ESCAPE_WINDOW_MS  3000        // 3-second reaction time window
#define COOLDOWN_MS 3000

// Global static variables for Red Light Green Light
static int game_status = 1;
static int game_seconds_count = 0;
static uint32_t game_last_tick = 0;
static uint32_t game_blink_tick = 0;
static uint8_t game_led_state = 0;
static uint8_t game_initialized = 0;  // 0 = not started, 1 = playing, 2 = waiting before start
static uint8_t game_calibrated = 0;
static uint8_t game_over_msg_sent = 0;  // Flag for sending game over message once

static float accel_const[3] = {0};
static float gyro_const[3] = {0};


/* --- Pins --- */
#define BUZZER_PIN         GPIO_PIN_14
#define BUZZER_PORT        GPIOD

/* --- Function Prototypes --- */
float Read_Magnetometer(void);
void UART_Send(char *msg);
uint8_t Button_Pressed(void);
void CatchAndRun(void);
void RedLightGreenLight(void);
static void UART1_Init(void);
static void MX_GPIO_Init(void);
void SystemClock_Config(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void I2C1_Init(void);

/* --- Misc variables --- */
char msg[100]; //message variable declaration

//uint8_t paused = 0;              // 0 = running, 1 = paused
uint32_t last_press_time = 0;    // last button press time (ms)
uint8_t press_pending = 0;       // 1 if a single press waiting for possible double
uint8_t double_press_slow = 0;
volatile uint8_t currentGame = 0; //0 for catcha nd run, 1 for red light green light
volatile uint8_t buzzer_active = 0;


int main(void){
	HAL_Init();
	SystemClock_Config(); //NOTE: ALWAYS INTIALIZE THE SYSTEM CLOCK AFTER HAL
    UART1_Init();
    MX_GPIO_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
	BSP_LED_Init(LED2);
	BSP_MAGNETO_Init();
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_HSENSOR_Init();
    BSP_TSENSOR_Init();
    BSP_PSENSOR_Init();
    HAL_TIM_Base_Start_IT(&htim16);
    HAL_TIM_Base_Start_IT(&htim17);
    I2C1_Init();
    ssd1306_Init();

    /*
    while (1){
    	float magValue = Read_Magnetometer();
    	char msg[50];
    	sprintf(msg, "Magnetometer: %.2f uT\r\n", magValue);
    	UART_Send(msg);
    	HAL_Delay(1000);
    }*/
    ssd1306_Fill(Black);  // Clear display buffer
        	ssd1306_SetCursor(0,32);
        	ssd1306_WriteString("Welcome to EE2028 (It has not been fun)", Font_7x10, White);
        	ssd1306_UpdateScreen();
        	HAL_Delay(2000);
        	ssd1306_Fill(Black);
        	ssd1306_UpdateScreen();
    while (1){
    	if (currentGame){
    		RedLightGreenLight();
        }
        else{
            CatchAndRun();
        }

    }
}

/* ============================================================= */
/*                          MAIN GAME                            */
/* ============================================================= */



// Call this to reset the game state before replay

void Reset_Game(void)
{
    game_status = 1;
    game_seconds_count = 0;
    game_last_tick = 0;
    game_blink_tick = 0;
    game_led_state = 0;
    game_initialized = 0;
    game_calibrated = 0;
    game_over_msg_sent = 0;
}

// Call this repeatedly in main when currentGame == 1
void RedLightGreenLight(void)
{
    uint32_t now = HAL_GetTick();

    if (game_status == 0)
    {
        // Send game over instructions once
        if (!game_over_msg_sent) {
            UART_Send("Game Over\r\n");
            UART_Send("Press double button slowly to replay. Press double button rapidly to switch.\r\n");
            game_over_msg_sent = 1;
        }
        return; // Do nothing more until reset
    }

    if (!game_initialized)
    {
        UART_Send("Entering Red Light, Green Light as Player \r\n");
        game_last_tick = now;
        game_initialized = 2;  // Waiting 5 seconds before green light
        BSP_LED_Off(LED2);
        return;
    }

    if (game_initialized == 2)
    {
        if (now - game_last_tick >= 5000)
        {
            UART_Send(" Green Light! \r\n");
            game_seconds_count = 1;
            game_last_tick = now;
            game_initialized = 1;
            BSP_LED_On(LED2);

        }
        return;
    }

    if (game_initialized != 1)
    {
        return;
    }

    if (game_seconds_count <= 10)
    {
        // Green light phase
        if (now - game_last_tick >= 1000)
        {
            game_last_tick = now;
            char msg[100];
            sprintf(msg, "\r\n %d: Green Light. Can Move \r\n", game_seconds_count);
            UART_Send(msg);
            ssd1306_Fill(Black);
            ssd1306_SetCursor(0,20);
            ssd1306_TestGreenLight();

            if (game_seconds_count % 2 == 0)
            {
                float temp_data = BSP_TSENSOR_ReadTemp();
                float hum_data = BSP_HSENSOR_ReadHumidity();
                float pres_data = BSP_PSENSOR_ReadPressure();
                sprintf(msg, " Temperature: %f \r\n Humidity: %f \r\n Pressure: %f \r\n", temp_data, hum_data, pres_data);
                UART_Send(msg);
            }
            game_seconds_count++;
        }
        BSP_LED_On(LED2);
    }
    else if (game_seconds_count == 11 && !game_calibrated)
    {
        // Calibrate sensors just after green light
        int16_t accel_const_i16[3] = {0};
        int16_t gyro_const_i16[3] = {0};

        BSP_ACCELERO_AccGetXYZ(accel_const_i16);
        accel_const[0] = (float)accel_const_i16[0] * (9.8 / 1000.0f);
        accel_const[1] = (float)accel_const_i16[1] * (9.8 / 1000.0f);
        accel_const[2] = (float)accel_const_i16[2] * (9.8 / 1000.0f);

        BSP_GYRO_GetXYZ(gyro_const_i16);
        gyro_const[0] = (float)gyro_const_i16[0] * (35 / 1000.0f);
        gyro_const[1] = (float)gyro_const_i16[1] * (35 / 1000.0f);
        gyro_const[2] = (float)gyro_const_i16[2] * (35 / 1000.0f);

        sprintf(msg, "\r\n Accel X: %f \r\n Accel Y: %f \r\n Accel Z: %f \r\n", accel_const[0], accel_const[1], accel_const[2]);
        UART_Send(msg);
        sprintf(msg, "\r\n Gyro X: %f \r\n Gyro Y: %f \r\n Gyro Z: %f \r\n", gyro_const[0], gyro_const[1], gyro_const[2]);
        UART_Send(msg);

        game_calibrated = 1;
        game_last_tick = now;
        BSP_LED_Off(LED2);
    }
    else if (game_seconds_count >= 11 && game_seconds_count <= 20)
    {
    	ssd1306_Fill(Black);
    	ssd1306_SetCursor(0,20);
    	ssd1306_TestRedLight();
        // Red Light phase (blink LED and check motion)
        if (now - game_blink_tick >= 500)
        {
            game_blink_tick = now;
            if (game_led_state)
            {
                BSP_LED_Off(LED2);
                game_led_state = 0;
            }
            else
            {
                BSP_LED_On(LED2);
                game_led_state = 1;
            }
        }

        if (now - game_last_tick >= 1000)
        {
            game_last_tick = now;
            char msg[200];
            sprintf(msg, "\r\n %d: Red Light. Cannot Move \r\n", game_seconds_count - 10);
            UART_Send(msg);

            if (game_seconds_count % 2 == 0)
            {
                int16_t accel_data_i16[3] = {0};
                float accel_data[3] = {0};
                BSP_ACCELERO_AccGetXYZ(accel_data_i16);
                accel_data[0] = (float)accel_data_i16[0] * (9.8 / 1000.0f);
                accel_data[1] = (float)accel_data_i16[1] * (9.8 / 1000.0f);
                accel_data[2] = (float)accel_data_i16[2] * (9.8 / 1000.0f);

                int16_t gyro_data_i16[3] = {0};
                float gyro_data[3] = {0};
                BSP_GYRO_GetXYZ(gyro_data_i16);
                gyro_data[0] = (float)gyro_data_i16[0] * (35 / 1000.0f);
                gyro_data[1] = (float)gyro_data_i16[1] * (35 / 1000.0f);
                gyro_data[2] = (float)gyro_data_i16[2] * (35 / 1000.0f);

                sprintf(msg, "\r\n Accel X: %f \r\n Accel Y: %f \r\n Accel Z: %f \r\n", accel_data[0], accel_data[1], accel_data[2]);
                UART_Send(msg);
                sprintf(msg, "\r\n Gyro X: %f \r\n Gyro Y: %f \r\n Gyro Z: %f \r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
                UART_Send(msg);

                if (fabs(accel_data[0] - accel_const[0]) >= 0.5f ||
                    fabs(accel_data[1] - accel_const[1]) >= 0.5f ||
                    fabs(accel_data[2] - accel_const[2]) >= 0.5f ||
                    fabs(gyro_data[0] - gyro_const[0]) >= 10.0f ||
                    fabs(gyro_data[1] - gyro_const[1]) >= 10.0f ||
                    fabs(gyro_data[2] - gyro_const[2]) >= 10.0f)
                {
                    game_status = 0;  // Set game over
                    ssd1306_Fill(Black);
                    ssd1306_SetCursor(0,20);
                    ssd1306_WriteString("You Moved", Font_7x10, White);
                    ssd1306_SetCursor(0,40);
                    ssd1306_WriteString("Game Over", Font_7x10, White);
                    ssd1306_UpdateScreen();
                }
            }
            game_seconds_count++;
        }
    }
    else
    {
        game_status = 0; // Game over if time exceeds expected
    }
}



void CatchAndRun(void)
{
    uint8_t detected = 0;
    uint32_t detectStart = 0;
    uint8_t game_over = 0;

    //sprintf(msg, "Entering Catch And Run as %s\r\n", (role == ROLE_PLAYER) ? "Player" : "Enforcer");
    UART_Send("Entering Catch And Run as Player\r\n");
    while (!currentGame)
    {
    	if (double_press_slow && game_over){
    		double_press_slow = 0;
    		game_over = 0;
    		UART_Send("Replaying Catch And Run as Player\r\n");
    		continue;
    	}
    	else if (game_over){
    		continue;
    	}
        float magValue = Read_Magnetometer();
        // Blink LED faster when proximity increases
        if (magValue > 5 * PROXIMITY_THRESHOLD){

        	__HAL_TIM_SET_AUTORELOAD(&htim16, 999);

        }
        else if (magValue > 4 * PROXIMITY_THRESHOLD){
        	__HAL_TIM_SET_AUTORELOAD(&htim16, 1999);

        }
        else if (magValue > 3 * PROXIMITY_THRESHOLD){
        	__HAL_TIM_SET_AUTORELOAD(&htim16, 3999);

        }
        else{
        	__HAL_TIM_SET_AUTORELOAD(&htim16, 7999);

        }



        // --- Detect proximity event ---
        if (magValue > MAG_THRESHOLD && !detected)
        {
        	press_pending = 0; // to clear out preempted presses.
        	buzzer_active = 1;
            detected = 1;
            detectStart = HAL_GetTick();


            if (role == ROLE_PLAYER)
            	UART_Send("Enforcer nearby! Be careful.\r\n");
            else
                UART_Send("Player is nearby! Move faster.\r\n");
        }

        // --- While detected ---
        if (detected)
        {
            uint32_t elapsed = HAL_GetTick() - detectStart;



            // Check for player/enforcer action (button press)
            if (elapsed <= ESCAPE_WINDOW_MS)
            {
                if (press_pending)
                {
                	press_pending = 0;
                	detected = 0;
                	buzzer_active = 0;
                    if (role == ROLE_PLAYER){
                    	UART_Send("Player escaped, good job!\r\n");
                    }
                    else{
                    	buzzer_active = 0;
                        UART_Send("Player captured, good job!\r\n");
                    	game_over = 1;
                    	UART_Send("Game Over. Press double button slowly to replay. Press double button rapidly to switch.\r\n");
                    	__HAL_TIM_SET_AUTORELOAD(&htim16, 7999);
                    }

                }
            }
            else
            {
            	detected = 0;
            	game_over = 1;
            	press_pending = 0;
            	buzzer_active = 0;
            	if (role == ROLE_PLAYER){
            		UART_Send("Game Over. Press double button slowly to replay. Press double button rapidly to switch.\r\n");
            		__HAL_TIM_SET_AUTORELOAD(&htim16, 7999);
            	}
                else{
                	UART_Send("Player escaped! Keep trying.\r\n");
                	detected = 0;
                	buzzer_active = 0;
                }
            }
        }

    }
}

/* ============================================================= */
/*                      HELPER FUNCTIONS                         */
/* ============================================================= */

//
float Read_Magnetometer(void)
{
	int16_t mag_data[3] = {0};
	BSP_MAGNETO_GetXYZ(mag_data); // Returns data in milli Gauss (1 mG = 0.1 micro Tesla)
	// convert to micro tesla
	float mag_strength = sqrtf((float)mag_data[0]*mag_data[0] + (float)mag_data[1]*mag_data[1] + (float)mag_data[2]*mag_data[2]) * 0.1f;
	return mag_strength;
}



// UART printing helper
void UART_Send(char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFF);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim16){
    	BSP_LED_Toggle(LED2);
    	//if (buzzer_active) {
    		//HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    	//}
    }
    else if ((htim == &htim17) && !(currentGame)) {
		float temperature = BSP_TSENSOR_ReadTemp();
		float humidity = BSP_HSENSOR_ReadHumidity();
		float pressure = BSP_PSENSOR_ReadPressure();

		if (temperature > 35.0f) {
			sprintf(msg, "Temperature spike detected! T:%fC. Dangerous environment!\r\n", temperature);
			UART_Send(msg);
		}
		else if (temperature < 5.0f) {
			sprintf(msg, "Temperature drop detected! T:%fC. Dangerous environment!\r\n", temperature);
			UART_Send(msg);
		}
		if (humidity > 75.0f) {
			sprintf(msg, "High humidity! %f. Heat stroke risk!\r\n", humidity);
			UART_Send(msg);
		}
		else if (humidity < 30.0f) {
			sprintf(msg, "Low humidity! %f. Dehydration risk!\r\n", humidity);
			UART_Send(msg);
		}
		if (pressure < 950.0f) {
			   sprintf(msg, "Low pressure! %f hPa. Risk of dizziness!\r\n", pressure);
			   UART_Send(msg);
		}
		else if (pressure > 1050.0f) {
			sprintf(msg, "High pressure! %f hPa. Risk of headache!\r\n", pressure);
			UART_Send(msg);
		}

    }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
   if (GPIO_Pin == BUTTON_EXTI13_Pin)
    {
        uint32_t now = HAL_GetTick();

        // --- Debounce (ignore within 50 ms) ---
        static uint32_t debounce_time = 0;
        if (now - debounce_time < 50)
            return;
        debounce_time = now;

        // --- Double-press detection ---
        if (press_pending && (now - last_press_time <= 1000))
        {
            // second press within 1 s → double press so switch game mode
            currentGame ^= 1;
            press_pending = 0;

            // Reset game states on switching to Red Light Green Light mode
            if (currentGame == 1) {
                Reset_Game();
            }

        }
        else if (press_pending && (now - last_press_time > 1000) && (now - last_press_time <= 5000)){
            // slow double press → replay current game
            double_press_slow = 1;
            press_pending = 0;

            if (currentGame == 1) {
                Reset_Game();
            }
            // Reset CatchAndRun similarly if needed
        }
        else if (press_pending && (now - last_press_time > 5000)){
                   press_pending = 0;

               }
        else
        {
            last_press_time = now;
            press_pending = 1;
        }
    }
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	// Enable NVIC EXTI line 13
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


	GPIO_InitStruct.Pin = BUZZER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

}

// Actual Period = (ARR + 1) * (PSC + 1) / TimerClockFreq. TimerClockFreq is 120 MHz
//
static void MX_TIM16_Init(void){
	__HAL_RCC_TIM16_CLK_ENABLE();
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 29999;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 7999;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
    {
		Error_Handler();
    }
	HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

static void MX_TIM17_Init(void){
	__HAL_RCC_TIM17_CLK_ENABLE();
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 29999;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 3999;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
    {
		Error_Handler();
    }
	HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
}


static void UART1_Init(void){

	// Pin configuration for UART. BSP_COM_Init() can do this automatically

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// Configuring UART1
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		while(1);
	}
}

static void I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // I2C Pins (PB8=SCL, PB9=SDA)
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10909CEC;  // Example timing for 100kHz at 80MHz
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        while(1);
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}
void TIM1_UP_TIM16_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim16);
}
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim17);
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
