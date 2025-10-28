#include "main.h"
#include "stdio.h"
#include "string.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"



static void UART1_Init(void);
UART_HandleTypeDef huart1;

int main(void)
{
    int seconds_count = 1;
    int move=1;
    float temp_data;
    float hum_data;
    float pres_data;
    int game=1;
    HAL_Init();
    UART1_Init();
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_HSENSOR_Init();
    BSP_TSENSOR_Init();
    BSP_PSENSOR_Init();

    char message_print[64]; // Increased buffer size
    snprintf(message_print, sizeof(message_print), "Entering Red Light, Green Light as Player/Enforcer\r\n");

    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
    HAL_Delay(5000);

    snprintf(message_print, sizeof(message_print), "Entering Green Light as Enforcer\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
    while (game){
		while (seconds_count<=10)
		{

			float accel_data[3];
			int16_t accel_data_i16[3] = { 0 };
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);

			snprintf(message_print, sizeof(message_print), " \r\n %d: Green Light. Can Move \r\n", seconds_count);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
			if (seconds_count%2==0){
				temp_data = BSP_TSENSOR_ReadTemp();
				hum_data = BSP_HSENSOR_ReadHumidity();
				pres_data = BSP_PSENSOR_ReadPressure();
				snprintf(message_print, sizeof(message_print), " Temperature: %f \r\n Humidity: %f \r\n Pressure: %f \r\n", temp_data, hum_data,pres_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

			}
			HAL_Delay(1000);

			seconds_count++;
		}

		float accel_const[3];
		int16_t accel_const_i16[3] = { 0 };// array to store the x, y and z readings.
		BSP_ACCELERO_AccGetXYZ(accel_const_i16);
		accel_const[0] = (float)accel_const_i16[0] * (9.8/1000.0f);
		accel_const[1] = (float)accel_const_i16[1] * (9.8/1000.0f);
		accel_const[2] = (float)accel_const_i16[2] * (9.8/1000.0f);

		snprintf(message_print, sizeof(message_print), "Entering Red Light as Player/Enforcer\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

		while (seconds_count>10 && seconds_count<=20)
			{
			float accel_data[3];
			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);
			seconds_count++;
			accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
			accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
			accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
			// Format the message with the temperature value
			snprintf(message_print, sizeof(message_print), "\r\n %d: Red Light. Cannot Move \r\n", seconds_count-11);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
			HAL_Delay(1000);
			if (seconds_count%2==0){
				snprintf(message_print, sizeof(message_print), "\r\n Accel X : %f \r\n Accel Y : %f \r\n Accel Z : %f \r\n", accel_data[0], accel_data[1], accel_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
			}
			if (accel_data[0]-accel_const[0]>=0.5|accel_data[1]-accel_const[1]>=0.5|accel_data[2]-accel_const[2]>=0.5){
				snprintf(message_print, sizeof(message_print), "\r\n Game Over\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
				game=0;
				}

			}
		seconds_count=0;
    }
}

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while(1);
    }
}
