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
    BSP_LED_Init(LED2);

    char message_print[64]; // Increased buffer size
    snprintf(message_print, sizeof(message_print), "Entering Red Light, Green Light as Player \r\n");
    HAL_Delay(5000);
    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
    HAL_Delay(5000);

    snprintf(message_print, sizeof(message_print), " Green Light! \r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
    while (game==1){
    	int seconds_count = 1;
		while (seconds_count<=10)
		{
			snprintf(message_print, sizeof(message_print), " \r\n %d: Green Light. Can Move \r\n", seconds_count);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
			BSP_LED_On(LED2);
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
		BSP_LED_Off(LED2);

		float accel_const[3];
		int16_t accel_const_i16[3] = { 0 };
		BSP_ACCELERO_AccGetXYZ(accel_const_i16);
		accel_const[0] = (float)accel_const_i16[0] * (9.8/1000.0f);
		accel_const[1] = (float)accel_const_i16[1] * (9.8/1000.0f);
		accel_const[2] = (float)accel_const_i16[2] * (9.8/1000.0f);

		float gyro_const[3];
		int16_t gyro_const_i16[3] = { 0 };
		BSP_ACCELERO_AccGetXYZ(gyro_const_i16);
		gyro_const[0] = (float)gyro_const_i16[0] * (35/1000.0f);
		gyro_const[1] = (float)gyro_const_i16[1] * (35/1000.0f);
		gyro_const[2] = (float)gyro_const_i16[2] * (35/1000.0f);

		snprintf(message_print, sizeof(message_print), "\r\n Gyro X : %f \r\n Gyro Y : %f \r\n Gyro Z : %f \r\n", gyro_const[0], gyro_const[1], gyro_const[2]);
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

		snprintf(message_print, sizeof(message_print), " Red Light \r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

		while (seconds_count>10 && seconds_count<=20)
			{
			float accel_data[3];
			int16_t accel_data_i16[3] = { 0 };
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);
			seconds_count++;
			accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
			accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
			accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);

			float gyro_data[3];
			int16_t gyro_data_i16[3] = { 0 };
			BSP_GYRO_GetXYZ(gyro_data_i16);
			gyro_data[0] = (float)gyro_data_i16[0] * (35/1000.0f);
			gyro_data[1] = (float)gyro_data_i16[1] * (35/1000.0f);
			gyro_data[2] = (float)gyro_data_i16[2] * (35/1000.0f);

			snprintf(message_print, sizeof(message_print), "\r\n %d: Red Light. Cannot Move \r\n", seconds_count-11);
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
			BSP_LED_Off(LED2);
			HAL_Delay(500);
			BSP_LED_On(LED2);
			HAL_Delay(500);
			if (seconds_count%2==0){
				snprintf(message_print, sizeof(message_print), "\r\n Accel X : %f \r\n Accel Y : %f \r\n Accel Z : %f \r\n", accel_data[0], accel_data[1], accel_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
				snprintf(message_print, sizeof(message_print), "\r\n Gyro X : %f \r\n Gyro Y : %f \r\n Gyro Z : %f \r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
			}
			if (accel_data[0]-accel_const[0]>=0.5 || accel_data[1]-accel_const[1]>=0.5 || accel_data[2]-accel_const[2]>=0.5){
				snprintf(message_print, sizeof(message_print), "\r\n Game Over\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
				game=0;
				break;
				}
			else if (gyro_data[0]-gyro_const[0]>=10 || gyro_data[1]-gyro_const[1]>=10 || gyro_data[2]-gyro_const[2]>=10){
				snprintf(message_print, sizeof(message_print), "\r\n Game Over\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);
				game=0;
				break;
				}
			}
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
