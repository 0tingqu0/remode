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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nrf24l01.h"
#include "key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ADC_CHANNELS     4     // 摇杆通道数
#define DEADZONE_THRESHOLD 200
#define FILTER_WINDOW 8

uint16_t values[4];
//char message[4][20];
// 在全局变量区域声明
#define MAX_MSG_LEN 20
char tx_buffer[MAX_MSG_LEN]; // 统一发送缓冲区

char remode1[50] = "";
char remode2[50] = "";
char Rx_data[10] = "";
char data_ready = 0;

uint8_t g_TxMode = 0, g_UartRxFlag = 0;
uint8_t g_UartRxBuffer[32] = { 0 };
uint8_t g_RF24L01RxBuffer[32] = { 0 };
// 定义3个不同的缓冲区用于存储接收数据
uint8_t g_RF24L01RxBuffer1[32] = { 0 };
uint8_t g_RF24L01RxBuffer2[32] = { 0 };
uint8_t g_RF24L01RxBuffer3[32] = { 0 };
uint8_t current_buffer = 1; // 当前使用的缓冲区编号(1-3)
uint8_t refresh_flag = 0;   // 数据刷新标志
uint8_t ADC_State = 0;
uint8_t conversion = 0; // nrf24l01转换标志
uint8_t TX_Errow = 0;

volatile uint16_t adc_raw[ADC_CHANNELS];          // DMA原始数据
volatile uint16_t filtered_values[ADC_CHANNELS];  // 滤波后数据
volatile uint8_t timerflag = 0;

extern volatile uint32_t clean;
extern uint8_t key1_state, key2_state , key3_state, key4_state ;
extern uint8_t key1, key2 , key3, key4;
extern uint8_t key[];
typedef struct
{
    uint16_t buffer[FILTER_WINDOW];
    uint8_t index;
} MovingFilter;

MovingFilter joystick_filters[ADC_CHANNELS]; // 每个通道独立滤波器

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        static uint8_t i = 0;
//    i++;
        // 遍历所有通道进行滤波
        for (uint8_t ch = 0; ch < ADC_CHANNELS; ch++)
        {
            // 更新滤波器
            joystick_filters[ch].buffer[joystick_filters[ch].index] = adc_raw[ch];
            joystick_filters[ch].index = (joystick_filters[ch].index + 1) % FILTER_WINDOW;

            // 计算滑动平均
            uint32_t sum = 0;
            for (uint8_t i = 0; i < FILTER_WINDOW; i++)
            {
                sum += joystick_filters[ch].buffer[i];
            }
            filtered_values[ch] = sum / FILTER_WINDOW;
            if (abs(filtered_values[ch] - 2048) <= DEADZONE_THRESHOLD)
            {
                filtered_values[ch] = 2048;
            }
        }
        if (filtered_values[0] == 2048)
            i = 1;
        if (i == 1)
        {
            filtered_values[4]=key[0];
            snprintf(tx_buffer , MAX_MSG_LEN , "%d,%d,%d,%d,%d," , filtered_values[0] , filtered_values[1] ,
                    filtered_values[3] , filtered_values[4], filtered_values[2]);

            sprintf(remode1 , "%d,%d " , filtered_values[0] , filtered_values[1]);
            sprintf(remode2 , "%d,%d %d" , filtered_values[2] , filtered_values[3],filtered_values[4] );

        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        timerflag++; // 设置2000ms标志
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    uint8_t i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    HAL_SYSTICK_IRQHandler();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    // 启动定时器中断
    HAL_TIM_Base_Start_IT(&htim1);

    // 初始化滤波器结构体
    memset(joystick_filters , 0 , sizeof(joystick_filters));
    HAL_ADCEx_Calibration_Start(&hadc1);
    hadc1.Instance->CR2 |= ADC_CR2_CAL;  // 启动内部校准
    while (hadc1.Instance->CR2 & ADC_CR2_CAL); // 等待校准完成

    OLED_Init();                           //OLED初始
    OLED_ShowString(42 , 3 , "hellow" , 12 , 0);    //反相显示8X16字符串

    //RF24L01引脚初始化
    NRF24L01_Gpio_Init();

    //检测nRF24L01
    while (NRF24L01_check_DMA() == 0);
    RF24L01_Init_DMA();
    RF24L01_Set_Mode_DMA(MODE_TX);        //发送模式

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        HAL_ADC_Start_DMA(&hadc1 , (uint32_t*) adc_raw , ADC_CHANNELS);
        key_scan();
        if (timerflag >= 2)
        {
            if ((NRF24L01_TxPacket_DMA((uint8_t*) tx_buffer , strlen(tx_buffer)) == TX_OK))
            {
            }
            else
            {
                if (clean >= 18)// oled页面刷新18次后显示
                {
                    TX_Errow++;
                    
                }
                else
                    TX_Errow = 0;
                if (TX_Errow >= 1)
                {
//                    HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13);
                    memset(g_RF24L01RxBuffer , 0 , sizeof(g_RF24L01RxBuffer)); // 清空接收缓冲区
                    TX_Errow = 0;
                    conversion = 1;
                }
            }
            timerflag = 0; // 清除标志
        }
        if (clean >= 18 && conversion == 1)
        {
            uint32_t timeout = 50000; // 设置超时计数
            uint8_t rx_status = 0;
            
            /* 切换LED状态，指示接收状态 */
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            
            /* 切换到接收模式 */
            RF24L01_Set_Mode_DMA(MODE_RX);
            
            /* 等待接收数据，添加超时机制 */
            while ((rx_status = NRF24L01_RxPacket_DMA(g_RF24L01RxBuffer)) == 0)
            {
                timeout--;
                if (timeout == 0)
                {
                    /* 超时处理 */
                    OLED_ShowString(0, 6, "RX Timeout", 12, 0);
                    break;
                }
            }
            
            /* 如果成功接收数据 */
            if (rx_status != 0 && timeout > 0)
            {
                /* 根据数据类型显示在不同位置 */
                if (g_RF24L01RxBuffer[0] == '1')
                {
                    OLED_ShowString(0, 3, g_RF24L01RxBuffer, 12, 0);
                }
                else if (g_RF24L01RxBuffer[0] == '2')
                {
                    OLED_ShowString(0, 4, g_RF24L01RxBuffer, 12, 0);
                }
                else if (g_RF24L01RxBuffer[0] == '3')
                {
                    OLED_ShowString(0, 5, g_RF24L01RxBuffer, 12, 0);
                }
//                else
//                {
//                    /* 默认显示在第3行 */
//                    OLED_ShowString(0, 3, g_RF24L01RxBuffer, 12, 0);
//                }
            }
            
            /* 切回发送模式 */
            RF24L01_Set_Mode_DMA(MODE_TX);
            conversion = 0; // 清除转换标志
        }
        OLED_ShowString(0 , 0 , remode1 , 12 , 0);
        OLED_ShowString(64 , 0 , remode2 , 12 , 0);



        if (clean >= 15 && i == 0) //oled刷新15次后显示
        {
            OLED_Clear();
            i = 1;

            HAL_GPIO_WritePin(GPIOC , GPIO_PIN_13 , 0);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
