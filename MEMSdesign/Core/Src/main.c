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
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define PRINT_INSTAND_SWJ
#include "STM32F1_porting.h"
#include "mpu6050_SL.h"

#include "stdio.h"
#include "string.h"

/* mpu6050 DMP库  begin */
#include "eMPL_outputs.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "log.h"
#include "mltypes.h"
#include "mpu.h"
#include "packet.h"

extern void get_ms_user(unsigned long *count); //定义在inv_mpu.c
#define get_tick_count get_ms_user
/* mpu6050 DMP库  end */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Data read from MPL. */
#define PRINT_ACCEL (0x01)
#define PRINT_GYRO (0x02)
#define PRINT_QUAT (0x04)
#define PRINT_COMPASS (0x08)
#define PRINT_EULER (0x10)
#define PRINT_ROT_MAT (0x20)
#define PRINT_HEADING (0x40)
#define PRINT_PEDO (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)


#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern struct inv_sensor_cal_t sensors;
volatile uint32_t hal_timestamp = 0;

volatile int pitch = 0;
volatile int roll = 0;
volatile int yaw = 0;

uint16_t adc_buffer[4];

char data_receive[32];
uint8_t rx_buf;

uint8_t data_to_send[49];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void uart_receive_start()
{
	HAL_UART_Receive_IT(&huart1,&rx_buf,1);
	//hdma_rx->Instance->CCR &= ~DMA_CCR_HTIE;
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart != &huart1)
		return;
	static uint8_t uart1_rx_count = 0;
	if (uart1_rx_count >= 255)
	{
		uart1_rx_count = 0;
		memset(data_receive,0x00,sizeof(data_receive));
	}
	else
	{
		data_receive[uart1_rx_count++] = rx_buf;
		if((data_receive[uart1_rx_count - 1] == 0x0A) && (data_receive[uart1_rx_count - 2] == 0x0D))
		{
			if ((data_receive[0]=='s') && (data_receive[1]=='t') && (data_receive[2]=='a') && (data_receive[3]=='r') && (data_receive[4]=='t'))
			{
				HAL_TIM_Base_Start_IT(&htim1);
			}
			else if ((data_receive[0]=='s') && (data_receive[1]=='t') && (data_receive[2]=='o') && (data_receive[3]=='p'))
			{
				HAL_TIM_Base_Stop_IT(&htim1);
			}
			uart1_rx_count = 0;
			memset(data_receive,0x00,sizeof(data_receive));
		}

	}

	uart_receive_start();
}



/**
 * @brief  read euler angle from MPL
 */
static void read_from_mpl(void)
{
  long data[9];
  int8_t accuracy;
  unsigned long timestamp;


  if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp))
  {
    pitch = data[0];
    roll = data[1];
    yaw = data[2];
  }
}


void ADC_Samp()
{
  //HAL_ADC_Start(&hadc2);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) adc_buffer, 4);

}

void send_data()
{
	data_to_send[0] = 0xAB;
	// pitch
	for (int i = 0;i < 4; i++)
	{
		data_to_send[i+1] = (pitch >> (8*i)) & 0xFF;
	}

	// roll
	for (int i = 0;i < 4; i++)
	{
		data_to_send[i+5] = (roll >> (8*i)) & 0xFF;
	}

	// yaw
	for (int i = 0;i < 4; i++)
	{
		data_to_send[i+9] = (yaw >> (8*i)) & 0xFF;
	}

	uint32_t crc = HAL_CRC_Calculate(&hcrc,(uint32_t*)(data_to_send+1), 11);


	data_to_send[45] = crc & 0xFF;
	data_to_send[46] = (crc >> 8) & 0xFF;
	data_to_send[47] = (crc >> 16) & 0xFF;
	data_to_send[48] = (crc >> 24) & 0xFF;


	HAL_UART_Transmit_DMA(&huart1,data_to_send,49);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static uint8_t samp_counter = 0;
  if (hadc == &hadc1)
  {
  	switch (samp_counter)
  	{
  	  case 0:
  	    {
  	      HAL_GPIO_WritePin(Sel0_GPIO_Port,Sel0_Pin,GPIO_PIN_SET);
  	      HAL_GPIO_WritePin(Sel1_GPIO_Port,Sel1_Pin,GPIO_PIN_RESET);
  	      break;
  	    }
  	  case 1:
  	    {
  	      HAL_GPIO_WritePin(Sel0_GPIO_Port,Sel0_Pin,GPIO_PIN_RESET);
  	      HAL_GPIO_WritePin(Sel1_GPIO_Port,Sel1_Pin,GPIO_PIN_SET);
  	      break;
  	    }
  	  case 2:
  	    {
  	      HAL_GPIO_WritePin(Sel0_GPIO_Port,Sel0_Pin,GPIO_PIN_SET);
  	      HAL_GPIO_WritePin(Sel1_GPIO_Port,Sel1_Pin,GPIO_PIN_SET);
  	      break;
  	    }
  	  case 3:
  	    {
  	      HAL_GPIO_WritePin(Sel0_GPIO_Port,Sel0_Pin,GPIO_PIN_RESET);
  	      HAL_GPIO_WritePin(Sel1_GPIO_Port,Sel1_Pin,GPIO_PIN_RESET);
  	      break;
  	    }
  	 case 4:
  	     {
           samp_counter = 0;

					//HAL_ADC_Stop(&hadc2);
  	  		HAL_ADC_Stop_DMA(&hadc1);
  	  		HAL_TIM_Base_Stop(&htim3);
  	  		send_data();

  	       return;
  	       break;
  	     }
  	  default:
  	    {
  	      HAL_GPIO_WritePin(Sel0_GPIO_Port,Sel0_Pin,GPIO_PIN_RESET);
  	      HAL_GPIO_WritePin(Sel1_GPIO_Port,Sel1_Pin,GPIO_PIN_RESET);
  	      break;
  	    }
  	}

  	data_to_send[13 + samp_counter * 8] = (adc_buffer[0]) & 0xFF;
  	data_to_send[14 + samp_counter * 8] = (adc_buffer[0] >> 8) & 0xFF;
  	data_to_send[15 + samp_counter * 8] = (adc_buffer[1]) & 0xFF;
  	data_to_send[16 + samp_counter * 8] = (adc_buffer[1] >> 8) & 0xFF;

  	data_to_send[17 + samp_counter * 8] = (adc_buffer[2]) & 0xFF;
  	data_to_send[18 + samp_counter * 8] = (adc_buffer[2] >> 8) & 0xFF;
  	data_to_send[19 + samp_counter * 8] = (adc_buffer[3]) & 0xFF;
  	data_to_send[20 + samp_counter * 8] = (adc_buffer[3] >> 8) & 0xFF;

//  	uint16_t data_0 = adc_buffer[0]& 0xFFFF;
//		uint16_t data_1 = (adc_buffer[0] >> 16) & 0xFFFF;
//  	uint16_t data_2 = adc_buffer[1]& 0xFFFF;
//  	uint16_t data_3 = (adc_buffer[1] >> 16) & 0xFFFF;
//
//  	printf("%x %X %X %X\r\n",data_0,data_1,data_2,data_3);

    samp_counter++;
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1) {
  	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
  	ADC_Samp();
  	HAL_TIM_Base_Start(&htim3);
    //		int p = pitch >> 16;
    //		int r = roll >> 16;
    //		int y = yaw >> 16;
    //	  printf ("pitch: %d\t roll: %d\t yaw: %d\t\r\n", p,r,y);
  }
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);
  //HAL_ADCEx_Calibration_Start(&hadc2);
  MPU6050_mpu_init();
  MPU6050_mpl_init();
  MPU6050_config();
  HAL_GPIO_WritePin(Sel0_GPIO_Port,Sel0_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Sel1_GPIO_Port,Sel1_Pin,GPIO_PIN_RESET);
	uart_receive_start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { unsigned long sensor_timestamp;
    int new_data = 0;
    if (!hal.sensors || !hal.new_gyro)
    {
      continue;
    }
    if (hal.new_gyro && hal.dmp_on)
        {
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long  quat[4];
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO)
            {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;

            }
        }

        if (new_data)
        {
            inv_execute_on_data();
            read_from_mpl();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
