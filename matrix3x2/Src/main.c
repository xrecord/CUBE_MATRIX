/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 26/01/2015 11:17:38
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "CHARS.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
char data_An[8] = {0x7F,0xBF,0xDF,0xEF,0xF7,0xFB,0xFD,0xFE};
char data_Gr[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
char data_Rd[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
char data_Bl[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

char char_Play[][2] = {
  2,21,
  0,239,
  1,223,
  2,223,
  3,191,
  4,191,
  5,191,
  6,191,
  7,191,
  0,231,
  1,219,
  2,219,
  3,189,
  4,189,
  5,189,
  6,189,
  7,189,
  4,157,
  4,141,
  4,133,
  4,129
}; 
int setcolour = WHITE;
short countLine = sizeof(data_An)-1;
int counterChanges = 0;
int counterLetters = 1;
short countColour = 0;
uint8_t pData[24];

char bufferData [8][4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */



/*Инициализация функции формирования посылки*/
void create_pData(void)
{
  pData[0] = bufferData[countLine][0];
  pData[1] = bufferData[countLine][1];
  pData[2] = bufferData[countLine][2];
  pData[3] = bufferData[countLine][3];
  pData[4] = bufferData[countLine][0];
  pData[5] = bufferData[countLine][1];
  pData[6] = bufferData[countLine][2];
  pData[7] = bufferData[countLine][3];
  pData[8] = bufferData[countLine][0];
  pData[9] = bufferData[countLine][1];
  pData[10] = bufferData[countLine][2];
  pData[11] = bufferData[countLine][3];
  pData[12] = bufferData[countLine][0];
  pData[13] = bufferData[countLine][1];
  pData[14] = bufferData[countLine][2];
  pData[15] = bufferData[countLine][3];
  pData[16] = bufferData[countLine][0];
  pData[17] = bufferData[countLine][1];
  pData[18] = bufferData[countLine][2];
  pData[19] = bufferData[countLine][3];
  pData[20] = bufferData[countLine][0];
  pData[21] = bufferData[countLine][1];
  pData[22] = bufferData[countLine][2];
  pData[23] = bufferData[countLine][3];
}

/*Инициализация функции формирования масива изображения*/
void create_bufferData(void)
{
  for (int i=0; i <=7 ; i++)
  {
    bufferData[i][0] = data_An[i];
    bufferData[i][1] = data_Gr[i];
    bufferData[i][2] = data_Rd[i];
    bufferData[i][3] = data_Bl[i];
  }
 }

/*Инициализация функции отправки данных*/
void send_Data(void)
{
  create_bufferData();
  create_pData();
  HAL_SPI_Transmit(&hspi1, pData, 24, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
}

/*Инициализация функции очистки изображения*/
void zero_matrix(void)
{
  for (int i=0; i<=7; i++)
  {
  data_Gr[i] = 0xFF;
  data_Rd[i] = 0xFF;
  data_Bl[i] = 0xFF;
  }
}

/*Инициализация функци отрисовки буквы нужного цвета*/
void writte_Char(int colour)
{
switch(colour)
    {
    case GREEN:
      {
        data_Gr[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        break;
      }
    case RED:
      {
        data_Rd[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        break;
      }
    case BLUE:
      {
        data_Bl[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        break;
      }
    case YELLOW:
      {
        data_Gr[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        data_Rd[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        break;
      }
    case MAGENTA:
      {
        data_Rd[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        data_Bl[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        break;
      }
    case CYAN:
      {
        data_Gr[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        data_Bl[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        break;
      }
    case WHITE:
      {
        data_Gr[char_Play[counterChanges+1][0]] = char_Play[counterChanges+1][1];
        data_Bl[char_Play[counterChanges+1][0]] = char_Play[counterChanges+1][1];
        data_Rd[char_Play[counterChanges+1][0]] = char_Play[counterChanges+1][1];
        break;
      }
    default:
      zero_matrix();
    }
  }

/*Инициализация функции присвоения масиву вывода выбранной буквы*/
void set_Char (char *letter, int i, int j)
{
  for (int k = 0; k < j; k++)
  {
    for (int z = 0; z < i; z++)
    {
      char_Play[k][z] = letter[(i*k)+z];
    }
  }
}

/*????Инициализация функции изменения цвета*/
void changecolour(void)
{  
  switch(countColour)
  {
  case 0:
    {
      setcolour = GREEN;
      countColour++;
      break;
    }
  case 1:
    {
      setcolour = RED;
      countColour++;
      break;
    }
  case 2:
    {
      setcolour = BLUE;
      countColour++;
      break;
    }
  case 3:
    {
      setcolour = YELLOW;
      countColour++;
      break;
    }
  case 4:
    {
      setcolour = MAGENTA;
      countColour++;
      break;
    }
  case 5:
    {
      setcolour = CYAN;
      countColour++;
      break;
    }
  case 6:
    {
      setcolour = WHITE;
      countColour++;
      break;
    }
  default:
    __NOP();
  }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
 
  while (1)
  {

    
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  __SYSCFG_CLK_ENABLE();

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;
  HAL_SPI_Init(&hspi1);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 24000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 200;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA11   ------> USB_DM
     PA12   ------> USB_DP
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE4 PE5 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE8 PE9 PE10 
                           PE11 PE12 PE13 PE14 
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
  if (htim == &htim6)
  {
    if (countLine < 0)
    {
      countLine=(sizeof(data_An)-1);
    }
    send_Data();
    countLine--;
  }
  else if (htim == &htim7)
  {
    char *p = lEtters[counterLetters-1];
    if(counterChanges > (((p[1])-1)+24))
    {
        zero_matrix();
        counterChanges = 0;
    }
    if (counterChanges < ((p[1])-1))
    {
      writte_Char(setcolour);
    }
      counterChanges++; 
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin0)
{ 
  char *p = lEtters[counterLetters];
  set_Char(p, p[0], p[1]);
  zero_matrix();
  counterChanges = 0;
  counterLetters++;
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
