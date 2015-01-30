/**
  ******************************************************************************
  * File Name          : main.c
<<<<<<< HEAD
  * Date               : 30/01/2015 15:15:45
=======
  * Date               : 28/01/2015 20:31:30
>>>>>>> origin/master
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "CHARS.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

<<<<<<< HEAD
TIM_HandleTypeDef htim9;
=======
>>>>>>> origin/master
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
char data_An[8] = {0x7F,0xBF,0xDF,0xEF,0xF7,0xFB,0xFD,0xFE};
<<<<<<< HEAD
char data_Gr[8]; 
char data_Rd[8];
char data_Bl[8];
char char_Play[30][2];
uint8_t pData[24];
=======
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
int countLine = sizeof(data_An)-1;
int counterChanges = 0;
int counterLetters = 2;
short countColour = 0;
uint8_t pData[24];

>>>>>>> origin/master
char bufferData [8][4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
<<<<<<< HEAD
static void MX_TIM9_Init(void);
=======
>>>>>>> origin/master
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);

/* USER CODE BEGIN PFP */

/*Инициализация функции формирования посылки*/
<<<<<<< HEAD
void create_pData(int cntLn)
{
  pData[0] = bufferData[cntLn][0];
  pData[1] = bufferData[cntLn][1];
  pData[2] = bufferData[cntLn][2];
  pData[3] = bufferData[cntLn][3];
  pData[4] = bufferData[cntLn][0];
  pData[5] = bufferData[cntLn][1];
  pData[6] = bufferData[cntLn][2];
  pData[7] = bufferData[cntLn][3];
  pData[8] = bufferData[cntLn][0];
  pData[9] = bufferData[cntLn][1];
  pData[10] = bufferData[cntLn][2];
  pData[11] = bufferData[cntLn][3];
  pData[12] = bufferData[cntLn][0];
  pData[13] = bufferData[cntLn][1];
  pData[14] = bufferData[cntLn][2];
  pData[15] = bufferData[cntLn][3];
  pData[16] = bufferData[cntLn][0];
  pData[17] = bufferData[cntLn][1];
  pData[18] = bufferData[cntLn][2];
  pData[19] = bufferData[cntLn][3];
  pData[20] = bufferData[cntLn][0];
  pData[21] = bufferData[cntLn][1];
  pData[22] = bufferData[cntLn][2];
  pData[23] = bufferData[cntLn][3];
=======
void create_pData(int cnL)
{
  pData[0] = bufferData[cnL][0];
  pData[1] = bufferData[cnL][1];
  pData[2] = bufferData[cnL][2];
  pData[3] = bufferData[cnL][3];
  pData[4] = bufferData[cnL][0];
  pData[5] = bufferData[cnL][1];
  pData[6] = bufferData[cnL][2];
  pData[7] = bufferData[cnL][3];
  pData[8] = bufferData[cnL][0];
  pData[9] = bufferData[cnL][1];
  pData[10] = bufferData[cnL][2];
  pData[11] = bufferData[cnL][3];
  pData[12] = bufferData[cnL][0];
  pData[13] = bufferData[cnL][1];
  pData[14] = bufferData[cnL][2];
  pData[15] = bufferData[cnL][3];
  pData[16] = bufferData[cnL][0];
  pData[17] = bufferData[cnL][1];
  pData[18] = bufferData[cnL][2];
  pData[19] = bufferData[cnL][3];
  pData[20] = bufferData[cnL][0];
  pData[21] = bufferData[cnL][1];
  pData[22] = bufferData[cnL][2];
  pData[23] = bufferData[cnL][3];
>>>>>>> origin/master
}

/*Инициализация функции формирования масива изображения*/
void create_bufferData(void)
{
<<<<<<< HEAD
  for (int j=0; j <=7 ; j++)
  {
    bufferData[j][0] = data_An[j];
    bufferData[j][1] = data_Gr[j];
    bufferData[j][2] = data_Rd[j];
    bufferData[j][3] = data_Bl[j];
=======
  for (int i=0; i <=7 ; i++)
  {
    bufferData[i][0] = data_An[i];
    bufferData[i][1] = data_Gr[i];
    bufferData[i][2] = data_Rd[i];
    bufferData[i][3] = data_Bl[i];
>>>>>>> origin/master
  }
 }

/*Инициализация функции отправки данных*/
<<<<<<< HEAD
void send_Data(int cntLn)
{
  create_bufferData(); 
  create_pData(cntLn);
=======
void send_Data(void)
{
  create_bufferData();
  create_pData(countLine);
>>>>>>> origin/master
  HAL_SPI_Transmit(&hspi1, pData, 24, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_RESET);
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
<<<<<<< HEAD
void writte_Char(int colour, int cntCh)
=======
void writte_Char(int colour)
>>>>>>> origin/master
{
switch(colour)
    {
    case GREEN:
      {
<<<<<<< HEAD
        data_Gr[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
=======
        data_Gr[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
>>>>>>> origin/master
        break;
      }
    case RED:
      {
<<<<<<< HEAD
        data_Rd[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
=======
        data_Rd[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
>>>>>>> origin/master
        break;
      }
    case BLUE:
      {
<<<<<<< HEAD
        data_Bl[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
=======
        data_Bl[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
>>>>>>> origin/master
        break;
      }
    case YELLOW:
      {
<<<<<<< HEAD
        data_Gr[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
        data_Rd[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
=======
        data_Gr[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        data_Rd[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
>>>>>>> origin/master
        break;
      }
    case MAGENTA:
      {
<<<<<<< HEAD
        data_Rd[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
        data_Bl[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
=======
        data_Rd[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        data_Bl[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
>>>>>>> origin/master
        break;
      }
    case CYAN:
      {
<<<<<<< HEAD
        data_Gr[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
        data_Bl[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
=======
        data_Gr[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
        data_Bl[char_Play[counterChanges][0]] = char_Play[counterChanges][1];
>>>>>>> origin/master
        break;
      }
    case WHITE:
      {
<<<<<<< HEAD
        data_Gr[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
        data_Bl[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
        data_Rd[char_Play[cntCh+1][0]] = char_Play[cntCh+1][1];
=======
        data_Gr[char_Play[counterChanges+1][0]] = char_Play[counterChanges+1][1];
        data_Bl[char_Play[counterChanges+1][0]] = char_Play[counterChanges+1][1];
        data_Rd[char_Play[counterChanges+1][0]] = char_Play[counterChanges+1][1];
>>>>>>> origin/master
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
<<<<<<< HEAD
/*void changecolour(int cntC)
{  
  switch(cntC)
=======
void changecolour(void)
{  
  switch(countColour)
>>>>>>> origin/master
  {
  case 0:
    {
      setcolour = GREEN;
<<<<<<< HEAD
=======
      countColour++;
>>>>>>> origin/master
      break;
    }
  case 1:
    {
      setcolour = RED;
<<<<<<< HEAD
=======
      countColour++;
>>>>>>> origin/master
      break;
    }
  case 2:
    {
      setcolour = BLUE;
<<<<<<< HEAD
=======
      countColour++;
>>>>>>> origin/master
      break;
    }
  case 3:
    {
      setcolour = YELLOW;
<<<<<<< HEAD
=======
      countColour++;
>>>>>>> origin/master
      break;
    }
  case 4:
    {
      setcolour = MAGENTA;
<<<<<<< HEAD
=======
      countColour++;
>>>>>>> origin/master
      break;
    }
  case 5:
    {
      setcolour = CYAN;
<<<<<<< HEAD
=======
      countColour++;
>>>>>>> origin/master
      break;
    }
  case 6:
    {
      setcolour = WHITE;
<<<<<<< HEAD
=======
      countColour++;
>>>>>>> origin/master
      break;
    }
  default:
    __NOP();
  }
<<<<<<< HEAD
}*/
=======
}
>>>>>>> origin/master
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
<<<<<<< HEAD
  MX_TIM9_Init();
=======
>>>>>>> origin/master
  MX_TIM10_Init();
  MX_TIM11_Init();

  /* USER CODE BEGIN 2 */
<<<<<<< HEAD
  HAL_TIM_Base_Start_IT(&htim9);
=======
>>>>>>> origin/master
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
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

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

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
  HAL_SPI_Init(&hspi1);

}

<<<<<<< HEAD
/* TIM9 init function */
void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8400;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 20000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim9);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig);

}

=======
>>>>>>> origin/master
/* TIM10 init function */
void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 4;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim10);

}

/* TIM11 init function */
void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 60000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim11);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PB10   ------> I2S2_CK
     PB12   ------> I2S2_WS
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
<<<<<<< HEAD
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
=======
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
>>>>>>> origin/master
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
<<<<<<< HEAD

int counterChanges = 0;
int countLine = 7;
int counterLetters = 0;
int setcolour = 0;


=======
>>>>>>> origin/master
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
  if (htim == &htim10)
  {
    if (countLine < 0)
    {
<<<<<<< HEAD
      countLine=7;
    }
    send_Data(countLine);
=======
      countLine=(sizeof(data_An)-1);
    }
    send_Data();
>>>>>>> origin/master
    countLine--;
  }
  else if (htim == &htim11)
  {
<<<<<<< HEAD
    /*char *y = lEtters[counterLetters-1];*/
    if (setcolour >= 7)
  {
    setcolour = 0;
  }
    if(counterChanges > (((char_Play[0][1])-1)+24))
=======
    char *y = lEtters[counterLetters-1];
    
    if(counterChanges > (((y[1])-1)+24))
>>>>>>> origin/master
    {
        zero_matrix();
        counterChanges = 0;
    }
<<<<<<< HEAD
    if (counterChanges < ((char_Play[0][1])-1))
    {
      writte_Char(setcolour, counterChanges);
    }
      counterChanges++; 
  }
  else if (htim == &htim9)
  {
    if (counterLetters == 26) /*3 - количесвто елементов в масиве букв*/
    {
      counterLetters = 0;
    }
    char *p = lEtters[counterLetters];
    set_Char(p, (int)p[0], (int)p[1]);
    zero_matrix();
    setcolour++;
    counterChanges = 0;
    counterLetters++;
  
  }
=======
    if (counterChanges < ((y[1])-1))
    {
      writte_Char(setcolour);
    }
      counterChanges++; 
  }
>>>>>>> origin/master
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin0)
{ 
<<<<<<< HEAD
  
=======
  if (counterLetters == 4)
  {
  counterLetters = 1;
  }
  char *p = lEtters[counterLetters];
  set_Char(p, (int)p[0], (int)p[1]);
  zero_matrix();
  counterChanges = 0;
  counterLetters++;
>>>>>>> origin/master
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
