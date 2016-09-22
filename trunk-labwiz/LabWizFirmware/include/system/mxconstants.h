/*
 * NOTE: This file is required by a third party library and must be in
 * a path searchable by the compiler and linker
 */

/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LCD_RST_Pin GPIO_PIN_13
#define LCD_RST_GPIO_Port GPIOC
#define LCD_A0_Pin GPIO_PIN_0
#define LCD_A0_GPIO_Port GPIOC
#define PC1_Pin GPIO_PIN_1
#define PC1_GPIO_Port GPIOC
#define MOD3_GPIO1_Pin GPIO_PIN_2
#define MOD3_GPIO1_GPIO_Port GPIOC
#define MOD2_GPIO1_Pin GPIO_PIN_3
#define MOD2_GPIO1_GPIO_Port GPIOC
#define SW_PWR_Pin GPIO_PIN_0
#define SW_PWR_GPIO_Port GPIOA
#define MOD2_GPIO0_Pin GPIO_PIN_1
#define MOD2_GPIO0_GPIO_Port GPIOA
#define MOD1_GPIO1_Pin GPIO_PIN_4
#define MOD1_GPIO1_GPIO_Port GPIOC
#define MOD4_GPIO0_Pin GPIO_PIN_5
#define MOD4_GPIO0_GPIO_Port GPIOC
#define MOD4_GPIO1_Pin GPIO_PIN_1
#define MOD4_GPIO1_GPIO_Port GPIOB
#define SW_E_BOOT1_Pin GPIO_PIN_2
#define SW_E_BOOT1_GPIO_Port GPIOB
#define WIFI_EN_Pin GPIO_PIN_6
#define WIFI_EN_GPIO_Port GPIOC
#define SW_C_Pin GPIO_PIN_7
#define SW_C_GPIO_Port GPIOC
#define BATT_STAT_Pin GPIO_PIN_9
#define BATT_STAT_GPIO_Port GPIOC
#define SW_D_BOOT0_Pin GPIO_PIN_8
#define SW_D_BOOT0_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWDCLK_Pin GPIO_PIN_14
#define SWDCLK_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOA
#define SW_B_Pin GPIO_PIN_10
#define SW_B_GPIO_Port GPIOC
#define SW_A_Pin GPIO_PIN_11
#define SW_A_GPIO_Port GPIOC
#define SDIO_CLK_Pin GPIO_PIN_12
#define SDIO_CLK_GPIO_Port GPIOC
#define LCD_SCK_SWO_Pin GPIO_PIN_3
#define LCD_SCK_SWO_GPIO_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_4
#define LCD_BL_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_5
#define LCD_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
