/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SI_SDN_Pin GPIO_PIN_0
#define SI_SDN_GPIO_Port GPIOF
#define SI_ENT_Pin GPIO_PIN_1
#define SI_ENT_GPIO_Port GPIOF
#define CH0_Pin GPIO_PIN_0
#define CH0_GPIO_Port GPIOA
#define CH1_Pin GPIO_PIN_1
#define CH1_GPIO_Port GPIOA
#define CH2_Pin GPIO_PIN_2
#define CH2_GPIO_Port GPIOA
#define CH3_Pin GPIO_PIN_3
#define CH3_GPIO_Port GPIOA
#define CH4_Pin GPIO_PIN_4
#define CH4_GPIO_Port GPIOA
#define CH5_Pin GPIO_PIN_5
#define CH5_GPIO_Port GPIOA
#define CH6_Pin GPIO_PIN_6
#define CH6_GPIO_Port GPIOA
#define CH7_Pin GPIO_PIN_7
#define CH7_GPIO_Port GPIOA
#define PWM_OUT_Pin GPIO_PIN_0
#define PWM_OUT_GPIO_Port GPIOB
#define ADC_BAT_Pin GPIO_PIN_1
#define ADC_BAT_GPIO_Port GPIOB
#define SI_IRQ_Pin GPIO_PIN_8
#define SI_IRQ_GPIO_Port GPIOA
#define SI_IRQ_EXTI_IRQn EXTI4_15_IRQn
#define SI_ENP_Pin GPIO_PIN_11
#define SI_ENP_GPIO_Port GPIOA
#define SYS_LED_Pin GPIO_PIN_12
#define SYS_LED_GPIO_Port GPIOA
#define SI_CSN_Pin GPIO_PIN_15
#define SI_CSN_GPIO_Port GPIOA
#define SI_SCK_Pin GPIO_PIN_3
#define SI_SCK_GPIO_Port GPIOB
#define SI_MISO_Pin GPIO_PIN_4
#define SI_MISO_GPIO_Port GPIOB
#define SI_MOSI_Pin GPIO_PIN_5
#define SI_MOSI_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
