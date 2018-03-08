/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
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
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN Private defines */

/* MOTORS INSTANCES */
#define MOTOR_FR 1
#define MOTOR_FL 2
#define MOTOR_RR 3
#define MOTOR_RL 4

/* MOTORS GPIO CONFIG */

#define MOTOR_FR_FORWARD_PORT GPIOE
#define MOTOR_FR_FORWARD_PIN GPIO_PIN_10

#define MOTOR_FR_BACKWARD_PORT GPIOE
#define MOTOR_FR_BACKWARD_PIN GPIO_PIN_11

#define MOTOR_FL_FORWARD_PORT GPIOE
#define MOTOR_FL_FORWARD_PIN GPIO_PIN_12

#define MOTOR_FL_BACKWARD_PORT GPIOE
#define MOTOR_FL_BACKWARD_PIN GPIO_PIN_13

#define MOTOR_RR_FORWARD_PORT GPIOE
#define MOTOR_RR_FORWARD_PIN GPIO_PIN_14

#define MOTOR_RR_BACKWARD_PORT GPIOE
#define MOTOR_RR_BACKWARD_PIN GPIO_PIN_15

#define MOTOR_RL_FORWARD_PORT GPIOB
#define MOTOR_RL_FORWARD_PIN GPIO_PIN_10

#define MOTOR_RL_BACKWARD_PORT GPIOB
#define MOTOR_RL_BACKWARD_PIN GPIO_PIN_11

/* DIRECTIONS*/
#define DIR_FORWARD 1
#define DIR_BACKWARD 2
#define DIR_NONE 0

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_TIM3_Init(void);
                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN Prototypes */
void HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(uint8_t duty_cycle, uint8_t channel);
void drive_forward(uint8_t speed);
void drive_backward(uint8_t speed);
void drive_stop();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
