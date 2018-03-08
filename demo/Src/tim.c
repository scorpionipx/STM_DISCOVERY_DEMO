/**
  ******************************************************************************
  * File Name          : TIM.c
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

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim3;

/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    /**TIM3 GPIO Configuration    
    PB1     ------> TIM3_CH4
    PC8     ------> TIM3_CH3
    PB4     ------> TIM3_CH1
    PB5     ------> TIM3_CH2 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */

void HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(uint8_t duty_cycle, uint8_t channel)
{
	unsigned int pulse_width;  // transform duty_cycle into pulse

	// make sure no invalid duty cycle is passed
	if (duty_cycle < 0)
	{
		duty_cycle = 0;
	}
	if(duty_cycle > 100)
	{
		duty_cycle = 100;
	}

	// max pulse is 500, as defined in MX_TIM2_Init: htim2.Init.Period = 500
	pulse_width = 5 * duty_cycle;

	if((channel < 1) || (channel > 4))
	{
		channel = 0;
	}

	switch(channel)
	{
		case 1:
		{
			channel = TIM_CHANNEL_1;
			break;
		}
		case 2:
		{
			channel = TIM_CHANNEL_2;
			break;
		}
		case 3:
		{
			channel = TIM_CHANNEL_3;
			break;
		}
		case 4:
		{
			channel = TIM_CHANNEL_4;
			break;
		}
		default:
		{
			return;
			break;
		}
	}

	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse_width;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, channel) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_TIM_PWM_Start(&htim3, channel);
}


void drive_forward(uint8_t speed)
{
	/*
	 * :param speed: Desired motors' speed as duty cycle.
	 * :type speed: 8 bits unsigned integer
	 * :range speed: [0 - 100]
	 */

	/* stop pwm and wait 10 ms to make sure there will be no short */
	if(DRIVE_DIRECTION == DIR_BACKWARD)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(10);
	}

	/* reset drive backward pins */
	HAL_GPIO_WritePin(MOTOR_FR_BACKWARD_PORT, MOTOR_FR_BACKWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_FL_BACKWARD_PORT, MOTOR_FL_BACKWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_RR_BACKWARD_PORT, MOTOR_RR_BACKWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_RL_BACKWARD_PORT, MOTOR_RL_BACKWARD_PIN, GPIO_PIN_RESET);

	/* set drive forward pins*/
	HAL_GPIO_WritePin(MOTOR_FR_FORWARD_PORT, MOTOR_FR_FORWARD_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_FL_FORWARD_PORT, MOTOR_FL_FORWARD_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_RR_FORWARD_PORT, MOTOR_RR_FORWARD_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_RL_FORWARD_PORT, MOTOR_RL_FORWARD_PIN, GPIO_PIN_SET);

	/* speed clamping */
	if(speed < 0)
	{
		speed = 0;
	}
	if(speed > 100)
	{
		speed = 100;
	}

	DRIVE_DIRECTION = DIR_FORWARD;
	HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(speed, MOTOR_FR);
	HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(speed, MOTOR_FL);
	HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(speed, MOTOR_RR);
	HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(speed, MOTOR_RL);
}


void drive_backward(uint8_t speed)
{
	/*
	 * :param speed: Desired motors' speed as duty cycle.
	 * :type speed: 8 bits unsigned integer
	 * :range speed: [0 - 100]
	 */

	/* stop pwm and wait 10 ms to make sure there will be no short */
	if(DRIVE_DIRECTION == DIR_FORWARD)
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
		HAL_Delay(10);
	}

	/* set drive backward pins */
	HAL_GPIO_WritePin(MOTOR_FR_BACKWARD_PORT, MOTOR_FR_BACKWARD_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_FL_BACKWARD_PORT, MOTOR_FL_BACKWARD_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_RR_BACKWARD_PORT, MOTOR_RR_BACKWARD_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_RL_BACKWARD_PORT, MOTOR_RL_BACKWARD_PIN, GPIO_PIN_SET);

	/* reset drive forward pins*/
	HAL_GPIO_WritePin(MOTOR_FR_FORWARD_PORT, MOTOR_FR_FORWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_FL_FORWARD_PORT, MOTOR_FL_FORWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_RR_FORWARD_PORT, MOTOR_RR_FORWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_RL_FORWARD_PORT, MOTOR_RL_FORWARD_PIN, GPIO_PIN_RESET);

	/* speed clamping */
	if(speed < 0)
	{
		speed = 0;
	}
	if(speed > 100)
	{
		speed = 100;
	}

	DRIVE_DIRECTION = DIR_BACKWARD;
	HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(speed, MOTOR_FR);
	HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(speed, MOTOR_FL);
	HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(speed, MOTOR_RR);
	HAL_TIM_IPX_CHANGE_TIM2_DUTY_CYCLE(speed, MOTOR_RL);
}


void drive_stop()
{
	/* reset drive forward pins*/
	HAL_GPIO_WritePin(MOTOR_FR_FORWARD_PORT, MOTOR_FR_FORWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_FL_FORWARD_PORT, MOTOR_FL_FORWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_RR_FORWARD_PORT, MOTOR_RR_FORWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_RL_FORWARD_PORT, MOTOR_RL_FORWARD_PIN, GPIO_PIN_RESET);

	/* reset drive backward pins */
	HAL_GPIO_WritePin(MOTOR_FR_BACKWARD_PORT, MOTOR_FR_BACKWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_FL_BACKWARD_PORT, MOTOR_FL_BACKWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_RR_BACKWARD_PORT, MOTOR_RR_BACKWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_RL_BACKWARD_PORT, MOTOR_RL_BACKWARD_PIN, GPIO_PIN_RESET);

	/* stop pwm channels */
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

	DRIVE_DIRECTION = DIR_NONE;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
