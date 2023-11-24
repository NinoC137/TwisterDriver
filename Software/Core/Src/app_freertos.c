/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void FOCTask(void const *argument) {
    HAL_GPIO_WritePin(Driver1_EN_GPIO_Port, Driver1_EN_Pin, 1);

//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//
//            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);
//            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
//            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500);

    FOC_Vbus(12.0f);
    FOC_alignSensor(7, 1);

    for (;;) {
        velocityOpenLoop(30);
        osDelay(1);
    }
}

void LCDTask(void const *argument) {
    uart3_printf("LCD Task Start\n");
    ST7735_Init();
    gui_draw_init("NinoC137", 1);
    for (;;) {
        osDelay(10);
    }
}

void UARTTask(void const *argument) {
    for (;;) {
        osDelay(300);
    }
}

void CANTask(void const *argument) {
    for (;;) {
        osDelay(1000);
    }
}

void ServoTask(void const *argument) {
    for (;;) {
        osDelay(300);
    }
}

void ButtonTask(void const *argument) {
    HAL_UART_Transmit(&huart3, "Button Task Start\n", sizeof("Button Task Start\n") - 1, 0xff);

    button_init(&KEY1, read_KEY1_GPIO, 0);
    button_init(&KEY2, read_KEY2_GPIO, 0);
    button_init(&KEY3, read_KEY3_GPIO, 0);
    button_init(&KEY4, read_KEY4_GPIO, 0);
    button_init(&KEY5, read_KEY5_GPIO, 0);
    button_init(&KEY6, read_KEY6_GPIO, 0);

    button_attach(&KEY1, PRESS_DOWN, KEY1_PRESS_DOWN_Handler);
    button_attach(&KEY2, PRESS_DOWN, KEY2_PRESS_DOWN_Handler);
    button_attach(&KEY3, PRESS_DOWN, KEY3_PRESS_DOWN_Handler);
    button_attach(&KEY4, PRESS_DOWN, KEY4_PRESS_DOWN_Handler);
    button_attach(&KEY5, PRESS_DOWN, KEY5_PRESS_DOWN_Handler);
    button_attach(&KEY6, PRESS_DOWN, KEY6_PRESS_DOWN_Handler);

    button_start(&KEY1);
    button_start(&KEY2);
    button_start(&KEY3);
    button_start(&KEY4);
    button_start(&KEY5);
    button_start(&KEY6);

    for (;;) {
        button_ticks();
        osDelay(5);
    }
}
/* USER CODE END Application */

