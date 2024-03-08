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
float targetAngle_left = 60;
float targetAngle_right = 0;
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
    float angle_p, angle_f;

    Pid_Value_Init();

    HAL_GPIO_WritePin(Driver1_EN_GPIO_Port, Driver1_EN_Pin, 1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 5000);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 5000);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 5000);

    FOC_Vbus(12.0f);
    FOC_alignSensor(7, 1);

    for (;;) {
//        velocityOpenLoop(30);
//        FOC_M0_set_Velocity_Angle(targetAngle_left);
        FOC_M0_setVelocity(20);
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
extern float angle_pi;
extern int dataPackageUpdate;
void UARTTask(void const *argument) {
    HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart3Buffer, 1);
    for (;;) {
//        float currentSpeed = FOC_M0_Velocity() * 180.0f / _PI;
//        uart_printf("Motor1 speed: %d\t",(int)currentSpeed);
//        uart_printf("Motor1 angle: %d\r\n",(int)angle_pi);
        if(dataPackageUpdate == 1){
            dataPackageUpdate = 0;
            uart3_printf("buffer: %s\r\n", uart3Buffer);
        }
        osDelay(100);
    }
}

void CANTask(void const *argument) {
    for (;;) {
        osDelay(1000);
    }
}

void ServoTask(void const *argument) {
    Servo_init();

    osDelay(500);

    setAngle_270(&Servo_LeftLeg, 60);
    setAngle_270(&Servo_RightLeg, 5);

    uart_printf("Servo init.\r\n");
    
    for (;;) {
        setAngle_270(&Servo_LeftLeg, targetAngle_left);
        setAngle_270(&Servo_RightLeg, targetAngle_right);
        osDelay(300);
    }
}

void ButtonTask(void const *argument) {
    uart3_printf("Button Task Start\n");

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

