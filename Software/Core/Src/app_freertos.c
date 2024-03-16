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
float targetMotorSpeed_Left;
float targetMotorSpeed_Right;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void FOC_LeftTask(void const *argument) {
    Pid_Value_Init();
    HAL_GPIO_WritePin(Driver1_EN_GPIO_Port, Driver1_EN_Pin, 1);

    FOC_Vbus(12.3f);    //3s 电池
    FOC_alignSensor(&FOCMotor_Left, 7, -1);

    for (;;) {
//        velocityOpenLoop(&FOCMotor_Left,10);
//        FOCMotor_Left.api_getMotorCurrent(FOCMotor_Left.current);
//        FOC_setAngle(&FOCMotor_Left, targetAngle_right);
//        FOC_setVelocity(&FOCMotor_Left, targetMotorSpeed_Left);
        FOC_setVelocityAngle(&FOCMotor_Left, targetAngle_left);
//        FOC_current_control_loop(&FOCMotor_Left, targetMotorSpeed_Left*10);
        osDelay(3);
    }
}

void FOC_RightTask(void const *argument) {
//    Pid_Value_Init();
//    FOC_Vbus(12.3f);    //3s 电池
    HAL_GPIO_WritePin(Driver2_EN_GPIO_Port, Driver2_EN_Pin, 1);
    FOC_alignSensor(&FOCMotor_Right, 7, 1);
    for (;;) {
//        velocityOpenLoop(&FOCMotor_Right,10);
//        FOC_setAngle(&FOCMotor_Right, targetAngle_right);
//        FOC_setVelocity(&FOCMotor_Right, targetMotorSpeed_Right);
//        FOC_current_control_loop(&FOCMotor_Right, targetMotorSpeed_Right*10);
        osDelay(3);
    }
}

void ServoTask(void const *argument) {
    Servo_init();

    setAngle_270(&Servo_LeftLeg, 60);
    setAngle_270(&Servo_RightLeg, 5);
    osDelay(500);

    for (;;) {
        uart3_printf("%f,%f,%f,%f,%f,%f\n",FOCMotor_Right.current[0] , FOCMotor_Right.current[1], FOCMotor_Right.current[2],
                     FOCMotor_Right.Iq,FOCMotor_Right.Id, FOCMotor_Right.angle_pi);
//        uart3_printf("%f,%f,%f,%f,%f,%f\n",FOCMotor_Left.current[0] , FOCMotor_Left.current[1], FOCMotor_Left.current[2],
//                     FOCMotor_Left.Iq,FOCMotor_Left.Id, FOCMotor_Left.angle_pi);
//        setAngle_270(&Servo_LeftLeg, targetAngle_left);
//        setAngle_270(&Servo_RightLeg, targetAngle_right);
        osDelay(5);
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
    HAL_UART_Receive_IT(&huart3, (uint8_t*)&uart3Buffer, 1);

    osEvent JsonQueueEvt;
    t_JsonPackage *JsonBuffer = NULL;

    for (;;) {
        JsonQueueEvt = osMessageGet(JsonQueueHandle, osWaitForever);

        if(JsonQueueEvt.status == osEventMessage){
            JsonBuffer = JsonQueueEvt.value.p;
            uart3_printf("buffer:\r\n %s\r\n", JsonBuffer->JsonString);
            cmd_startParse(JsonBuffer->JsonString);
            osPoolFree(JsonQ_Mem, JsonBuffer);
        }
    }
}

void CANTask(void const *argument) {
    for (;;) {
        osDelay(1000);
    }
}

void ButtonTask(void const *argument) {
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

