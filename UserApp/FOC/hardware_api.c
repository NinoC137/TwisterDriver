//
// Created by Yoshi on 2023/11/6.
//

#include "hardware_api.h"

#include "main.h"

void _init3PWM(){
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c){
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, dc_a * htim1.Init.Period);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, dc_b * htim1.Init.Period);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, dc_c * htim1.Init.Period);
}

void _initCurrentSample(){
//    HAL_ADCEx_Calibration_Start(&hadc1);
}

void _currentGetValue(float *cs_value){
    static LowPass_Filter current[3];
//    for (int i = 0; i < 3; ++i) {
//        // Enables ADC, starts conversion of regular group.
//        HAL_ADC_Start(&hadc1);
//        // Wait for regular group conversion to be completed.
//        HAL_ADC_PollForConversion(&hadc1, 100);
//
//        // 该函数读取寄存器DR同时自动清除了EOC(End Of unitary Conversation)标志位
//        // cs_zero_value需要实际测量得出
//        cs_value[i] = Low_Pass_Filter(&current[i] , HAL_ADC_GetValue(&hadc1) - cs_zero_value);
//    }
//    // Stop ADC conversion of regular group, disable ADC peripheral.
//    HAL_ADC_Stop(&hadc1);
}