//
// Created by Yoshi on 2023/11/6.
//

#include "hardware_api.h"
#include "main.h"

extern uint32_t cs1_zero_value[3];
extern uint32_t cs2_zero_value[3];

void _init3PWM(){
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void _writeDutyCycle3PWM_1(float dc_a, float dc_b, float dc_c){
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, dc_a * htim1.Init.Period);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, dc_b * htim1.Init.Period);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, dc_c * htim1.Init.Period);
}

void _writeDutyCycle3PWM_2(float dc_a, float dc_b, float dc_c){
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, dc_a * htim2.Init.Period);
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, dc_b * htim2.Init.Period);
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, dc_c * htim2.Init.Period);
}

void _initCurrentSample(){
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    while(HAL_ADC_GetState(&hadc1) != HAL_ADC_STATE_READY);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    while(HAL_ADC_GetState(&hadc2) != HAL_ADC_STATE_READY);
}

void _getCurrentZeroValue(uint32_t *cs1_zeroValue, uint32_t *cs2_zeroValue){
    for(int i = 0; i < 999; i++){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        cs1_zeroValue[i % 3] = (uint32_t)movingAverageFilter(&moving_filter_cs1ZeroValue, HAL_ADC_GetValue(&hadc1));
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 100);
        cs2_zeroValue[i % 3] = (uint32_t)movingAverageFilter(&moving_filter_cs2ZeroValue, HAL_ADC_GetValue(&hadc2));
    }
}

void _currentGetValue(float *cs_value){
    static LowPass_Filter current_1[3];
    static float currentGained[3];
    for (int i = 0; i < 3; i++) {
        // Enables ADC, starts conversion of regular group.
        HAL_ADC_Start(&hadc1);
        // Wait for regular group conversion to be completed.
        HAL_ADC_PollForConversion(&hadc1, 100);
        int data = (int)(HAL_ADC_GetValue(&hadc1) - cs1_zero_value[i]);
        // 该函数读取寄存器DR同时自动清除了EOC(End Of unitary Conversation)标志位
        // cs_zero_value需要实际测量得出
        currentGained[i] = Low_Pass_Filter(&current_1[i] , (float)data, 0.1f);
        cs_value[i] = 0.00080586f * currentGained[i] / (CURRENT_GAIN * CURRENT_SAMPLE_RESISTER);
    }
    // Stop ADC conversion of regular group, disable ADC peripheral.
    HAL_ADC_Stop(&hadc1);
}

void _currentGetValue2(float *cs_value){
    static LowPass_Filter current_2[3];
    static float currentGained[3];
    for (int i = 0; i < 3; i++) {
        // Enables ADC, starts conversion of regular group.
        HAL_ADC_Start(&hadc2);
        // Wait for regular group conversion to be completed.
        HAL_ADC_PollForConversion(&hadc2, 100);
        int data = (int)(HAL_ADC_GetValue(&hadc2) - cs2_zero_value[i]);
        // 该函数读取寄存器DR同时自动清除了EOC(End Of unitary Conversation)标志位
        // cs_zero_value需要实际测量得出
        currentGained[i] = Low_Pass_Filter(&current_2[i] , (float)data, 0.1f);
        cs_value[i] = 0.00080586f * currentGained[i] / (CURRENT_GAIN * CURRENT_SAMPLE_RESISTER);
    }
    // Stop ADC conversion of regular group, disable ADC peripheral.
    HAL_ADC_Stop(&hadc2);
}