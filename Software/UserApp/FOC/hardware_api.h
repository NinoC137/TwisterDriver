//
// Created by Yoshi on 2023/11/6.
//

#ifndef FOC_CONTROL_HARDWARE_API_H
#define FOC_CONTROL_HARDWARE_API_H

#include "FOC_math.h"

void _init3PWM();

/**
 * Function setting the duty cycle to the pwm pin (ex. analogWrite())
 * - BLDC driver - 3PWM setting
 * - hardware specific
 *
 * @param dc_a  duty cycle phase A [0, 1]
 * @param dc_b  duty cycle phase B [0, 1]
 * @param dc_c  duty cycle phase C [0, 1]
 */
void _writeDutyCycle3PWM_1(float dc_a, float dc_b, float dc_c);

void _writeDutyCycle3PWM_2(float dc_a, float dc_b, float dc_c);

void _initCurrentSample();

/**
 * Function getting the value of current. (ex. getAnalogValue())
 * Current sensor Sample api
 *
 * @param cs_value pointer of a float array, stored current value
 */
void _currentGetValue(float *cs_value);

#endif //FOC_CONTROL_HARDWARE_API_H
