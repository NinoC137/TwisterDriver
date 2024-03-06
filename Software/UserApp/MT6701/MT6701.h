#ifndef __MT6701_H__
#define __MT6701_H__

#include <stdio.h>

#define MT6701_SLAVE_ADDR         0x06 << 1
#define MT6701_Timeout            50

#define MT6701_REG_ANGLE_14b      0x03    // 14Bit角度信息，存储在0x03[13:6]、0x04[5:0]两个寄存器中，高位在前，原始读数0~16383

#define mt6701_log		uart_printf

void i2c_mt6701_get_angle(float *angle_pi, float *angle_f);

void i2c2_mt6701_get_angle(float *angle_pi, float *angle_f);

#endif

