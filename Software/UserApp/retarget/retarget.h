
//
// Created by nino on 23-6-18.
//

#ifndef STM32H7_TEST_RETARGET_H
#define STM32H7_TEST_RETARGET_H

#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "string.h"

extern char uart3Buffer[128];

void uart_printf(const char *format, ...);

void uart3_printf(const char* format, ...);

void ReformatBuffer(uint8_t *buffer, float *afterReformat);

float Reformat_Float(const char *format);

uint32_t Reformat_TOF(const char *format);

#endif //STM32H7_TEST_RETARGET_H
