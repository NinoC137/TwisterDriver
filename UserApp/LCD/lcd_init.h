#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "main.h"
#include "stm32g4xx_hal.h"

#include "lcd.h"

#define USE_HORIZONTAL 0

#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
#define LCD_W 80
#define LCD_H 160

#else
#define LCD_W 480
#define LCD_H 320
#endif

#define LCD_RES_Clr()  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, 0)
#define LCD_RES_Set()  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, 1)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, 0)
#define LCD_DC_Set()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, 1)

#define LCD_CS_Clr()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, 0)
#define LCD_CS_Set()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, 1)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(BLK_GPIO_Port, BLK_Pin, 0)
#define LCD_BLK_Set()  HAL_GPIO_WritePin(BLK_GPIO_Port, BLK_Pin, 1)

void LCD_Writ_Bus(uint8_t dat);

void LCD_WR_DATA8(uint8_t dat);

void LCD_WR_DATA(uint16_t dat);

void LCD_WR_REG(uint8_t dat);

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void LCD_Init(void);

#endif




