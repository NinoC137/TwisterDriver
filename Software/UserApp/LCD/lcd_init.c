#include "lcd_init.h"

extern SPI_HandleTypeDef hspi1;

void LCD_Writ_Bus(uint8_t dat) {
    LCD_CS_Clr();
//    while (HAL_SPI_Transmit(&hspi1, &dat, 1, 0xffff) != HAL_OK);
    while (HAL_SPI_Transmit_DMA(&hspi1, &dat, 1) != HAL_OK);
    LCD_CS_Set();
}

void LCD_WR_DATA8(uint8_t dat) {
    LCD_Writ_Bus(dat);
}

//Write data
void LCD_WR_DATA(uint16_t dat) {
    uint8_t buf[2];
    buf[0] = dat >> 8;
    buf[1] = dat;
    LCD_CS_Clr();
    while (HAL_SPI_Transmit(&hspi1, buf, 2, 0xffff) != HAL_OK);
//    while (HAL_SPI_Transmit_DMA(&hspi1, buf, 2) != HAL_OK);
    LCD_CS_Set();
}

//Write command
void LCD_WR_REG(uint8_t dat) {
    LCD_DC_Clr();
    LCD_Writ_Bus(dat);
    LCD_DC_Set();
}

void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    LCD_WR_REG(0x2a);
    LCD_WR_DATA(x1);
    LCD_WR_DATA(x2);
    LCD_WR_REG(0x2b);
    LCD_WR_DATA(y1);
    LCD_WR_DATA(y2);
    LCD_WR_REG(0x2c);
}

void LCD_Init(void) {
    LCD_RES_Clr();
    HAL_Delay(1);
    LCD_RES_Set();
    HAL_Delay(2);

//    LCD_WR_REG(0x10);
//    HAL_Delay(5000);
    //************* Start Initial Sequence **********//
    LCD_WR_REG(0x11); //Sleep out
    HAL_Delay(120);    //Delay 120ms
    //************* Start Initial Sequence **********//
    LCD_WR_REG(0Xf0);
    LCD_WR_DATA8(0xc3);
    LCD_WR_REG(0Xf0);
    LCD_WR_DATA8(0x96);
    LCD_WR_REG(0x36);    // Memory Access Control
    if (USE_HORIZONTAL == 0)LCD_WR_DATA8(0x48);
    else if (USE_HORIZONTAL == 1)LCD_WR_DATA8(0x88);
    else if (USE_HORIZONTAL == 2)LCD_WR_DATA8(0x28);
    else LCD_WR_DATA8(0xE8);

    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);

    LCD_WR_REG(0Xe8);
    LCD_WR_DATA8(0x40);
    LCD_WR_DATA8(0x82);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x27);
    LCD_WR_DATA8(0x0a);
    LCD_WR_DATA8(0xb6);
    LCD_WR_DATA8(0x33);

    LCD_WR_REG(0Xc5);
    LCD_WR_DATA8(0x27);

    LCD_WR_REG(0Xc2);
    LCD_WR_DATA8(0xa7);

    LCD_WR_REG(0Xe0);
    LCD_WR_DATA8(0xf0);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x06);
    LCD_WR_DATA8(0x0f);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x1d);
    LCD_WR_DATA8(0x36);
    LCD_WR_DATA8(0x54);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x0c);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x16);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x15);

    LCD_WR_REG(0Xe1);
    LCD_WR_DATA8(0xf0);
    LCD_WR_DATA8(0x01);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x0a);
    LCD_WR_DATA8(0x0b);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x32);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x0c);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x17);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x16);

    LCD_WR_REG(0Xf0);
    LCD_WR_DATA8(0x3c);

    LCD_WR_REG(0Xf0);
    LCD_WR_DATA8(0x69);

    LCD_WR_REG(0X29);

    LCD_Address_Set(0, 0, LCD_W, LCD_H);
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
} 
