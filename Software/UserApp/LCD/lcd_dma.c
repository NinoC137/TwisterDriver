#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"
#include "pic.h"

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

uint8_t spiDMAFinshedFlag = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
    UNUSED(hspi);
    if(hspi == &hspi1){
//        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        spiDMAFinshedFlag = 1;
    }
}

void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi){
    UNUSED(hspi);
    if(hspi == &hspi1){
//        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
}

const unsigned int color_Fill[21] = {
        0xFFFF, 0x0000, 0x001F, 0XF81F, 0XFFE0, 0X07FF, 0xF800, 0xF81F,
        0x07E0, 0x7FFF, 0xFFE0, 0XBC40, 0XFC07, 0X8430, 0X01CF, 0X7D7C,
        0X5458, 0X841F, 0XC618, 0XA651, 0X2B12
};

void LCD_Fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color) {
    uint16_t i, j;
    uint16_t* buf = &color;

    uint16_t xLen, yLen;
    xLen = xend - xsta;
    yLen = yend - ysta;

    if(xLen * yLen * 4 <= 65535){
        LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);
        LCD_CS_Clr();
        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)buf, (xLen * yLen * 4));
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        LCD_CS_Set();
    }else{
        LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);
        LCD_CS_Clr();
        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)buf, (xLen * yLen * 4) / 10);
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        LCD_CS_Set();

        LCD_Address_Set(xsta, ysta + (yLen * 2 / 10), xend - 1, yend - 1);
        LCD_CS_Clr();
        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)buf, (xLen * yLen * 4) / 10);
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        LCD_CS_Set();

        LCD_Address_Set(xsta, ysta + (yLen * 4 / 10), xend - 1, yend - 1);
        LCD_CS_Clr();
        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)buf, (xLen * yLen * 4) / 10);
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        LCD_CS_Set();

        LCD_Address_Set(xsta, ysta + (yLen * 6 / 10), xend - 1, yend - 1);
        LCD_CS_Clr();
        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)buf, (xLen * yLen * 4) / 10);
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        LCD_CS_Set();

        LCD_Address_Set(xsta, ysta + (yLen * 8 / 10), xend - 1, yend - 1);
        LCD_CS_Clr();
        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)buf, (xLen * yLen * 4) / 10);
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        LCD_CS_Set();
    }
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color) {
    LCD_Address_Set(x, y, x, y);//���ù��λ��
    LCD_WR_DATA(color);
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; //������������
    delta_y = y2 - y1;
    uRow = x1;//�����������
    uCol = y1;
    if (delta_x > 0)incx = 1; //���õ�������
    else if (delta_x == 0)incx = 0;//��ֱ��
    else {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)incy = 1;
    else if (delta_y == 0)incy = 0;//ˮƽ��
    else {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)distance = delta_x; //ѡȡ��������������
    else distance = delta_y;
    for (t = 0; t < distance + 1; t++) {
        LCD_DrawPoint(uRow, uCol, color);//����
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

void Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color) {
    int a, b;
    a = 0;
    b = r;
    while (a <= b) {
        LCD_DrawPoint(x0 - b, y0 - a, color);             //3
        LCD_DrawPoint(x0 + b, y0 - a, color);             //0
        LCD_DrawPoint(x0 - a, y0 + b, color);             //1
        LCD_DrawPoint(x0 - a, y0 - b, color);             //2
        LCD_DrawPoint(x0 + b, y0 + a, color);             //4
        LCD_DrawPoint(x0 + a, y0 - b, color);             //5
        LCD_DrawPoint(x0 + a, y0 + b, color);             //6
        LCD_DrawPoint(x0 - b, y0 + a, color);             //7
        a++;
        if ((a * a + b * b) > (r * r))//�ж�Ҫ���ĵ��Ƿ��Զ
        {
            b--;
        }
    }
}

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, TypefaceNum;//һ���ַ���ռ�ֽڴ�С
    uint16_t x0 = x;
    sizex = sizey / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';    //�õ�ƫ�ƺ��ֵ
    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);  //���ù��λ��
    for (i = 0; i < TypefaceNum; i++) {
        if (sizey == 12)temp = ascii_1206[num][i];               //����6x12����
        else if (sizey == 16)temp = ascii_1608[num][i];         //����8x16����
        else if (sizey == 24)temp = ascii_2412[num][i];         //����12x24����
        else if (sizey == 32)temp = ascii_3216[num][i];         //����16x32����
        else return;
        for (t = 0; t < 8; t++) {
            if (!mode)//�ǵ���ģʽ
            {
                if (temp & (0x01 << t))LCD_WR_DATA(fc);
                else LCD_WR_DATA(bc);
                m++;
                if (m % sizex == 0) {
                    m = 0;
                    break;
                }
            } else//����ģʽ
            {
                if (temp & (0x01 << t))LCD_DrawPoint(x, y, fc);//��һ����
                x++;
                if ((x - x0) == sizex) {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
}

void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
    while (*p != '\0') {
        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
        x += sizey / 2;
        p++;
    }
}

uint32_t mypow(uint8_t m, uint8_t n) {
    uint32_t result = 1;
    while (n--)result *= m;
    return result;
}

void LCD_ShowIntNum(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey) {
    uint8_t t, temp;
    uint8_t enshow = 0;
    uint8_t sizex = sizey / 2;
    for (t = 0; t < len; t++) {
        temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0);
                continue;
            } else enshow = 1;

        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

void LCD_ShowFloatNum1(uint16_t x, uint16_t y, float num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey) {
    uint8_t t, temp, sizex;
    uint16_t num1;
    sizex = sizey / 2;
    num1 = num * 100;
    for (t = 0; t < len; t++) {
        temp = (num1 / mypow(10, len - t - 1)) % 10;
        if (t == (len - 2)) {
            LCD_ShowChar(x + (len - 2) * sizex, y, '.', fc, bc, sizey, 0);
            t++;
            len += 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t length, uint16_t width, const uint8_t pic[]) {
    uint16_t i, j;
    uint32_t k = 0;
    LCD_Address_Set(x, y, x + length - 1, y + width - 1);
    for (i = 0; i < length; i++) {
        for (j = 0; j < width; j++) {
            LCD_WR_DATA8(pic[k * 2]);
            LCD_WR_DATA8(pic[k * 2 + 1]);
            k++;
        }
    }
}


