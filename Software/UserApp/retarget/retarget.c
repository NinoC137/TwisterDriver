//
// Created by nino on 23-6-18.
//

#include "retarget.h"
#include "cmsis_os.h"

#define Usart_Number 20

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern osMutexId printfMutex;

char uart3Buffer[128];

void uart_printf(const char* format, ...) {
//    osMutexWait(printfMutex, portMAX_DELAY);
    char buffer[128];  // 缓冲区用于存储格式化后的字符串
    va_list args;
    va_start(args, format);

    vsnprintf(buffer, sizeof(buffer), format, args);  // 格式化字符串到缓冲区
    va_end(args);

    for (size_t i = 0; buffer[i] != '\0'; ++i) {
        HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[i], 1, HAL_MAX_DELAY);
    }
//    osMutexRelease(printfMutex);
}

void uart3_printf(const char* format, ...) {
    osMutexWait(printfMutex, portMAX_DELAY);
    char buffer[128];  // 缓冲区用于存储格式化后的字符串
    va_list args;
    va_start(args, format);

    vsnprintf(buffer, sizeof(buffer), format, args);  // 格式化字符串到缓冲区
    va_end(args);

    for (size_t i = 0; buffer[i] != '\0'; ++i) {
        HAL_UART_Transmit(&huart3, (uint8_t *) &buffer[i], 1, HAL_MAX_DELAY);
    }
    osMutexRelease(printfMutex);
}

void ReformatBuffer(uint8_t *buffer, float *afterReformat) {
    uint16_t i, j;
    uint8_t array_flag = 0;//标志位初始为0
    float a_first = 0, b_first = 0, a_sec = 0, a_trd = 0, b_sec = 0, b_trd = 0;
    uint8_t *temp_buf;
    static float temp_1 = 0, temp_2 = 0;

    temp_buf = buffer;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
        /*当出现“a:”和“，“，则输出a：和，之间的数据，出现b：和，类似*/
        for (i = 0; i < Usart_Number; i++) {
            //字符转义, 对照ASCII表 字符型数字-48后成为原数字.
            if (temp_buf[i] == 'x' && temp_buf[i + 1] == ':') {
                if(temp_buf[i+2] == '-')
                {
                    a_first = (float) temp_buf[i + 3] - 48;//逗号后面的第一个数据被赋值给a_first
                    a_sec = (float) temp_buf[i + 4] - 48;
                    a_trd = (float) temp_buf[i + 5] - 48;
                    afterReformat[0] = -(a_first*100 + a_sec*10 + a_trd);
                }else{
                    a_first = (float) temp_buf[i + 3] - 48;//逗号后面的第一个数据被赋值给a_first
                    a_sec = (float) temp_buf[i + 4] - 48;
                    a_trd = (float) temp_buf[i + 5] - 48;
                    afterReformat[0] = a_first*100 + a_sec*10 + a_trd;
                }
            } else if (temp_buf[i] == 'y' && temp_buf[i + 1] == ':') {
                if(temp_buf[i+2] == '-'){
                    b_first = (float) temp_buf[i + 3] - 48;//逗号后面的第一个数据被赋值给b_first
                    b_sec = (float) temp_buf[i + 4] - 48;
                    b_trd = (float) temp_buf[i + 5] - 48;
                    afterReformat[1] = -(b_first * 100 + b_sec * 10 + b_trd);
                }else{
                    b_first = (float) temp_buf[i + 3] - 48;//逗号后面的第一个数据被赋值给b_first
                    b_sec = (float) temp_buf[i + 4] - 48;
                    b_trd = (float) temp_buf[i + 5] - 48;
                    afterReformat[1] = b_first*100 + b_sec*10 + b_trd;
                }
            }
        }
    }

}

float Reformat_Float(const char* format){
    // find ":" or sth else
    char *xPtr = strchr(format, 'c');
    char *colonPtr = strchr(format, ':');
    if (xPtr == NULL && colonPtr == NULL) {
        uart_printf("Invalid input format.\n");
        return 0;
    }

    // 提取冒号后的浮点数部分
    char *floatStr = colonPtr + 1;
    float value = atof(floatStr); // 使用 atof 函数将字符串转换为浮点数

    /*
     * input: uint8_t uartBuffer[];
     * example:
     * uart_printf("Value: %.2f\r\n",Reformat_Float((char*)uartBuffer));
     * */

    return value;
}

uint8_t hexCharToUint8(char c1, char c2) {
    uint8_t value = 0;

    if (c1 >= '0' && c1 <= '9')
        value = (c1 - '0') << 4;
    else if (c1 >= 'A' && c1 <= 'F')
        value = (c1 - 'A' + 10) << 4;

    if (c2 >= '0' && c2 <= '9')
        value |= (c2 - '0');
    else if (c2 >= 'A' && c2 <= 'F')
        value |= (c2 - 'A' + 10);

    return value;
}

uint32_t Reformat_TOF(const char *format){
    uint8_t dataArray[7] = {0};

    int dataIndex = 0;
    char buffer[3];
    for (int i = 0; i < strlen(format); i += 3) {
        strncpy(buffer, format + i, 2);
        buffer[2] = '\0';  // 添加字符串结尾
        dataArray[dataIndex++] = hexCharToUint8(buffer[0], buffer[1]);
    }

    uint32_t combinedData = 0;
    // 将第一个数组的值放入高8位
    combinedData = ((uint32_t)dataArray[3]) << 24;
    // 将第二个数组的值放入次高8位
    combinedData |= ((uint32_t)dataArray[4]) << 16;
    // 将第三个数组的值放入次低8位
    combinedData |= ((uint32_t)dataArray[5]) << 8;
    // 将第四个数组的值放入低8位
    combinedData |= dataArray[6];

    return combinedData / (1 << 16);
}
