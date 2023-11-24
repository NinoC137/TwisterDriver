/*
 * Copyright (c) 2016 Zibin Zheng <znbin@qq.com>
 * All rights reserved
 */

#include "multi_button.h"

#define EVENT_CB(ev)   if(handle->cb[ev])handle->cb[ev]((Button*)handle)

struct Button KEY1;
struct Button KEY2;
struct Button KEY3; //步进电机切换
struct Button KEY4; //下药模式切换
struct Button KEY5; //开启下药
struct Button KEY6; //待定

extern int gMode;
extern int StepID;
extern int SubBox_Buffer[4];

//button handle list head.
static struct Button *head_handle = NULL;

/**
  * @brief  Initializes the button struct handle.
  * @param  handle: the button handle strcut.
  * @param  pin_level: read the HAL GPIO of the connet button level.
  * @param  active_level: pressed GPIO level.
  * @retval None
  */
void button_init(struct Button *handle, uint8_t(*pin_level)(), uint8_t active_level) {
    memset(handle, 0, sizeof(struct Button));
    handle->event = (uint8_t) NONE_PRESS;
    handle->hal_button_Level = pin_level;
    handle->button_level = handle->hal_button_Level();
    handle->active_level = active_level;
}

/**
  * @brief  Attach the button event callback function.
  * @param  handle: the button handle strcut.
  * @param  event: trigger event type.
  * @param  cb: callback function.
  * @retval None
  */
void button_attach(struct Button *handle, PressEvent event, BtnCallback cb) {
    handle->cb[event] = cb;
}

/**
  * @brief  Inquire the button event happen.
  * @param  handle: the button handle strcut.
  * @retval button event.
  */
PressEvent get_button_event(struct Button *handle) {
    return (PressEvent) (handle->event);
}

/**
  * @brief  Button driver core function, driver state machine.
  * @param  handle: the button handle strcut.
  * @retval None
  */
void button_handler(struct Button *handle) {
    uint8_t read_gpio_level = handle->hal_button_Level();

    //ticks counter working..
    if ((handle->state) > 0) handle->ticks++;

    /*------------button debounce handle---------------*/
    if (read_gpio_level != handle->button_level) { //not equal to prev one
        //continue read 3 times same new level change
        if (++(handle->debounce_cnt) >= DEBOUNCE_TICKS) {
            handle->button_level = read_gpio_level;
            handle->debounce_cnt = 0;
        }
    } else { //leved not change ,counter reset.
        handle->debounce_cnt = 0;
    }

    /*-----------------State machine-------------------*/
    switch (handle->state) {
        case 0:
            if (handle->button_level == handle->active_level) {    //start press down
                handle->event = (uint8_t) PRESS_DOWN;
                EVENT_CB(PRESS_DOWN);
                handle->ticks = 0;
                handle->repeat = 1;
                handle->state = 1;
            } else {
                handle->event = (uint8_t) NONE_PRESS;
            }
            break;

        case 1:
            if (handle->button_level != handle->active_level) { //released press up
                handle->event = (uint8_t) PRESS_UP;
                EVENT_CB(PRESS_UP);
                handle->ticks = 0;
                handle->state = 2;

            } else if (handle->ticks > LONG_TICKS) {
                handle->event = (uint8_t) LONG_PRESS_START;
                EVENT_CB(LONG_PRESS_START);
                handle->state = 5;
            }
            break;

        case 2:
            if (handle->button_level == handle->active_level) { //press down again
                handle->event = (uint8_t) PRESS_DOWN;
                EVENT_CB(PRESS_DOWN);
                handle->repeat++;
                EVENT_CB(PRESS_REPEAT); // repeat hit
                handle->ticks = 0;
                handle->state = 3;
            } else if (handle->ticks > SHORT_TICKS) { //released timeout
                if (handle->repeat == 1) {
                    handle->event = (uint8_t) SINGLE_CLICK;
                    EVENT_CB(SINGLE_CLICK);
                } else if (handle->repeat == 2) {
                    handle->event = (uint8_t) DOUBLE_CLICK;
                    EVENT_CB(DOUBLE_CLICK); // repeat hit
                }
                handle->state = 0;
            }
            break;

        case 3:
            if (handle->button_level != handle->active_level) { //released press up
                handle->event = (uint8_t) PRESS_UP;
                EVENT_CB(PRESS_UP);
                if (handle->ticks < SHORT_TICKS) {
                    handle->ticks = 0;
                    handle->state = 2; //repeat press
                } else {
                    handle->state = 0;
                }
            } else if (handle->ticks > SHORT_TICKS) { // long press up
                handle->state = 0;
            }
            break;

        case 5:
            if (handle->button_level == handle->active_level) {
                //continue hold trigger
                handle->event = (uint8_t) LONG_PRESS_HOLD;
                EVENT_CB(LONG_PRESS_HOLD);

            } else { //releasd
                handle->event = (uint8_t) PRESS_UP;
                EVENT_CB(PRESS_UP);
                handle->state = 0; //reset
            }
            break;
    }
}

/**
  * @brief  Start the button work, add the handle into work list.
  * @param  handle: target handle strcut.
  * @retval 0: succeed. -1: already exist.
  */
int button_start(struct Button *handle) {
    struct Button *target = head_handle;
    while (target) {
        if (target == handle) return -1;    //already exist.
        target = target->next;
    }
    handle->next = head_handle;
    head_handle = handle;
    return 0;
}

/**
  * @brief  Stop the button work, remove the handle off work list.
  * @param  handle: target handle strcut.
  * @retval None
  */
void button_stop(struct Button *handle) {
    struct Button **curr;
    for (curr = &head_handle; *curr;) {
        struct Button *entry = *curr;
        if (entry == handle) {
            *curr = entry->next;
//			free(entry);
            return;//glacier add 2021-8-18
        } else
            curr = &entry->next;
    }
}

/**
  * @brief  background ticks, timer repeat invoking interval 5ms.
  * @param  None.
  * @retval None
  */
void button_ticks() {
    struct Button *target;
    for (target = head_handle; target; target = target->next) {
        button_handler(target);
    }
}

uint8_t read_KEY1_GPIO() {
    return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
}

uint8_t read_KEY2_GPIO() {
    return HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
}

uint8_t read_KEY3_GPIO() {
    return HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
}

uint8_t read_KEY4_GPIO() {
    return HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin);
}

uint8_t read_KEY5_GPIO() {
    return HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin);
}

uint8_t read_KEY6_GPIO() {
    return HAL_GPIO_ReadPin(ENCD_KEY_GPIO_Port, ENCD_KEY_Pin);
}

void KEY1_PRESS_DOWN_Handler(void *btn){
    HAL_UART_Transmit(&huart3, "key1 press\n", sizeof("key1 press\n") - 1, 0xffff);
}

void KEY2_PRESS_DOWN_Handler(void *btn){
    HAL_UART_Transmit(&huart3, "key2 press\n", sizeof("key1 press\n") - 1, 0xffff);
}

void KEY3_PRESS_DOWN_Handler(void *btn) {
    HAL_UART_Transmit(&huart3, "key3 press\n", sizeof("key1 press\n") - 1, 0xffff);
}

void KEY4_PRESS_DOWN_Handler(void *btn) {
    HAL_UART_Transmit(&huart3, "key4 press\n", sizeof("key1 press\n") - 1, 0xffff);
}

void KEY5_PRESS_DOWN_Handler(void *btn) {
    HAL_UART_Transmit(&huart3, "key5 press\n", sizeof("key1 press\n") - 1, 0xffff);
}

void KEY6_PRESS_DOWN_Handler(void *btn) {
    HAL_UART_Transmit(&huart3, "Encoder press\n", sizeof("Encoder press\n") - 1, 0xffff);
}
