#ifndef __SG90_H
#define __SG90_H

//include
#include "main.h"
#include "math.h"

#define Servo_log uart_printf
typedef float ServoAngle;
typedef float TimerPeriod;

//Servo OOP_By_C
typedef struct Servo_OOC {
    //elements
    char *name;
    ServoAngle prTarget_Angle;
    TimerPeriod prHighLevelTimes_Ms;
    ServoAngle prAngle_Offset;

    TIM_HandleTypeDef *TIMER;
    uint32_t TIM_Channel;
    uint32_t TIMFreq;

    ServoAngle prLastAngle;

    //functions
    float (*GetAngle2Pulse)(struct Servo_OOC *);

    void (*Move2TargetAngle)(struct Servo_OOC *);
} Servo;

//extern
extern Servo Servo_LeftLeg;
extern Servo Servo_RightLeg;

//Create an Oriented
Servo Servo_Create(char *name, TIM_HandleTypeDef *TIMER, uint32_t Channel, ServoAngle angle_Offset);

//OOP function define
TimerPeriod getAngle2Pulse(struct Servo_OOC *Servo);

void move2TargetAngle(struct Servo_OOC *Servo);

//Application functions
void Servo_init();

void setAngle_180(struct Servo_OOC *Servo, ServoAngle target_Angle);

void setAngle_270(struct Servo_OOC *Servo, ServoAngle target_Angle);

void Servo_move_pitch(struct Servo_OOC *Servo, ServoAngle start_pitch, ServoAngle end_pitch, ServoAngle start_yaw, int time);

void Servo_move_yaw(struct Servo_OOC *Servo, ServoAngle start_yaw, ServoAngle end_yaw, ServoAngle start_pitch, int time);

#endif