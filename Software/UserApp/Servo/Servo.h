#ifndef __SG90_H
#define __SG90_H

//include
#include "main.h"
#include "math.h"
//#include "cmsis_os.h"

#define SG90_FINISH 1
#define SG90_RUNNING 0
//extern

//SG90 OOP_By_C
typedef struct SG90_OOC {
    //elements
    char *name;
    float prTarget_Angle;
    float prHighLevelTimes_Ms;

    TIM_HandleTypeDef *TIMER;
    uint32_t TIM_Channel;

    float prLastAngle;

    //functions
    float (*GetAngle2Pulse)(struct SG90_OOC *);

    void (*SetAngle_180)(struct SG90_OOC *, float target_Angle);

    void (*SetAngle_270)(struct SG90_OOC *, float target_Angle);

    void (*Move2TargetAngle)(struct SG90_OOC *);
} SG90;

//Create an Oriented
SG90 SG90_Create(char *name, TIM_HandleTypeDef *TIMER, uint32_t Channel);

//OOP function define
float getAngle2Pulse(struct SG90_OOC *SG90);

void setAngle_180(struct SG90_OOC *SG90, float target_Angle);

void setAngle_270(struct SG90_OOC *SG90, float target_Angle);

void SG90_scan(void);

void move2TargetAngle(struct SG90_OOC *SG90);

void SG90_move_pitch(struct SG90_OOC *SG90, float start_pitch, float end_pitch, float start_yaw, int time);

void SG90_move_yaw(struct SG90_OOC *SG90, float start_yaw, float end_yaw, float start_pitch, int time);

void setAngle_180_slowly(struct SG90_OOC *SG90, float target_Angle, int time);

void setAngle_270_slowly(struct SG90_OOC *SG90, float target_Angle, int time);

//Application functions
void SG90_init();

char *Click_Once(struct SG90_OOC *aSG90);

char *Click_Num(struct SG90_OOC *aSG90, uint8_t Num);

extern SG90 Servo_pitch;
extern SG90 Servo_yaw;
#endif