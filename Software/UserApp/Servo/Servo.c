#include "Servo.h"

Servo Servo_LeftLeg;
Servo Servo_RightLeg;

void Servo_init() {
    Servo_LeftLeg = Servo_Create("LeftLeg", &htim8, TIM_CHANNEL_1, 60);
    Servo_RightLeg = Servo_Create("RightLeg", &htim8, TIM_CHANNEL_2, 0);

    HAL_TIM_PWM_Start(Servo_LeftLeg.TIMER, Servo_LeftLeg.TIM_Channel);
    HAL_TIM_PWM_Start(Servo_RightLeg.TIMER, Servo_RightLeg.TIM_Channel);
}

TimerPeriod getAngle2Pulse(struct Servo_OOC *Servo) {
    return Servo->prHighLevelTimes_Ms;
}

void move2TargetAngle(struct Servo_OOC *Servo) {
            __HAL_TIM_SetCompare(Servo->TIMER, Servo->TIM_Channel, Servo->prHighLevelTimes_Ms);
}

void setAngle_180(struct Servo_OOC *Servo, ServoAngle target_Angle) {
    Servo->prLastAngle = target_Angle;
    target_Angle += Servo->prAngle_Offset;

    /********角度限幅, 使用反馈控制时可按需定义*************/
    if(target_Angle <= 2)
        target_Angle = 2;
    if(target_Angle >= 60)
        target_Angle = 60;
    /************************************************/

    Servo->prTarget_Angle = target_Angle;
    //换算角度至脉冲时长对应的占空比比较值
    Servo->prHighLevelTimes_Ms = (5e-4f * (float) Servo->TIMFreq +
                                  (Servo->prTarget_Angle / 180.00f) * (2e-3f * (float) Servo->TIMFreq));

    if (Servo->prHighLevelTimes_Ms > Servo->TIMER->Init.Period) {
        Servo_log("Counter Period overflow!\r\n");
    } else {
        move2TargetAngle(Servo);
    }
}

void setAngle_270(struct Servo_OOC *Servo, ServoAngle target_Angle) {
    Servo->prLastAngle = target_Angle;
    target_Angle += Servo->prAngle_Offset;

    /********角度限幅, 使用反馈控制时可按需定义*************/
    if(target_Angle <= 2)
        target_Angle = 2;
    if(target_Angle >= 60)
        target_Angle = 60;
    /************************************************/

    Servo->prTarget_Angle = target_Angle;
    //换算角度至脉冲时长对应的占空比比较值
    Servo->prHighLevelTimes_Ms = (5e-4f * (float) Servo->TIMFreq +
                                  (Servo->prTarget_Angle / 270.00f) * (2e-3f * (float) Servo->TIMFreq));

    if (Servo->prHighLevelTimes_Ms > Servo->TIMER->Init.Period) {
        Servo_log("Counter Period overflow!\r\n");
    } else {
        move2TargetAngle(Servo);
    }
}

void Servo_move_pitch(struct Servo_OOC *Servo, ServoAngle start_pitch, ServoAngle end_pitch, ServoAngle start_yaw, int time) {
    int i;
    float width;
    if (start_yaw >= 0) {
        width = (100 / cosf(fabsf(start_pitch))) * tanf(fabsf(start_yaw));
    } else {
        width = -(100 / cosf(fabsf(start_pitch))) * tanf(fabsf(start_yaw));
    }
    float temp = (end_pitch - start_pitch) / 100;
    for (i = 0; i < time; i++) {
        start_pitch += temp;
        if (start_pitch > end_pitch)break;
        setAngle_180(&Servo_LeftLeg, start_pitch);
        move2TargetAngle(&Servo_LeftLeg);
        start_yaw = atanf(width / (100 / cosf(fabsf(start_pitch))));
        setAngle_270(&Servo_RightLeg, start_yaw);
        move2TargetAngle(&Servo_RightLeg);
        HAL_Delay(10);
    }
}

void Servo_move_yaw(struct Servo_OOC *Servo, ServoAngle start_yaw, ServoAngle end_yaw, ServoAngle start_pitch, int time) {
    int i;
    float high;
    if (start_pitch >= 0) {
        high = (100 / cosf(fabsf(start_yaw))) * tanf(fabsf(start_pitch));

    } else {
        high = -(100 / cosf(fabsf(start_yaw))) * tanf(fabsf(start_pitch));
    }
    float temp = (end_yaw - start_yaw) / 100;
    for (i = 0; i < time; i++) {
        start_yaw += temp;
        if (start_yaw > end_yaw)break;
        setAngle_270(&Servo_RightLeg, start_yaw);
        move2TargetAngle(&Servo_RightLeg);
        start_pitch = atanf(high / (100 / cosf(fabsf(start_yaw))));
        setAngle_180(&Servo_LeftLeg, start_pitch);
        move2TargetAngle(&Servo_LeftLeg);
        HAL_Delay(10);
    }
}

Servo Servo_Create(char *name, TIM_HandleTypeDef *TIMER, uint32_t Channel, ServoAngle angle_Offset) {
    Servo servo_temple;

    servo_temple.name = name;
    servo_temple.TIMER = TIMER;
    servo_temple.TIM_Channel = Channel;
    //默认各个时钟都是相同的频率值 故此处用PCLK1来统一代替, 不做区分
    servo_temple.TIMFreq = HAL_RCC_GetPCLK1Freq() / servo_temple.TIMER->Init.Prescaler;

    //default arguments
    servo_temple.prTarget_Angle = 0;
    servo_temple.prHighLevelTimes_Ms = 5;
    servo_temple.prAngle_Offset = angle_Offset;

    //functions define
    servo_temple.GetAngle2Pulse = getAngle2Pulse;
    servo_temple.Move2TargetAngle = move2TargetAngle;

    return servo_temple;
}