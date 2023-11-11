#include "Servo.h"

SG90 Servo_pitch;
SG90 Servo_yaw;

float getAngle2Pulse(struct SG90_OOC *SG90) {
    return SG90->prHighLevelTimes_Ms;
}

void setAngle_180(struct SG90_OOC *SG90, float target_Angle) {
    SG90->prLastAngle = target_Angle;
    target_Angle += 40.2f;
    if(target_Angle <= 20)
        target_Angle = 20;
    if(target_Angle >= 75)
        target_Angle = 75;
    SG90->prTarget_Angle = target_Angle;
//    SG90->prHighLevelTimes_Ms = (500.00f + (SG90->prTarget_Angle/ 18.00f) * 20.00f);
    SG90->prHighLevelTimes_Ms = (4000.00f + (SG90->prTarget_Angle / 18.00f) * 800.00f);

    move2TargetAngle(SG90);
}

void setAngle_180_slowly(struct SG90_OOC *SG90, float target_Angle, int time)
{
    float start_angle = SG90->prLastAngle;
    float temp_angle = start_angle;
    float temp = (target_Angle - temp_angle) / (float)time;
    for(int i=0;i<time;i++)
    {
        temp_angle += temp;
//        if((start_angle<target_Angle)&&(temp_angle > target_Angle))break;
//        else if((start_angle>target_Angle)&&(temp_angle < target_Angle))break;
        setAngle_180(&Servo_pitch, temp_angle);
        HAL_Delay(10);

    }
}

void setAngle_270_slowly(struct SG90_OOC *SG90, float target_Angle, int time)
{
    float start_angle = SG90->prLastAngle;
    float temp_angle = start_angle;
    float temp = (target_Angle - temp_angle) / (float)time;
    for(int i=0;i<time;i++)
    {
        temp_angle += temp;
//        if((start_angle<target_Angle)&&(temp_angle > target_Angle))
//            break;
//        else if((start_angle>target_Angle)&&(temp_angle < target_Angle))break;
        setAngle_270(&Servo_yaw, temp_angle);
        HAL_Delay(10);

    }
}

void setAngle_270(struct SG90_OOC *SG90, float target_Angle) {
    move2TargetAngle(SG90);
    SG90->prLastAngle = target_Angle;
    target_Angle += 62.3f;
    if(target_Angle <= 40)
        target_Angle = 40;
    if(target_Angle >= 95)
        target_Angle = 95;
    SG90->prTarget_Angle = target_Angle;
//    SG90->prHighLevelTimes_Ms = (5000.00f + (SG90->prTarget_Angle / 27.00f) * 200.00f);
    SG90->prHighLevelTimes_Ms = (40000.00f + (SG90->prTarget_Angle / 27.00f) * 8000.00f);
    move2TargetAngle(SG90);
}

void SG90_scan(void)     //左移为正，上移为正
{
    //原点为坐标原点，位置固定通过atan计算开环控制固定打角
    setAngle_270(&Servo_yaw, 14.036f); //-14.036度
    HAL_Delay(500);
    setAngle_180(&Servo_pitch, -14.036f+1.5f); //26.565度
    HAL_Delay(500);
    setAngle_270(&Servo_yaw, -14.036f-0.05f); //14.036度
    HAL_Delay(500);
    setAngle_180(&Servo_pitch, 14.036f+1.2f); //度
    HAL_Delay(500);
    setAngle_270(&Servo_yaw, 14.036f); //0度
    HAL_Delay(500);
    setAngle_180(&Servo_pitch, 0); //0度
    HAL_Delay(500);
    setAngle_270(&Servo_yaw, 0); //0度
    HAL_Delay(500);
}
void SG90_move_pitch(struct SG90_OOC *SG90, float start_pitch, float end_pitch, float start_yaw, int time)
{
    int i;
    float width;
    if(start_yaw >= 0)
    {
        width = (100 / cosf(fabsf(start_pitch))) * tanf(fabsf(start_yaw));

    }
    else
    {
        width = -(100 / cosf(fabsf(start_pitch))) * tanf(fabsf(start_yaw));
    }
    float temp = (end_pitch - start_pitch) / 100;
    for(i=0; i<time; i++)
    {
        start_pitch += temp;
        if(start_pitch > end_pitch)break;
        setAngle_180(&Servo_pitch, start_pitch);
        move2TargetAngle(&Servo_pitch);
        start_yaw = atanf(width / (100 / cosf(fabsf(start_pitch))));
        setAngle_270(&Servo_yaw, start_yaw);
        move2TargetAngle(&Servo_yaw);
        HAL_Delay(10);
    }
}

void SG90_move_yaw(struct SG90_OOC *SG90, float start_yaw, float end_yaw, float start_pitch, int time)
{
    int i;
    float high;
    if(start_pitch >= 0)
    {
        high = (100 / cosf(fabsf(start_yaw))) * tanf(fabsf(start_pitch));

    }
    else
    {
        high = -(100 / cosf(fabsf(start_yaw))) * tanf(fabsf(start_pitch));
    }
    float temp = (end_yaw - start_yaw) / 100;
    for(i=0; i<time; i++)
    {
        start_yaw += temp;
        if(start_yaw > end_yaw)break;
        setAngle_270(&Servo_yaw, start_yaw);
        move2TargetAngle(&Servo_yaw);
        start_pitch = atanf(high / (100 / cosf(fabsf(start_yaw))));
        setAngle_180(&Servo_pitch, start_pitch);
        move2TargetAngle(&Servo_pitch);
        HAL_Delay(10);
    }
}

void move2TargetAngle(struct SG90_OOC *SG90) {
            __HAL_TIM_SetCompare(SG90->TIMER, SG90->TIM_Channel, SG90->prHighLevelTimes_Ms);
}

SG90 SG90_Create(char* name, TIM_HandleTypeDef* TIMER, uint32_t Channel){
    SG90 sg90_temple;

    sg90_temple.name = name;
    sg90_temple.TIMER = TIMER;
    sg90_temple.TIM_Channel = Channel;

    //default arguments
    sg90_temple.prTarget_Angle = 0;
    sg90_temple.prHighLevelTimes_Ms = 5;

    //functions define
    sg90_temple.GetAngle2Pulse = getAngle2Pulse;
    sg90_temple.SetAngle_180 = setAngle_180;
    sg90_temple.SetAngle_270 = setAngle_270;
    sg90_temple.Move2TargetAngle = move2TargetAngle;

    return sg90_temple;
}

void SG90_init() {
//    Servo_pitch = SG90_Create("First", &htim15, TIM_CHANNEL_1);
//    Servo_yaw = SG90_Create("Second", &htim5, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(Servo_pitch.TIMER, Servo_pitch.TIM_Channel);
    HAL_TIM_PWM_Start(Servo_yaw.TIMER, Servo_yaw.TIM_Channel);
}

char *Click_Once(struct SG90_OOC *aSG90) {
    //click out two pills
    aSG90->SetAngle_180(aSG90, 180.0f);
    aSG90->Move2TargetAngle(aSG90);
    HAL_Delay(500);

    //return to default
    aSG90->SetAngle_180(aSG90, 0.0f);
    aSG90->Move2TargetAngle(aSG90);
    HAL_Delay(500);

    return aSG90->name;
}

/*the number must >= 2, because SG90 must reset, and reset will also click out one pill,
 * It is meaning we at least click two pill at once*/
char *Click_Num(struct SG90_OOC *aSG90, uint8_t Num) {
    for (uint8_t i = 1; i <= (Num / 2); i++) {
        //click out o pieces
        aSG90->SetAngle_180(aSG90, 180.0f);
        aSG90->Move2TargetAngle(aSG90);
        HAL_Delay(800);

        //return to default
        aSG90->SetAngle_180(aSG90, 0.0f);
        aSG90->Move2TargetAngle(aSG90);
        HAL_Delay(800);
    }

    return aSG90->name;
}