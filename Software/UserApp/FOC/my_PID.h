#ifndef PID_H__
#define PID_H__

typedef struct
{
    float   Kp;                        //PID系数kp
    float   Ki;                        //PID系数ki
    float   Kd;                        //PID系数kd

    float   Proportion;                //比例项
    float   Integral;                  //积分项
    float   Differential;              //微分项
    float   Differential_Last;         //上一个微分项
    float   IntegralMax;               //积分项所能累加到的极值

    float   Error;                     //当前误差
    float   Error_Last;                //上次误差
    float   Error_Prev;                //上上次误差

    float   Target;              //期望值
    float   Actual;              //实际值

    float   Kp_max;                    //变结构pi中最大Kp
    float   Kp_min;                    //变结构pi中最小Kp
    float   Ki_max;                    //变结构pi中最大Ki
    float   Ki_min;                    //变结构pi中最小Ki

    float   Output;                    //PID的输出
    float   OutputLast;                //PID的上一次输出
    float   OutputMax;                 //PID输出的最大值
    float   OutputMin;                 //PID输出的最小值

}PID;

extern PID Motor_1;
extern PID Motor_Uq;
extern PID Motor_Ud;

void Pid_Value_Init(void);
void Pid_Init(PID *param,float Kp_Init,float Ki_Init,float Kd_Init);
float Position_Pid_Calculate(PID *param);
float Incremental_Pid_Calculate(PID *param);

#endif



