#include "my_PID.h"

PID Motor_Left_speed;
PID Motor_Left_position;
PID Motor_Left_Uq;
PID Motor_Left_Ud;

PID Motor_Right_speed;
PID Motor_Right_position;
PID Motor_Right_Uq;
PID Motor_Right_Ud;

/*************************************************************************
*  函数名称：Pid_Value_Init()
*  功能说明：直接给PID系数赋值，用于软件保存PID参数
*  参数说明：无
*  函数返回：无
*  修改时间：2022年3月14日
*  备    注：在main中调用（初始化），在调好PID系数后把参数值保存在此函数中
*************************************************************************/
void Pid_Value_Init(void)
{
    Pid_Init(&Motor_Left_position, 2.0f, 0.00f, 0.0f);
    Motor_Left_position.IntegralMax = 10.0f;
    Pid_Init(&Motor_Left_speed, 0.08f, 0.008f, 0.0f);
    Motor_Left_speed.IntegralMax = 6.0f;

    Pid_Init(&Motor_Right_position, 2.0f, 0.00f, 0.0f);
    Motor_Right_position.IntegralMax = 3.0f;
    Pid_Init(&Motor_Right_speed, 0.08f, 0.01f, 0.0f);
    Motor_Right_speed.IntegralMax = 6.0f;

    Pid_Init(&Motor_Left_Uq, 0.8f, 1.5f, 1.0f);
    Motor_Left_Uq.OutputMax = 6.0f;
    Motor_Left_Uq.OutputMin = -6.0f;
    Motor_Left_Uq.IntegralMax = 0.04f;
    Pid_Init(&Motor_Left_Ud, 0.8f, 1.5f, 1.0f);
    Motor_Left_Ud.OutputMax = 6.0f;
    Motor_Left_Ud.OutputMin = -6.0f;
    Motor_Left_Ud.IntegralMax = 0.04f;
    Pid_Init(&Motor_Right_Uq, 1.2f, 10.0f, 0.0f);
    Motor_Right_Uq.OutputMax = 6.0f;
    Motor_Right_Uq.OutputMin = -6.0f;
//    Motor_Right_Uq.IntegralMax = 0.04f;
    Pid_Init(&Motor_Right_Ud, 1.2f, 10.0f, 0.0f);
    Motor_Right_Ud.OutputMax = 6.0f;
    Motor_Right_Ud.OutputMin = -6.0f;
//    Motor_Right_Ud.IntegralMax = 0.04f;
}

/*************************************************************************
*  函数名称：Pid_Init()
*  功能说明：给PID系数赋值，用于软件保存PID参数
*  参数说明：PID相关参数的结构体指针(param)，调好的PID系数
*  函数返回：无
*  修改时间：2022年3月14日
*  备    注：在Pid_Value_Init函数中调用，在调好PID系数后把参数值保存在此函数中
*************************************************************************/
void Pid_Init(PID *param,float Kp_Init,float Ki_Init,float Kd_Init)
{
    param->Kp = Kp_Init;
    param->Ki = Ki_Init;
    param->Kd = Kd_Init;
}

/*************************************************************************
*  函数名称：Position_Pid_Calculate()
*  功能说明：位置式PID算法
*  参数说明：位置式PID相关参数的结构体指针
*  函数返回：位置式PID计算出的结果
*  修改时间：2022年7月15日
*  备    注：积分项加入抗积分饱和与梯形积分，微分项可加入一阶低通滤波
*************************************************************************/
float Position_Pid_Calculate(PID *param)
{
//    param->Error = param->Target - param->Actual;
    /*位置式PID比例项的计算*/
    param->Proportion = param->Error;
    /*位置式PID积分项的计算*/
    param->Integral+=param->Error;
    /*抗积分饱和*/
    if(param->Integral>0 && param->Integral>param->IntegralMax)
        param->Integral = param->IntegralMax;
    if(param->Integral<0 && param->Integral<-param->IntegralMax)
        param->Integral = -param->IntegralMax;
    /*位置式PID微分项的计算*/
    param->Differential = param->Error - param->Error_Last;
    /*位置式PID的输出*/
    param->Output = (float)(param->Kp * param->Proportion + param->Ki * param->Integral + param->Kd * param->Differential);
    if(param->OutputMin==0)param->OutputMin=-param->OutputMax;
    /*对位置式PID的输出进行限幅*/
    if(param->OutputMax!=0&&param->OutputMin!=0)
    {
        if(param->Output>param->OutputMax)param->Output=param->OutputMax;
        if(param->Output<param->OutputMin)param->Output=param->OutputMin;
    }
    /*迭代上次的误差*/
    param->Error_Last = param->Error;
    return param->Output;
}

/*************************************************************************
*  函数名称：Incremental_Pid_Calculate()
*  功能说明：增量式PID算法
*  参数说明：增量式PID相关参数的结构体指针
*  函数返回：增量式PID计算出的结果
*  修改时间：2022年3月15日
*  备    注：积分项加入抗积分饱和与梯形积分，微分项可加入一阶低通滤波
*************************************************************************/
float Incremental_Pid_Calculate(PID *param)
{
    param->Error = param->Target - param->Actual;

    /*增量式PID比例项的计算*/
    param->Proportion = param->Error - param->Error_Last;

    /*增量式PID积分项的计算*/
    param->Integral=param->Error;

    /*增量式PID微分项的计算*/
    param->Differential = param->Error - 2*param->Error_Last + param->Error_Prev;
    /*增量式PID的输出*/
    param->Output += (float)(param->Kp * param->Proportion + param->Ki * param->Integral + param->Kd * param->Differential);

    //对增量式PID的输出进行限幅
    if(param->OutputMax!=0&&param->OutputMin!=0)
    {
        if(param->Output>param->OutputMax)param->Output=param->OutputMax;
        if(param->Output<param->OutputMin)param->Output=param->OutputMin;
    }
    /*迭代上次的误差和上上次的误差*/
    param->Error_Prev = param->Error_Last;
    param->Error_Last = param->Error;

    return param->Output;
}




