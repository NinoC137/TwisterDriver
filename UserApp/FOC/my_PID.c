#include "my_PID.h"

PID Motor_1;

PID Motor_Uq;
PID Motor_Ud;

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
    Pid_Init(&Motor_1, 0.06f, 0.0005f, 0.00f);
    Pid_Init(&Motor_Uq, 0.004f, 0.00f, 0.00f);
    Motor_Uq.OutputMax = 3.0f;
    Motor_Uq.OutputMin = -3.0f;
    Motor_Uq.IntegralMax = 200.0f;
    Pid_Init(&Motor_Ud, 0.004f, 0.00f, 0.00f);
    Motor_Ud.OutputMax = 3.0f;
    Motor_Ud.OutputMin = -3.0f;
    Motor_Ud.IntegralMax = 200.0f;
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
    param->Output = (float)(param->Kp * param->Proportion + param->Ki * param->Integral +param->Kd * param->Differential);
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




