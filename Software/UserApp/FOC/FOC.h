#ifndef _FOC_H__
#define _FOC_H__

enum Mode{
    SPWM = 0,
    SVPWM = 1
};
//FOC模式设定   ---     SVPWM未测试
#define FOC_MODE 0

#include "main.h"

#include "MT6701.h"
#include "hardware_api.h"

#include "FOC_math.h"
#include "my_PID.h"
#include "Filter.h"

#define VOLTAGE_LIMIT (voltage_power_supply / 3)

//传感器读取
float FOC_M0_Velocity();
float FOC_M0_Angle();
//PID
void FOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp);
void FOC_M0_SET_VEL_PID(float P, float I, float D, float ramp);
float FOC_M0_VEL_PID(float error);
float FOC_M0_ANGLE_PID(float error);

//电角度求解
float _electricalAngle(float shaft_angle, int pole_pairs);
//角度归一化
float _normalizeAngle(float angle);
//输出PWM
void setPWM(float Ua, float Ub, float Uc);
//设置相电压
void setPhaseVoltage(float Uq, float Ud, float angle_elec);
/**
 * SVPWM计算式
 * 求解得出PWM输出, 内部通过api设定PWM输出
 * @param Uq    力矩电压
 * @param Ud    励磁电压    在不使用电流环控制时可设定为0
 * @param angle 当前电角度
 */
void FOC_SVPWM(float Uq, float Ud, float angle);
/**
 * 电流环克拉克-帕克变换
 * 用于求解Id, Iq
 * @param Ia    电流传感器采样得到的a相电流值
 * @param Ib    电流传感器采样得到的b相电流值
 * @param Ic    电流传感器采样得到的c相电流值
 * @param angle 当前电角度
 * @result Id    求解得出的Id值
 * @result Iq    求解得出的Iq值
 */
void FOC_Clarke_Park(float Ia, float Ib, float Ic, float angle, float *Id, float *Iq);

//开环速度接口函数
float velocityOpenLoop(float target_velocity);

//闭环部分
void setTorque(float Uq, float angle_el);
void FOC_Vbus(float _Vbus);
void FOC_alignSensor(int _PP, int _DIR);

//闭环控制接口函数
void FOC_M0_set_Velocity_Angle(float Target);
void FOC_M0_setVelocity(float Target);
void FOC_M0_set_Force_Angle(float Target);
void FOC_M0_setTorque(float Target);

#endif