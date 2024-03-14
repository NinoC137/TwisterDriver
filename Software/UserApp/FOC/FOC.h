#ifndef _FOC_H__
#define _FOC_H__

//FOC模式设定
#define mode_SPWM 0
#define mode_SVPWM 1
#define FOC_MODE mode_SVPWM

#define FOC_CONTROL_MODE_NUM 10

#define VOLTAGE_LIMIT 6

#include "MT6701.h"
#include "hardware_api.h"

#include "FOC_math.h"
#include "my_PID.h"
#include "Filter.h"

typedef struct{
    float angle_now;
    float angle_old;
    int full_rotations;
    int full_rotations_old;
    long timeStamp;
    long timeStamp_old;
}FOC_calculateUint;

typedef struct{
    PID *position;
    PID *speed;
    PID *Uq;
    PID *Ud;
}FOC_PIDUint;

typedef struct{
    LowPass_Filter *position;
    LowPass_Filter *speed;
    LowPass_Filter *current_d;
    LowPass_Filter *current_q;
}FOC_Filter;

typedef struct{
    char* name;

    float Uq;
    float Ud;
    float current[3];
    float Iq;
    float Id;

    FOC_calculateUint speedUint;
    FOC_PIDUint PIDUint;
    FOC_Filter FilterUint;

    int PolePair;
    int direct;

    float speed;

    float angle_pi;
    float angle_f;
    float electrical_angle;
    float zero_electrical_angle;
    float shaft_angle;

    void (*api_writeDutyCyclePWM)(float dutyCycle_a, float dutyCycle_b, float dutyCycle_c);
    void (*api_getMotorAngle)(float *angle_Pi, float *angle_f);
    void (*api_getMotorCurrent)(float *currentArray);
}FOC_Motor;

typedef enum{
    OPEN_LOOP_POSITION_CONTROL = 0,
    OPEN_LOOP_SPEED_CONTROL,
    TORQUE_CONTROL,
    SPEED_CONTROL,
    POSITION_CONTROL,
    SPRING,
    SPRING_WITH_DAMP,
    DAMP,
    KNOB,
    ZERO_RESISTANCE
} FOC_CONTROL_MODE;

extern FOC_Motor FOCMotor_Left;
extern FOC_Motor FOCMotor_Right;

//传感器读取
float FOC_getVelocity(FOC_Motor *Motor);
//PID
float FOC_VEL_PID(FOC_Motor *Motor, float error);
float FOC_ANGLE_PID(FOC_Motor *Motor, float error);

//电角度求解
float _electricalAngle(FOC_Motor *Motor, float shaft_angle, int pole_pairs);
//角度归一化
float _normalizeAngle(float angle);
//输出PWM
void setPWM(FOC_Motor *Motor, float Ua, float Ub, float Uc);
//设置相电压
void setPhaseVoltage(FOC_Motor *Motor, float Uq, float Ud, float angle_elec);
/**
 * SVPWM计算式
 * 求解得出PWM输出, 内部通过api设定PWM输出
 * @param Uq    力矩电压
 * @param Ud    励磁电压    在不使用电流环控制时可设定为0
 * @param angle 当前电角度
 */
void FOC_SVPWM(FOC_Motor *Motor, float Uq, float Ud, float angle);
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
float velocityOpenLoop(FOC_Motor *Motor, float target_velocity);

//闭环部分
void setTorque(FOC_Motor *Motor, float Uq, float angle_el);
void FOC_Vbus(float _Vbus);
void FOC_alignSensor(FOC_Motor *Motor, int _PP, int _DIR);

//闭环控制接口函数
void FOC_setAngle(FOC_Motor *Motor, float Target);
void FOC_setVelocityAngle(FOC_Motor *Motor, float TargetAngle, float TargetSpeed);
void FOC_setVelocity(FOC_Motor *Motor, float Target);
void FOC_current_control_loop(FOC_Motor *Motor, float target_Iq);

#endif