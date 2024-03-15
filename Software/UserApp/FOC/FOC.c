#include "FOC.h"
#include "main.h"

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float open_loop_timestamp = 0.0f;
float voltage_power_supply = 12.0f;
uint32_t cs1_zero_value[3];
uint32_t cs2_zero_value[3];

FOC_Motor FOCMotor_Left = {
        .name = "left leg",
        .api_writeDutyCyclePWM = _writeDutyCycle3PWM_1,
        .api_getMotorAngle = i2c_mt6701_get_angle,
        .api_getMotorCurrent = _currentGetValue,
        .PIDUint.speed = &Motor_Left_speed,
        .PIDUint.position = &Motor_Left_position,
        .PIDUint.Uq = &Motor_Left_Uq,
        .PIDUint.Ud = &Motor_Left_Ud,
        .FilterUint.speed = &lpf_Motor_Left_speed,
        .FilterUint.position = &lpf_Motor_Left_position,
        .FilterUint.current_d = &lpf_Motor_Left_Id,
        .FilterUint.current_q = &lpf_Motor_Left_Iq
};
FOC_Motor FOCMotor_Right = {
        .name = "right leg",
        .api_writeDutyCyclePWM = _writeDutyCycle3PWM_2,
        .api_getMotorAngle = i2c2_mt6701_get_angle,
        .api_getMotorCurrent = _currentGetValue2,
        .PIDUint.speed = &Motor_Right_speed,
        .PIDUint.position = &Motor_Right_position,
        .PIDUint.Uq = &Motor_Right_Uq,
        .PIDUint.Ud = &Motor_Right_Ud,
        .FilterUint.speed = &lpf_Motor_Right_speed,
        .FilterUint.position = &lpf_Motor_Right_position,
        .FilterUint.current_d = &lpf_Motor_Right_Id,
        .FilterUint.current_q = &lpf_Motor_Right_Iq
};

void FOC_Vbus(float _Vbus) {
    voltage_power_supply = _Vbus;

    _init3PWM();
    _initCurrentSample();
    _getCurrentZeroValue(cs1_zero_value, cs2_zero_value);
}

//电角度求解
float _electricalAngle(FOC_Motor *Motor, float _shaft_angle, int pole_pairs) {
    return _normalizeAngle(((float)(Motor->direct * pole_pairs) * _shaft_angle) - Motor->zero_electrical_angle);
}

//角度归一化
float _normalizeAngle(float angle) {
    float a = fmod(angle, 2*_PI);
    return a >= 0 ? a : (a + 2*_PI);
}

void FOC_alignSensor(FOC_Motor *Motor, int _PP, int _DIR) {
    Motor->PolePair = _PP;
    Motor->direct = _DIR;

    setTorque(Motor, 1, _3PI_2);  //起劲
    HAL_Delay(400);
    Motor->api_getMotorAngle(&Motor->angle_pi, &Motor->angle_f); //更新传感器数值
    Motor->zero_electrical_angle = _electricalAngle(Motor, Motor->angle_pi, Motor->PolePair);
    HAL_Delay(100);
    setTorque(Motor, 0, _3PI_2);  //松劲（解除校准）

    Motor->angle_pi = 0.0f;
    Motor->angle_f = 0.0f;
    Motor->PIDUint.speed->Error = 0;
}

void setPWM(FOC_Motor *Motor, float Ua, float Ub, float Uc) {
    // 限制上限
    Ua = _constrain(Ua, 0.0f, voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, voltage_power_supply);

    //计算占空比, 并限制其在0~1
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    //写入PWM通道
    Motor->api_writeDutyCyclePWM(dc_a, dc_b, dc_c);
}

void setTorque(FOC_Motor *Motor, float Uq, float angle_el) {
    Uq = _constrain(Uq, -(voltage_power_supply) / 3, (voltage_power_supply) / 3);   //力矩限制, 限定为上限电压的n分之一, 保证安全性

    angle_el = _normalizeAngle(angle_el);

    // 帕克逆变换
    float Ualpha = -Uq * (float) sin(angle_el);
    float Ubeta = Uq * (float) cos(angle_el);

    //克拉克逆变换
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (float) (_SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (float) (-Ualpha - _SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;

    setPWM(Motor, Ua, Ub, Uc);
}

//设置相电压
void setPhaseVoltage(FOC_Motor *Motor, float Uq, float Ud, float angle_elec) {
    angle_elec = _normalizeAngle(angle_elec);
    // 帕克逆变换
    float Ualpha = -Uq * sin(angle_elec);
    float Ubeta = Uq * cos(angle_elec);

    // 克拉克逆变换
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (_SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - _SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;

    setPWM(Motor, Ua, Ub, Uc);
}

//开环速度函数
float velocityOpenLoop(FOC_Motor *Motor, float target_velocity) {
    unsigned long now_us = HAL_GetTick();  //获取从开启芯片以来的微秒数，它的精度是 1ms

    //计算当前每个Loop的运行时间间隔
    float Ts = (now_us - open_loop_timestamp) * 3e-3f;

    // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 Motor->shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
    Motor->shaft_angle = _normalizeAngle(Motor->shaft_angle + target_velocity * Ts);
    //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
    //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

    // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
    // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
    float Uq = voltage_power_supply / 9.0f;

#if FOC_MODE == mode_SPWM
    setPhaseVoltage(Motor, Uq, 0, _electricalAngle(Motor, Motor->shaft_angle, Motor->PolePair));
#else
    FOC_SVPWM(Motor, Uq, 0, _electricalAngle(Motor, Motor->shaft_angle, Motor->PolePair));
#endif
    open_loop_timestamp = now_us;  //用于计算下一个时间间隔

    return Uq;
}

//================简易接口函数================
void FOC_setAngle(FOC_Motor *Motor, float Target) {
    Motor->api_getMotorAngle(&Motor->angle_pi, &Motor->angle_f);
    Motor->PIDUint.position->Target = _normalizeAngle(Target);;
    float angle_error = _normalizeAngle(Target - Motor->angle_pi * (float)Motor->direct);

    if(angle_error > M_PI){
        angle_error -= 2.0f * _PI;
    }

    Motor->Uq = _constrain( FOC_ANGLE_PID(Motor, angle_error),
                            Motor->PIDUint.position->OutputMin, Motor->PIDUint.position->OutputMax);
    Motor->electrical_angle = _electricalAngle(Motor, Motor->angle_pi, Motor->PolePair);

#if FOC_MODE == mode_SPWM
    setTorque(Motor, Motor->Uq, Motor->electrical_angle);   //角度闭环
#else
    FOC_SVPWM(Motor,  Motor->Uq,0, Motor->electrical_angle);
#endif
}

//不对劲
void FOC_setVelocityAngle(FOC_Motor *Motor, float TargetAngle, float TargetSpeed){
    Motor->api_getMotorAngle(&Motor->angle_pi, &Motor->angle_f);
    Motor->PIDUint.position->Target = _normalizeAngle(TargetAngle);
    float angle_error = _normalizeAngle(TargetAngle - Motor->angle_pi * (float)Motor->direct);

    if(angle_error > M_PI){
        angle_error -= 2.0f * _PI;
    }

    Motor->PIDUint.speed->Target = _constrain(FOC_ANGLE_PID(Motor, angle_error * 180.0f / _PI), -0.2, 0.2);

    FOC_getVelocity(Motor);
    Motor->Uq = _constrain( FOC_VEL_PID(Motor, Motor->speed),
                            Motor->PIDUint.speed->OutputMin, Motor->PIDUint.speed->OutputMax);
    Motor->electrical_angle = _electricalAngle(Motor, Motor->angle_pi, Motor->PolePair);

#if FOC_MODE == mode_SPWM
    setTorque(Motor, Motor->Uq, Motor->electrical_angle);   //角度闭环
#else
    FOC_SVPWM(Motor, Motor->Uq, 0, Motor->electrical_angle);
#endif
}

void FOC_setVelocity(FOC_Motor *Motor, float Target) {
    Motor->PIDUint.speed->Target = Target;
    FOC_getVelocity(Motor);
    Motor->api_getMotorAngle(&Motor->angle_pi, &Motor->angle_f);

    Motor->Uq = _constrain( FOC_VEL_PID(Motor, (Motor->PIDUint.speed->Target - Motor->PIDUint.speed->Actual)),
                            Motor->PIDUint.speed->OutputMin, Motor->PIDUint.speed->OutputMax);
    Motor->electrical_angle = _electricalAngle(Motor, Motor->angle_pi, Motor->PolePair);

    // Current sense
    Motor->api_getMotorCurrent(Motor->current);
    Motor->current[0] = Motor->current[0] * (float)Motor->direct;
//    Motor->current[0] = Motor->current[0];
    Motor->current[1] = Motor->current[1] * (float)Motor->direct;
//    Motor->current[1] = Motor->current[1];
    Motor->current[2] = Motor->current[2];
    FOC_Clarke_Park(Motor->current[0], Motor->current[1], Motor->current[2], Motor->electrical_angle, &Motor->Id, &Motor->Iq);

//    Motor->Iq = Low_Pass_Filter(Motor->FilterUint.current_q, Motor->Iq, 0.2f);

#if FOC_MODE == mode_SPWM
    setTorque(Motor, Motor->Uq, Motor->electrical_angle);
#else
    FOC_SVPWM(Motor, Motor->Uq,0, Motor->electrical_angle);   //速度闭环
#endif
}

void FOC_current_control_loop(FOC_Motor *Motor, float target_Iq){
    Motor->api_getMotorAngle(&Motor->angle_pi, &Motor->angle_f);
    Motor->electrical_angle = _electricalAngle(Motor, Motor->angle_pi, Motor->PolePair);
    // Current sense
    Motor->api_getMotorCurrent(Motor->current);
    Motor->current[0] = Motor->current[0];
    Motor->current[1] = Motor->current[1];
    Motor->current[2] = Motor->current[2];
    FOC_Clarke_Park(Motor->current[0], Motor->current[1], Motor->current[2], Motor->electrical_angle, &Motor->Id, &Motor->Iq);

    Motor->Iq = Low_Pass_Filter(Motor->FilterUint.current_q, Motor->Iq, 0.2f);
    Motor->Id = Low_Pass_Filter(Motor->FilterUint.current_d, Motor->Id, 0.2f);

    // back feed
    Motor->PIDUint.Uq->Target = target_Iq;
    Motor->PIDUint.Uq->Error = target_Iq - Motor->Iq;
    Motor->Uq = Position_Pid_Calculate(Motor->PIDUint.Uq);
    Motor->PIDUint.Ud->Target = 0;
    Motor->PIDUint.Ud->Error = -Motor->Id;
    Motor->Ud = Position_Pid_Calculate(Motor->PIDUint.Ud);

//     front feed
    Motor->Uq += 1.0f * Motor->Iq;

//    setTorque(Motor, Motor->Uq, Motor->electrical_angle);
    FOC_SVPWM(Motor, Motor->Uq, Motor->Ud, Motor->electrical_angle);
//    FOC_SVPWM(Motor, Motor->Uq, 0, Motor->electrical_angle);
}

//速度PID接口
float FOC_VEL_PID(FOC_Motor *Motor, float error)
{
    Motor->PIDUint.speed->Error = error;
    return Position_Pid_Calculate(Motor->PIDUint.speed);
}

//角度PID接口
float FOC_ANGLE_PID(FOC_Motor *Motor, float error) {
//    Motor->PIDUint.position->Error = Low_Pass_Filter(Motor->FilterUint.position, error, 0.4f);
    Motor->PIDUint.position->Error = error;
    return Position_Pid_Calculate(Motor->PIDUint.position);
}
static float Ts = 1;
float FOC_getVelocity(FOC_Motor *Motor) {
    Motor->api_getMotorAngle(&Motor->angle_pi, &Motor->angle_f);
    Motor->speedUint.angle_now = Motor->angle_pi;

    if(Motor->speedUint.angle_old == 0){
        Motor->speedUint.angle_old = Motor->speedUint.angle_now;
        return 0;
    }

    float delta_angle = (Motor->speedUint.angle_now - Motor->speedUint.angle_old);

//    if (delta_angle >= 1.6f * M_PI){
//        delta_angle -= 2.0f * (float)M_PI;
//    }
//    if (delta_angle <= -1.6f * M_PI){
//        delta_angle += 2.0f * (float)M_PI;
//    }

    if (fabs(delta_angle) > _PI) {
        if (delta_angle > 0) {
            delta_angle -= _2PI;
        } else {
            delta_angle += _2PI;
        }
    }

    float vel_speed_ori = Motor->direct * (delta_angle) / (float)Ts ;

    Motor->speedUint.angle_old = Motor->speedUint.angle_now;
    Motor->speedUint.full_rotations_old = Motor->speedUint.full_rotations;

    float vel_flit = Low_Pass_Filter(Motor->FilterUint.speed, vel_speed_ori, 0.3f);
    Motor->PIDUint.speed->Actual = vel_flit;
    Motor->speed = vel_flit;

    return vel_flit;
}

//current sample version
void FOC_Clarke_Park(float Ia, float Ib, float Ic, float angle, float *Id, float *Iq) {
    // Clarke transform
//    float mid = (1.0f / 3) * (Ia + Ib + Ic);
//    float a = Ia - mid;
//    float b = Ib - mid;
    float a = Ia;
    float b = Ib;
    float i_alpha = a;
    float i_beta = _1_SQRT3 * a + _2_SQRT3 * b;

    // Park transform
    float ct = cos(angle);
    float st = sin(angle);
    *Iq = i_beta * ct - i_alpha * st;
    *Id = i_alpha * ct + i_beta * st;
}

void FOC_SVPWM(FOC_Motor *Motor, float Uq, float Ud, float angle) {
    int sector;
    // Nice video explaining the SpaceVectorModulation (FOC_SVPWM) algorithm
    // https://www.youtube.com/watch?v=QMSWUMEAejg

    float Uout = sqrt(Ud * Ud + Uq * Uq) / VOLTAGE_LIMIT; // Actually, Uout is a ratio
    angle = _normalizeAngle(angle + atan2(Uq, Ud));

    // find the sector we are in currently
    sector = floor(angle / _PI_3) + 1;
    // calculate the duty cycles
    float T1 = _SQRT3 * sin(sector * _PI_3 - angle) * Uout;
    float T2 = _SQRT3 * sin(angle - (sector - 1.0f) * _PI_3) * Uout;
    // two versions possible
//    float T0 = 0; // pulled to 0 - better for low power supply voltage
    float T0 = 1 - T1 - T2; // modulation_centered around driver->voltage_limit/2

    // calculate the duty cycles(times)
    float Ta, Tb, Tc;
    switch (sector) {
        case 1:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            // possible error state
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }

    // Ta, Tb, Tc range [0,1]
    Motor->api_writeDutyCyclePWM(Ta, Tb, Tc);
}
