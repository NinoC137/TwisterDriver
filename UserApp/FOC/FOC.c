#include "FOC.h"
#include "cmsis_os.h"

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float shaft_angle = 0.0f, open_loop_timestamp = 0.0f;
float zero_electric_angle = 0.0f;
float voltage_power_supply = 12.0f;

int PP, DIR;

//传感器数值
float angle_pi;
float angle_f;

//M0速度PID接口
float FOC_M0_VEL_PID(float error)   //M0速度环
{
//    Motor_1.Error = error;
    Motor_1.Error = Low_Pass_Filter(&lpf_Motor1_error, error, 0.7f);
    return Position_Pid_Calculate(&Motor_1);
}

//M0角度PID接口
float FOC_M0_ANGLE_PID(float error) {
    Motor_1.Error = Low_Pass_Filter(&lpf_Motor1_error, error, 0.7f);
    return Position_Pid_Calculate(&Motor_1);
}
//=====================================================================================

void FOC_Vbus(float _Vbus) {
    voltage_power_supply = _Vbus;

    _init3PWM();
    _initCurrentSample();

//    struct LowPassFilter filter= {.Tf=0.01,.y_prev=0.0f}; //Tf=10ms
//    struct PIDController pid_controller = {.P=0.5,.I=0.1,.D=0.0,.output_ramp=100.0,.limit=6,.error_prev=0,.output_prev=0,.integral_prev=0};

}

void FOC_alignSensor(int _PP, int _DIR) {
    PP = _PP;
    DIR = _DIR;

    setTorque(3, _3PI_2);  //起劲
    HAL_Delay(1000);
    i2c_mt6701_get_angle(&angle_pi, &angle_f); //更新传感器数值
    zero_electric_angle = _electricalAngle(angle_pi, (int) PP);
    setTorque(0, _3PI_2);  //松劲（解除校准）

    uart_printf("zero electric angle: %f\r\n", zero_electric_angle);
}

//电角度求解
float _electricalAngle(float _shaft_angle, int pole_pairs) {
//    return (shaft_angle * pole_pairs);
    return _normalizeAngle(((float)(DIR * pole_pairs)*_shaft_angle)-zero_electric_angle);
}

//角度归一化
float _normalizeAngle(float angle) {
    float a = fmod(angle, 2*_PI);
    return a >= 0 ? a : (a + 2*_PI);
}

//输出PWM
void setPWM(float Ua, float Ub, float Uc) {
    // 限制上限
    Ua = _constrain(Ua, 0.0f, voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, voltage_power_supply);

    //计算占空比, 并限制其在0~1
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

    //写入PWM通道
    _writeDutyCycle3PWM(dc_a, dc_b, dc_c);
}

void setTorque(float Uq, float angle_el) {
    Uq = _constrain(Uq, -(voltage_power_supply) / 3, (voltage_power_supply) / 3);   //力矩限制, 限定为上限电压的五分之一, 保证安全性

    angle_el = _normalizeAngle(angle_el);

    // 帕克逆变换
    float Ualpha = -Uq * (float) _sin(angle_el);
    float Ubeta = Uq * (float) _cos(angle_el);

    //克拉克逆变换
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (float) (_SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (float) (-Ualpha - _SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;

    setPWM(Ua, Ub, Uc);
}

void FOC_SVPWM(float Uq, float Ud, float angle) {

    int sector;

    // Nice video explaining the SpaceVectorModulation (FOC_SVPWM) algorithm
    // https://www.youtube.com/watch?v=QMSWUMEAejg

    // the algorithm goes
    // 1) Ualpha, Ubeta
    // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
    // 3) angle_el = atan2(Ubeta, Ualpha)
    //
    // equivalent to 2)  because the magnitude does not change is:
    // Uout = sqrt(Ud^2 + Uq^2)
    // equivalent to 3) is
    // angle_el = angle_el + atan2(Uq,Ud)

    float Uout = _sqrt(Ud * Ud + Uq * Uq) / VOLTAGE_LIMIT; // Actually, Uout is a ratio
    angle = _normalizeAngle(angle + atan2(Uq, Ud));

    // find the sector we are in currently
    sector = floor(angle / _PI_3) + 1;
    // calculate the duty cycles
    float T1 = _SQRT3 * _sin(sector * _PI_3 - angle) * Uout;
    float T2 = _SQRT3 * _sin(angle - (sector - 1.0f) * _PI_3) * Uout;
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
    _writeDutyCycle3PWM(Ta, Tb, Tc);
}

//current sample version
void FOC_Clarke_Park(float Ia, float Ib, float Ic, float angle, float *Id, float *Iq) {
    // Clarke transform
    float mid = (1.f / 3) * (Ia + Ib + Ic);
    float a = Ia - mid;
    float b = Ib - mid;
    float i_alpha = a;
    float i_beta = _1_SQRT3 * a + _2_SQRT3 * b;

    // Park transform
    float ct = _cos(angle);
    float st = _sin(angle);
    *Id = i_alpha * ct + i_beta * st;
    *Iq = i_beta * ct - i_alpha * st;

    return;
}

//设置相电压
void setPhaseVoltage(float Uq, float Ud, float angle_elec) {
    angle_elec = _normalizeAngle(angle_elec);
    // 帕克逆变换
    float Ualpha = -Uq * _sin(angle_elec);
    float Ubeta = Uq * _cos(angle_elec);

    // 克拉克逆变换
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (_SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - _SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;

    setPWM(Ua, Ub, Uc);
}

//开环速度函数
float velocityOpenLoop(float target_velocity) {
    unsigned long now_us = HAL_GetTick();  //获取从开启芯片以来的微秒数，它的精度是 1ms

    //计算当前每个Loop的运行时间间隔
    float Ts = (now_us - open_loop_timestamp) * 1e-3f;

    // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
    //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

    // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
    // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
    float Uq = voltage_power_supply / 8.0f;

#if FOC_MODE == SPWM
    setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 7));
#else
    FOC_SVPWM(Uq, 0, _electricalAngle(shaft_angle, 7));
#endif
    open_loop_timestamp = now_us;  //用于计算下一个时间间隔

    return Uq;
}

//================简易接口函数================
void FOC_M0_set_Velocity_Angle(float Target) {
    i2c_mt6701_get_angle(&angle_pi, &angle_f);
    float angle_error = Target - angle_pi * DIR;

    angle_error = _normalizeAngle(angle_error);
    if(angle_error > M_PI){
        angle_error -= M_PI_2;
    }
#if FOC_MODE == SPWM
    setTorque(_constrain( FOC_M0_ANGLE_PID(angle_error) * 180.0f / _PI , Motor_1.OutputMin, Motor_1.OutputMax),
    _electricalAngle(angle_pi, PP));   //角度闭环
#else
    FOC_SVPWM(_constrain( FOC_M0_ANGLE_PID(angle_error) * 180.0f / PI , Motor_1.OutputMin, Motor_1.OutputMax), 0,
              _electricalAngle(angle_pi, PP));
#endif
}

void FOC_M0_setVelocity(float Target) {
    Motor_1.Target = Target;
    setTorque(FOC_M0_VEL_PID((Target - FOC_M0_Velocity()) * 180 / _PI), _electricalAngle(angle_pi, PP));   //速度闭环
}

void FOC_M0_set_Force_Angle(float Target)   //力位
{
    setTorque(FOC_M0_ANGLE_PID((Target - FOC_M0_Angle()) * 180 / _PI), _electricalAngle(angle_pi, PP));
}

void FOC_M0_setTorque(float Target) {
    setTorque(Target, _electricalAngle(Motor_1.Error, PP));
}

void FOC_current_control_loop(float target_Iq){
    static float electrical_angle;
    i2c_mt6701_get_angle(&angle_pi, &angle_f);
    electrical_angle = _electricalAngle(angle_pi, PP);
    // Current sense
    static float Id, Iq;
    static float cs_current[3];
    _currentGetValue(cs_current);
    FOC_Clarke_Park(cs_current[0], cs_current[1], cs_current[2], electrical_angle, &Id, &Iq);
    Id = Low_Pass_Filter(&lpf_current_d, Id, 0.6f);
    Iq = Low_Pass_Filter(&lpf_current_q, Iq, 0.6f);

    // back feed
    static float Uq, Ud;
    Motor_Uq.Error = target_Iq - Iq;
    Uq = Position_Pid_Calculate(&Motor_Uq);
    Motor_Ud.Error = -Id;
    Ud = Position_Pid_Calculate(&Motor_Ud);

    // front feed
    Uq += 0.008 * Iq;

    FOC_SVPWM(Uq, Ud, electrical_angle);

    // debug
//    printf("%.1f,%.1f,%.1f,%.1f,%.1f\n", cs_value[0], cs_value[1], cs_value[2], Id, Iq);
}

float FOC_M0_Velocity() {
    static float angle_now, angle_old;
    static long sampleTimeStamp;
    i2c_mt6701_get_angle(&angle_pi, &angle_f); //更新传感器数值
    sampleTimeStamp = osKernelSysTick();
    angle_now = angle_pi;

    float delta_angle = angle_now - angle_old;
    if (delta_angle >= 1.6 * M_PI) {
        delta_angle -= 2.0f * M_PI;
    }
    if (delta_angle <= -1.6 * M_PI) {
        delta_angle += 2.0f * M_PI;
    }

    float vel_speed_ori = delta_angle / 3e-3;  //采样时间以3ms为单位, 乘1e3后为每秒的角速度

    angle_old = angle_now;

    float vel_M0_flit = Low_Pass_Filter(&lpf_Motor1_speed, DIR * vel_speed_ori, 0.7f);
//    Motor_1.Actual = vel_M0_flit;

    return vel_M0_flit;
}

float FOC_M0_Angle() {
    i2c_mt6701_get_angle(&angle_pi, &angle_f);
    return DIR * angle_pi;
}