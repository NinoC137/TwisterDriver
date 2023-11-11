#include "Filter.h"

kalman1_filter_t kalman_filter;
Sliding_Mean_Filter moving_filter;

LowPass_Filter lpf_Motor1_error;
LowPass_Filter lpf_Motor1_speed;

LowPass_Filter lpf_Motor1_current;
LowPass_Filter lpf_current_d;
LowPass_Filter lpf_current_q;

/**
 ** 函数功能:  一阶滞后滤波算法，Bias滤波器
 ** 参    数: value：需要进行滤波的值
 ** 返 回 值: 滤波后的值
 */
float FirstOrderLagFilter(float value) {
    static float last_value;
    value = FIRST_LAG_P * value + (1.0f - FIRST_LAG_P) * last_value; //一阶滞后滤波
    last_value = value;
    return value;
}

/**
 ** 函数功能:  一阶滞后滤波算法，Slope滤波器
 ** 参    数: value：需要进行滤波的值
 ** 返 回 值: 滤波后的值
*/
float FirstOrderLagFilter_Slope(float value) {
    static float last_value;
    value = FIRST_LAG_P * value + (1.0f - FIRST_LAG_P) * last_value; //一阶滞后滤波
    last_value = value;
    return value;
}

/**
 *  函数功能:  二阶滞后滤波算法
 ** 参    数: value：需要进行滤波的值
 ** 返 回 值: 滤波后的值
 */
float SecondOrderLagFilter(float value) {
    static float last_value, last_2_value;
    value = 0.2f * value + 0.4f * last_value + 0.4f * last_2_value;     //二阶滞后滤波
    last_2_value = last_value;
    last_value = value;
    return value;
}

/**
 *  函数功能:  滑动平均滤波算法
 ** 参    数: value：需要进行滤波的值
 ** 返 回 值: 滤波后的值
 */
float movingAverageFilter(Sliding_Mean_Filter *filter, float value) {
    filter->sum -= filter->Filter_Buffer[filter->index];        //减去最旧的数
    filter->sum += value;                                       //加进最新的数
    filter->Filter_Buffer[filter->index] = value;               //将最新的数覆盖最旧的数
    filter->average = filter->sum / MVF_BUFFER;                 //求均值
    if (++filter->index == MVF_BUFFER)
        filter->index = 0;
    return filter->average;
}

/**
 * 函数功能:    一阶卡尔曼滤波初始化
 * 参    数:   q,r: 预测噪声方差, 测量噪声方差
 * 返 回 值:   state : 滤波结构数据指针
 */
void kalman1_init(kalman1_filter_t *state, float q, float r) {
    state->x = 0;
    state->p = 0.0f;
    state->A = 1.0f;
    state->H = 1.0f;
    state->q = q;
    state->r = r;
}

/**
 * 函数功能:    一阶卡尔曼滤波
 * 参    数:   z_measure : 原始数据
 * 返 回 值:   state : 滤波结构数据指针
 */
float kalman1_filter(kalman1_filter_t *state, float z_measure) {
    /* Predict */
    // 时间更新(预测): X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
    state->x = state->A * state->x;
    // 更新先验协方差: P(k|k-1) = A(k,k-1)*A(k,k-1)^T*P(k-1|k-1)+Q(k)
    state->p = state->A * state->A * state->p + state->q;

    /* Measurement */
    // 计算卡尔曼增益: K(k) = P(k|k-1)*H(k)^T/(P(k|k-1)*H(k)*H(k)^T + R(k))
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    // 测量更新(校正): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    // 更新后验协方差: P(k|k) =（I-K(k)*H(k))*P(k|k-1)
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

/*********************************一阶低通滤波***********************************************************/
float Low_Pass_Filter(LowPass_Filter *filter,float data,float param)
{
    filter->parameter=param;
    filter->sample_data=data;
    filter->output=filter->parameter*filter->sample_data+(1.0-filter->parameter)*filter->output_last;
    filter->output_last=filter->output;
    return filter->output;
}

/*********************************窗口滑动滤波***********************************************************/
float Window_Slide_Filter(Slide_Filter *filter,float data)
{
    filter->slide_count++;
    filter->slide_temp1 = data;
    filter->slide_sum = filter->slide_temp1 + filter->slide_temp2 + filter->slide_temp3 + filter->slide_temp4 + filter->slide_temp5;
    if(filter->slide_count > 5)filter->slide_count--;
    filter->slide_temp5 = filter->slide_temp4;
    filter->slide_temp4 = filter->slide_temp3;
    filter->slide_temp3 = filter->slide_temp2;
    filter->slide_temp2 = filter->slide_temp1;
    return filter->slide_sum/filter->slide_count;
}

