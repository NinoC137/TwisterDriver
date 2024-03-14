#include "Filter.h"

kalman1_filter_t kalman_filter;

LowPass_Filter lpf_Motor_Left_position;
LowPass_Filter lpf_Motor_Left_speed;
LowPass_Filter lpf_Motor_Left_Iq;
LowPass_Filter lpf_Motor_Left_Id;

LowPass_Filter lpf_Motor_Right_position;
LowPass_Filter lpf_Motor_Right_speed;
LowPass_Filter lpf_Motor_Right_Iq;
LowPass_Filter lpf_Motor_Right_Id;

Slide_Filter wsf_Motor_Left_speed;
Slide_Filter wsf_Motor_Right_speed;

Sliding_Mean_Filter moving_filter_cs1ZeroValue;
Sliding_Mean_Filter moving_filter_cs2ZeroValue;

float FirstOrderLagFilter(float value) {
    static float last_value;
    value = FIRST_LAG_P * value + (1.0f - FIRST_LAG_P) * last_value;
    last_value = value;
    return value;
}

float FirstOrderLagFilter_Slope(float value) {
    static float last_value;
    value = FIRST_LAG_P * value + (1.0f - FIRST_LAG_P) * last_value;
    last_value = value;
    return value;
}

float SecondOrderLagFilter(float value) {
    static float last_value, last_2_value;
    value = 0.2f * value + 0.4f * last_value + 0.4f * last_2_value;     //�����ͺ��˲�
    last_2_value = last_value;
    last_value = value;
    return value;
}

float movingAverageFilter(Sliding_Mean_Filter *filter, float value) {
    filter->sum -= filter->Filter_Buffer[filter->index];        //��ȥ��ɵ���
    filter->sum += value;                                       //�ӽ����µ���
    filter->Filter_Buffer[filter->index] = value;               //�����µ���������ɵ���
    filter->average = filter->sum / MVF_BUFFER;                 //���ֵ
    if (++filter->index == MVF_BUFFER)
        filter->index = 0;
    return filter->average;
}

void kalman1_init(kalman1_filter_t *state, float q, float r) {
    state->x = 0;
    state->p = 0.0f;
    state->A = 1.0f;
    state->H = 1.0f;
    state->q = q;
    state->r = r;
}

float kalman1_filter(kalman1_filter_t *state, float z_measure) {
    /* Predict */
    // ʱ�����(Ԥ��): X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
    state->x = state->A * state->x;
    // ��������Э����: P(k|k-1) = A(k,k-1)*A(k,k-1)^T*P(k-1|k-1)+Q(k)
    state->p = state->A * state->A * state->p + state->q;

    /* Measurement */
    // ���㿨��������: K(k) = P(k|k-1)*H(k)^T/(P(k|k-1)*H(k)*H(k)^T + R(k))
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    // ��������(У��): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    // ���º���Э����: P(k|k) =��I-K(k)*H(k))*P(k|k-1)
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}

float Low_Pass_Filter(LowPass_Filter *filter,float data,float param)
{
    filter->parameter=param;
    filter->sample_data=data;
    filter->output=filter->parameter*filter->sample_data+(1.0-filter->parameter)*filter->output_last;
    filter->output_last=filter->output;
    return filter->output;
}

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

