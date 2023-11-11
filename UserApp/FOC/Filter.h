#ifndef CODE_FILTER_H_
#define CODE_FILTER_H_

#define FIRST_LAG_P 0.8f        //һ���ͺ��˲�ϵ����ԽС�ͺ�Ч��Խǿ
#define MVF_BUFFER 8

typedef struct
{
    float Filter_Buffer[MVF_BUFFER];
    int index;
    float sum;
    float average;
}Sliding_Mean_Filter;

typedef struct {
    float x;        // state
    float A;        // x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
    float H;        // z(n)=H*x(n)+w(n),w(n)~N(0,r)
    float q;        // process(predict) noise convariance
    float r;        // measure noise(error) convariance
    float p;        // estimated error convariance
    float gain;     // kalman gain
} kalman1_filter_t;

extern Sliding_Mean_Filter moving_filter;  //���廬���˲��ṹ��
extern kalman1_filter_t kalman_filter;       //���忨�����ṹ��

float FirstOrderLagFilter(float value);         //һ���ͺ��˲��㷨
float FirstOrderLagFilter_Slope(float value);   //һ��slope�ͺ��˲��㷨
float SecondOrderLagFilter(float value);        //�����ͺ��˲��㷨
float movingAverageFilter(Sliding_Mean_Filter *filter, float value);

void kalman1_init(kalman1_filter_t* state, float q, float r);   //��ʼ���������˲���
float kalman1_filter(kalman1_filter_t* state, float z_measure); //�������˲��㷨

typedef struct
{
    float parameter;
    float sample_data;
    float output;
    float output_last;
    float filtered_data;
    unsigned char first_filter_flag;
}LowPass_Filter;

extern LowPass_Filter lpf_Motor1_error;
extern LowPass_Filter lpf_Motor1_speed;

extern LowPass_Filter lpf_Motor1_current;
extern LowPass_Filter lpf_current_d;
extern LowPass_Filter lpf_current_q;

float Low_Pass_Filter(LowPass_Filter *filter,float data,float param);

typedef struct
{
    short int slide_count;
    float slide_temp1;
    float slide_temp2;
    float slide_temp3;
    float slide_temp4;
    float slide_temp5;
    float slide_sum;
}Slide_Filter;

float Window_Slide_Filter(Slide_Filter *filter,float data);
float Low_Pass_Filter(LowPass_Filter *filter,float data,float param);

#endif /* CODE_FILTER_H_ */

