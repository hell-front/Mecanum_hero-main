#ifndef PID_H
#define PID_H


#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif




#include "main.h"


#define NB -3   //负大
#define NM -2   //负中
#define NS -1   //负小
#define ZO  0   //零
#define PS  1   //正小
#define PM  2   //正中
#define PB  3   //正大








float Limit_fuction_p(float *value,float limit);//限制value的大小
float Limit_fuction(float value,float limit);//限制value的大小


class Class_PID
{


private:
        float Kp;//PID的三个参数
        float Ki;
        float Kd;
public:
/*常规PID使用到的函数*/
        void PID_init(float KP,float KI,float KD);//PID初始化，初始化PID的三个参数
        void PID_clear();//清楚PID中已有的积分项等内容
        float get_PID_Kp(){return Kp;}//返回PID的比例增益
        float get_PID_Ki(){return Ki;}//返回PID的积分增益
        float get_PID_Kd(){return Kd;}//返回PID的微分增益
        float PID_absolute(float value_target,float value_real);//位置式PID
        float PID_increment(float value_target,float value_real);//增量式PID

/*PID衍生算法使用到的函数*/
        void PID_integ_separated_init(float ERROR_MAX){error_max_inte_sepa=ERROR_MAX;}
        float PID_integral_separated(float value_target,float value_real);//积分分离PID

        void PID_differ_advanced_init(){error_differ_last=0;}
        float PID_differ_advanced(float value_target,float value_real);//微分先行PID

        void PID_differ_filter_init(float FILTER_PROPORTION){Filter_proportion=FILTER_PROPORTION;error_differ_last=0;}
        float PID_differ_filter(float value_target,float value_real);//不完全微分PID

        void PID_anti_integ_saturated_init(float RESULT_MAX,float RESULT_MIN){Result_MAX=RESULT_MAX;Result_MIN=RESULT_MIN;}
        float PID_anti_integral_saturated(float value_target,float value_real);//抗饱和积分PID


/*混合了抗积分饱和的PID函数*/
        float PID_integ_separated_anti_saturated(float value_target,float value_real);//抗饱和积分的积分分离PID
        float PID_differ_advanced_anti_saturated(float value_target,float value_real);//抗饱和积分的微分先行PID
        float PID_differ_filter_anti_saturated(float value_target,float value_real);//抗饱和积分的不完全微分PID


/*混合了低通滤波器和抗积分饱和的积分分离PID函数*/
        float PID_differ_filter_integ_separated_antisaturated(float value_target,float value_real);//抗饱和积分的不完全微分PID








/*常规PID使用到的参数*/
        float error_differ;
        float error_now;
        float error_integral;
        float error_past;
        float error_past_past;
        float Result;
        float Result_delta;
/*积分分离PID使用到的参数*/
        float error_max_inte_sepa;
/*微分先行PID使用到的参数*/
        float value_real_last;
/*不完全微分PID使用到的参数*/
        float Filter_proportion;//数值越小，滤波作用越显著，并且该值必须在0到1之间
        float error_differ_last;
/*抗饱和积分PID使用到的参数*/
        float Result_MAX;
        float Result_MIN;


};





#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 




#endif
