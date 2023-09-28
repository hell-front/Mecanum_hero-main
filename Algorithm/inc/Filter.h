#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif

#include "main.h"
	

#define Damping 3.0f







	
class Class_Average_Filter
{
private:
    
public:

    void Average_Filter_init();
    
    uint8_t number;
    float value_0;           //队列均值滤�?
    float value_1;
    float value_2;
    float value_3;
    float value_4;




    
    float Average_2(float value);
    float Average_3(float value);
    float Average_5(float value);





};


class Class_one_order_lowpass_filter
{
private:
    uint8_t number;
    float value_last;
    float value_now;
    float value_init;
    float RC;

public:

               
    
    void One_LowPass_Filter_init(float K,float ESTIMATE);//K为一街低通滤波的参数，K越大，则滤波效果越弱小
    float One_LowPass_Filter(float value);


};
class Class_two_order_lowpass_filter
{
private:
    float value_X0;
    float value_X1;
    float value_X2;
    float value_Y0;
    float value_Y1;
    float value_Y2;
    uint8_t number;
    float A;
    float B;
    float C;
    
public:
    void Two_LowPass_Filter_init(float w_c,float Tsw);
    float Two_LowPass_Filter(float value);
};


class Class_Kalman_filter
{
  private:
    float Now_P;
    float Kg;
    float Last_P;
    float out;
    float Q;
    float R;

    float out_last;

  public:
    void Kalman_filter_init(float lastp,float q,float r);
    float kalman_Filter(float input);
    float kalman_Filter(float value_measured,float value_guessd);

};


































#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 




#endif


