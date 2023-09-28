#include "main.h"
#include "Filter.h"

void Class_Average_Filter::Average_Filter_init(){
        number=0;
        value_0=0;
        value_1=0;
        value_2=0;
        value_3=0;
        value_4=0;
}


float Class_Average_Filter::Average_2(float value){

    if (number==2){
        value_0=value_1;
        value_1=value;
        return  (value_0+value_1)/2;

    }else if(number==1){
        number++;
        value_1=value;
        return (value_0+value_1)/2;

    }else{
        number=0;
        number++;
        value_0=value;
        return value;
    }
    
}

float Class_Average_Filter::Average_3(float value){

    if (number==0){
        value_0=value_1;
        value_1=value_2;
        value_2=value;
        return  (value_0+value_1+value_2)/3;

    }else if(number==1){
        number++;
        value_1=value;
        return (value_0+value_1)/2;

    }else if(number==2){
        number++;
        value_2=value;
        return  (value_0+value_1+value_2)/3;
    }else{
        number=0;
        number++;
        value_0=value;
        return value;
    }
    
}

float Class_Average_Filter::Average_5(float value){

    if (number==5){
        value_0=value_1;
        value_1=value_2;
        value_2=value_3;
        value_3=value_4;
        value_4=value;
        return  (value_0+value_1+value_2+value_3+value_4)/5;


    }else if(number==1){
        number++;
        value_1=value;
        return (value_0+value_1)/2;

    }else if(number==2){
        number++;
        value_2=value;
        return  (value_0+value_1+value_2)/3;
    }else if(number==3){
        number++;
        value_3=value;
        return  (value_0+value_1+value_2+value_3)/4;
    }else if(number==4){
        number++;
        value_4=value;
        return  (value_0+value_1+value_2+value_3+value_4)/5;
    }else{
        number=0;
        number++;
        value_0=value;
        return value;
    }
    
}


void Class_one_order_lowpass_filter::One_LowPass_Filter_init(float K,float INIT){
        RC=K;
        value_init=INIT;
        number=0;
}


float Class_one_order_lowpass_filter::One_LowPass_Filter(float value){
    if(number==0){
        number++;
        value_now=(1-RC)*value_init+RC*value;
        value_last=value_now;

        return value_now;
    }else{
        value_now=(1-RC)*value_last+RC*value;
        value_last=value_now;
        return value_now;
    }
}


void Class_two_order_lowpass_filter::Two_LowPass_Filter_init(float w_c,float Tsw){

        A=(w_c*w_c*Tsw*Tsw)/(4+4*Damping*w_c*Tsw+w_c*w_c*Tsw*Tsw);
        B=(-8+2*w_c*w_c*Tsw*Tsw)/(4+4*Damping*w_c*Tsw+w_c*w_c*Tsw*Tsw);
        C=(4-4*Damping*w_c*Tsw+4+w_c*Tsw+w_c*w_c*Tsw*Tsw)/(4+4*Damping*w_c*Tsw+w_c*w_c*Tsw*Tsw);
        value_X0=0;
        value_X1=0;
        value_X2=0;
        value_Y0=0;
        value_Y1=0;
        value_Y2=0;
        number=0;
}



float Class_two_order_lowpass_filter::Two_LowPass_Filter(float value){
        
        
        if (number==3)
        {
            value_X0=value_X1;
            value_X1=value_X2;
            value_X2=value;
            value_Y0=value_Y1;
            value_Y1=value_Y2;
            value_Y2=A*(value_X2+2*value_X1+value_X0)-B*value_Y1-C*value_Y0;
            return value_Y2;

        }else if(number==2){
            number++;
            value_X2=value;
            value_Y2=A*(value_X2+2*value_X1+value_X0)-B*value_Y1-C*value_Y0;
            return value_Y2;

        }else if(number==1){
            number++;
            value_X1=value;
            value_Y1=value;
            return value;

        }else{
            number=1;
            value_X0=value;
            value_Y0=value;
            return value;
        }
    

    
}
void Class_Kalman_filter::Kalman_filter_init(float lastp,float q,float r){
    Now_P=0;
    Kg=0;
    Last_P=lastp;
    out=0;
    Q=q;
    R=r;
}



float Class_Kalman_filter::kalman_Filter(float input)
{
     
    //预测协方�?方程：k时刻系统估算协方�? = k-1时刻的系统协方差 + 过程�?声协方差
    Now_P=Last_P+Q;
    //卡尔曼�?�益方程：卡尔曼增益 = k时刻系统估算协方�? / （k时刻系统估算协方�? + 观测�?声协方差�?
    Kg=Now_P/(Now_P+R);
    //更新最优值方程：k时刻状态变量的最优�? = 状态变量的预测�? + 卡尔曼�?�益 * （测量�? - 状态变量的预测值）
    out=out+out-out_last;//计算预测�?=上一次的输出量加上变化量（�?��?�假设每次卡尔曼滤波计算的间隔是一样的�?
    out=out+Kg*(input-out);//因为这一次的预测值就�?上一次的输出�?
    //更新协方�?方程: �?次的系统协方�?付给 kfp->LastP 威下一次运算准备�?
    Last_P=(1-Kg)*Now_P;
    
    
    out_last=out;


    return out;
}


float Class_Kalman_filter::kalman_Filter(float value_measured,float value_guessd){
        //预测协方�?方程：k时刻系统估算协方�? = k-1时刻的系统协方差 + 过程�?声协方差
    Now_P=Last_P+Q;
    //卡尔曼�?�益方程：卡尔曼增益 = k时刻系统估算协方�? / （k时刻系统估算协方�? + 观测�?声协方差�?
    Kg=Now_P/(Now_P+R);
    //更新最优值方程：k时刻状态变量的最优�? = 状态变量的预测�? + 卡尔曼�?�益 * （测量�? - 状态变量的预测值）
    
    out=value_guessd+Kg*(value_measured-value_guessd);//因为这一次的预测值就�?上一次的输出�?
    //更新协方�?方程: �?次的系统协方�?付给 kfp->LastP 威下一次运算准备�?
    Last_P=(1-Kg)*Now_P;
    
    
    out_last=out;


    return out;
}
