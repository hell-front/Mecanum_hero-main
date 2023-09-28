#include "PID.h"
#include "string.h"


float Limit_fuction_p(float *value,float limit){
    if (*value>limit)
    {
        *value=limit;
        return limit;
    }
    if (*value<-limit)
    {
        *value=-limit;
        return -limit;
    }
    
    return *value;    
}

float Limit_fuction(float value,float limit){
    if (value>limit)
    {
        value=limit;
        return limit;
    }
    if (value<-limit)
    {
        value=-limit;
        return -limit;
    }
    
    return value;    
}


void Class_PID::PID_init(float KP,float KI,float KD){
        Kp=KP;
        Ki=KI;
        Kd=KD;
        error_integral=0;
        error_past=0;
        error_past_past=0;
        Result=0;
        error_differ=0;
        
}
void Class_PID::PID_clear(){
    error_differ=0;
    error_differ_last=0;
    error_integral=0;
}

float Class_PID::PID_absolute(float value_target,float value_real){
        
        error_now=value_target-value_real;
        error_integral+=error_now;
        error_differ=error_now-error_past;
    
        Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
        error_past=error_now;
        return Result;
}

float Class_PID::PID_increment(float value_target,float value_real){
        
        error_now=value_target-value_real;
        
        
        Result_delta=Kp*(error_now-error_past)+Ki*error_now+Kd*(error_now-2*error_past+error_past_past);
        Result+=Result_delta;
        error_past_past=error_past;
        error_past=error_now;
        
        
        return Result;

        
}

//积分分离PID
float Class_PID::PID_integral_separated(float value_target,float value_real){


        error_now=value_target-value_real;
        if(error_now<error_max_inte_sepa&&error_now>-error_max_inte_sepa){
            error_integral+=error_now;
            error_differ=error_now-error_past;
    
            Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
            error_past=error_now;
            return Result;
        }else{
            
            error_differ=error_now-error_past;
    
            Result=Kp*error_now+Kd*error_differ;
            error_past=error_now;
            return Result;
        }

}

//微分先行PID
float Class_PID::PID_differ_advanced(float value_target,float value_real){


        error_now=value_target-value_real;
        error_integral+=error_now;

        error_differ=value_real_last-value_real;
        

        Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
        error_past=error_now;
        value_real_last=value_real;
        return Result;
}

//不完全微分PID
float Class_PID::PID_differ_filter(float value_target,float value_real){


        error_now=value_target-value_real;
        error_integral+=error_now;
        error_differ=Filter_proportion*(error_now-error_past)+(1-Filter_proportion)*error_differ_last;
    
        

        Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
        error_past=error_now;
        error_differ_last=error_differ;
        return Result;
}

//抗饱和积分的PID
float Class_PID::PID_anti_integral_saturated(float value_target,float value_real){

        error_now=value_target-value_real;
        if(Result>Result_MAX){
            if(error_now>0){

            }else{
                error_integral+=error_now;
            }
        }else if(Result<Result_MIN){
            if(error_now<0){

            }else{
                error_integral+=error_now;
            }
        }else{
                error_integral+=error_now;
        }
        
 
        error_differ=error_now-error_past;
    
        Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
        error_past=error_now;
        if(Result>Result_MAX){
            return Result_MAX;
        }else if(Result<Result_MIN){
            return Result_MIN;
        }else{
            return Result;
        }

         
}

//抗饱和积分的积分分离PID
float Class_PID::PID_integ_separated_anti_saturated(float value_target,float value_real){
        error_now=value_target-value_real;


        if(error_now<error_max_inte_sepa&&error_now>-error_max_inte_sepa){
            if(Result>Result_MAX){
                if(error_now>0){

                }else{
                    error_integral+=error_now;
                }
            }else if(Result<Result_MIN){
                if(error_now<0){

                }else{
                    error_integral+=error_now;
                }
            }else{
                error_integral+=error_now;
            }
        
 
            error_differ=error_now-error_past;
    
            Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
            error_past=error_now;
            if(Result>Result_MAX){
                return Result_MAX;
            }else if(Result<Result_MIN){
                return Result_MIN;
            }else{
                return Result;
            }
        }else{
 
            error_differ=error_now-error_past;
    
            Result=Kp*error_now+Kd*error_differ;
            error_past=error_now;
            if(Result>Result_MAX){
                return Result_MAX;
            }else if(Result<Result_MIN){
                return Result_MIN;
            }else{
                return Result;
            }
        }


}

//抗饱和积分的微分先行PID
float Class_PID::PID_differ_advanced_anti_saturated(float value_target,float value_real){
        error_now=value_target-value_real;
        if(Result>Result_MAX){
            if(error_now>0){

            }else{
                error_integral+=error_now;
            }
        }else if(Result<Result_MIN){
            if(error_now<0){

            }else{
                error_integral+=error_now;
            }
        }else{
                error_integral+=error_now;
        }
        
 
        
        error_differ=value_real_last-value_real;
        Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
        error_past=error_now;
        if(Result>Result_MAX){
            return Result_MAX;
        }else if(Result<Result_MIN){
            return Result_MIN;
        }else{
            return Result;
        }



}


//抗饱和积分的不完全微分PID
float Class_PID::PID_differ_filter_anti_saturated(float value_target,float value_real){
        error_now=value_target-value_real;
        if(Result>Result_MAX){
            if(error_now>0){

            }else{
                error_integral+=error_now;
            }
        }else if(Result<Result_MIN){
            if(error_now<0){

            }else{
                error_integral+=error_now;
            }
        }else{
                error_integral+=error_now;
        }
        
 
        error_differ=Filter_proportion*(error_now-error_past)+(1-Filter_proportion)*error_differ_last;
    
        Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
        error_past=error_now;
        error_differ_last=error_differ;
        if(Result>Result_MAX){
            return Result_MAX;
        }else if(Result<Result_MIN){
            return Result_MIN;
        }else{
            return Result;
        }


}

//混合了低通滤波器和抗积分饱和的积分分离PID函数
float Class_PID::PID_differ_filter_integ_separated_antisaturated(float value_target,float value_real){
        error_now=value_target-value_real;


        if(error_now<error_max_inte_sepa&&error_now>-error_max_inte_sepa){
            if(Result>Result_MAX){
                if(error_now>0){

                }else{
                    error_integral+=error_now;
                }
            }else if(Result<Result_MIN){
                if(error_now<0){

                }else{
                    error_integral+=error_now;
                }
            }else{
                error_integral+=error_now;
            }
        
 
            error_differ=Filter_proportion*(error_now-error_past)+(1-Filter_proportion)*error_differ_last;
    
            Result=Kp*error_now+Ki*error_integral+Kd*error_differ;
            error_past=error_now;
            error_differ_last=error_differ;
            if(Result>Result_MAX){
                return Result_MAX;
            }else if(Result<Result_MIN){
                return Result_MIN;
            }else{
                return Result;
            }
        }else{
 
            error_differ=Filter_proportion*(error_now-error_past)+(1-Filter_proportion)*error_differ_last;
    
            Result=Kp*error_now+Kd*error_differ;
            error_past=error_now;
            error_differ_last=error_differ;
            if(Result>Result_MAX){
                return Result_MAX;
            }else if(Result<Result_MIN){
                return Result_MIN;
            }else{
                return Result;
            }
        }
}










