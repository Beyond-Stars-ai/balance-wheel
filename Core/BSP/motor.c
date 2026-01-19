#include "motor.h" 

// float Velocity_Left,Velocity_Right;	//车轮速度(mm/s)

static int16_t PWM_Limit(int16_t IN,int16_t Limit)
{
	if(IN > Limit) return Limit;
	if(IN < -Limit) return -Limit;
	return IN;
}

void Motor_SetPWM(int16_t left, int16_t right)
{
  left  = PWM_Limit(left ,MOTOR_MAX_PWM);
  right = PWM_Limit(right ,MOTOR_MAX_PWM);

  // 左轮
  if (left == 0) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  } else if (left > 0) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)left);
  } else { 
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)(-left));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  }

  // 右轮
  if (right == 0) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
  } else if (right > 0) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)right);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
  } else { 
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)(-right));
  }
}

// 通过编码器获取速度  
// void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
// { 	
// 	float Rotation_Speed_L,Rotation_Speed_R;						//电机转速  转速=编码器读数（5ms每次）*读取频率/倍频数/减速比/编码器精度 
// 	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
// 	Velocity_Left = Rotation_Speed_L*PI*Diameter;		
// 	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
// 	Velocity_Right = Rotation_Speed_R*PI*Diameter;		
// }

int Position_PID (int position,int target)
{ 	
		static float error,Pwm,Integral_error,Last_error;
		error=target-position;                                  //计算偏差 Calculate deviation
		Integral_error+=error;	                                 //求出偏差的积分 Calculate the integral of the deviation

		Integral_error=PWM_Limit(Integral_error,Target_Velocity); //积分限幅 Integral limit

		Pwm=Position_KP*error+Position_KI*Integral_error+Position_KD*(error-Last_error);       //位置式PID控制器 Position based PID controller

		Last_error=error;                                       //保存上一次偏差  Save the previous deviation
	
		Pwm=PWM_Limit(Pwm,Target_Velocity);  //位置转增量输出限幅 Position conversion increase output limit
		
		return Pwm;                                           //增量输出 Incremental output
}

int Incremental_PI (int Encoder,int Target)
{ 	
		static float error,Pwm,Last_error,Last_last_error;
		error=Target-Encoder;                                  //计算偏差 Calculate deviation
		Pwm+=Incremental_KP*(error-Last_error)+
				 Incremental_KI*error + 
				 Incremental_KD*(error-2*Last_error+Last_last_error);   //增量式PID控制器 Incremental PID controller

		Last_error=error;	                                   //保存上一次偏差 Save the previous deviation
		Last_last_error = Last_error;  //保存上上次偏差 Save the previous deviation

		return Pwm;                                           //增量输出 Incremental output
}