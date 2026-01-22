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
  left  = PWM_Limit(left*1.04 ,MOTOR_MAX_PWM) *0.7;
  right = PWM_Limit(right*0.96 ,MOTOR_MAX_PWM) *0.7;

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
