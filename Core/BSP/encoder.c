#include "encoder.h"

int Read_Encoder(Motor_ID MYTIMX)
{
    int Encoder_TIM;
    switch(MYTIMX)
    {
        case MOTOR_ID_ML:
            Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim3);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            break;
        case MOTOR_ID_MR:
            Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim4);
            __HAL_TIM_SET_COUNTER(&htim4, 0);
            break;
        default:
            Encoder_TIM = 0;
    }
    return Encoder_TIM;
}

// int Read_Encoder(Motor_ID MYTIMX)
// {
//    int Encoder_TIM;    
//    switch(MYTIMX)
// 	 {
// 		 case MOTOR_ID_ML:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
// 		 case MOTOR_ID_MR:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
// 		 default: Encoder_TIM=0;
// 	 }
// 		return Encoder_TIM;
// }