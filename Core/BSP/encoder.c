#include "encoder.h"

// int Read_Encoder(Motor_ID MYTIMX)
// {
//     int Encoder_TIM;
//     switch(MYTIMX)
//     {
//         case MOTOR_ID_ML:
//             Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim3);
//             __HAL_TIM_SET_COUNTER(&htim3, 0);
//             break;
//         case MOTOR_ID_MR:
//             Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim4);
//             __HAL_TIM_SET_COUNTER(&htim4, 0);
//             break;
//         default:
//             Encoder_TIM = 0;
//     }
//     return Encoder_TIM;
// }

int Read_Encoder(Motor_ID MYTIMX)
{
    static int Encoder_Last_ML = 0, Encoder_Last_MR = 0;  // 存储上一次的滤波值
    int Encoder_TIM, Encoder_Least;
    
    switch(MYTIMX)
    {
        case MOTOR_ID_ML:
            Encoder_Least = (short)__HAL_TIM_GET_COUNTER(&htim3);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            // 一阶低通滤波
            Encoder_Last_ML = Encoder_Last_ML * 0.7 + Encoder_Least * 0.3;
            Encoder_TIM = Encoder_Last_ML;
            break;
            
        case MOTOR_ID_MR:
            Encoder_Least = (short)__HAL_TIM_GET_COUNTER(&htim4);
            __HAL_TIM_SET_COUNTER(&htim4, 0);
            // 一阶低通滤波
            Encoder_Last_MR = Encoder_Last_MR * 0.7 + Encoder_Least * 0.3;
            Encoder_TIM = Encoder_Last_MR;
            break;
            
        default:
            Encoder_TIM = 0;
    }
    return Encoder_TIM;
}


int Read_Position(Motor_ID MYTIMX)
{
    int Encoder_TIM;
    switch(MYTIMX)
    {
        case MOTOR_ID_ML:
            Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim3);
            //__HAL_TIM_SET_COUNTER(&htim3, 0);
            break;
        case MOTOR_ID_MR:
            Encoder_TIM = (short)__HAL_TIM_GET_COUNTER(&htim4);
            //__HAL_TIM_SET_COUNTER(&htim4, 0);
            break;
        default:
            Encoder_TIM = 0;
    }
    return Encoder_TIM;
}