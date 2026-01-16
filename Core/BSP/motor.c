#include "motor.h" 

static int16_t clip_pwm(int16_t v)
{
  if (v > MOTOR_MAX_PWM) return MOTOR_MAX_PWM;
  if (v < -MOTOR_MAX_PWM) return -MOTOR_MAX_PWM;
  return v;
}


void Motor_SetPWM(int16_t left, int16_t right)
{
  left  = clip_pwm(left);
  right = clip_pwm(right);

  // 左轮
  if (left == 0) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
  } else if (left > 0) {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)left);
  } else { // left < 0
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
  } else { // right < 0
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)(-right));
  }
}
