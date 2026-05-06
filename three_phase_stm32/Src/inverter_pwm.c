#include "inverter_pwm.h"

extern TIM_HandleTypeDef htim1;

void inverter_pwm_start(void)
{
    HAL_StatusTypeDef s1, s2, s3, n1, n2, n3;

    s1 = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    n1 = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    s2 = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    n2 = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

    s3 = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    n3 = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    if (s1 != HAL_OK || n1 != HAL_OK ||
        s2 != HAL_OK || n2 != HAL_OK ||
        s3 != HAL_OK || n3 != HAL_OK)
    {
        Error_Handler();
    }
}

void inverter_pwm_set_ccr(uint16_t a, uint16_t b, uint16_t c)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, c);
}

uint32_t inverter_pwm_get_arr(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&htim1);
}                                            