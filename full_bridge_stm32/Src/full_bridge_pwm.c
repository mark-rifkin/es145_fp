#include "full_bridge_pwm.h"

extern TIM_HandleTypeDef htim1;

void full_bridge_pwm_start(void)
{
    HAL_StatusTypeDef s1, n1, s2, n2;

    s1 = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    n1 = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

    s2 = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    n2 = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

    if (s1 != HAL_OK || n1 != HAL_OK || s2 != HAL_OK || n2 != HAL_OK)
    {
        Error_Handler();
    }
}

void full_bridge_pwm_set_duty(float duty_a, float duty_b)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    // Conservative rail margin for bring-up
    if (duty_a < 0.05f) duty_a = 0.05f;
    if (duty_a > 0.95f) duty_a = 0.95f;
    if (duty_b < 0.05f) duty_b = 0.05f;
    if (duty_b > 0.95f) duty_b = 0.95f;

    uint16_t ccr_a = (uint16_t)((arr + 1U) * duty_a);
    uint16_t ccr_b = (uint16_t)((arr + 1U) * duty_b);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
}

uint32_t full_bridge_pwm_get_arr(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&htim1);
}