#ifndef FULL_BRIDGE_PWM_H
#define FULL_BRIDGE_PWM_H

#include "main.h"
#include <stdint.h>

void full_bridge_pwm_start(void);
void full_bridge_pwm_set_duty(float duty_a, float duty_b);
uint32_t full_bridge_pwm_get_arr(void);

#endif