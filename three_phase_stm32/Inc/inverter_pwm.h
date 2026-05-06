#ifndef INVERTER_PWM_H
#define INVERTER_PWM_H

#include "main.h"
#include <stdint.h>

void inverter_pwm_start(void);
void inverter_pwm_set_ccr(uint16_t a, uint16_t b, uint16_t c);
uint32_t inverter_pwm_get_arr(void);

#endif