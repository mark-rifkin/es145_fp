#include "spwm.h"
#include "inverter_pwm.h"
#include <math.h>
#include <stdint.h>

static float theta = 0.0f;
static float fout = 10.0f;
static float m = 0.8f;
static float fs_update = 1000.0f;


void spwm_init(float fout_hz, float modulation, float update_rate_hz)
{
    fout = fout_hz;
    m = modulation;
    fs_update = update_rate_hz;
    theta = 0.0f;
}

void spwm_step(void)
{
    const float two_pi = 6.283185307f;
    const float phase120 = 2.094395102f;
    float Ts = 1.0f / fs_update;

    theta += two_pi * fout * Ts;
    if (theta >= two_pi)
        theta -= two_pi;

    float va = sinf(theta);
    float vb = sinf(theta - phase120);
    float vc = sinf(theta + phase120);

    uint32_t arr = inverter_pwm_get_arr();
    float center = ((float)arr + 1.0f) * 0.5f;
    float amp = center * m;

    uint16_t ccr_a = (uint16_t)(center + amp * va);
    uint16_t ccr_b = (uint16_t)(center + amp * vb);
    uint16_t ccr_c = (uint16_t)(center + amp * vc);

    if (ccr_a > arr) ccr_a = arr;
    if (ccr_b > arr) ccr_b = arr;
    if (ccr_c > arr) ccr_c = arr;

    inverter_pwm_set_ccr(ccr_a, ccr_b, ccr_c);
}