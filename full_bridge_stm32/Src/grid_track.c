#include "grid_track.h"
#include "full_bridge_pwm.h"
#include <math.h>

static volatile uint16_t adc_grid_raw = 2048;
static volatile uint16_t adc_aux_raw  = 2048;   // spare / current later

static float fs = 10000.0f;
static float Ts = 0.0001f;

static float f_nom = 50.0f;
static float f_est = 50.0f;
static float omega = 2.0f * 3.1415926535f * 50.0f;
static float theta = 0.0f;

static float m = 0.20f;    // small starter modulation

static uint16_t adc_mid = 2048;

// simple hysteresis for zero-cross detection
static float zc_threshold = 50.0f; // ADC counts after midpoint subtraction

static int last_above = 0;
static uint32_t ticks_since_rising = 0;
static int lock_counter = 0;
static int locked = 0;

void grid_track_init(float fs_ctrl, float f_nom_hz, float modulation)
{
    fs = fs_ctrl;
    Ts = 1.0f / fs_ctrl;

    f_nom = f_nom_hz;
    f_est = f_nom_hz;
    omega = 2.0f * 3.1415926535f * f_nom_hz;
    theta = 0.0f;

    if (modulation < 0.0f) modulation = 0.0f;
    if (modulation > 0.5f) modulation = 0.5f;  // conservative first limit
    m = modulation;

    ticks_since_rising = 0;
    last_above = 0;
    lock_counter = 0;
    locked = 0;
}

void grid_track_set_adc(uint16_t grid_raw, uint16_t aux_raw)
{
    adc_grid_raw = grid_raw;
    adc_aux_raw = aux_raw;
    (void)adc_aux_raw;
}

static float wrap_angle(float x)
{
    const float two_pi = 6.283185307f;
    while (x >= two_pi) x -= two_pi;
    while (x < 0.0f)    x += two_pi;
    return x;
}

void grid_track_step(void)
{
    // Signed grid sample around zero
    int32_t grid_signed_counts = (int32_t)adc_grid_raw - (int32_t)adc_mid;
    float vg = (float)grid_signed_counts;

    // rising zero-cross with hysteresis:
    // below = vg < -thr
    // above = vg > +thr
    int above = (vg > zc_threshold) ? 1 : 0;
    int below = (vg < -zc_threshold) ? 1 : 0;

    ticks_since_rising++;

    // detect a rising crossing only after having been below threshold previously
    static int seen_negative_half = 0;
    if (below) {
        seen_negative_half = 1;
    }

    if (seen_negative_half && above && !last_above)
    {
        if (ticks_since_rising > 0)
        {
            float Tline = ticks_since_rising * Ts;
            float f_new = 1.0f / Tline;

            // accept only plausible bench values
            if (f_new > 40.0f && f_new < 70.0f)
            {
                f_est = f_new;
                omega = 2.0f * 3.1415926535f * f_est;

                if (lock_counter < 20) lock_counter++;
                if (lock_counter > 5) locked = 1;
            }
            else
            {
                if (lock_counter > 0) lock_counter--;
                if (lock_counter == 0) locked = 0;
            }
        }

        theta = 0.0f;
        ticks_since_rising = 0;
        seen_negative_half = 0;
    }

    last_above = above;

    // advance angle between zero crossings
    theta += omega * Ts;
    theta = wrap_angle(theta);

    // generate synchronized bridge command
    float u = m * sinf(theta);

    // unipolar full-bridge duties
    float duty_a = 0.5f + 0.5f * u;
    float duty_b = 0.5f - 0.5f * u;

    full_bridge_pwm_set_duty(duty_a, duty_b);
}

float grid_track_get_freq_est(void)
{
    return f_est;
}

float grid_track_get_theta(void)
{
    return theta;
}

int grid_track_is_locked(void)
{
    return locked;
}