#ifndef SPWM_H
#define SPWM_H

void spwm_init(float fout_hz, float modulation, float update_rate_hz);
void spwm_step(void);

#endif