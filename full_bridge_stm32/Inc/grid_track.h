#ifndef GRID_TRACK_H
#define GRID_TRACK_H

#include <stdint.h>

void grid_track_init(float fs_ctrl, float f_nom_hz, float modulation);
void grid_track_set_adc(uint16_t grid_raw, uint16_t aux_raw);
void grid_track_step(void);

float grid_track_get_freq_est(void);
float grid_track_get_theta(void);
int   grid_track_is_locked(void);

#endif