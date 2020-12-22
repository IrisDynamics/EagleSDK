#include "Periodic.h"

int16_t SineEffect::sine[EFFECT_SAMPLES];
int SineEffect::is_setup = 0;


int16_t SawtoothDownEffect::sawtoothdown[EFFECT_SAMPLES];
int SawtoothDownEffect::is_setup = 0;

int16_t SquareWaveEffect::square[EFFECT_SAMPLES];
int SquareWaveEffect::is_setup = 0;