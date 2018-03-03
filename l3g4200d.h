#include "stdint.h"

const int L3G4200D_DPS_250  = 0;
const int L3G4200D_DPS_500  = 1;
const int L3G4200D_DPS_2000 = 2;

void l3g4200d_setup();
void l3g4200d_read_gyro(int16_t *x, int16_t *y, int16_t *z);
void l3g4200d_calibrate(int n, float *threshX, float *threshY, float *threshZ);
