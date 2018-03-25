#ifndef _BNO055_H
#define _BNO055_H

#include "linmath.h"
#include "stdint.h"

#define AXIS_X 1
#define AXIS_Y 2
#define AXIS_Z 3

int bno055_setup(int8_t map_x, int8_t map_y, int8_t map_z);
void bno055_read_abs_orientation(quat out);

#endif // _BNO055_H
