#ifndef _BNO055_H
#define _BNO055_H

#include "linmath.h"
#include "stdint.h"

int bno055_setup();
void bno055_read_abs_orientation(quat out);

#endif // _BNO055_H
