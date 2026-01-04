#ifndef SITL_H
#define SITL_H

#include "stdint.h"

void sitl_init();

void sitl_update();

float get_current_vertical_accl();
double get_current_baro_alt();

#endif