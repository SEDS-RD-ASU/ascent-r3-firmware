#ifndef AIRBRAKES_CONTROLLER_H
#define AIRBRAKES_CONTROLLER_H

#include <math.h>
#include "esp_log.h"

typedef struct {
    float def;
    float vel;
    float altitude_agl;
    float elevation;
    float m;
    float cd;
} physics_state_t;

typedef struct {
    int64_t t;
    float def;
} controller_state_t;

float AirDensity(float altitude_agl, float elevation);
float BodyDragForce(float altitude_agl, float elevation, float vel, float cd);
float AirBrakesDragForce(float def, float vel, float altitude_agl, float elevation);
float PredictApogee(float m, float altitude_agl, float elevation, float vel, float def);


#endif