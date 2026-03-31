#include "airbrakes_controller.h"
#include <math.h>

float AirDensity(float altitude_agl, float elevation)
{
    float z = altitude_agl + elevation;
    float ip = 1.225;
    float rho = ip*expf(-(z)/10000);
    return rho;
}

float BodyDragForce(float altitude_agl, float elevation, float vel, float cd)
{
    float A = 8.075e-3f;

    float rho = AirDensity(altitude_agl, elevation);

    float q = 0.5f * rho * vel * vel;
    return -q * cd * A;
}


float AirBrakesDragForce(float def, float vel, float altitude_agl, float elevation)
{
    float cd = 1.28f; // flat plate drag coefficient assumption

    float rho = AirDensity(altitude_agl, elevation);

    // 5th-order polynomial for effective brake area (in^2 -> converted to m^2 via /1550)
    float brakes_area = (19.15959f * powf(def, 5)
                       - 47.14582f * powf(def, 4)
                       + 36.18457f * powf(def, 3)
                       -  8.73138f * powf(def, 2)
                       + expf(1.0f) * def) / 1550.0f;

    // Factor of 3 accounts for 3 airbrake fins
    return -1.0f * (0.5f * rho * vel * vel * brakes_area * cd) * 3.0f;
}

float PredictApogee(float m, float altitude_agl, float elevation, float vel, float def)
{
    const float g        = -9.81f;
    const float body_cd  = 0.8017444f;
    const float dt       = 0.01f;       // time step (s)
    const int   max_iter = 1000000;

    float h = altitude_agl;
    float v = vel;

    int i = 0; // tracking iterations
    

    while (v > 0)
    {
        if (i > max_iter)
        {
            ESP_LOGE("PREDICT APOGEE", "REACHED MAXIMUM ITERATIONS!!");
            break;
        }

        float body_drag = BodyDragForce(h, elevation, v, body_cd);
        float airbrakes_drag = AirBrakesDragForce(def, v, h, elevation);
        float weight = m * g;

        float a = (body_drag + airbrakes_drag + weight) / m;

        float v_new = v + a * dt;
        float h_new = h + v * dt + 0.5f * a * dt * dt;

        if (v_new <= 0.0f) {
            // Interpolate to find exact apogee within the last step
            if (a < 0.0f) {
                float t_stop = -v / a;
                return h + v * t_stop + 0.5f * a * t_stop * t_stop;
            }
            return h;
        }

        v = v_new;
        h = h_new;

        i++;
    }

    return h;
}