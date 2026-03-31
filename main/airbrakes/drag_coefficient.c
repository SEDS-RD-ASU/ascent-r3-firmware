#include "drag_coefficient.h"

// this is one big spline that i make in matlab from the rasaero drag coefficient information.

float drag_coefficient(float t)
{
    float dt;

    /* Segment 1: 0.000 <= t < 0.417 */
    if (t < 0.417f) {
        dt = t - 0.000f;
        return -0.0784187f*dt*dt*dt + 0.181039f*dt*dt + -0.115666f*dt + 0.800195f;
    }
    /* Segment 2: 0.417 <= t < 0.833 */
    if (t < 0.833f) {
        dt = t - 0.417f;
        return -0.0784187f*dt*dt*dt + 0.0830157f*dt*dt + -0.0056431f*dt + 0.777759f;
    }
    /* Segment 3: 0.833 <= t < 1.250 */
    if (t < 1.250f) {
        dt = t - 0.833f;
        return 0.0951196f*dt*dt*dt + -0.0150077f*dt*dt + 0.0226936f*dt + 0.784147f;
    }
    /* Segment 4: 1.250 <= t < 1.667 */
    if (t < 1.667f) {
        dt = t - 1.250f;
        return -0.256836f*dt*dt*dt + 0.103892f*dt*dt + 0.0597287f*dt + 0.797878f;
    }
    /* Segment 5: 1.667 <= t < 2.083 */
    if (t < 2.083f) {
        dt = t - 1.667f;
        return 0.678411f*dt*dt*dt + -0.217153f*dt*dt + 0.0125368f*dt + 0.822223f;
    }
    /* Segment 6: 2.083 <= t < 2.500 */
    if (t < 2.500f) {
        dt = t - 2.083f;
        return -0.722962f*dt*dt*dt + 0.630861f*dt*dt + 0.184915f*dt + 0.838822f;
    }
    /* Segment 7: 2.500 <= t < 2.917 */
    if (t < 2.917f) {
        dt = t - 2.500f;
        return 0.0247769f*dt*dt*dt + -0.272841f*dt*dt + 0.334091f*dt + 0.973097f;
    }
    /* Segment 8: 2.917 <= t < 3.333 */
    if (t < 3.333f) {
        dt = t - 2.917f;
        return 0.101072f*dt*dt*dt + -0.24187f*dt*dt + 0.119628f*dt + 1.06673f;
    }
    /* Segment 9: 3.333 <= t < 3.750 */
    if (t < 3.750f) {
        dt = t - 3.333f;
        return 0.210673f*dt*dt*dt + -0.11553f*dt*dt + -0.0292889f*dt + 1.08189f;
    }
    /* Segment 10: 3.750 <= t < 4.167 */
    if (t < 4.167f) {
        dt = t - 3.750f;
        return -0.318243f*dt*dt*dt + 0.147811f*dt*dt + -0.0158384f*dt + 1.06487f;
    }
    /* Segment 11: 4.167 <= t < 4.583 */
    if (t < 4.583f) {
        dt = t - 4.167f;
        return 0.217969f*dt*dt*dt + -0.249993f*dt*dt + -0.058414f*dt + 1.06091f;
    }
    /* Segment 12: 4.583 <= t < 5.000 */
    if (t < 5.000f) {
        dt = t - 4.583f;
        return 0.0284789f*dt*dt*dt + 0.0224682f*dt*dt + -0.153216f*dt + 1.00894f;
    }
    /* Segment 13: 5.000 <= t < 7.499 */
    if (t < 7.499f) {
        dt = t - 5.000f;
        return -0.00918812f*dt*dt*dt + 0.0580668f*dt*dt + -0.11966f*dt + 0.951058f;
    }
    /* Segment 14: 7.499 <= t < 9.998 */
    if (t < 9.998f) {
        dt = t - 7.499f;
        return 0.00216576f*dt*dt*dt + -0.0108113f*dt*dt + -0.00157726f*dt + 0.871263f;
    }
    /* Segment 15: 9.998 <= t < 12.496 */
    if (t < 12.496f) {
        dt = t - 9.998f;
        return -0.000815867f*dt*dt*dt + 0.00542412f*dt*dt + -0.0150389f*dt + 0.833607f;
    }
    /* Segment 16: 12.496 <= t < 14.995 */
    if (t < 14.995f) {
        dt = t - 12.496f;
        return 0.000178189f*dt*dt*dt + -0.000691967f*dt*dt + -0.00321411f*dt + 0.817167f;
    }
    /* Segment 17: 14.995 <= t < 17.494 */
    if (t < 17.494f) {
        dt = t - 14.995f;
        return -0.000112907f*dt*dt*dt + 0.000643818f*dt*dt + -0.00333443f*dt + 0.807595f;
    }
    /* Segment 18: 17.494 <= t < 19.993 */
    if (t < 19.993f) {
        dt = t - 17.494f;
        return 0.000260643f*dt*dt*dt + -0.000202583f*dt*dt + -0.00223187f*dt + 0.801521f;
    }
    /* Segment 19: 19.993 <= t < 22.492 */
    if (t < 22.492f) {
        dt = t - 19.993f;
        return -0.000216905f*dt*dt*dt + 0.00175131f*dt*dt + 0.00163811f*dt + 0.798746f;
    }
    /* Segment 20: 22.492 <= t < 24.990 (clamp beyond upper bound) */
    dt = t - 22.492f;
    return -0.000216905f*dt*dt*dt + 0.000125301f*dt*dt + 0.00632741f*dt + 0.81039f;
}