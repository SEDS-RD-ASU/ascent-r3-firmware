float irec_rocket_mass(float t)
{
    float dt;

    if (t < 0.754600f) {
        dt = t - 0.000000f;
        return  2.219779e-05f * dt*dt*dt
               + 0.0048280345f * dt*dt
               - 1.0647117f    * dt
               + 17.84403f;
    } else if (t < 1.509200f) {
        dt = t - 0.754600f;
        return -0.078465575f * dt*dt*dt
               + 0.064038158f * dt*dt
               - 1.0573873f   * dt
               + 17.043357f;
    } else if (t < 2.263800f) {
        dt = t - 1.509200f;
        return  0.34381793f * dt*dt*dt
               - 0.31823442f * dt*dt
               - 1.0947808f  * dt
               + 16.248202f;
    } else if (t < 3.018400f) {
        dt = t - 2.263800f;
        return  0.69814375f  * dt*dt*dt
               - 0.37324751f * dt*dt
               - 0.98772856f * dt
               + 15.388604f;
    } else if (t < 3.773000f) {
        dt = t - 3.018400f;
        return  0.16286139f * dt*dt*dt
               + 0.05314741f * dt*dt
               - 0.35842023f * dt
               + 14.730712f;
    } else {
        /* Past burnout — return end-of-spline value */
        dt = 3.773000f - 3.018400f;
        return  0.16286139f * dt*dt*dt
               + 0.05314741f * dt*dt
               - 0.35842023f * dt
               + 14.730712f;
    }
}