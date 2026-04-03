
typedef struct {
    float kp;
    float ki;
    float kd;

    float limit_min;
    float limit_max;

    float T; // time step (s)

    float integral;
    float prev_error;
    float differentiator;

    float output;
} PIDController;

void init_pid(PIDController *pid);
float update_pid(PIDController *pid, float error);
