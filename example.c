#include "fuzzyPID.h"

#define DOF 6

int main() {
    // Default fuzzy rule base of delta kp/ki/kd
    int rule_base[][qf_default] = {
            //delta kp rule base
            {PB, PB, PM, PM, PS, ZO, ZO},
            {PB, PB, PM, PS, PS, ZO, NS},
            {PM, PM, PM, PS, ZO, NS, NS},
            {PM, PM, PS, ZO, NS, NM, NM},
            {PS, PS, ZO, NS, NS, NM, NM},
            {PS, ZO, NS, NM, NM, NM, NB},
            {ZO, ZO, NM, NM, NM, NB, NB},
            //delta ki rule base
            {NB, NB, NM, NM, NS, ZO, ZO},
            {NB, NB, NM, NS, NS, ZO, ZO},
            {NB, NM, NS, NS, ZO, PS, PS},
            {NM, NM, NS, ZO, PS, PM, PM},
            {NM, NS, ZO, PS, PS, PM, PB},
            {ZO, ZO, PS, PS, PM, PB, PB},
            {ZO, ZO, PS, PM, PM, PB, PB},
            //delta kd rule base
            {PS, NS, NB, NB, NB, NM, PS},
            {PS, NS, NB, NM, NM, NS, ZO},
            {ZO, NS, NM, NM, NS, NS, ZO},
            {ZO, NS, NS, NS, NS, NS, ZO},
            {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
            {PB, PS, PS, PS, PS, PS, PB},
            {PB, PM, PM, PM, PS, PS, PB}};

    // Default parameters of membership function
    int mf_params[4 * qf_default] = {-3, -3, -2, 0,
                                     -3, -2, -1, 0,
                                     -2, -1,  0, 0,
                                     -1,  0,  1, 0,
                                      0,  1,  2, 0,
                                      1,  2,  3, 0,
                                      2,  3,  3, 0};

    // Default parameters of pid controller
    float fuzzy_pid_params[DOF][pid_params_count] = {{0.65f,  0,     0,    0, 0, 0, 1},
                                                     {-0.34f, 0,     0,    0, 0, 0, 1},
                                                     {-1.1f,  0,     0,    0, 0, 0, 1},
                                                     {-2.4f,  0,     0,    0, 0, 0, 1},
                                                     {1.2f,   0,     0,    0, 0, 0, 1},
                                                     {1.2f,   0.05f, 0.1f, 0, 0, 0, 1}};

    // Obtain the PID controller vector according to the parameters
    struct PID **pid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);

    printf("output:\n");
    int control_id = 5;
    float real = 0;
    float idea = max_error * 0.9f;
    printf("idea value: %f\n", idea);
    bool direct[DOF] = {true, false, false, false, true, true};
    for (int j = 0; j < 1000; ++j) {
        int out = fuzzy_pid_motor_pwd_output(real, idea, direct[control_id], pid_vector[control_id]);
        real += (float) (out - middle_pwm_output) / (float) middle_pwm_output * (float) max_error * 0.1f;
        printf("%d,%f\n", out, real);
    }

    delete_pid_vector(pid_vector, DOF);
    return 0;
}