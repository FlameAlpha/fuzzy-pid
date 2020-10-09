# fuzzyPID

#### 介绍
模糊PID项目

···c
#include "stdio.h"
#include "fuzzyPID.h"

#define DOF 6

int main() {
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
            {PB, NS, PS, PS, PS, PS, PB},
            {PB, PM, PM, PM, PS, PS, PB}};
    int mf_params[
            qf_default * 4] = {-3, -3, -2, 0, -3, -2, -1, 0, -2, -1, 0, 0, -1, 0, 1, 0, 0, 1, 2, 0, 1, 2, 3, 0, 2, 3, 3,
                               0};
    float fuzzy_pid_params[DOF][pid_params_count] = {{0.65f,  0,     0,    0, 0.5f, -2.4f,  1},//MOTOR_NUM_LEFT_HIP {0.65,0,0,0,0.5,-2.4,1}
                                                     {-0.34f, 0,     0,    0, 0.5f, 0.536f, 2.3f},//MOTOR_NUM_LEFT_KNEE {-0.34,0,0,0,0.5,0.536,2.3}
                                                     {-1.1f,  0,     0,    0, 0.5f, 0.00f,  1},//MOTOR_NUM_LEFT_ANKLE {-1.1,0,0,0,0.5,0.00,1}
                                                     {-2.4f,  0,     0,    0, 0.5f, 2.3f,   1},//MOTOR_NUM_RIGHT_HIP {-0.65,0,0,0,0.5,2.3,1}
                                                     {1.2f,   0,     0,    0, 0.5f, -0.7f,  2.3f},//MOTOR_NUM_RIGHT_KNEE {1,0,0,0,0.5,-0.7,2.3} {0.17,0,0,0,0.5,-0.7,2.3}
                                                     {1.2f,   0.05f, 0.1f, 0, 0,    0,      1}};//MOTOR_NUM_RIGHT_ANKLE {0.85,0,0,0,0.5,-0.05,1}

    struct PID **pid_vector = fuzzy_vector_pid_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);

    printf("output:\n");

    float real = 0;
    float idea = max_error * 0.9f;
    for (int j = 0; j < 500; ++j) {
        int out = fuzzy_pid_motor_pwd_output(real, idea, pid_vector[5]);
        real += (float) (out - middle_pwm_output) / (float) middle_pwm_output * (float) max_error * 0.1f;
        printf("%d,%f\n", out, real);
    }

    delete_pid_vector(pid_vector, DOF);
    return 0;
}
```