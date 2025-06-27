#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

void init_pwm();
void init_direction();
void motor_pwm_task(void *param);

#endif