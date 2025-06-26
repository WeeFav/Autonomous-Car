#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

void init_pwm_dual();
void init_direction_dual();
void motor_pwm_task(void *param);

#endif