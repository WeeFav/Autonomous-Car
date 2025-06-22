#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

void init_pwm_dual(void);
void init_direction_dual(void);
void set_pwm_A(int duty_cycle);
void set_pwm_B(int duty_cycle);
void set_direction_A(int dir);
void set_direction_B(int dir);
void motor_pwm_task(void *param);

#endif