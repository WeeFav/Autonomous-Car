#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#define MOTOR_STOP 0
#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 2

void init_pwm_dual(void);
void set_pwm_A(int duty_cycle);
void set_pwm_B(int duty_cycle);
void start_pwm_dual(void);
void init_direction_dual(void);
void set_direction_A(int dir);
void set_direction_B(int dir);
void motor_pwm_task(void *param);

#endif