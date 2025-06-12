#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

void init_pwm_A(void);
void init_pwm_B(void);

void set_pwm_A(int);
void set_pwm_B(int);

void start_pwm_A(void);
void start_pwm_B(void);

void init_direction_A(void);
void init_direction_B(void);

void set_direction_A(int);
void set_direction_B(int);

#endif