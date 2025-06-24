#ifndef ENCODER_H
#define ENCODER_H

void encoder_init(void);
void IRAM_ATTR encoder_l_isr_handler(void* arg);
void IRAM_ATTR encoder_r_isr_handler(void* arg);
void encoder_task(void *args);

#endif