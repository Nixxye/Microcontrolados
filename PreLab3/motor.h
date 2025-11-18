#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1
} Motor_Dir_t;

void Motor_Init(void);
void Motor_Enable(void);   // liga EN (PF2)
// void Motor_Disable(void);  // desliga EN (PF2)
// void Motor_SetDirection(Motor_Dir_t dir);

#endif // MOTOR_H