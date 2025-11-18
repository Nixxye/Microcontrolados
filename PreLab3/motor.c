#include "motor.h"
#include "tm4c1294ncpdt.h"



// void Motor_Disable(void)
// {
//     GPIO_PORTF_DATA_R &= ~MOTOR_EN_MASK; // PF2 = 0 -> desabilita L293
//     // garantir pinos em 0
//     GPIO_PORTE_AHB_DATA_R &= ~0x03;
// }

// void Motor_SetDirection(Motor_Dir_t dir)
// {
//     if (dir == MOTOR_DIR_FORWARD) {
//         // Para um sentido: PE0 = 1
//         GPIO_PORTE_AHB_DATA_R |= pwm_pin_mask;
//     } else {
//         // Sentido inverso: PE0 = 0
//         GPIO_PORTE_AHB_DATA_R &= ~0x01; // PE0 = 0
//     }
// }