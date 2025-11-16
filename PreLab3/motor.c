#include "motor.h"
#include "tm4c1294ncpdt.h"
#include "main.h"

#define MOTOR_EN_MASK  (1 << 2) // PF2

void Motor_Init(void)
{
    GPIO_PORTF_DATA_R &= ~MOTOR_EN_MASK; // inicialmente desabilitado
    GPIO_PORTE_DATA_R &= ~0x03; // garantir pinos em 0
}

void Motor_Enable(void)
{
    GPIO_PORTF_DATA_R |= MOTOR_EN_MASK; // PF2 = 1 -> habilita L293
}

void Motor_Disable(void)
{
    GPIO_PORTF_DATA_R &= ~MOTOR_EN_MASK; // PF2 = 0 -> desabilita L293
    // garantir pinos em 0
    GPIO_PORTE_DATA_R &= ~0x03;
}

void Motor_SetDirection(Motor_Dir_t dir)
{
    if (dir == MOTOR_DIR_FORWARD) {
        // Para um sentido: PE0 = 1
        GPIO_PORTE_DATA_R |= pwm_pin_mask;
    } else {
        // Sentido inverso: PE0 = 0
        GPIO_PORTE_DATA_R &= ~0x01; // PE0 = 0
    }
}