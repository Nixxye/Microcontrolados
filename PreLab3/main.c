// main.c
// Desenvolvido para a placa EK-TM4C1294XL
// Verifica o estado das chaves USR_SW1 e USR_SW2, acende os LEDs 1 e 2 caso estejam pressionadas independentemente
// Caso as duas chaves estejam pressionadas ao mesmo tempo pisca os LEDs alternadamente a cada 500ms.
// Prof. Guilherme Peron

#include "main.h"

/* Flag setada pela ISR quando USR_SW1 é pressionada (falling edge) */
volatile int usr_sw1_event = 0;

int main(void)
{
	PLL_Init();
    SysTick_Init();
    GPIO_Init();

    while (1) {
        sendUART('k');
        // pequeno intervalo para evitar busy-loop extremo
        SysTick_Wait1ms(50);
    }
}
