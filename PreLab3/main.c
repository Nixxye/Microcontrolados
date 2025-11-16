// main.c
// Desenvolvido para a placa EK-TM4C1294XL
// Verifica o estado das chaves USR_SW1 e USR_SW2, acende os LEDs 1 e 2 caso estejam pressionadas independentemente
// Caso as duas chaves estejam pressionadas ao mesmo tempo pisca os LEDs alternadamente a cada 500ms.
// Prof. Guilherme Peron

#include "main.h"
// #include <string> 
#define MAX_POT 4095
/* Flag setada pela ISR quando USR_SW1 é pressionada (falling edge) */
volatile int usr_sw1_event = 0;


int main(void)
{
	PLL_Init();
    SysTick_Init();
    GPIO_Init();
    initUART();
    ADC_Init();
    Motor_Init();
    Motor_SetDirection(MOTOR_DIR_FORWARD);
    Motor_Enable();
    SysTick_Wait1ms(1000);


    int valorADC = 0;
    char receivedChar = '/0';
    char str[10];
    int direction = 1; // 1 para horário, 0 para anti-horário
    int speed = 0;
    while (1) {
        sendStringUART("Motor parado, pressione '*' para iniciar.");
        setDCMotorSpeed(0);
        // Passo 2
        do {
            receivedChar = receiveCharUART();
            SysTick_Wait1ms(50);
        } while (receivedChar != '*');
        // Passo 3
        sendStringUART("Pressione 'p' (controle pelo potenciometro) ou 't' (controle pelo terminal): ");
        do {
            receivedChar = receiveCharUART();
            SysTick_Wait1ms(50);
        } while (receivedChar != 'p' && receivedChar != 't');
    
        switch (receivedChar) {
            case 'p':
                sendStringUART("Controle pelo potenciometro selecionado. Gire o potenciometro para ajustar a velocidade do motor.");
                sendStringUART("\r\n");
                sendStringUART("\r\n");
                while (receivedChar != 's') {
                    valorADC = converte();
                    direction = (valorADC > MAX_POT / 2);
                    switch (direction) {
                        case 1:
                            speed = ((valorADC - (MAX_POT / 2)) * 100) / (MAX_POT / 2);
                            break;
                        case 0:
                            speed = -((valorADC - (MAX_POT / 2)) * 100) / (MAX_POT / 2);
                            break;
                        default:
                            break;
                    }
                    sendStringUART("Direção: ");
                    if (direction) {
                        sendStringUART("Horário.        ");
                    } else {
                        sendStringUART("Anti-horário.   ");
                    }
                    sendStringUART("");
                    sendStringUART("Velocidade: ");
                    intToStr(speed, str);
                    sendStringUART(str);
                    sendStringUART("\r\n");
                    SysTick_Wait1ms(1000);
                }
            case 't':
                sendStringUART("Controle via terminal selecionado. Envie '0' a '5' para definir a velocidade do motor, 'h' para sentido horario, 'a' para sentido anti-horario, e 's' para parar.\n");
                while (receivedChar != 's') {
                    receivedChar = receiveCharUART();
                    if (receivedChar >= '0' && receivedChar <= '5') {
                        speed = (receivedChar - '0');
                        setDCMotorSpeed(speed);
                        sendStringUART("Velocidade do motor ajustada para: ");
                        sendCharUART(receivedChar);
                        sendCharUART('\n');
                    } else if (receivedChar == 'h') {
                        // Lógica para o motor no sentido horário
                    } else if (receivedChar == 'a') {
                        // Lógica para o motor no sentido anti-horário
                    }
                }
            default:
                break;
        }
    }
}
