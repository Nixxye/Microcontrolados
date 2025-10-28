#ifndef MAIN_H__
#define MAIN_H_

#include <stdint.h>
#include <string.h>

/* Máquina de estados e variáveis */
typedef enum {
    STATE_ABERTO,
    STATE_FECHANDO,
    STATE_FECHADO,
    STATE_ABRINDO,
    STATE_TRAVADO
} CofreState;

#define USER_PASS_LEN 4
#define MAX_INPUT_LEN 16

/* Flag setada pela ISR quando USR_SW1 é pressionada (falling edge) */
extern volatile int usr_sw1_event;
extern uint8_t sentido;
extern uint8_t velocidade;
extern uint16_t posicao_motor;

/* Protótipo da função de inicialização da interrupção */
void AcenderTodosLEDs(void);
void collect_password(char *buf, int *len, int maxlen);
void stepper_close(void);
void stepper_open(void);

void PLL_Init(void);
void SysTick_Init(void);
void SysTick_Wait1ms(uint32_t delay);
void SysTick_Wait1us(uint32_t delay);

uint32_t PortJ_Input(void);
void PortN_Output(uint32_t leds);
void resetLCD();
void lcd_data(uint8_t data);
void lcd_puts(char *s);
char Keypad_Scan(void);
void stepper_move(void);
void GPIO_Init(void);

void GPIOPortJ_Handler(void);


#endif // MAIN_H__