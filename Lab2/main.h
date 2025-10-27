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

/* Protótipo da função de inicialização da interrupção */
void Pisca_leds(void);
void collect_password(char *buf, int *len, int maxlen);
void stepper_close(void);
void stepper_open(void);
int usr_sw1_pressed(void);

void PLL_Init(void);
void SysTick_Init(void);
void SysTick_Wait1ms(uint32_t delay);
void SysTick_Wait1us(uint32_t delay);

uint32_t PortJ_Input(void);
void USR_SW1_IntInit(void);
void PortN_Output(uint32_t leds);
void resetLCD();
void lcd_data(uint8_t data);
void lcd_puts(char *s);
char Keypad_Scan(void);
void GPIO_Init(void);


#endif // MAIN_H__