#ifndef MAIN_H__
#define MAIN_H_

#include <stdint.h>
#include <string.h>


/* Flag setada pela ISR quando USR_SW1 é pressionada (falling edge) */
extern volatile int usr_sw1_event;


/* Protótipo da função de inicialização da interrupção */
void AcenderTodosLEDs(void);

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
void GPIO_Init(void);
void lcd_command(uint8_t command);
void ADC_Init(void);
int converte();
void initUART(void);
void sendCharUART(char c);
void sendIntUART(int c);

void GPIOPortJ_Handler(void);


#endif // MAIN_H__