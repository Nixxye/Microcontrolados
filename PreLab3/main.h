#ifndef MAIN_H__
#define MAIN_H_

#include <stdint.h>
#include <string.h>


/* Flag setada pela ISR quando USR_SW1 é pressionada (falling edge) */
extern volatile int usr_sw1_event;

extern volatile uint32_t pwm_period_ticks; // 1 ms @ 80 MHz
extern volatile uint32_t pwm_high_ticks;
extern volatile uint32_t pwm_low_ticks;
extern volatile uint8_t pwm_duty; // percent
extern volatile uint8_t pwm_state; // 0 = low, 1 = high
extern volatile uint32_t pwm_pin_mask

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
void sendStringUART(char* str);
void setDCMotorSpeed(int speed);
char receiveCharUART(void);
void GPIOPortJ_Handler(void);
void intToStr(int N, char *str);

#endif // MAIN_H__