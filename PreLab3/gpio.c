// gpio.c
// Desenvolvido para a placa EK-TM4C1294XL
// Inicializa as portas J, N, K e M
// Prof. Guilherme Peron

#include <stdint.h>

#include "tm4c1294ncpdt.h"
#include "main.h"

#define GPIO_PORTA (0x0001)
#define GPIO_PORTE (0x0010)
#define GPIO_PORTJ (0x0100)
#define GPIO_PORTK (0x0200)
#define GPIO_PORTL (0x0400)
#define GPIO_PORTM (0x0800)
#define GPIO_PORTN (0x1000)
#define GPIO_PORTH (0x0080)
#define GPIO_PORTQ (0x4000)
#define GPIO_PORTP (0x2000)

void SysTick_Wait1ms(uint32_t delay);
void SysTick_Wait1us(uint32_t delay);

// --- PWM globals (acessíveis pela ISR) ---
volatile uint32_t pwm_period_ticks = 80000; // 1 ms @ 80 MHz
volatile uint32_t pwm_high_ticks = 0;
volatile uint32_t pwm_low_ticks = 0;
volatile uint8_t pwm_duty = 60; // percent
volatile uint8_t pwm_state = 1; // 0 = low, 1 = high

// seleciona qual pino de PORTE será utilizado para PWM: 0x01 = PE0, 0x02 = PE1
volatile uint32_t pwm_pin_mask = 0x01;

int velocidade_atual = 0;
int velocidade_alvo = 0;
int velocidade_modulo = 0;
int direcao = 0;
int contadorPrint = 0;
char str[10];

// Atualiza o duty cycle do PWM (0..100). Ajusta ticks correspondentes
void PWM_SetDuty(uint8_t duty, int direction)
{
    if (duty > 100) 
        duty = 100;

    pwm_duty = duty;
    pwm_high_ticks = (pwm_period_ticks * pwm_duty) / 100;
    pwm_low_ticks = pwm_period_ticks - pwm_high_ticks;
    pwm_pin_mask = direction ? 0x01 : 0x02; // PE0 para horário, PE1 para anti-horário

    // Ajusta imediatamente o reload do timer conforme o estado atual
    if (pwm_state == 1) {
        if (pwm_high_ticks == 0) pwm_high_ticks = 1;
        TIMER0_TAILR_R = pwm_high_ticks - 1;
        GPIO_PORTE_AHB_DATA_R |= pwm_pin_mask;
    } else {
        if (pwm_low_ticks == 0) pwm_low_ticks = 1;
        TIMER0_TAILR_R = pwm_low_ticks - 1;
        GPIO_PORTE_AHB_DATA_R &= ~0x03;
    }
}

// -------------------------------------------------------------------------------
// Função PortJ_Input
// Lê os valores de entrada do port J
// Parâmetro de entrada: Não tem
// Parâmetro de saída: o valor da leitura do port
uint32_t PortJ_Input(void)
{
    return GPIO_PORTJ_AHB_DATA_R;
}

void Timer0A_Handler(void) {
    // Acknowledge timer interrupt
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;

    // PWM software toggle on PE0 (alternância entre HIGH/LOW)
    if (pwm_state == 1) {
        // estava HIGH -> passar para LOW
        GPIO_PORTE_AHB_DATA_R &= ~pwm_pin_mask; // PE0 = 0
        pwm_state = 0;
        if (pwm_low_ticks == 0)
            pwm_low_ticks = 1;
        TIMER0_TAILR_R = pwm_low_ticks - 1;
    } else {
        // estava LOW -> passar para HIGH
        GPIO_PORTE_AHB_DATA_R |= pwm_pin_mask; // PE0 = 1
        pwm_state = 1;
        if (pwm_high_ticks == 0) 
            pwm_high_ticks = 1;
        TIMER0_TAILR_R = pwm_high_ticks - 1;
    }

    contadorPrint = (contadorPrint + 1) % 5000;
    if (contadorPrint == 0) {
        sendStringUART("Direcao: ");
        if (velocidade_atual > 0) {
            sendStringUART("Horario.        ");
        } else {
            sendStringUART("Anti-horario.   ");
        }
        sendStringUART("");
        sendStringUART("Velocidade: ");
        intToStr(velocidade_modulo, str);
        sendStringUART(str);
        sendStringUART("\r\n");
        sendStringUART("Velocidade alvo: ");
        intToStr(velocidade_alvo, str);
        sendStringUART(str);
        sendStringUART("\r\n");
    }
}

// -------------------------------------------------------------------------------
// Função GPIOPortJ_Handler
// Rotina de tratamento da interrupção do PortJ (USR_SW1)
void GPIOPortJ_Handler(void)
{
    if ((GPIO_PORTJ_AHB_RIS_R & 0x01) == 0)
        return; // Verifica se a interrupção foi no pin0
    // Limpa o evento de interrupção para o pin0
    GPIO_PORTJ_AHB_ICR_R = 0x01;

    // Sinaliza para o código principal que a tecla foi pressionada
    usr_sw1_event = 1;
}

// -------------------------------------------------------------------------------
// Função PortN_Output
// Escreve os valores no port N
// Parâmetro de entrada: Valor a ser escrito
// Parâmetro de saída: não tem
void PortN_Output(uint32_t valor)
{
    uint32_t temp;
    // vamos zerar somente os bits menos significativos
    // para uma escrita amigável nos bits 0 e 1
    temp = GPIO_PORTN_DATA_R & 0xFC;
    // agora vamos fazer o OR com o valor recebido na função
    temp = temp | valor;
    GPIO_PORTN_DATA_R = temp;
}
/**
 * Acende TODOS os LEDs da PAT.
 */
void AcenderTodosLEDs(void)
{
    GPIO_PORTA_AHB_DATA_R = 0xF0;
    GPIO_PORTQ_DATA_R = 0xF;
    GPIO_PORTP_DATA_R = 0x20;
    SysTick_Wait1ms(1);
    GPIO_PORTP_DATA_R = 0xDF;
    SysTick_Wait1ms(1);
}
// --- Funções do LCD ---

/**
 * Envia um comando para o LCD.
 * - Barramento de dados 8-bit no GPIO_PORTK_DATA_R (PK0-PK7)
 * - RS = PM0
 * - R/W = PM1 (sempre 0)
 * - E  = PM2
 */
void lcd_command(uint8_t command)
{
    // Configura RS=0, R/W=0, E=0
    // (Limpa bits PM0, PM1, PM2)
    GPIO_PORTM_DATA_R &= ~0x07;

    // Coloca o comando no barramento de dados (Port K)
    GPIO_PORTK_DATA_R = command;

    // Gera um pulso em 'E' (PM2)
    // Seta PM2 (E=1)
    GPIO_PORTM_DATA_R |= 0x04;
    SysTick_Wait1ms(2); // Duração do pulso

    // Finaliza o pulso em 'E' (PM2)
    // Limpa PM2 (E=0)
    GPIO_PORTM_DATA_R &= ~0x04;
}

void resetLCD()
{
    lcd_command(0x01);
    SysTick_Wait1ms(2); // Espera 2ms (1.64ms é o mínimo)
}

/**
 * Envia DADOS (um caractere) para o LCD.
 * - Barramento de dados 8-bit no GPIO_PORTK_DATA_R (PK0-PK7)
 * - RS = PM0
 * - R/W = PM1 (sempre 0)
 * - E  = PM2
 */
void lcd_data(uint8_t data)
{

    // 1. Configura RS=1, R/W=0, E=0
    //    Seta PM0 (RS=1), Limpa PM1 (R/W=0), Limpa PM2 (E=0)
    GPIO_PORTM_DATA_R = (GPIO_PORTM_DATA_R & ~0x06) | 0x01;

    // 2. Coloca o caractere ASCII no barramento de dados (Port K)
    GPIO_PORTK_DATA_R = data;

    // 3. Gera um pulso em 'E' (PM2)
    //    Seta PM2 (E=1)
    GPIO_PORTM_DATA_R |= 0x04;
    SysTick_Wait1ms(2);

    //    Limpa PM2 (E=0)
    GPIO_PORTM_DATA_R &= ~0x04;

    // 4. Espera o tempo necessário para o LCD processar o caractere
    SysTick_Wait1ms(40);
}

/**
 * Escreve uma string (vários caracteres) no LCD
 */
void lcd_puts(char *s)
{
    while (*s)
    {
        lcd_data(*s);
        s++;
    }
}

void initLCD()
{
    // 1. Inicializar no modo 2 linhas / caracter matriz 5x7 (0x38)
    lcd_command(0x38);
    SysTick_Wait1ms(40);

    // 2. Cursor com autoincremento para direita (0x06)
    lcd_command(0x06);
    SysTick_Wait1ms(40);

    // 3. Configurar o cursor (habilitar o display + cursor + não-pisca) (0x0E)
    lcd_command(0x0E);
    SysTick_Wait1ms(40);

    // 4. Resetar: Limpar o display e levar o cursor para o home (0x01)
    lcd_command(0x01);
    SysTick_Wait1ms(2); // Espera 2ms (1.64ms é o mínimo)
}

void initTimer() {
    // Disable timer while configuring
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;

    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;

    // Calcular ticks para PWM com base em pwm_period_ticks e pwm_duty
    pwm_high_ticks = (pwm_period_ticks * pwm_duty) / 100;
    pwm_low_ticks = pwm_period_ticks - pwm_high_ticks;

    // Inicialmente contar pelo intervalo HIGH (assumimos iniciar em HIGH)
    if (pwm_high_ticks == 0) 
        pwm_high_ticks = 1;

    TIMER0_TAILR_R = pwm_high_ticks - 1;

    TIMER0_TAPR_R = 0;

    // Limpar event/interrupt
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;

    // Habilitar interrupção de timeout
    TIMER0_IMR_R |= TIMER_IMR_TATOIM;

    // Prioridade e habilitação no NVIC (Timer0A = IRQ 19)
    NVIC_PRI4_R |= (2 << 29);
    NVIC_EN0_R |= (1 << 19);

    // Começar com o pino em HIGH e habilitar o timer
    GPIO_PORTE_AHB_DATA_R |= 0x01; // PE0 = 1
    pwm_state = 1;
    TIMER0_CTL_R |= TIMER_CTL_TAEN;
}

void initUART() {
  SYSCTL_RCGCUART_R = 0x1; // Ativa o clock para UART0
  while ((SYSCTL_PRUART_R & 0x1) == 0) {}; // Espera a UART0 ficar pronta
  UART0_CTL_R = 0;
  
  // 520,8333
  // 53,3333
  UART0_IBRD_R = 520;
  UART0_FBRD_R = 53;

  UART0_LCRH_R = 0x7A; // Paridade ímpar, 2 stop bits
  UART0_CC_R = 0;
  
  UART0_CTL_R = 0x301;
  
  // UART_CTL_UARTEN = 0;
  // UART_CTL_HSE = 0;
  // UART_CTL_UARTEN = 1;
  // UART_CTL_TXE = 1;
  // UART_CTL_RXE = 1;

  GPIO_PORTA_AHB_PCTL_R = 0x11; // Configura PA0 e PA1 para função UART
  GPIO_PORTA_AHB_AFSEL_R = 0x03; // Habilita função alternativa em PA0 e PA1

}

void sendCharUART(char c) {
  while (UART0_FR_R & UART_FR_TXFF); // Espera até que o buffer de transmissão esteja vazio
  UART0_DR_R = c; // Envia o caractere
}

void sendIntUART(int c) {
  while (UART0_FR_R & UART_FR_TXFF); // Espera até que o buffer de transmissão esteja vazio
  UART0_DR_R = c; // Envia o inteiro
}

void sendStringUART(char* str) {
    while (*str) {
        sendCharUART(*str);
        str++;
    }
}

char receiveCharUART(void) {
    // while (UART0_FR_R & UART_FR_RXFE); // Espera até que haja dados para ler
    return (char)(UART0_DR_R & 0xFF); // Lê e retorna o caractere recebido
}

void intToStr(int N, char *str) {
    int i = 0;
  
    // Save the copy of the number for sign
    int sign = N;

    // If the number is negative, make it positive
    if (N < 0)
        N = -N;

    // Extract digits from the number and add them to the
    // string
    while (N > 0) {
      
        // Convert integer digit to character and store
      	// it in the str
        str[i++] = N % 10 + '0';
      	N /= 10;
    } 

    // If the number was negative, add a minus sign to the
    // string
    if (sign < 0) {
        str[i++] = '-';
    }

    // Null-terminate the string
    str[i] = '\0';

    // Reverse the string to get the correct order
    for (int j = 0, k = i - 1; j < k; j++, k--) {
        char temp = str[j];
        str[j] = str[k];
        str[k] = temp;
    }
}
// --- (NOVO) Funções do Teclado (Port L e M) ---

/**
 * Faz a varredura do teclado matricial 4x4.
 * Colunas (Saída): PM4, PM5, PM6, PM7
 * Linhas (Entrada): PL0, PL1, PL2, PL3
 * Retorna o caractere da tecla pressionada, ou '\0' (nulo) se nenhuma.
 */
char Keypad_Scan(void)
{
    // Mapeamento de teclas [coluna][linha]
    // Col 0 = PM4, Col 1 = PM5, Col 2 = PM6, Col 3 = PM7
    // Linha 0 = PL0, Linha 1 = PL1, Linha 2 = PL2, Linha 3 = PL3
    const char keymap[4][4] = {
        {'1', '4', '7', '*'}, // Coluna 0 (PM4)
        {'2', '5', '8', '0'}, // Coluna 1 (PM5)
        {'3', '6', '9', '#'}, // Coluna 2 (PM6)
        {'A', 'B', 'C', 'D'}  // Coluna 3 (PM7)
    };

    uint32_t col_pin;
    uint32_t lines;
    int i, j;

    for (i = 0; i < 4; i++)
    {
        // 'i' representa a coluna (0 a 3)
        // O pino da coluna atual é (PM4 + i)
        col_pin = (1 << (i + 4));

        // Configurar colunas:
        //    - Todas as colunas do teclado (PM4-PM7) como entrada alta impedância
        //    - Manter LCD (PM0-PM2) como saída
        GPIO_PORTM_DIR_R = (GPIO_PORTM_DIR_R & ~0xF0) | 0x07;

        // Configurar a coluna ATUAL (col_pin) como saída
        GPIO_PORTM_DIR_R |= col_pin;

        // Colocar 0 na coluna atual
        //    (Garante que não afeta os pinos do LCD PM0-PM2)
        uint32_t current_portm_data = GPIO_PORTM_DATA_R;
        GPIO_PORTM_DATA_R = (current_portm_data & ~col_pin);

        SysTick_Wait1ms(5); // Pequeno delay para estabilizar o sinal

        // Verificar o valor de leitura das linhas (PL0-PL3)
        lines = GPIO_PORTL_DATA_R & 0x0F;

        // Se algum bit for 0, uma tecla foi pressionada
        if (lines != 0x0F)
        {
            for (j = 0; j < 4; j++)
            {
                // 'j' representa a linha (0 a 3)
                if ((lines & (1 << j)) == 0)
                {
                    // Encerra a varredura e retorna a tecla

                    // Restaura colunas para entrada antes de sair
                    GPIO_PORTM_DIR_R = (GPIO_PORTM_DIR_R & ~0xF0) | 0x07;

                    return keymap[i][j];
                }
            }
        }
    }

    // Se todos os bits = 1 (após varrer tudo), nenhuma tecla pressionada
    // Restaura colunas para entrada
    GPIO_PORTM_DIR_R = (GPIO_PORTM_DIR_R & ~0xF0) | 0x07;
    return '\0'; // Retorna nulo
}

int converte() {
    ADC0_PSSI_R = 0x8; // Inicia a conversão no SS3
    while ((ADC0_RIS_R & 0x8) == 0) {}; // Espera a conversão completar
    int valorConvertido = ADC0_SSFIFO3_R; // Lê o valor convertido
    ADC0_ISC_R = 0x8; // ACK: limpar o bit de conversão (SS3) em ADC RIS
    return valorConvertido;
}

void atualiza_velocidade(){
    if(velocidade_atual < velocidade_alvo){
        velocidade_atual++;
    }
    else if(velocidade_atual > velocidade_alvo){
        velocidade_atual--;
    }
    if (velocidade_atual < 0) {
        direcao = 0;
    } else {
        direcao = 1;
    }
    velocidade_modulo = (velocidade_atual < 0) ? -velocidade_atual : velocidade_atual;
}

void setDCMotorSpeed(int speed, int direction) {
    velocidade_alvo = speed;
    while (velocidade_atual != velocidade_alvo) {
        atualiza_velocidade();
        PWM_SetDuty(velocidade_modulo, direcao);
        SysTick_Wait1ms(100);
    }
    PWM_SetDuty(velocidade_modulo, direcao);
    SysTick_Wait1ms(30);
}

void ADC_Init(void) {
    SYSCTL_RCGCADC_R |= 0x1; // Ativa o clock para ADC0
    while (SYSCTL_PRADC_R & 0x1 == 0) {}; // Espera o ADC0 ficar pronto
    ADC0_PC_R = 0x226;
    ADC0_SSPRI_R = 0x3210; // Prioridade das SS
    ADC0_ACTSS_R &= ~0x8; // Desabilita o SS3 para configuração
    ADC0_EMUX_R = 0x0F; // trigger continuo
    // Seleciona o canal (bits 3:0) para a entrada analógica em PE4.
    ADC0_SSMUX3_R |= 0x9;
    // Configura o controle da amostra: para SS3 (apenas uma amostra)
    // habilitar IE0 e END0 -> 0110b = 0x6
    ADC0_SSCTL3_R = 0x6;

    // Habilitar interrupção do sequenciador 3 no ADC
    // ADC0_IM_R |= 0x8; // descomente se desejar usar interrupção

    // Habilitar o sequenciador 3 (ASEN3)
    ADC0_ACTSS_R |= 0x8;
}

#define MOTOR_EN_MASK  (1 << 2) // PF2

void Motor_Init(void)
{
    // GPIO_PORTF_AHB_DATA_R &= ~MOTOR_EN_MASK; // inicialmente desabilitado
    GPIO_PORTE_AHB_DATA_R &= ~0x03; // garantir pinos em 0
}

void Motor_Enable(void)
{
    // GPIO_PORTF_AHB_DATA_R |= MOTOR_EN_MASK; // PF2 = 1 -> habilita L293
}

// -------------------------------------------------------------------------------
// Função GPIO_Init
// Inicializa os ports J, N (para usuário) e K, M (para o LCD)
// Parâmetro de entrada: Não tem
// Parâmetro de saída: Não tem
void GPIO_Init(void)
{
    // 1a. Ativar o clock para a porta setando o bit correspondente no registrador
    // RCGCGPIO
    SYSCTL_RCGCGPIO_R =
        (GPIO_PORTA | GPIO_PORTE | GPIO_PORTJ | GPIO_PORTK | GPIO_PORTL | GPIO_PORTM |
         GPIO_PORTN | GPIO_PORTH | GPIO_PORTQ | GPIO_PORTP);
    // 1b.   ap�s isso verificar no PRGPIO se a porta est� pronta para uso.
        while ((SYSCTL_PRGPIO_R &
            (GPIO_PORTA | GPIO_PORTE | GPIO_PORTJ | GPIO_PORTK | GPIO_PORTL | GPIO_PORTM |
             GPIO_PORTN | GPIO_PORTH | GPIO_PORTQ | GPIO_PORTP)) !=
           (GPIO_PORTA | GPIO_PORTE | GPIO_PORTJ | GPIO_PORTK | GPIO_PORTL | GPIO_PORTM |
            GPIO_PORTN | GPIO_PORTH | GPIO_PORTQ | GPIO_PORTP))
    {
    };
    SYSCTL_RCGCTIMER_R = 0x1;
    while ((SYSCTL_PRTIMER_R & (0x1)) != (0x1))
        ;

    // Limpar o AMSEL para desabilitar a anal�gica
    GPIO_PORTJ_AHB_AMSEL_R = 0x00;
    GPIO_PORTN_AMSEL_R = 0x00;
    GPIO_PORTL_AMSEL_R = 0x00;
    GPIO_PORTM_AMSEL_R = 0x00;
    GPIO_PORTA_AHB_AMSEL_R = 0x00;
    GPIO_PORTQ_AMSEL_R = 0x00;
    GPIO_PORTK_AMSEL_R = 0x00;
    GPIO_PORTH_AHB_AMSEL_R = 0x00;
    GPIO_PORTP_AMSEL_R = 0x00;
    GPIO_PORTE_AHB_AMSEL_R = 0x00; // desabilita analógico em PORTE

    // Limpar PCTL para selecionar o GPIO
    GPIO_PORTJ_AHB_PCTL_R = 0x00;
    GPIO_PORTN_PCTL_R = 0x00;
    GPIO_PORTL_PCTL_R = 0x00;
    GPIO_PORTM_PCTL_R = 0x00;
    GPIO_PORTA_AHB_PCTL_R = 0x00;
    GPIO_PORTQ_PCTL_R = 0x00;
    GPIO_PORTK_PCTL_R = 0x00;
    GPIO_PORTP_PCTL_R = 0x00;
    GPIO_PORTH_AHB_PCTL_R = 0x00;
    GPIO_PORTE_AHB_PCTL_R = 0x00;

    // DIR para 0 se for entrada, 1 se for sa�da
    GPIO_PORTJ_AHB_DIR_R = 0x00;
    GPIO_PORTL_DIR_R = 0x00;
    GPIO_PORTM_DIR_R = 0x07;
    GPIO_PORTN_DIR_R = 0x03;
    GPIO_PORTA_AHB_DIR_R = 0xF0; // BIT4 AO BIT7
    GPIO_PORTQ_DIR_R = 0x0F;     // BIT0, 1, 2, 3
    GPIO_PORTK_DIR_R = 0xFF;
    GPIO_PORTP_DIR_R = 0x20;
    GPIO_PORTH_AHB_DIR_R = 0x0F;
    GPIO_PORTE_AHB_DIR_R = 0x03; // PE0 e PE1 como saída
    // GPIO_PORTF_AHB_DIR_R = 0x04; // PF2 saída

    // Limpar os bits AFSEL para 0 para selecionar GPIO sem fun��o alternativa
    GPIO_PORTJ_AHB_AFSEL_R = 0x00;
    GPIO_PORTL_AFSEL_R = 0x00;
    GPIO_PORTM_AFSEL_R = 0x00;
    GPIO_PORTN_AFSEL_R = 0x00;
    GPIO_PORTA_AHB_AFSEL_R = 0x00;
    GPIO_PORTQ_AFSEL_R = 0x00;
    GPIO_PORTK_AFSEL_R = 0x00;
    GPIO_PORTP_AFSEL_R = 0x00;
    GPIO_PORTH_AHB_AFSEL_R = 0x00;
    GPIO_PORTE_AHB_AFSEL_R = 0x00;

    // Setar os bits de DEN para habilitar I/O digital
    GPIO_PORTJ_AHB_DEN_R = 0x03; // Bit0 e bit1
    GPIO_PORTL_DEN_R = 0x0F;     // Bit0, 1, 2, 3
    GPIO_PORTM_DEN_R = 0xF7;     //
    GPIO_PORTN_DEN_R = 0x03;     // Bit0 e bit1
    GPIO_PORTA_AHB_DEN_R = 0xF3; // BIT4 AO BIT7
    GPIO_PORTQ_DEN_R = 0x0F;     // BIT0, 1, 2, 3
    GPIO_PORTK_DEN_R = 0xFF;     // todos
    GPIO_PORTP_DEN_R = 0x20;     // todos
    GPIO_PORTH_AHB_DEN_R = 0x0F; // BIT4 AO BIT7
    GPIO_PORTE_AHB_DEN_R = 0x03; // habilita PE0 e PE1
    // GPIO_PORTF_AHB_DEN_R = 0x04; // PF2 digital enable

    // Habilitar resistor de pull-up interno, setar PUR para 1
    GPIO_PORTJ_AHB_PUR_R = 0x03; // Bit0 e bit1
    GPIO_PORTL_PUR_R = 0x0F;     // Bit0, 1, 2, 3
    GPIO_PORTE_AHB_PUR_R = 0x00;
    GPIO_PORTJ_AHB_IM_R = 0x0;
    GPIO_PORTJ_AHB_IS_R = 0x0;
    GPIO_PORTJ_AHB_IBE_R = 0x0;
    GPIO_PORTJ_AHB_IEV_R = 0x0; // Falling edge
    GPIO_PORTJ_AHB_ICR_R = 0x03;
    GPIO_PORTJ_AHB_IM_R = 0x03;
    // Habilitar interrupção no NVIC para GPIOJ
    NVIC_PRI12_R = 0;
    NVIC_EN1_R = (1 << 19);
    // Inicializa o LCD após configurar todas as portas
    // initUART();
    // initLCD();

    initTimer();
}