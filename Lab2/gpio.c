// gpio.c
// Desenvolvido para a placa EK-TM4C1294XL
// Inicializa as portas J, N, K e M
// Prof. Guilherme Peron


#include <stdint.h>

#include "tm4c1294ncpdt.h"

 
#define GPIO_PORTJ  (0x0100) //bit 8
#define GPIO_PORTK  (0x0200) //bit 9
#define GPIO_PORTM  (0x0800) //bit 11
#define GPIO_PORTN  (0x1000) //bit 12

void SysTick_Wait1ms(uint32_t delay);
void SysTick_Wait1ms(uint32_t delay);

// -------------------------------------------------------------------------------
// Função PortJ_Input
// Lê os valores de entrada do port J
// Parâmetro de entrada: Não tem
// Parâmetro de saída: o valor da leitura do port
uint32_t PortJ_Input(void)
{
    return GPIO_PORTJ_AHB_DATA_R;
}

// -------------------------------------------------------------------------------
// Função PortN_Output
// Escreve os valores no port N
// Parâmetro de entrada: Valor a ser escrito
// Parâmetro de saída: não tem
void PortN_Output(uint32_t valor)
{
    uint32_t temp;
    //vamos zerar somente os bits menos significativos
    //para uma escrita amigável nos bits 0 e 1
    temp = GPIO_PORTN_DATA_R & 0xFC;
    //agora vamos fazer o OR com o valor recebido na função
    temp = temp | valor;
    GPIO_PORTN_DATA_R = temp; 
}

// --- Funções do LCD ---

/**
 * Envia um comando para o LCD.
 * - Barramento de dados 8-bit no GPIO_PORTK_DATA_R (PK0-PK7)
 * - RS = PM0
 * - R/W = PM1 (sempre 0)
 * - E  = PM2
 */
void lcd_command(uint8_t command) {
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

void resetLCD() {
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
void lcd_data(uint8_t data) {
    
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
void lcd_puts(char *s) {
    while (*s) {
        lcd_data(*s);
        s++;
    }
}

void initLCD() {
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


// -------------------------------------------------------------------------------
// Função GPIO_Init
// Inicializa os ports J, N (para usuário) e K, M (para o LCD)
// Parâmetro de entrada: Não tem
// Parâmetro de saída: Não tem
void GPIO_Init(void)
{
    // 1a. Ativar o clock para as portas J, N, K e M
    SYSCTL_RCGCGPIO_R = (GPIO_PORTJ | GPIO_PORTN | GPIO_PORTK | GPIO_PORTM);
    // 1b. verificar no PRGPIO se as portas estão prontas
  while((SYSCTL_PRGPIO_R & (GPIO_PORTJ | GPIO_PORTN | GPIO_PORTK | GPIO_PORTM) ) != (GPIO_PORTJ | GPIO_PORTN | GPIO_PORTK | GPIO_PORTM) ){};
    
    // --- Configuração Port J (Conforme original) ---
    GPIO_PORTJ_AHB_AMSEL_R = 0x00;
    GPIO_PORTJ_AHB_PCTL_R = 0x00;
    GPIO_PORTJ_AHB_DIR_R = 0x00;      // Entrada
    GPIO_PORTJ_AHB_AFSEL_R = 0x00;
    GPIO_PORTJ_AHB_DEN_R = 0x03;      // Bit0 e bit1
    GPIO_PORTJ_AHB_PUR_R = 0x03;      // Bit0 e bit1 
    
    // --- Configuração Port N (Conforme original) ---
    GPIO_PORTN_AMSEL_R = 0x00;
    GPIO_PORTN_PCTL_R = 0x00;
    GPIO_PORTN_DIR_R = 0x03;          // Saída (BIT0 | BIT1)
    GPIO_PORTN_AFSEL_R = 0x00; 
    GPIO_PORTN_DEN_R = 0x03;          // Bit0 e bit1

    // --- (NOVO) Configuração Port K (Barramento de Dados LCD D0-D7) ---
    GPIO_PORTK_AMSEL_R = 0x00;        // 2. Desabilita analógica
    GPIO_PORTK_PCTL_R = 0x00;         // 3. Limpa PCTL (GPIO)
    GPIO_PORTK_DIR_R = 0xFF;          // 4. PK0-PK7 como Saída (0b11111111)
    GPIO_PORTK_AFSEL_R = 0x00;        // 5. Desabilita função alternativa
    GPIO_PORTK_DEN_R = 0xFF;          // 6. Habilita digital (PK0-PK7)
    
    // --- (NOVO) Configuração Port M (Controle LCD RS, R/W, EN) ---
    GPIO_PORTM_AMSEL_R = 0x00;        // 2. Desabilita analógica
    GPIO_PORTM_PCTL_R = 0x00;         // 3. Limpa PCTL (GPIO)
    GPIO_PORTM_DIR_R = 0x07;          // 4. PM0, PM1, PM2 como Saída (0b00000111)
    GPIO_PORTM_AFSEL_R = 0x00;        // 5. Desabilita função alternativa
    GPIO_PORTM_DEN_R = 0x07;          // 6. Habilita digital (PM0, PM1, PM2)
    
    // Seta R/W (PM1) para 0 (modo escrita) permanentemente
    GPIO_PORTM_DATA_R &= ~0x02; 

    // Inicializa o LCD após configurar todas as portas
    initLCD();
}