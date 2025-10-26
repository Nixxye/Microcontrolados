// main.c
// Desenvolvido para a placa EK-TM4C1294XL
// Verifica o estado das chaves USR_SW1 e USR_SW2, acende os LEDs 1 e 2 caso estejam pressionadas independentemente
// Caso as duas chaves estejam pressionadas ao mesmo tempo pisca os LEDs alternadamente a cada 500ms.
// Prof. Guilherme Peron

#include <stdint.h>

void PLL_Init(void);
void SysTick_Init(void);
void SysTick_Wait1ms(uint32_t delay);
void SysTick_Wait1us(uint32_t delay);
void GPIO_Init(void);
uint32_t PortJ_Input(void);
void PortN_Output(uint32_t leds);
void Pisca_leds(void);
void lcd_data(uint8_t data);
void lcd_puts(char *s);
void resetLCD();
char Keypad_Scan(void);

int main(void)
{
	PLL_Init();
	SysTick_Init();
	GPIO_Init();

	uint32_t senha_mestra = 1234;
	char tecla = '\0';

	while(1) {
		resetLCD();
		// Quando o sistema iniciar, o cofre devera estar aberto, sendo indicado pelo LCD "Cofre aberto".
		lcd_puts("Cofre aberto");
		// Se o usuario quiser fechar o cofre, basta ele digitar no teclado uma senha de 4 digitos e em seguida o "#""; 
		// Assim que a "#" for pressionada, o cofre deve esperar por 1 segundo e, em seguida, o motor de passo deve girar 2 voltas no sentido anti-horario no modo meio passo. Neste momento, o  LCD deve indicar a mensagem "Cofre fechando";
		lcd_puts("Cofre fechando");
		// Assim que o cofre for fechado e em todo momento em que estiver fechado, o LCD deve mostrar a mensagem 'Cofre fechado'. Sendo aceita somente a senha previamente cadastrada para abrir o cofre;
		lcd_puts("Cofre fechado");
		// Se a senha for digitada corretamente, o motor de passo deve girar 2 voltas no sentido horario no modo passo completo. Neste momento, o  LCD deve indicar a mensagem 'Cofre abrindo'; 
		lcd_puts("Cofre abrindo");
		// Se, enquanto o cofre estiver fechado, a senha for digitada incorretamente por 3 vezes, o cofre travara. Os LEDs da PAT devem ficar piscando e o LCD deve apresentar a mensagem "Cofre Travado". Neste caso, o cofre so podera ser aberto pressionando a chave USR_SW1 acionada por interrupcao de GPIO. Em seguida a senha mestra devera ser requisitada. A senha mestra so podera ser digitada se a chave USR_SW1 for pressionada.
		lcd_puts("Cofre Travado");

		// Assim que o cofre abrir completamente, voltar ao passo 1, ou seja, o LCD deve indicar a mensagem 'Cofre aberto'.
		// O sistema deve ter uma senha mestra que sera inicializada como 1234, para permitir a abertura do cofre em caso de travamento.
		// Se a senha mestra for digitada corretamente, os 8 LEDs da PAT devem parar de piscar, e, em seguida, o cofre deve ser aberto, indicado pelo LCD como do passo 5.

		// Utilizar o algoritmo de varredura para realizar a leitura das teclas do teclado matricial. Para colocar um pino em alta impedancia atribuir o respectivo bit do GPIO_DIR para entrada. A cada troca de entrada para saida e saida para entrada, esperar no minimo 1 ms. 
		
		tecla = Keypad_Scan();
		if (tecla != '\0') {
			lcd_data(tecla);
			tecla = '\0';
		}
		SysTick_Wait1ms(1000);
	}
}

void Pisca_leds(void)
{
	PortN_Output(0x2);
	SysTick_Wait1ms(250);
	PortN_Output(0x1);
	SysTick_Wait1ms(250);
}
