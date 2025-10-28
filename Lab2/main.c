// main.c
// Desenvolvido para a placa EK-TM4C1294XL
// Verifica o estado das chaves USR_SW1 e USR_SW2, acende os LEDs 1 e 2 caso estejam pressionadas independentemente
// Caso as duas chaves estejam pressionadas ao mesmo tempo pisca os LEDs alternadamente a cada 500ms.
// Prof. Guilherme Peron

#include "main.h"

static char user_password[USER_PASS_LEN + 1] = "0000"; // inicial temporária
static const char master_password[] = "1234";

/* Flag setada pela ISR quando USR_SW1 é pressionada (falling edge) */
volatile int usr_sw1_event = 0;

int main(void)
{
	PLL_Init();
    SysTick_Init();
    GPIO_Init();

	uint32_t senha_mestra = 1234;

	CofreState state = STATE_ABERTO;
    int failed_attempts = 0;
    char input_buf[MAX_INPUT_LEN];
    int input_len = 0;

    while (1) {
        switch (state) {
            case STATE_ABERTO:
                resetLCD();
                lcd_puts("Cofre aberto");
                // Espera senha de 4 dígitos seguida de '#'
                collect_password(input_buf, &input_len, USER_PASS_LEN + 1);
                if (input_len == USER_PASS_LEN) {
                    // Armazena senha do usuário
                    memcpy(user_password, input_buf, USER_PASS_LEN);
                    user_password[USER_PASS_LEN] = '\0';
                    // Fecha o cofre
					state = STATE_FECHANDO;
                } else {
                    resetLCD();
                	lcd_puts("Tamanho invalido");
                    SysTick_Wait1ms(1000);
                }
                break;

            case STATE_FECHADO:
                resetLCD();
                lcd_puts("Cofre fechado");
                // Aguarda tentativa de abertura (senha seguida de '#')
                collect_password(input_buf, &input_len, USER_PASS_LEN + 1);
                if (input_len == USER_PASS_LEN) {
                    if (strncmp(input_buf, user_password, USER_PASS_LEN) == 0) {
                        // Senha correta -> abre
                        state = STATE_ABRINDO;
                    } else {
                        // Senha incorreta
                        failed_attempts++;
                        resetLCD();
                        lcd_puts("Senha incorreta");
                        SysTick_Wait1ms(1000);
                        if (failed_attempts >= 3) {
                            state = STATE_TRAVADO;
                        }
                    }
                } else {
					resetLCD();
                	lcd_puts("Tamanho invalido");
                    SysTick_Wait1ms(1000);
                }
                break;

            case STATE_TRAVADO:
                // Exibe mensagem e pisca LEDs até que USR_SW1 seja pressionada
                resetLCD();
                lcd_puts("Cofre Travado");
                // Loop de espera que pisca LEDs e checa USR_SW1
				AcenderTodosLEDs(); // pisca alternadamente
				if (usr_sw1_event) {
					usr_sw1_event = 0;
					
					// Quando USR_SW1 for detectada, pede senha mestra
					collect_password(input_buf, &input_len, USER_PASS_LEN + 1);
					if (input_len == USER_PASS_LEN) {
						if(strncmp(input_buf, master_password, USER_PASS_LEN) == 0) {
							// Mestre ok -> abrir e voltar a aberto
							stepper_open();
							state = STATE_ABRINDO;
							failed_attempts = 0;
						} else {
							resetLCD();
							lcd_puts("Mestre invalida");
							SysTick_Wait1ms(1000);
						}
					} else {
						resetLCD();
						lcd_puts("Tamanho invalido");
						SysTick_Wait1ms(1000);
					}
				}
                break;

            case STATE_FECHANDO:
                // Estado transitório (se precisar de algo específico)
				resetLCD();
				lcd_puts("Cofre fechando");
				stepper_close();
				state = STATE_FECHADO;
				failed_attempts = 0;
                break;

            case STATE_ABRINDO:
                // Estado transitório (se precisar de algo específico)
				resetLCD();
				lcd_puts("Cofre abrindo");
				stepper_open();
                state = STATE_ABERTO;
                break;

            default:
                state = STATE_ABERTO;
                break;
        } // switch

        // pequeno intervalo para evitar busy-loop extremo
        SysTick_Wait1ms(50);
    }
}

void Pisca_leds(void)
{
	// Aqui precisa ser os LEDs da PAT não da placa vermelha
	/* Cada LED mapeado em:
		PA7
		PA6
		PA5
		PA4
		PQ3
		PQ2
		PQ1
		PQ0
	*/
	// Precisa fazer o negocio com o transistor lá usando a porta PP5

	PortN_Output(0x2);
	SysTick_Wait1ms(250);
	PortN_Output(0x1);
	SysTick_Wait1ms(250);
}

/* Coleta uma senha até '#' ser pressionado.
   - buf: buffer de saída
   - len: saída com comprimento (sem o terminador)
   - maxlen: tamanho máximo (inclui espaço para terminador, e.g. USER_PASS_LEN+1)
   Observação: bloqueante até '#' ser pressionado. Aceita tecla '*' para limpar. */
void collect_password(char *buf, int *len, int maxlen) {
    int idx = 0;
    *len = 0;
	char k;
    // indica visualmente entrada opcionalmente
    while (1) {
		k = '\0';
        k = Keypad_Scan();
        if (k == '*') {
            // limpar
            idx = 0;
            *len = 0;
            continue;
        }
        if (k == '#') {
            if (idx >= 0 && idx < maxlen) 
				buf[idx] = '\0';
            else 
				buf[maxlen - 1] = '\0';
            *len = idx;
            return;
        }
        // aceita apenas dígitos e caracteres aceitos (A-D,*,# também podem aparecer)
        if (k != '\0' && idx < maxlen - 1) {
            buf[idx++] = k;
            SysTick_Wait1ms(200);
        }
    }
}

/* Simulações simples do motor de passo (apenas delays e mensagem) */
void stepper_close(void) {
    // aguarda 1s antes de girar (conforme enunciado)
    SysTick_Wait1ms(1000);
    // simula tempo de 2 voltas no sentido anti-horário no modo meio passo
    SysTick_Wait1ms(2000);
}

void stepper_open(void) {
	// simula tempo de 2 voltas no sentido horário no modo passo completo
    SysTick_Wait1ms(2000);
}

/* Leitura simples da chave USR_SW1: Port J bit 0 (pull-up, 0 quando pressionado) */
int usr_sw1_pressed(void) {
    return (PortJ_Input() & 0x01) == 0;
}