; main.s
; Desenvolvido para a placa EK-TM4C1294XL
; Prof. Guilherme Peron
; 15/03/2018
; Este programa espera o usu�rio apertar a chave USR_SW1 e/ou a chave USR_SW2.
; Caso o usu�rio pressione a chave USR_SW1, acender� o LED2. Caso o usu�rio pressione 
; a chave USR_SW2, acender� o LED1. Caso as duas chaves sejam pressionadas, os dois 
; LEDs acendem.

; -------------------------------------------------------------------------------
        THUMB                        ; Instru��es do tipo Thumb-2
; -------------------------------------------------------------------------------

; -------------------------------------------------------------------------------
; �rea de Dados - Declara��es de vari�veis
		AREA  DATA, ALIGN=2
		; Se alguma vari�vel for chamada em outro arquivo
		;EXPORT  <var> [DATA,SIZE=<tam>]   ; Permite chamar a vari�vel <var> a 
		                                   ; partir de outro arquivo
;<var>	SPACE <tam>                        ; Declara uma vari�vel de nome <var>
                                           ; de <tam> bytes a partir da primeira 
                                           ; posi��o da RAM		

; -------------------------------------------------------------------------------
; �rea de C�digo - Tudo abaixo da diretiva a seguir ser� armazenado na mem�ria de 
;                  c�digo
        AREA    |.text|, CODE, READONLY, ALIGN=2

		; Se alguma fun��o do arquivo for chamada em outro arquivo	
        EXPORT Start                ; Permite chamar a fun��o Start a partir de 
                       ; outro arquivo. No caso startup.s
									
		; Se chamar alguma fun��o externa	
        ;IMPORT <func>              ; Permite chamar dentro deste arquivo uma 
									; fun��o <func>
		IMPORT PLL_Init
		IMPORT SysTick_Init
		IMPORT SysTick_Wait1ms										
		IMPORT GPIO_Init
        IMPORT PortN_Output
		IMPORT PortA_Output
		IMPORT PortQ_Output
		IMPORT PortB_Output
		IMPORT PortP_Output
		IMPORT GPIO_PORTA_DATA_R
		IMPORT GPIO_PORTB_DATA_R
		IMPORT GPIO_PORTQ_DATA_R

; -------------------------------------------------------------------------------
; Fun��o main()
Start  		
	BL PLL_Init                  ;Chama a subrotina para alterar o clock do microcontrolador para 80MHz
	BL SysTick_Init              ;Chama a subrotina para inicializar o SysTick
	BL GPIO_Init                 ;Chama a subrotina que inicializa os GPIO
	MOV R10, #15 ; Temperatura atual				
	MOV R11, #25 ; Temperatura alvo
	LDR R6, =GPIO_PORTB_DATA_R
	LDR R7, [R6]
	; Setando bit para 1
	LDR R8, =2_10000
	STR R8, [R6]
MainLoop
	BL ValorDisplay
	B MainLoop
	
ValorDisplay
	; Recebe um número no R10 e mostra nos displays de 7 segmentos
	MOV R1, #10
	UDIV R0, R10, R1 ; R0 = R10 / 10

	MLS R1, R0, R1, R10 ; R1 = R10 - R0 * 10
	; R0 = dezena, R1 = unidade
	BL LigaDisplayDezena

LigaDisplayDezena
	; Liga o display da dezena
	PUSH {R0,R1}
	BL SysTick_Wait1ms
	POP {R0,R1}

	MOV R3, #2_1111
	LDR R4, =GPIO_PORTA_DATA_R
	LDR R4, [R4]
	AND R4, R4, R3 ; Apaga todos os bits do display e mantém os 4 primeiros

	LDR R5, =GPIO_PORTQ_DATA_R
	LDR R5, [R5]
	BIC R5, R5, R3 ; Mantém todos os bits do display e apaga os 4 primeiros

	CMP R0, #0
	BEQ Acende0
    CMP R0, #1
    BEQ Acende1
    CMP R0, #2
    BEQ Acende2
    CMP R0, #3
    BEQ Acende3
    CMP R0, #4
    BEQ Acende4
    CMP R0, #5
    BEQ Acende5
    CMP R0, #6
    BEQ Acende6
    CMP R0, #7
    BEQ Acende7
    CMP R0, #8
    BEQ Acende8
    CMP R0, #9
    BEQ Acende9
    B FimAcendeDezena 

FimAcendeDezena
	LDR R6, =GPIO_PORTA_DATA_R
	LDR R7, =GPIO_PORTQ_DATA_R
	STR R4, [R6]
	STR R5, [R7]
	
	; Atualiza transistor
	LDR R6, =GPIO_PORTB_DATA_R
	LDR R7, [R6]
	; Setando bit para 1
	;LDR R8, =2_10000
	ORR R7, R7, R8
	;STR R7, [R6]

	BL SysTick_Wait1ms
	; Setando bit para 0
	BIC R7, R7, R8
	;STR R7, [R6]

	BL SysTick_Wait1ms

	B MainLoop

Acende0
	MOV R6, #2_110000 ; PA
	MOV R7, #2_1111   ; PQ
	ORR R4, R4, R6
	ORR R5, R5, R7
	B FimAcendeDezena

Acende1     ; Acende segmentos: b, c
    MOV R6, #2_0000000  ; PA (gfe = 000)
    MOV R7, #2_0110     ; PQ (dcba = 0110)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

Acende2     ; Acende segmentos: a, b, d, e, g
    MOV R6, #2_1010000  ; PA (gfe = 101)
    MOV R7, #2_1011     ; PQ (dcba = 1011)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

Acende3     ; Acende segmentos: a, b, c, d, g
    MOV R6, #2_1000000  ; PA (gfe = 100)
    MOV R7, #2_1111     ; PQ (dcba = 1111)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

Acende4     ; Acende segmentos: b, c, f, g
    MOV R6, #2_1100000  ; PA (gfe = 110)
    MOV R7, #2_0110     ; PQ (dcba = 0110)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

Acende5     ; Acende segmentos: a, c, d, f, g
    MOV R6, #2_1100000  ; PA (gfe = 110)
    MOV R7, #2_1101     ; PQ (dcba = 1101)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

Acende6     ; Acende segmentos: a, c, d, e, f, g
    MOV R6, #2_1110000  ; PA (gfe = 111)
    MOV R7, #2_1101     ; PQ (dcba = 1101)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

Acende7     ; Acende segmentos: a, b, c
    MOV R6, #2_0000000  ; PA (gfe = 000)
    MOV R7, #2_0111     ; PQ (dcba = 0111)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

Acende8     ; Acende segmentos: a, b, c, d, e, f, g
    MOV R6, #2_1110000  ; PA (gfe = 111)
    MOV R7, #2_1111     ; PQ (dcba = 1111)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

Acende9     ; Acende segmentos: a, b, c, d, f, g
    MOV R6, #2_1100000  ; PA (gfe = 110)
    MOV R7, #2_1111     ; PQ (dcba = 1111)
    ORR R4, R4, R6
    ORR R5, R5, R7
    B FimAcendeDezena

LigaDisplayUnidade
	; Liga o display da unidade

    ALIGN                        ;Garante que o fim da se��o est� alinhada 
    END                          ;Fim do arquivo