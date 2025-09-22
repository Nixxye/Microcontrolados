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

; -------------------------------------------------------------------------------
; Fun��o main()
Start  		
	BL PLL_Init                  ;Chama a subrotina para alterar o clock do microcontrolador para 80MHz
	BL SysTick_Init              ;Chama a subrotina para inicializar o SysTick
	BL GPIO_Init                 ;Chama a subrotina que inicializa os GPIO
	MOV R10, #15 ; Temperatura atual				
	MOV R11, #25 ; Temperatura alvo
    MOV R6, #0 ; contador de tempo
	B MainLoop

Acende0Display  ; Acende segmentos: a, b, c, d, e, f
	MOV R9, #2_110000 ; PA
    BL PortA_Output
	MOV R9, #2_1111   ; PQ
    BL PortQ_Output
    B FimAcende

Acende1Display  ; Acende segmentos: b, c
    MOV R9, #2_0000000  ; PA (gfe = 000)
    BL PortA_Output
    MOV R9, #2_0110     ; PQ (dcba = 0110)
    BL PortQ_Output
    B FimAcende

Acende2Display  ; Acende segmentos: a, b, d, e, g
    MOV R9, #2_1010000  ; PA (gfe = 101)
    BL PortA_Output
    MOV R9, #2_1011     ; PQ (dcba = 1011)
    BL PortQ_Output
    B FimAcende

Acende3Display  ; Acende segmentos: a, b, c, d, g
    MOV R9, #2_1000000  ; PA (gfe = 100)
    BL PortA_Output
    MOV R9, #2_1111     ; PQ (dcba = 1111)
    BL PortQ_Output
    B FimAcende

Acende4Display  ; Acende segmentos: b, c, f, g
    MOV R9, #2_1100000  ; PA (gfe = 110)
	BL PortA_Output
    MOV R9, #2_0110     ; PQ (dcba = 0110)
    BL PortQ_Output
    B FimAcende

Acende5Display  ; Acende segmentos: a, c, d, f, g
    MOV R9, #2_1100000  ; PA (gfe = 110)
    BL PortA_Output
    MOV R9, #2_1101     ; PQ (dcba = 1101)
    BL PortQ_Output
    B FimAcende

Acende6Display  ; Acende segmentos: a, c, d, e, f, g
    MOV R9, #2_1110000  ; PA (gfe = 111)
    BL PortA_Output
    MOV R9, #2_1101     ; PQ (dcba = 1101)
    BL PortQ_Output
    B FimAcende

Acende7Display  ; Acende segmentos: a, b, c
    MOV R9, #2_0000000  ; PA (gfe = 000)
    BL PortA_Output
    MOV R9, #2_0111     ; PQ (dcba = 0111)
    BL PortQ_Output
    B FimAcende

Acende8Display  ; Acende segmentos: a, b, c, d, e, f, g
    MOV R9, #2_1110000  ; PA (gfe = 111)
    BL PortA_Output
    MOV R9, #2_1111     ; PQ (dcba = 1111)
    BL PortQ_Output
    B FimAcende

Acende9Display  ; Acende segmentos: a, b, c, d, f, g
    MOV R9, #2_1100000  ; PA (gfe = 110)
    BL PortA_Output
    MOV R9, #2_1111     ; PQ (dcba = 1111)
    BL PortQ_Output
    B FimAcende

MainLoop
	; Recebe um número no R10 e mostra nos displays de 7 segmentos
	MOV R1, #10
	UDIV R0, R10, R1 ; R0 = R10 / 10

	MLS R1, R0, R1, R10 ; R1 = R10 - R0 * 10
	; R0 = dezena, R1 = unidade

    ADD R6, R6, #1
    CMP R6, #400
    BLT  PulaTempo
    ; A cada 1 segundo ajusta a temperatura
    MOV R6, #0
    CMP R10, R11
    BLT aumenta
    BGT diminui
    ; igual
    MOV R9, #2_11
    PUSH {LR}
    BL PortN_Output
    POP {LR}
    B PulaTempo
diminui
    MOV R9, #2_10
    PUSH {LR}
    BL PortN_Output
    POP {LR}
    SUB R10, R10, #1
    B PulaTempo
aumenta
    MOV R9, #2_1
    PUSH {LR}
    BL PortN_Output
    POP {LR}
    ADD R10, R10, #1
PulaTempo
    

AcendeDisplay

    CMP R8, #1
    BEQ unidade
    BGT acendeLEDs
    MOV R7, R0
    B dezena
unidade
    MOV R7, R1
dezena

	CMP R7, #0
	BEQ Acende0Display
    CMP R7, #1
    BEQ Acende1Display
    CMP R7, #2
    BEQ Acende2Display
    CMP R7, #3
    BEQ Acende3Display
    CMP R7, #4
    BEQ Acende4Display
    CMP R7, #5
    BEQ Acende5Display
    CMP R7, #6
    BEQ Acende6Display
    CMP R7, #7
    BEQ Acende7Display
    CMP R7, #8
    BEQ Acende8Display
    CMP R7, #9
    BEQ Acende9Display

acendeLEDs
    BIC R9, R11, #2_00001111
    BL PortA_Output
    AND R9, R11, #2_00001111
    BL PortQ_Output
    B FimAcende

FimAcende 
    ; Verifica se a unidade ou a dezena deve ser acesa
    CMP R8, #1
    BEQ FimAcendeUnidade
    BLT FimAcendeDezena
    B FimAcendeLEDs

FimAcendeDezena
	; Liga transistor da dezena (Q2)
    BL HabilitarDisplayDezena

	PUSH {R0}
    MOV R0, #1
    BL SysTick_Wait1ms
    POP {R0}
    
    ; Desliga transistor da dezena (Q2)
    BL DesabilitarDisplayDezena
    
	PUSH {R0}
    MOV R0, #1
    BL SysTick_Wait1ms
    POP {R0}

    ADD R8, #1

	B MainLoop

FimAcendeUnidade
	; Liga transistor da unidade (Q1)
	BL HabilitarDisplayUnidade

	PUSH {R0}
    MOV R0, #1
    BL SysTick_Wait1ms
    POP {R0}

    ; Desliga transistor da unidade (Q1)
    BL DesabilitarDisplayUnidade
    
	PUSH {R0}
    MOV R0, #1
    BL SysTick_Wait1ms
    POP {R0}

    ADD R8, #1

	B MainLoop

FimAcendeLEDs
	; Liga transistor da unidade (Q0)
	BL HabilitarLEDs

	PUSH {R0}
    MOV R0, #1
    BL SysTick_Wait1ms
    POP {R0}

    ; Desliga transistor da unidade (Q0)
    BL DesabilitarLEDs
    
	PUSH {R0}
    MOV R0, #1
    BL SysTick_Wait1ms
    POP {R0}
    
    MOV R8, #0

	B MainLoop

HabilitarDisplayUnidade
    ; Atualiza transistor
    MOV R9, #2_00100000 ; PB
    PUSH { LR }
    BL PortB_Output
    POP { LR }
    BX LR

HabilitarDisplayDezena
    ; Atualiza transistor
    MOV R9, #2_00010000 ; PB
    PUSH { LR }
    BL PortB_Output
    POP { LR }
    BX LR

DesabilitarDisplayUnidade
    ; Atualiza transistor
    MOV R9, #2_11001111 ; PB
    PUSH { LR }
    BL PortB_Output
    POP { LR }
    BX LR

DesabilitarDisplayDezena
    ; Atualiza transistor
    MOV R9, #2_11001111 ; PB
    PUSH { LR }
    BL PortB_Output
    POP { LR }
    BX LR

HabilitarLEDs
    ; Atualiza transistor
    MOV R9, #2_00100000 ; PB
    PUSH { LR }
    BL PortP_Output
    POP { LR }
    BX LR

DesabilitarLEDs
    ; Atualiza transistor
    MOV R9, #2_11011111 ; PB
    PUSH { LR }
    BL PortP_Output
    POP { LR }
    BX LR

    ALIGN                        ;Garante que o fim da se��o est� alinhada 
    END                          ;Fim do arquivo