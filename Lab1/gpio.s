; gpio.s
; Desenvolvido para a placa EK-TM4C1294XL
; Prof. Guilherme Peron
; 19/03/2018

; -------------------------------------------------------------------------------
        THUMB                        ; Instru��es do tipo Thumb-2
; -------------------------------------------------------------------------------
; Declara��es EQU - Defines
; ========================
; Defini��es dos Registradores Gerais
SYSCTL_RCGCGPIO_R	 EQU	0x400FE608
SYSCTL_PRGPIO_R		 EQU    0x400FEA08
    
; ========================
; NVIC
NVIC_EN2_R           EQU    0xE000E108
NVIC_EN1_R			 EQU	0xE000E104
NVIC_PRI18_R		 EQU    0xE000E448     
NVIC_PRI12_R		 EQU	0xE000E430
; ========================
; Defini��es dos Ports

; PORT K
GPIO_PORTK_IS_R      	EQU    0x40061404
GPIO_PORTK_IBE_R      	EQU    0x40061408
GPIO_PORTK_IEV_R      	EQU    0x4006140C
GPIO_PORTK_IM_R      	EQU    0x40061410
GPIO_PORTK_RIS_R      	EQU    0x40061414
GPIO_PORTK_ICR_R      	EQU    0x4006141C    
GPIO_PORTK_LOCK_R    	EQU    0x40061520
GPIO_PORTK_CR_R      	EQU    0x40061524
GPIO_PORTK_AMSEL_R   	EQU    0x40061528
GPIO_PORTK_PCTL_R    	EQU    0x4006152C
GPIO_PORTK_DIR_R     	EQU    0x40061400
GPIO_PORTK_AFSEL_R   	EQU    0x40061420
GPIO_PORTK_DEN_R     	EQU    0x4006151C
GPIO_PORTK_PUR_R     	EQU    0x40061510	
GPIO_PORTK_DATA_R    	EQU    0x400613FC
GPIO_PORTK              EQU    2_00001000000000

; PORT M
GPIO_PORTM_IS_R      	EQU    0x40063404
GPIO_PORTM_IBE_R      	EQU    0x40063408
GPIO_PORTM_IEV_R      	EQU    0x4006340C
GPIO_PORTM_IM_R      	EQU    0x40063410
GPIO_PORTM_RIS_R      	EQU    0x40063414
GPIO_PORTM_ICR_R      	EQU    0x4006341C    
GPIO_PORTM_LOCK_R    	EQU    0x40063520
GPIO_PORTM_CR_R      	EQU    0x40063524
GPIO_PORTM_AMSEL_R   	EQU    0x40063528
GPIO_PORTM_PCTL_R    	EQU    0x4006352C
GPIO_PORTM_DIR_R     	EQU    0x40063400
GPIO_PORTM_AFSEL_R   	EQU    0x40063420
GPIO_PORTM_DEN_R     	EQU    0x4006351C
GPIO_PORTM_PUR_R     	EQU    0x40063510	
GPIO_PORTM_DATA_R    	EQU    0x400633FC
GPIO_PORTM              EQU    2_00100000000000

; PORT J
GPIO_PORTJ_AHB_LOCK_R    	EQU    0x40060520
GPIO_PORTJ_AHB_CR_R      	EQU    0x40060524
GPIO_PORTJ_AHB_AMSEL_R   	EQU    0x40060528
GPIO_PORTJ_AHB_PCTL_R    	EQU    0x4006052C
GPIO_PORTJ_AHB_DIR_R     	EQU    0x40060400
GPIO_PORTJ_AHB_AFSEL_R   	EQU    0x40060420
GPIO_PORTJ_AHB_DEN_R     	EQU    0x4006051C
GPIO_PORTJ_AHB_PUR_R     	EQU    0x40060510	
GPIO_PORTJ_AHB_DATA_R    	EQU    0x400603FC
GPIO_PORTJ_AHB_IM_R			EQU	   0x40060410
GPIO_PORTJ_AHB_IS_R			EQU    0x40060404
GPIO_PORTJ_AHB_IBE_R		EQU    0x40060408
GPIO_PORTJ_AHB_IEV_R		EQU    0x4006040C
GPIO_PORTJ_AHB_ICR_R		EQU    0x4006041C
GPIO_PORTJ_AHB_RIS_R		EQU	   0x40060414
GPIO_PORTJ               	EQU    2_000000100000000

; PORT N
GPIO_PORTN_DATA_BITS_R  EQU 0x40064000
GPIO_PORTN_DATA_R       EQU 0x400643FC
GPIO_PORTN_DIR_R        EQU 0x40064400
GPIO_PORTN_IS_R         EQU 0x40064404
GPIO_PORTN_IBE_R        EQU 0x40064408
GPIO_PORTN_IEV_R        EQU 0x4006440C
GPIO_PORTN_IM_R         EQU 0x40064410
GPIO_PORTN_RIS_R        EQU 0x40064414
GPIO_PORTN_MIS_R        EQU 0x40064418
GPIO_PORTN_ICR_R        EQU 0x4006441C
GPIO_PORTN_AFSEL_R      EQU 0x40064420
GPIO_PORTN_DR2R_R       EQU 0x40064500
GPIO_PORTN_DR4R_R       EQU 0x40064504
GPIO_PORTN_DR8R_R       EQU 0x40064508
GPIO_PORTN_ODR_R        EQU 0x4006450C
GPIO_PORTN_PUR_R        EQU 0x40064510
GPIO_PORTN_PDR_R        EQU 0x40064514
GPIO_PORTN_SLR_R        EQU 0x40064518
GPIO_PORTN_DEN_R        EQU 0x4006451C
GPIO_PORTN_LOCK_R       EQU 0x40064520
GPIO_PORTN_CR_R         EQU 0x40064524
GPIO_PORTN_AMSEL_R      EQU 0x40064528
GPIO_PORTN_PCTL_R       EQU 0x4006452C
GPIO_PORTN_ADCCTL_R     EQU 0x40064530
GPIO_PORTN_DMACTL_R     EQU 0x40064534
GPIO_PORTN_SI_R         EQU 0x40064538
GPIO_PORTN_DR12R_R      EQU 0x4006453C
GPIO_PORTN_WAKEPEN_R    EQU 0x40064540
GPIO_PORTN_WAKELVL_R    EQU 0x40064544
GPIO_PORTN_WAKESTAT_R   EQU 0x40064548
GPIO_PORTN_PP_R         EQU 0x40064FC0
GPIO_PORTN_PC_R         EQU 0x40064FC4
GPIO_PORTN              EQU 2_001000000000000

; PORT A
GPIO_PORTA_DATA_BITS_R  EQU 0x40058000
GPIO_PORTA_DATA_R       EQU 0x400583FC
GPIO_PORTA_DIR_R        EQU 0x40058400
GPIO_PORTA_IS_R         EQU 0x40058404
GPIO_PORTA_IBE_R        EQU 0x40058408
GPIO_PORTA_IEV_R        EQU 0x4005840C
GPIO_PORTA_IM_R         EQU 0x40058410
GPIO_PORTA_RIS_R        EQU 0x40058414
GPIO_PORTA_MIS_R        EQU 0x40058418
GPIO_PORTA_ICR_R        EQU 0x4005841C
GPIO_PORTA_AFSEL_R      EQU 0x40058420
GPIO_PORTA_DR2R_R       EQU 0x40058500
GPIO_PORTA_DR4R_R       EQU 0x40058504
GPIO_PORTA_DR8R_R       EQU 0x40058508
GPIO_PORTA_ODR_R        EQU 0x4005850C
GPIO_PORTA_PUR_R        EQU 0x40058510
GPIO_PORTA_PDR_R        EQU 0x40058514
GPIO_PORTA_SLR_R        EQU 0x40058518
GPIO_PORTA_DEN_R        EQU 0x4005851C
GPIO_PORTA_LOCK_R       EQU 0x40058520
GPIO_PORTA_CR_R         EQU 0x40058524
GPIO_PORTA_AMSEL_R      EQU 0x40058528
GPIO_PORTA_PCTL_R       EQU 0x4005852C
GPIO_PORTA_ADCCTL_R     EQU 0x40058530
GPIO_PORTA_DMACTL_R     EQU 0x40058534
GPIO_PORTA_SI_R         EQU 0x40058538
GPIO_PORTA_DR12R_R      EQU 0x4005853C
GPIO_PORTA_WAKEPEN_R    EQU 0x40058540
GPIO_PORTA_WAKELVL_R    EQU 0x40058544
GPIO_PORTA_WAKESTAT_R   EQU 0x40058548
GPIO_PORTA_PP_R         EQU 0x40058FC0
GPIO_PORTA_PC_R         EQU 0x40058FC4
GPIO_PORTA              EQU 2_000000000000001

; PORT Q
GPIO_PORTQ_DATA_BITS_R  EQU 0x40066000
GPIO_PORTQ_DATA_R       EQU 0x400663FC
GPIO_PORTQ_DIR_R        EQU 0x40066400
GPIO_PORTQ_IS_R         EQU 0x40066404
GPIO_PORTQ_IBE_R        EQU 0x40066408
GPIO_PORTQ_IEV_R        EQU 0x4006640C
GPIO_PORTQ_IM_R         EQU 0x40066410
GPIO_PORTQ_RIS_R        EQU 0x40066414
GPIO_PORTQ_MIS_R        EQU 0x40066418
GPIO_PORTQ_ICR_R        EQU 0x4006641C
GPIO_PORTQ_AFSEL_R      EQU 0x40066420
GPIO_PORTQ_DR2R_R       EQU 0x40066500
GPIO_PORTQ_DR4R_R       EQU 0x40066504
GPIO_PORTQ_DR8R_R       EQU 0x40066508
GPIO_PORTQ_ODR_R        EQU 0x4006650C
GPIO_PORTQ_PUR_R        EQU 0x40066510
GPIO_PORTQ_PDR_R        EQU 0x40066514
GPIO_PORTQ_SLR_R        EQU 0x40066518
GPIO_PORTQ_DEN_R        EQU 0x4006651C
GPIO_PORTQ_LOCK_R       EQU 0x40066520
GPIO_PORTQ_CR_R         EQU 0x40066524
GPIO_PORTQ_AMSEL_R      EQU 0x40066528
GPIO_PORTQ_PCTL_R       EQU 0x4006652C
GPIO_PORTQ_ADCCTL_R     EQU 0x40066530
GPIO_PORTQ_DMACTL_R     EQU 0x40066534
GPIO_PORTQ_SI_R         EQU 0x40066538
GPIO_PORTQ_DR12R_R      EQU 0x4006653C
GPIO_PORTQ_WAKEPEN_R    EQU 0x40066540
GPIO_PORTQ_WAKELVL_R    EQU 0x40066544
GPIO_PORTQ_WAKESTAT_R   EQU 0x40066548
GPIO_PORTQ_PP_R         EQU 0x40066FC0
GPIO_PORTQ_PC_R         EQU 0x40066FC4
GPIO_PORTQ              EQU 2_100000000000000

; PORT B
GPIO_PORTB_DATA_BITS_R  EQU 0x40059000
GPIO_PORTB_DATA_R       EQU 0x400593FC
GPIO_PORTB_DIR_R        EQU 0x40059400
GPIO_PORTB_IS_R         EQU 0x40059404
GPIO_PORTB_IBE_R        EQU 0x40059408
GPIO_PORTB_IEV_R        EQU 0x4005940C
GPIO_PORTB_IM_R         EQU 0x40059410
GPIO_PORTB_RIS_R        EQU 0x40059414
GPIO_PORTB_MIS_R        EQU 0x40059418
GPIO_PORTB_ICR_R        EQU 0x4005941C
GPIO_PORTB_AFSEL_R      EQU 0x40059420
GPIO_PORTB_DR2R_R       EQU 0x40059500
GPIO_PORTB_DR4R_R       EQU 0x40059504
GPIO_PORTB_DR8R_R       EQU 0x40059508
GPIO_PORTB_ODR_R        EQU 0x4005950C
GPIO_PORTB_PUR_R        EQU 0x40059510
GPIO_PORTB_PDR_R        EQU 0x40059514
GPIO_PORTB_SLR_R        EQU 0x40059518
GPIO_PORTB_DEN_R        EQU 0x4005951C
GPIO_PORTB_LOCK_R       EQU 0x40059520
GPIO_PORTB_CR_R         EQU 0x40059524
GPIO_PORTB_AMSEL_R      EQU 0x40059528
GPIO_PORTB_PCTL_R       EQU 0x4005952C
GPIO_PORTB_ADCCTL_R     EQU 0x40059530
GPIO_PORTB_DMACTL_R     EQU 0x40059534
GPIO_PORTB_SI_R         EQU 0x40059538
GPIO_PORTB_DR12R_R      EQU 0x4005953C
GPIO_PORTB_WAKEPEN_R    EQU 0x40059540
GPIO_PORTB_WAKELVL_R    EQU 0x40059544
GPIO_PORTB_WAKESTAT_R   EQU 0x40059548
GPIO_PORTB_PP_R         EQU 0x40059FC0
GPIO_PORTB_PC_R         EQU 0x40059FC4
GPIO_PORTB              EQU 2_000000000000010

; PORT P
GPIO_PORTP_DATA_BITS_R  EQU 0x40065000
GPIO_PORTP_DATA_R       EQU 0x400653FC
GPIO_PORTP_DIR_R        EQU 0x40065400
GPIO_PORTP_IS_R         EQU 0x40065404
GPIO_PORTP_IBE_R        EQU 0x40065408
GPIO_PORTP_IEV_R        EQU 0x4006540C
GPIO_PORTP_IM_R         EQU 0x40065410
GPIO_PORTP_RIS_R        EQU 0x40065414
GPIO_PORTP_MIS_R        EQU 0x40065418
GPIO_PORTP_ICR_R        EQU 0x4006541C
GPIO_PORTP_AFSEL_R      EQU 0x40065420
GPIO_PORTP_DR2R_R       EQU 0x40065500
GPIO_PORTP_DR4R_R       EQU 0x40065504
GPIO_PORTP_DR8R_R       EQU 0x40065508
GPIO_PORTP_ODR_R        EQU 0x4006550C
GPIO_PORTP_PUR_R        EQU 0x40065510
GPIO_PORTP_PDR_R        EQU 0x40065514
GPIO_PORTP_SLR_R        EQU 0x40065518
GPIO_PORTP_DEN_R        EQU 0x4006551C
GPIO_PORTP_LOCK_R       EQU 0x40065520
GPIO_PORTP_CR_R         EQU 0x40065524
GPIO_PORTP_AMSEL_R      EQU 0x40065528
GPIO_PORTP_PCTL_R       EQU 0x4006552C
GPIO_PORTP_ADCCTL_R     EQU 0x40065530
GPIO_PORTP_DMACTL_R     EQU 0x40065534
GPIO_PORTP_SI_R         EQU 0x40065538
GPIO_PORTP_DR12R_R      EQU 0x4006553C
GPIO_PORTP_WAKEPEN_R    EQU 0x40065540
GPIO_PORTP_WAKELVL_R    EQU 0x40065544
GPIO_PORTP_WAKESTAT_R   EQU 0x40065548
GPIO_PORTP_PP_R         EQU 0x40065FC0
GPIO_PORTP_PC_R         EQU 0x40065FC4
GPIO_PORTP              EQU 2_010000000000000

; DISPLAY
ADC0_DCCMP4_R          EQU 0x40038E50
ADC0_DCCMP5_R          EQU 0x40038E54
ADC0_DCCMP6_R          EQU 0x40038E58
ADC0_DCCMP7_R          EQU 0x40038E5C
; -------------------------------------------------------------------------------
; �rea de C�digo - Tudo abaixo da diretiva a seguir ser� armazenado na mem�ria de 
;                  c�digo
        AREA    |.text|, CODE, READONLY, ALIGN=2

		; Se alguma fun��o do arquivo for chamada em outro arquivo	
        EXPORT GPIO_Init            ; Permite chamar GPIO_Init de outro arquivo
		EXPORT GPIOPortJ_Handler
		EXPORT PortN_Output
		EXPORT PortP_Output
		EXPORT PortB_Output
		EXPORT PortQ_Output
		EXPORT PortA_Output
		
		; Se chamar alguma fun��o externa	
        IMPORT EnableInterrupts
        IMPORT DisableInterrupts
		IMPORT SysTick_Wait1ms

;--------------------------------------------------------------------------------
; Fun��o GPIO_Init
; Par�metro de entrada: N�o tem
; Par�metro de sa�da: N�o tem
GPIO_Init
;=====================
; 1. Ativar o clock para a porta setando o bit correspondente no registrador RCGCGPIO,
; ap�s isso verificar no PRGPIO se a porta est� pronta para uso.
; enable clock to GPIOF at clock gating register
            LDR     R0, =SYSCTL_RCGCGPIO_R  		;Carrega o endere�o do registrador RCGCGPIO

			MOV		R1, #GPIO_PORTN                 ;Seta o bit da porta N

			ORR     R1, #GPIO_PORTJ					;Seta o bit da porta J, fazendo com OR

			ORR     R1, #GPIO_PORTA					;Seta o bit da porta A, fazendo com OR

			ORR     R1, #GPIO_PORTQ					;Seta o bit da porta Q, fazendo com OR

			ORR     R1, #GPIO_PORTB					;Seta o bit da porta B, fazendo com OR

			ORR     R1, #GPIO_PORTP					;Seta o bit da porta P, fazendo com OR

            STR     R1, [R0]						;Move para a mem�ria os bits das portas no endere�o do RCGCGPIO
 
            LDR     R0, =SYSCTL_PRGPIO_R			;Carrega o endere�o do PRGPIO para esperar os GPIO ficarem prontos
EsperaGPIO  LDR     R1, [R0]						;L� da mem�ria o conte�do do endere�o do registrador

			MOV     R2, #GPIO_PORTN                 ;Seta os bits correspondentes �s portas para fazer a compara��o

			ORR     R2, #GPIO_PORTJ                 ;Seta o bit da porta J, fazendo com OR

			ORR     R2, #GPIO_PORTA                 ;Seta o bit da porta A, fazendo com OR

			ORR     R2, #GPIO_PORTQ                 ;Seta o bit da porta Q, fazendo com OR

			ORR     R2, #GPIO_PORTB                 ;Seta o bit da porta B, fazendo com OR

			ORR     R2, #GPIO_PORTP                 ;Seta o bit da porta P, fazendo com OR

            TST     R1, R2							;ANDS de R1 com R2
            BEQ     EsperaGPIO					    ;Se o flag Z=1, volta para o la�o. Sen�o continua executando
 
; 2. Limpar o AMSEL para desabilitar a anal�gica
            MOV     R1, #0x00						;Colocar 0 no registrador para desabilitar a fun��o anal�gica

            LDR     R0, =GPIO_PORTJ_AHB_AMSEL_R     ;Carrega o R0 com o endere�o do AMSEL para a porta J
            STR     R1, [R0]						;Guarda no registrador AMSEL da porta J da mem�ria

            LDR     R0, =GPIO_PORTN_AMSEL_R			;Carrega o R0 com o endere�o do AMSEL para a porta N
            STR     R1, [R0]					    ;Guarda no registrador AMSEL da porta N da mem�ria

			LDR     R0, =GPIO_PORTA_AMSEL_R			;Carrega o R0 com o endere�o do AMSEL para a porta A
            STR     R1, [R0]					    ;Guarda no registrador AMSEL da porta A da mem�ria

			LDR     R0, =GPIO_PORTQ_AMSEL_R			;Carrega o R0 com o endere�o do AMSEL para a porta Q
            STR     R1, [R0]					    ;Guarda no registrador AMSEL da porta Q da mem�ria

			LDR     R0, =GPIO_PORTB_AMSEL_R			;Carrega o R0 com o endere�o do AMSEL para a porta B
            STR     R1, [R0]					    ;Guarda no registrador AMSEL da porta B da mem�ria

			LDR     R0, =GPIO_PORTP_AMSEL_R			;Carrega o R0 com o endere�o do AMSEL para a porta P
            STR     R1, [R0]					    ;Guarda no registrador AMSEL da porta P da mem�ria

; 3. Limpar PCTL para selecionar o GPIO
            MOV     R1, #0x00					    ;Colocar 0 no registrador para selecionar o modo GPIO
            LDR     R0, =GPIO_PORTJ_AHB_PCTL_R		;Carrega o R0 com o endere�o do PCTL para a porta J
            STR     R1, [R0]                        ;Guarda no registrador PCTL da porta J da mem�ria

            LDR     R0, =GPIO_PORTN_PCTL_R     		;Carrega o R0 com o endere�o do PCTL para a porta N
            STR     R1, [R0]                        ;Guarda no registrador PCTL da porta N da mem�ria

			LDR     R0, =GPIO_PORTA_PCTL_R     		;Carrega o R0 com o endere�o do PCTL para a porta A
            STR     R1, [R0]                        ;Guarda no registrador PCTL da porta A da mem�ria
			
			LDR     R0, =GPIO_PORTQ_PCTL_R     		;Carrega o R0 com o endere�o do PCTL para a porta Q
            STR     R1, [R0]                        ;Guarda no registrador PCTL da porta Q da mem�ria

			LDR     R0, =GPIO_PORTB_PCTL_R     		;Carrega o R0 com o endere�o do PCTL para a porta B
            STR     R1, [R0]                        ;Guarda no registrador PCTL da porta B da mem�ria

			LDR     R0, =GPIO_PORTP_PCTL_R     		;Carrega o R0 com o endere�o do PCTL para a porta P
            STR     R1, [R0]                        ;Guarda no registrador PCTL da porta P da mem�ria

; 4. DIR para 0 se for entrada, 1 se for sa�da
			LDR     R0, =GPIO_PORTN_DIR_R			;Carrega o R0 com o endere�o do DIR para a porta N
			MOV     R1, #2_00000011					;1 no registrador DIR dos pinos PN1 & PN0 para funcionarem como saida
            STR     R1, [R0]						;Guarda no registrador
			
			LDR     R0, =GPIO_PORTA_DIR_R			;Carrega o R0 com o endere�o do DIR para a porta A
			MOV     R1, #2_11110000					;1 no registrador DIR dos pinos PA7 a PA4 para funcionarem como saida
            STR     R1, [R0]						;Guarda no registrador
			
			LDR     R0, =GPIO_PORTQ_DIR_R			;Carrega o R0 com o endere�o do DIR para a porta Q
			MOV     R1, #2_00001111					;1 no registrador DIR dos pinos PQ3 a PQ0 para funcionarem como saida
            STR     R1, [R0]						;Guarda no registrador
			
			LDR     R0, =GPIO_PORTB_DIR_R			;Carrega o R0 com o endere�o do DIR para a porta B
			MOV     R1, #2_00110000					;1 no registrador DIR dos pinos PB4 & PB5 para funcionarem como saida
            STR     R1, [R0]						;Guarda no registrador
			
			LDR     R0, =GPIO_PORTP_DIR_R			;Carrega o R0 com o endere�o do DIR para a porta p
			MOV     R1, #2_00100000					;1 no registrador DIR dos pinos PP5 para funcionarem como saida
            STR     R1, [R0]						;Guarda no registrador
			
            LDR     R0, =GPIO_PORTJ_AHB_DIR_R		;Carrega o R0 com o endere�o do DIR para a porta J
            MOV     R1, #2_00               		;0 no registrador DIR dos pinos PJ1 & PJ0 (SW2 & SW1) para funcionarem como entrada
            STR     R1, [R0]						;Guarda no registrador

; 5. Limpar os bits AFSEL para 0 para selecionar GPIO 
;    Sem fun��o alternativa
            MOV     R1, #0x00						;Colocar o valor 0 para n�o setar fun��o alternativa
			
            LDR     R0, =GPIO_PORTN_AFSEL_R			;Carrega o endere�o do AFSEL da porta N
            STR     R1, [R0]						;Escreve na porta

            LDR     R0, =GPIO_PORTJ_AHB_AFSEL_R     ;Carrega o endere�o do AFSEL da porta J
            STR     R1, [R0]                        ;Escreve na porta

			LDR     R0, =GPIO_PORTA_AFSEL_R     	;Carrega o endere�o do AFSEL da porta A
            STR     R1, [R0]                        ;Escreve na porta

			LDR     R0, =GPIO_PORTQ_AFSEL_R     	;Carrega o endere�o do AFSEL da porta Q
            STR     R1, [R0]                        ;Escreve na porta

			LDR     R0, =GPIO_PORTB_AFSEL_R     	;Carrega o endere�o do AFSEL da porta B
            STR     R1, [R0]                        ;Escreve na porta

			LDR     R0, =GPIO_PORTP_AFSEL_R     	;Carrega o endere�o do AFSEL da porta P
            STR     R1, [R0]                        ;Escreve na porta

; 6. Setar os bits de DEN para habilitar I/O digital
            LDR     R0, =GPIO_PORTN_DEN_R			;Carrega o endere�o do DEN
            MOV     R1, #2_00000011               	;Ativa os pinos PN0 e PN1 como I/O Digital
            STR     R1, [R0]						;Escreve no registrador da mem�ria funcionalidade digital 
			
			LDR     R0, =GPIO_PORTA_DEN_R			;Carrega o endere�o do DEN
            MOV     R1, #2_11110000               	;Ativa os pinos PA7 ate PA4 como I/O Digital
            STR     R1, [R0]						;Escreve no registrador da mem�ria funcionalidade digital 
			
			LDR     R0, =GPIO_PORTQ_DEN_R			;Carrega o endere�o do DEN
            MOV     R1, #2_00001111               	;Ativa os pinos PQ3 ate PQ0 como I/O Digital
            STR     R1, [R0]						;Escreve no registrador da mem�ria funcionalidade digital 
			
			LDR     R0, =GPIO_PORTB_DEN_R			;Carrega o endere�o do DEN
			MOV     R1, #2_00110000         		;Ativa os pinos PB5 e PB4 como I/O Digital      
            STR     R1, [R0]                        ;Escreve no registrador da mem�ria funcionalidade digital
			
			LDR     R0, =GPIO_PORTP_DEN_R			;Carrega o endere�o do DEN
			MOV     R1, #2_00100000         		;Ativa o pino PP5 como I/O Digital      
            STR     R1, [R0]                        ;Escreve no registrador da mem�ria funcionalidade digital
 
            LDR     R0, =GPIO_PORTJ_AHB_DEN_R		;Carrega o endere�o do DEN
			MOV     R1, #2_00000011         		;Ativa os pinos PJ0 e PJ1 como I/O Digital      
            STR     R1, [R0]                        ;Escreve no registrador da mem�ria funcionalidade digital

; 7. Para habilitar resistor de pull-up interno, setar PUR para 1
			LDR     R0, =GPIO_PORTJ_AHB_PUR_R		;Carrega o endere�o do PUR para a porta J
			MOV     R1, #2_11						;Habilitar funcionalidade digital de resistor de pull-up                                                     
            STR     R1, [R0]						;Escreve no registrador da mem�ria do resistor de pull-up


;Interrup��es


; 8. Desabilitar a interrup��o no registrador IM
			LDR     R0, =GPIO_PORTJ_AHB_IM_R			;Carrega o endere�o do IM para a porta J
			MOV     R1, #2_00							;Desabilitar as interrup��es  
            STR     R1, [R0]							;Escreve no registrador


; 9. Configurar o tipo de interrup��o por borda no registrador IS
			LDR     R0, =GPIO_PORTJ_AHB_IS_R			;Carrega o endere�o do IS para a porta J
			MOV     R1, #2_00							;Por Borda  
            STR     R1, [R0]							;Escreve no registrador

; 10. Configurar borda �nica no registrador IBE
			LDR     R0, =GPIO_PORTJ_AHB_IBE_R			;Carrega o endere�o do IBE para a porta J
			MOV     R1, #2_00							;Borda �nica  para j0 e j1  
            STR     R1, [R0]							;Escreve no registrador
			
; 11. Configurar borda de descida no registrador IEV
			LDR     R0, =GPIO_PORTJ_AHB_IEV_R			;Carrega o endere�o do IEV para a porta J
			MOV     R1, #2_00							;Borda de descida para j0 e j1  
            STR     R1, [R0]							;Escreve no registrador

; 12. Realizando o ACK no registrador GPIOICR para ambos os pinos.
			LDR     R0, =GPIO_PORTJ_AHB_ICR_R			;Carrega o endere�o do ICR para a porta J	
			MOV     R1, #2_11							;Realizar o ACK para os dois pinos
            STR     R1, [R0]							;Escreve no registrador
  
; 13. Habilitar a interrup��o no registrador IM
			LDR     R0, =GPIO_PORTJ_AHB_IM_R			;Carrega o endere�o do IM para a porta J
			MOV     R1, #2_11							;Habilitar as interrup��es  
            STR     R1, [R0]							;Escreve no registrador
     
; 14. Setar a prioridade no NVIC
			LDR     R0, =NVIC_PRI12_R           		;Carrega o do NVIC para o grupo que tem o J entre 48 e 51
			MOV     R1, #0  		                    ;Prioridade 0
            STR     R1, [R0]							;Escreve no registrador da mem�ria

; 15. Habilitar a interrup��o no NVIC
			LDR     R0, =NVIC_EN1_R           			;Carrega o do NVIC para o grupo que tem o J entre 32 e 61
			MOV     R1, #1								;O bit para habilitar
			LSL     R1, #19								;Desloca 19 bits para a esquerda j� que o J � a interrup��o do bit 19 no EN1
            STR     R1, [R0]


			BX  LR

; -------------------------------------------------------------------------------
; Fun��o PortN_Output
; Par�metro de entrada: R9 --> se os BIT1 e BIT0 est�o ligado ou desligado
; Par�metro de sa�da: N�o tem
PortN_Output
	LDR	R1, =GPIO_PORTN_DATA_R		    	;Carrega o valor do offset do data register
	;Read-Modify-Write para escrita
	LDR R2, [R1]
	BIC R2, #2_11                     		;Primeiro limpamos os dois bits do lido da porta R2 = R2 & 11111100
	ORR R9, R9, R2                          ;Fazer o OR do lido pela porta com o par�metro de entrada
	STR R9, [R1]                            ;Escreve na porta N o barramento de dados dos pinos [N5-N0]
	BX LR									;Retorno
 

; -------------------------------------------------------------------------------
; Fun��o PortA_Output
; Par�metro de entrada: R9 --> BIT7 a BIT4 = bits mais significativos do numero
; Par�metro de sa�da: N�o tem
PortA_Output
	LDR	R1, =GPIO_PORTA_DATA_R		    ;Carrega o valor do offset do data register
	AND R2, R9, #2_11110000             ;Fazer o AND da par�metro de entrada com os bits que queremos ler
	STR R2, [R1]                        ;Escreve na porta Q o barramento de dados dos pinos [Q3-Q0]
	BX LR								;Retorno


; -------------------------------------------------------------------------------
; Fun��o PortQ_Output
; Par�metro de entrada: R9 --> BIT3 a BIT0 = bits menos significativos do numero
; Par�metro de sa�da: N�o tem
PortQ_Output
	LDR	R1, =GPIO_PORTQ_DATA_R		    ;Carrega o valor do offset do data register
	AND R2, R9, #2_00001111             ;Fazer o AND da par�metro de entrada com os bits que queremos ler
	STR R2, [R1]                        ;Escreve na porta Q o barramento de dados dos pinos [Q3-Q0]
	BX LR								;Retorno


; -------------------------------------------------------------------------------
; Fun��o PortB_Output
; Par�metro de entrada: R9 --> se os BIT5 ou BIT4 est�o ligado ou desligado
; Par�metro de sa�da: N�o tem
PortB_Output
	LDR	R1, =GPIO_PORTB_DATA_R		    ;Carrega o valor do offset do data register
	LDR R2, [R1]
	BIC R2, #2_00110000                	;Primeiro limpamos os dois bits do lido da porta 
	ORR R9, R9, R2                      ;Fazer o OR do lido pela porta com o par�metro de entrada
	STR R9, [R1]                        ;Escreve na porta N o barramento de dados dos pinos [N5-N0]
	BX LR								;Retorno
	
; -------------------------------------------------------------------------------
; Fun��o PortP_Output
; Par�metro de entrada: R9 --> se o BIT5 est�o ligado ou desligado
; Par�metro de sa�da: N�o tem
PortP_Output
	LDR	R1, =GPIO_PORTP_DATA_R		    ;Carrega o valor do offset do data register
	LDR R2, [R1]
	BIC R2, #2_00100000                	;Primeiro limpamos o bit do lido da porta 
	ORR R9, R9, R2                      ;Fazer o OR do lido pela porta com o par�metro de entrada
	STR R9, [R1]                        ;Escreve na porta N o barramento de dados dos pinos [N5-N0]
	BX LR								;Retorno


; -------------------------------------------------------------------------------
; Fun��o ISR GPIOPortJ_Handler (Tratamento da interrup��o)
GPIOPortJ_Handler
    LDR R0, =GPIO_PORTJ_AHB_RIS_R        
	LDR R1, [R0]
	CMP R1, #2_01
	BNE CHAVE2
	LDR R0, =GPIO_PORTJ_AHB_ICR_R
	MOV R1, #2_01
	STR R1, [R0]
	PUSH {LR}
	; Lógica ao pressionar a chave1
	ADD R11, R11, #1
	CMP R11, #50 ; Compara se ficou maior que 50
	BLT PULO_MAIOR
	MOV R11, #50 ; Limita a 50 graus
PULO_MAIOR
	POP {LR}
    B FIM_INTERRUPT


CHAVE2 
	LDR R0, =GPIO_PORTJ_AHB_ICR_R    
	MOV R1, #2_10
	STR R1, [R0]
	PUSH {LR}
	; Diminui temperatura
	SUB R11, R11, #1
	CMP R11, #5 ; Compara se ficou menor que 5
	BGT PULO_MENOR
	MOV R11, #5 ; Limita a 5 graus
PULO_MENOR
	

	POP {LR}

CHAVE1
	
FIM_INTERRUPT
	BX LR
	
    ALIGN                           ; garante que o fim da se��o est� alinhada 
    END                             ; fim do arquivo