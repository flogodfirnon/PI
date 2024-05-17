processor 16f1789
#include <xc.inc>
CONFIG  FOSC = INTOSC         ; Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
CONFIG  WDTE = OFF            ; Watchdog Timer Enable (WDT disabled)
CONFIG  PWRTE = OFF           ; Power-up Timer Enable (PWRT disabled)
CONFIG  MCLRE = OFF           ; MCLR Pin Function Select (MCLR/VPP pin function is digital input)
CONFIG  CP = OFF              ; Flash Program Memory Code Protection (Program memory code protection is disabled)
CONFIG  CPD = OFF             ; Data Memory Code Protection (Data memory code protection is disabled)
CONFIG  BOREN = ON            ; Brown-out Reset Enable (Brown-out Reset enabled)
CONFIG  CLKOUTEN = OFF        ; Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
CONFIG  IESO = ON             ; Internal/External Switchover (Internal/External Switchover mode is enabled)
CONFIG  FCMEN = ON            ; Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)
CONFIG  WRT = OFF             ; Flash Memory Self-Write Protection (Write protection off)
CONFIG  VCAPEN = OFF          ; Voltage Regulator Capacitor Enable bit (Vcap functionality is disabled on RA6.)
CONFIG  PLLEN = OFF           ; PLL Enable (4x PLL disabled)
CONFIG  STVREN = ON           ; Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
CONFIG  BORV = LO             ; Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
CONFIG  LPBOR = OFF           ; Low Power Brown-Out Reset Enable Bit (Low power brown-out is disabled)
CONFIG  LVP = OFF  
    
#define	    RC3	    3
#define	    RC4	    4
#define	    SSP1IF  3
#define	    SEN	    0
#define	    RSEN    1
#define	    ACKSTAT 6
#define	    PEN	    2
#define	    RCEN    3
#define	    ACKDT   5
#define	    ACKEN   4

#define	    ADDRESS_TRANSMITTER	    11001000B
#define	    ADDRESS_RECEIVER	    11001001B
#define	    REG_DATA_INIT_SENSOR    01110000B
#define	    REG_CLEAR_REG_STATE	    01100101B
#define	    WEIGHT_DATA_REGISTER    01100110B
    
#define	    RAW_DATA_SIZE		    4

#define	    GOAL_WEIGHT		    01110011B
#define	    TOLERANCE		    00000011B
#define	    KP			    00000011B
#define	    NB_BITS_REG_B		    8
    
    
#define STEP_COUNT 7
#define HIGH_POSITION 10
#define LOW_POSITION 16
#define SERVO_STEP 160
#define END_DESTINATION 5
#define SERVO_WAIT_UP 190
#define SERVO_WAIT_DOWN 20
#define SERVO_WAIT_TIME_PROG 6
#define PUMP_POSITION 32
#define ALCOHOL_1_POS 113
#define ALCOHOL_2_POS 180
#define ALCOHOL_3_POS 250
#define SEC_WAITING 35
      
    
PSECT udata_bank2
STEPPER_OR_SERVO: DS 1 ;bit 0 equal to 1 if servo is on, bit 0 is equal to 0 is stepper is on
; For the weight sensor 
RAW_DATA:	  DS	    RAW_DATA_SIZE
DATA_MSB:	  DS	    1
DATA_LSB:	  DS	    1
OFFSET_MSB:	  DS	    1
OFFSET_LSB:	  DS	    1
DATA_MUL:	  DS	    1
INDEX:		  DS	    1
    
;Flag register
OFFSET_FLAG:	  DS	    1
    
; Control part
E_K:		  DS	    1
SHIFT_COUNTER:	  DS	    1
TEMP_VAR:	  DS	    1
KP_REG:		  DS	    1
UP:		  DS	    1
PUMP_VALUE:	  DS	    1
    
COUNTER_STEP: DS 1
COUNTER_TURN: DS 1
BUTTONS_VALUES_INT: DS 1
OLD_BUTTONS_VALUES_INT: DS 1
BUTTONS_VALUES: DS 1
OLD_BUTTONS_VALUES: DS 1
; Bit 0: task button is ready
; Bit 1: the stop switch has been pushed
; Bit 2: the stepper is arrived at destination
; Bit 3: the servo has finished its operation
; Bit 4: the timer of 10ms has finished
; Bit 5: the timer of 1s has finished
TASK_READY: DS 1
; Bit 0 is set: the first alcohol has been choosen
; Bit 1 is set: the second alcohol has been choosen
; Bit 2 is set: the third alcohol has been choosen
; Bit 4 is set: not water is choosen
CHOICE: DS 1

SAME_BUTTONS_1: DS 1
SAME_BUTTONS_0: DS 1
BUTTONS_PRESS: DS 1 ; tell when the button was already press at the previous loop iteration
BUTTONS_USE: DS 1 ; tell if the button must be use

POSITION: DS 1  ; specify the position wanted for the servo
CURRENT_POSITION: DS 1 ;current position of the servos
COUNTER_SERVO: DS 1
; Count to be sure that the 'doseur' has completed its operation
WAIT_SERVO: DS 1
SERVO_WAIT_TIME: DS 1
STEPPER_TARGET: DS 1
; the state 0 is used for debugging
STATE: DS 1
PREVIOUS_STATE: DS 1
; variable to store a reading
READING: DS 1 
;to use the 1 ms timer, the prescaler of timer 1 must be eigth
SEC_WAITING_COUNTER: DS 1

    
PSECT rst_vec, abs, class = CODE, delta=2
ORG 0x00h
    goto    initialisation
    
PSECT isr_vec, class = CODE, delta = 2
ORG 0x04h
    goto interrupt
    
PSECT code
initialisation:
    
    ; Device frequency 
    banksel OSCCON
    movlw   11111100B ; 32 MHz FOSC
    movwf   OSCCON
    banksel ADCON1
    movlw 01000000B
    movwf ADCON1
    
    ;Timer1 d?clench? for the stepper motor
    banksel T1CON
    movlw   00000000B ; Prescale value of 1
    movwf   T1CON
    ;initialise the value of the counter
    banksel TMR1L
    clrf TMR1L
    clrf TMR1H
    banksel PIR1
    bcf PIR1, 0
    
    ; timer0 en mode timer
    banksel OPTION_REG
    bcf OPTION_REG, 5
    
    ; initialisation des interrupts du timer 0
    banksel INTCON
    bsf INTCON, 5
    bcf INTCON, 2
    
    ; Allow the prescaler of the timer0
    banksel OPTION_REG
    bcf OPTION_REG, 3
    
    ; Prescaler of 256
    movlw 07h
    iorwf OPTION_REG, f
    
    ; Disable the weak pull-up (for IOC)
    bsf OPTION_REG, 7

    
    ;Allow timer1 interrupts
    banksel PIE1
    bsf PIE1, 0
    
    ;reset Timer1 flag
    banksel PIR1
    bcf PIR1, 0
    
        ; initialisation of the pins
    ; Port A
    banksel TRISA
    movlw 00000011B
    movwf TRISA
    
    banksel LATA
    clrf LATA
    
    banksel ANSELA
    clrf ANSELA
    
    ; Port B
    banksel TRISB
    movlw 00111000B
    movwf TRISB
    
    banksel LATB
    clrf LATB
    
    banksel ANSELB
    clrf ANSELB
    


    ;Setup i2c pins
    banksel TRISC
    clrf    TRISC
    bsf	    TRISC, RC3
    bsf	    TRISC, RC4
    banksel LATC
    clrf    LATC
    banksel ANSELC
    clrf    ANSELC
    
    ;Setup LED test pin 
    banksel TRISD
    clrf    TRISD
    banksel LATD
    movlw   00001000B ; disable the stepper
    movwf   LATD
    
    ; Put the RA0 (for the water level sensor) and RA1 in Schmidt-Trigger mode
    banksel INLVLA
    bsf INLVLA, 0
    bsf INLVLA, 1
    
    ; IOC for RA1
    banksel INTCON
    ; enable the IOC on pins
    bsf INTCON, 3
    ; enable the rising edge on RA1
    banksel IOCAN
    bsf IOCAN, 1


    ; I2C CONFIGURATION
    banksel SSP1CON1	; Serial Communication COnfiguration Register
    movlw   00101000B	; bit 3-0 configures the type of communication. 
			; 1000 corresponds to I2C Master mode 
    movwf   SSP1CON1 
    
    banksel SSP1ADD	; modify the clock frequency
    ;movlw   00100111B	; SSP1ADD = 39 to have 200kHz f clock and so a 100kHz pulse 
    movlw   00011011B	;seems to be closer to 100kHz with that value 
    movwf   SSP1ADD
    
    
    banksel PIR1
    bcf	    PIR1, SSP1IF
    
    ;PWM CONFIGURATION
    ; Step 1
    banksel TRISC
    bsf TRISC, 2
    ;Step 2
    banksel PR2
    movlw 0xFF
    movwf PR2
    movlw 03h
    banksel T2CON
    movwf T2CON ;set the prescale value to 64
    ; Step 3
    banksel CCP1CON
    movlw 00111111B ; le 2 est pour le duty cycle
    movwf CCP1CON
    ;Step 4
    banksel CCPR1L
    movlw 01111111B
    movwf CCPR1L
     ; Step 5
    banksel PIR1
    bcf PIR1, 1

    banksel TRISC
    bcf TRISC, 2
    
    ;clear flag
    banksel TASK_READY
    clrf    TASK_READY
    banksel OFFSET_FLAG
    clrf    OFFSET_FLAG
    
     ;Initialisation of the variables
    banksel COUNTER_STEP
    
    clrf TASK_READY
    
    clrf COUNTER_STEP
    movlw 0xFE
    movwf COUNTER_TURN
    
    clrf SAME_BUTTONS_0
    clrf SAME_BUTTONS_1
    clrf BUTTONS_PRESS
    clrf STEPPER_OR_SERVO
    clrf BUTTONS_USE
    bsf STEPPER_OR_SERVO, 0
    clrf WAIT_SERVO
    
    movlw LOW_POSITION
    movwf POSITION
    
    movlw SERVO_WAIT_UP
    movwf SERVO_WAIT_TIME
    
    movlw 0x04
    movwf STATE
    movwf PREVIOUS_STATE
    ;clrf STATE
    
    movlw 0xFF
    movwf STEPPER_TARGET
    
    clrf CHOICE
    
    movlw LOW_POSITION
    movwf CURRENT_POSITION
    
    clrf SEC_WAITING_COUNTER
    
    ;force the machine to check first if there is water
    bsf TASK_READY, 1

    
    ;Allow global interrupts
    banksel INTCON
    bsf INTCON, 6
    bsf	INTCON, 7

    goto sensor_init
    
interrupt:
    banksel PIR1
    btfsc PIR1, 0
    goto timer1_selection
    
    banksel INTCON
    btfsc   INTCON, 2
    goto buttons_int
    
    banksel IOCAF
    btfsc IOCAF, 1
    goto stop_switch
    
    retfie

buttons_int:
    
    banksel TASK_READY
    bsf TASK_READY, 0
    
    banksel BUTTONS_VALUES_INT
    movf BUTTONS_VALUES_INT, w
    movwf OLD_BUTTONS_VALUES_INT
    
    ; read the values at the inputs
    ; read the values at RB3, RB4 and RB5
    banksel PORTB
    movf PORTB, w
    banksel READING
    movwf READING
    ; take only the input at 3, 4, 5
    movlw 00111000B
    andwf READING, w
    
    ; move the values read in the variable
    banksel BUTTONS_VALUES_INT
    movwf BUTTONS_VALUES_INT
    
    ;read the value at RA0 for the wls
    banksel PORTA
    movf PORTA, w
    banksel READING
    movwf READING
    ; take only care of 0
    movlw 00000001B
    andwf READING, w
    
    ; update the variable with this new input
    banksel BUTTONS_VALUES_INT
    iorwf BUTTONS_VALUES_INT, f
    
    banksel INTCON
    bcf INTCON, 5
    bcf INTCON, 2
    bsf INTCON, 5
    
    retfie
    
timer1_selection:
    
    banksel STEPPER_OR_SERVO
    btfsc   STEPPER_OR_SERVO, 2
    goto    timer1_waiting_1s
    
    banksel STEPPER_OR_SERVO
    btfsc   STEPPER_OR_SERVO, 1
    goto    timer1_waiting_10ms
    
    btfss   STEPPER_OR_SERVO, 0
    goto    stepper
    
    goto    servo

timer1_waiting_1s:
    banksel PIR1
    bcf PIR1, 0
    
    ;reset all the counters to have the maximal delay
    banksel T1CON
    bcf T1CON, 0
    clrf TMR1L
    clrf TMR1H
    bsf T1CON, 0
    
    ;check if the value is equal to the max value
    banksel SEC_WAITING_COUNTER
    incf SEC_WAITING_COUNTER
    movlw SEC_WAITING
    xorwf SEC_WAITING_COUNTER, 0
    banksel STATUS
    btfss STATUS, 2
    retfie
    
    banksel TASK_READY
    bsf TASK_READY, 5
    
    clrf SEC_WAITING_COUNTER
    
    retfie
    
    
timer1_waiting_10ms:
    banksel PIR1
    bcf PIR1, 0
    
    banksel TASK_READY
    bsf TASK_READY, 4
    
    retfie
    
stepper:
    banksel PIR1
    bcf PIR1, 0
    
    ; inverse the value on RD2 (CLK+ for the stepper
    banksel LATD	
    movlw   04h
    xorwf   LATD, 1
    
    ;value of the counter to reach 500Hz
    banksel T1CON
    bcf T1CON, 0
    movlw 11000000B
    movwf TMR1L
    movlw 11100000B
    movwf TMR1H
    bsf T1CON, 0
    
    ; increment the counter if a complete period has occured
    banksel LATD
    btfsc LATD, 2
    call increment_counter_step
    movlw STEP_COUNT
    banksel COUNTER_STEP
    xorwf COUNTER_STEP, 0
    banksel STATUS
    btfsc STATUS, 2
    call change
    
    retfie
    
increment_counter_step:
    banksel COUNTER_STEP
    incf COUNTER_STEP
    return
    
incrementation:
    banksel COUNTER_TURN
    incf COUNTER_TURN, 1
    return
    
decrementation:
    
    banksel COUNTER_TURN
    decf COUNTER_TURN, 1
    return

change:
    
    ; increment if we are going to the write
    banksel LATD
    btfss LATD, 0
    call incrementation
    
    ; decrement if we are going to the left
    btfsc LATD, 0
    call decrementation
    
    banksel COUNTER_STEP
    clrf COUNTER_STEP
    
    movf STEPPER_TARGET, w
    xorwf COUNTER_TURN, w
    banksel STATUS
    btfss STATUS, 2
    return
    
    ; cut the stepper
    banksel LATD
    bsf LATD, 3
    
    banksel T1CON
    bcf T1CON, 0
    
    banksel TASK_READY
    bsf TASK_READY, 2
    
    return

servo:
    banksel PIR1
    bcf PIR1, 0
    
    ;value of the counter to reach 8kHz
    banksel T1CON
    bcf T1CON, 0
    movlw 00110000B
    movwf TMR1L
    movlw 11111100B
    movwf TMR1H
    bsf T1CON, 0

    banksel COUNTER_SERVO
    incf COUNTER_SERVO
    movf CURRENT_POSITION, w
    xorwf COUNTER_SERVO, 0
    banksel STATUS
    btfsc STATUS, 2
    call clear_output
    
    banksel COUNTER_SERVO
    movlw SERVO_STEP
    xorwf COUNTER_SERVO, 0
    banksel STATUS
    btfsc STATUS, 2
    call reset_servo_counter
    
    retfie
    
    
reset_servo_counter:
    
    banksel COUNTER_SERVO
    clrf COUNTER_SERVO
    
    ; set again RC1 (PWM servo) to 1
    banksel LATC
    bsf LATC, 1
    
    banksel POSITION
    movf POSITION, w
    banksel CURRENT_POSITION
    xorwf CURRENT_POSITION, 0
    banksel STATUS
    btfss STATUS, 2
    goto progressive
    
    banksel WAIT_SERVO
    incf WAIT_SERVO
    movf SERVO_WAIT_TIME, w
    banksel WAIT_SERVO
    xorwf WAIT_SERVO, 0
    banksel STATUS
    btfss STATUS, 2
    return
    
    banksel TASK_READY
    bsf TASK_READY, 3
    
    banksel T1CON
    bcf T1CON, 0
    
    return

clear_output:
    
    ; clear the output of the PWM servo
    banksel LATC
    bcf LATC, 1
    
    return
    
progressive:
    
    banksel WAIT_SERVO
    incf WAIT_SERVO
    movlw SERVO_WAIT_TIME_PROG
    banksel WAIT_SERVO
    xorwf WAIT_SERVO, 0
    banksel STATUS
    btfss STATUS, 2
    return
    
    banksel WAIT_SERVO
    clrf WAIT_SERVO
    
    ; check wich of position and current_position is the highest
    banksel POSITION
    movf POSITION, w
    subwf   CURRENT_POSITION, w  ; POSITION - CURRENT_POSITION
    banksel STATUS
    btfsc   STATUS, 0
    goto    decrease_pos
    
increase_pos:
    
    banksel CURRENT_POSITION
    incf CURRENT_POSITION
    return
    
decrease_pos:
    
    banksel CURRENT_POSITION
    decf CURRENT_POSITION
    return
    
stop_switch:
    
    ; clear the interrupt flag
    banksel IOCAF
    bcf IOCAF, 1
    
    ; discard if the stepper is not moving
    banksel LATD
    btfsc LATD, 3
    retfie
    
    banksel LATD
    bcf LATD, 0 ; the direction of the platform
    bcf LATD, 2 ; the output of the PWM for the stepper
    
    banksel COUNTER_STEP
    clrf COUNTER_STEP
    clrf COUNTER_TURN
    
    banksel TASK_READY
    btfsc TASK_READY, 1
    retfie
    
    ; Disable the stepper if we want the stepper to stop
    banksel LATD
    bsf LATD, 3
    
    banksel T1CON
    bcf T1CON, 0
    
    banksel PIR1
    bcf PIR1, 0
    
    banksel TASK_READY
    bsf TASK_READY, 1
    
    retfie
    
    
sensor_init:
    ;######################## 
    ; 20 ms delay before asking anything 
    banksel STEPPER_OR_SERVO
    bsf	    STEPPER_OR_SERVO, 1
    
    banksel T1CON
    movlw 00110000B
    movwf T1CON
    
    banksel TMR1H
    movlw 11100000B
    movwf TMR1L
    movlw 10110001B
    movwf TMR1H
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel TASK_READY
    btfss   TASK_READY, 4
    goto    $-1
    
    banksel TASK_READY
    bcf TASK_READY, 4
    
    banksel T1CON
    bcf T1CON, 0
    clrf    T1CON
    
    banksel STEPPER_OR_SERVO
    bcf	    STEPPER_OR_SERVO, 1
    
    
    banksel INTCON
    bcf	    INTCON, 7

     
    banksel SSP1CON2	        ; Second configuration register
    bsf	    SSP1CON2, SEN	; set the SEN bit which initializes the communication
    

    
    call    wait_SSPIF
    

    
    ; send slave address
    banksel SSP1BUF
    movlw   ADDRESS_TRANSMITTER	    ; last bit to 0 for master transmitter
    movwf   SSP1BUF
 
    call     wait_SSPIF
   
    banksel  SSP1CON2
    btfsc    SSP1CON2, ACKSTAT ; check if ACKSTAT has been received
    goto     i2c_fail
    
	
    ;send register adress
    banksel  SSP1BUF
    movlw    REG_DATA_INIT_SENSOR  ; Adress of the sensor init register
    movwf    SSP1BUF
    
    call    wait_SSPIF
    
    banksel SSP1CON2
    btfsc   SSP1CON2, ACKSTAT  ; check if ACKSTAT has been received
    goto    i2c_fail

    
    ;send second register adress
    banksel  SSP1BUF
    movlw    REG_CLEAR_REG_STATE ; Adress of the sensor clear register
    movwf    SSP1BUF
    
    call    wait_SSPIF
    
    banksel  SSP1CON2
    btfsc    SSP1CON2, ACKSTAT   ; check if ACKSTAT has been received
    goto     i2c_fail

    
    banksel SSP1CON2	   ; Second configuration register
    bsf	    SSP1CON2, PEN  ; set the PEN bit which stops the communication
    
    call    wait_SSPIF
    
    banksel INTCON
    bsf	    INTCON, 7
    
    ;make a first reading to compute the offset,
    ;we use busy waiting but only for the first time
    call read_weight_1
    
    banksel STEPPER_OR_SERVO
    bsf	    STEPPER_OR_SERVO, 1
    
    banksel T1CON
    movlw 00110000B
    movwf T1CON
    
    banksel TMR1H
    movlw 11100000B
    movwf TMR1L
    movlw 10110001B
    movwf TMR1H
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel TASK_READY
    btfss   TASK_READY, 4
    goto    $-1
    
    banksel TASK_READY
    bcf TASK_READY, 4
    
    banksel T1CON
    clrf    T1CON
    bcf T1CON, 0
    
    banksel STEPPER_OR_SERVO
    bcf	    STEPPER_OR_SERVO, 1
    
    call read_weight_2

    goto    buttons

read_weight_1:
    
    banksel INTCON
    bcf	    INTCON, 7

    
    banksel SSP1CON2		; Second configuration register
    bsf	    SSP1CON2, SEN	; set the SEN bit which initializes the communication
    
    call    wait_SSPIF
    
    ; send slave address
    banksel SSP1BUF
    movlw   ADDRESS_TRANSMITTER   ; last bit to 0 for master transmitter
    movwf   SSP1BUF
    
    call    wait_SSPIF
    
    banksel SSP1CON2
    btfsc   SSP1CON2, ACKSTAT  ; check if ACKSTAT has been received
    goto    i2c_fail
	
    
    ;send data register adress
    banksel  SSP1BUF
    movlw    WEIGHT_DATA_REGISTER ; Adress of the data register
    movwf    SSP1BUF
    
    call    wait_SSPIF
    
    
    banksel SSP1CON2
    btfsc   SSP1CON2, ACKSTAT  ; check if ACKSTAT has been received
    goto    i2c_fail

	
    banksel SSP1CON2	 ; Second configuration register
    bsf	    SSP1CON2,PEN ; set the PEN bit which stops the communication
    
    call    wait_SSPIF
    
    banksel INTCON
    bsf INTCON, 7
    
    return 
    
read_weight_2:
    ;###########################
    ; Data has been required, we now ask for an answer
    
    ;initialize index and memory 
    
    banksel INTCON
    bcf INTCON, 7

    banksel INDEX
    movlw   RAW_DATA_SIZE
    movwf   INDEX
    
    movlw   LOW	RAW_DATA
    movwf   FSR0L
    movlw   HIGH RAW_DATA
    movwf   FSR0H
    
    
    ; Start communication
    banksel SSP1CON2	
    bsf	    SSP1CON2, SEN  ; set the SEN bit which initializes the communication
    
    call    wait_SSPIF
   
    
    ; send slave address
    banksel SSP1BUF
    movlw   ADDRESS_RECEIVER	    ; last bit to 1 for master receiver
    movwf   SSP1BUF
    
    call    wait_SSPIF
    
    banksel SSP1CON2
    btfsc   SSP1CON2, ACKSTAT	; check if ACKSTAT has been received
    goto    i2c_fail
    
    
    
    acquire_data:

	banksel SSP1CON2
	bsf	SSP1CON2, RCEN	; enable receive mode
	
	call    wait_SSPIF	; wait until data is received
	
	banksel SSP1BUF		; read into buffer
	movf    SSP1BUF, W
	movwi	FSR0++		; store its content into DATA register, and go to 
				; the next memory location by using the ++ operator
	
	banksel	INDEX		
	decf	INDEX		; decrease index
	banksel	STATUS
	btfsc	STATUS, 2	; check if result is zero
	goto	data_acquired	; if yes, all data has been acquired
			    
	BANKSEL SSP1CON2	; if not, send ACK to keep receiving data
	bcf	SSP1CON2, ACKDT ; ACK DATA to send is 0, which is ACK.
	bsf	SSP1CON2, ACKEN ; Send ACK DATA now.
	
	call	wait_SSPIF
	
	goto	acquire_data
	
    data_acquired:
	
	BANKSEL SSP1CON2
	bsf	SSP1CON2, ACKDT ; ACK DATA to send is 1, which is NACK.
	bsf	SSP1CON2, ACKEN ; Send ACK DATA now.
	
	call	wait_SSPIF

	banksel SSP1CON2	
	bsf	SSP1CON2,PEN ; set the PEN bit which stops the communication

	call    wait_SSPIF
	
    call    process_data
    
    ;if first bit of w is equal to 0, it means the data is corrupted
    ;---> come back to main loop
    ;movwf   INDEX
    ;btfss   INDEX , 0
    ;goto    main_loop
    
    banksel INTCON
    bsf	    INTCON, 7
    return

    
main_loop:
    
reset_state:
    
    banksel STATE
    movlw 0x01
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto reset_wait
    
    banksel T1CON
    clrf    T1CON
    
    banksel STEPPER_OR_SERVO
    clrf STEPPER_OR_SERVO
    
    ;set pwm in "setup" mode
    banksel TRISC
    bsf	TRISC, 2
    banksel T2CON
    bcf	T2CON, 2

    banksel CCPR1L
    clrf    CCPR1L

     ;re-enable pwm
    banksel PIR1
    bcf PIR1, 1
    banksel TRISC
    bcf TRISC, 2

    banksel	CCP1CON
    clrf	CCP1CON

    banksel	TMR2
    clrf	TMR2

    banksel LATC
    bcf	LATC, 2
    
    ; cut the stepper
    banksel T1CON
    bcf T1CON, 0
    
    banksel LATD
    bsf LATD, 3
    
    ; reset the different tasks because we don't know when the reset button is push
    banksel TASK_READY
    bcf TASK_READY, 2
    bcf TASK_READY, 3
    bcf TASK_READY, 4
    bcf TASK_READY, 5
    
    ; move all the servos down
    ; all the LATx are in the same bank
    banksel LATA
    bsf LATA, 2
    bsf LATA, 3
    bsf LATA, 4
    ;cut all the LEDS
    bcf LATC, 7
    movlw 00101111B
    andwf LATD, f
    
    ;clear the output for the servo
    banksel LATC
    bcf LATC, 1
    
    banksel STEPPER_OR_SERVO
    bsf STEPPER_OR_SERVO, 0
    
    banksel POSITION
    movlw LOW_POSITION
    movwf POSITION
    movwf CURRENT_POSITION
    
    ; clear the wait of the servo
    clrf WAIT_SERVO
    
    ; wait of the servo when it is going down
    movlw SERVO_WAIT_DOWN
    movwf SERVO_WAIT_TIME
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x02
    movwf STATE
    
reset_wait:

    banksel STATE
    movlw 0x02
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto not_enough_water
    
    banksel TASK_READY
    btfss TASK_READY, 3
    goto not_enough_water
    
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto not_enough_water
    
    bcf BUTTONS_USE, 1
    ; a better idea than going directly to the init state would be instead that going
    ; to the end state where the machine wait that the user remove its glass
    movlw 0x05
    movwf STATE
    
    ;reset the LED of the wls if now there is water
    banksel SAME_BUTTONS_0
    btfss SAME_BUTTONS_0, 0
    goto not_enough_water
    
    ; clear the LED of the wls
    banksel LATD
    bcf LATD, 5
      
not_enough_water:
    
    banksel STATE
    movlw 0x03
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto init_servo
    
    banksel SAME_BUTTONS_0
    btfss SAME_BUTTONS_0, 0
    goto init_servo
    
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto init_servo
    
    ; needed to cut properly the pump
    
    bcf BUTTONS_USE, 1
    movf PREVIOUS_STATE, w
    movwf STATE
    
    ; clear the LED of the wls
    banksel LATD
    bcf LATD, 5
    
init_servo:
    
    banksel STATE
    movlw 0x04
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto init_platform
    
    ; enable all the servos
    banksel LATA
    bsf LATA, 2
    bsf LATA, 3
    bsf LATA, 4
    
    banksel STEPPER_OR_SERVO
    bsf STEPPER_OR_SERVO, 0
    
    banksel POSITION
    movlw LOW_POSITION
    movwf POSITION
    movwf CURRENT_POSITION
    
    ; clear the wait of the servo
    clrf WAIT_SERVO
    
    movlw SERVO_WAIT_DOWN
    movwf SERVO_WAIT_TIME
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x05
    movwf STATE
    
init_platform:
    
    banksel STATE
    movlw 0x05
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto alcohol_selection
    
    banksel TASK_READY
    btfss TASK_READY, 3
    goto alcohol_selection
    
    ; clear the flag of the servo
    bcf TASK_READY, 3
    
    ;clear the outputs to enable the servos
    banksel LATA
    bcf LATA, 2
    bcf LATA, 3
    bcf LATA, 4
    ;turn off all the LEDS except the wls LED
    bcf LATC, 7
    movlw 00101111B
    andwf LATD, f
    ; begin with choice 1 for the alcohol
    bsf LATD, 7
    bsf LATD, 4
    
    ; clear the choice of the alcohol
    banksel CHOICE
    clrf CHOICE
    bsf CHOICE, 0
    
    ; skip if the stop switch is already press
    banksel PORTA
    btfss PORTA, 1
    goto move_to_alcohol_selection
    
    banksel TASK_READY
    bcf TASK_READY, 1
    
    ; enable the stepper motor and make it move to the left
    banksel LATD
    bsf LATD, 0 ; the direction of the stepper
    bcf LATD, 3 ; the enable of the stepper
    
    ; reset the position and the target of the stepper
    banksel STEPPER_TARGET
    movlw 0xFF
    movwf STEPPER_TARGET
    
    clrf COUNTER_STEP
    movlw 0xFE
    movwf COUNTER_TURN
    
    banksel STEPPER_OR_SERVO
    clrf STEPPER_OR_SERVO
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x06
    movwf STATE
    
    goto alcohol_selection
    
move_to_alcohol_selection:
    banksel STATE
    movlw 0x06
    movwf STATE
    
    banksel TASK_READY
    bsf TASK_READY, 1
    
    clrf COUNTER_STEP
    clrf COUNTER_TURN
    
alcohol_selection:
    banksel STATE
    movlw 0x06
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto water_selection
    
    banksel TASK_READY
    btfss TASK_READY, 1
    goto water_selection
    
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 0
    goto validation_alcohol
    
    bcf BUTTONS_USE, 0
    
    ;clear the carry flag
    banksel STATUS
    bcf STATUS, 0
    
    banksel CHOICE
    rlf CHOICE, f
    
    btfss CHOICE, 3
    goto choice_1
    
    bcf CHOICE, 3
    bsf CHOICE, 0
    
choice_1: 
    banksel CHOICE
    btfss CHOICE, 0
    goto choice_2
    
    ; Turn off LED 3 and turn on LED 1
    banksel LATD
    bcf LATC, 7
    bsf LATD, 7

choice_2:
    banksel CHOICE
    btfss CHOICE, 1
    goto choice_3
    
    ; Turn off LED 1 and turn on LED 2
    banksel LATD
    bcf LATD, 7
    bsf LATD, 6

choice_3:
    banksel CHOICE
    btfss CHOICE, 2
    goto validation_alcohol
    
    ; Turn off LED 2 and turn on LED 3
    banksel LATC
    bcf LATD, 6
    bsf LATC, 7

validation_alcohol:
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto water_selection
    
    banksel CHOICE
    bcf CHOICE, 4
    
    banksel STATE
    movlw 0x07
    movwf STATE
    
    bcf BUTTONS_USE, 1
    
water_selection:
    banksel STATE
    movlw 0x07
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto displacement_to_alcohol
    
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 0
    goto validation_water
    
    movlw 00010000B
    banksel CHOICE
    xorwf CHOICE, f
    bcf BUTTONS_USE, 0
    
    ; xor the LED 4
    banksel LATD
    movlw 00010000B
    xorwf LATD, f
    
validation_water:
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto displacement_to_alcohol
    
    ; cut all the LED
    banksel LATC
    bcf LATC, 7
    movlw 00101111B
    andwf LATD, f
    
    banksel STATE
    movlw 0x08
    movwf STATE
    bcf BUTTONS_USE, 1
    
displacement_to_alcohol:
    banksel STATE
    movlw 0x08
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto move_up_servo
    
    banksel LATD
    bcf LATD, 0 ; choose the direction of the stepper
    bcf LATD, 3 ; turn on the stepper
    
    ;Turn on the first LED for the dowload bar
    banksel LATD
    bsf LATD, 7
    
    banksel STEPPER_TARGET
    btfsc CHOICE, 0
    movlw ALCOHOL_1_POS
    
    btfsc CHOICE, 1
    movlw ALCOHOL_2_POS
    
    btfsc CHOICE, 2
    movlw ALCOHOL_3_POS
    
    movwf STEPPER_TARGET
    
    banksel STEPPER_OR_SERVO
    clrf STEPPER_OR_SERVO
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x09
    movwf STATE
    
    goto move_up_servo
    
move_up_servo:
    
    banksel STATE
    movlw 0x09
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto move_down_servo
    
    banksel TASK_READY
    btfss TASK_READY, 2
    goto move_down_servo
    
    ; reset the flag of end of destination
    bcf TASK_READY, 2
    
    ;Turn on the second LED for the dowload bar
    banksel LATD
    bsf LATD, 6
    
    banksel CHOICE
    btfss CHOICE, 0
    goto alcohol_2

   ; turn on the first servo
    banksel LATA
    bsf LATA, 2
    
    goto launch_servo
alcohol_2:
    banksel CHOICE
    btfss CHOICE, 1
    goto alcohol_3
    
    ; turn on the second servo
    banksel LATA
    bsf LATA, 3
    
    goto launch_servo

alcohol_3:
    ; turn on the third servo
    banksel LATA
    bsf LATA, 4

launch_servo:
    banksel STEPPER_OR_SERVO
    bsf STEPPER_OR_SERVO, 0
    
    banksel POSITION
    movlw HIGH_POSITION
    movwf POSITION
    
    ; clear the wait of the servo
    clrf WAIT_SERVO
    
    movlw SERVO_WAIT_UP
    movwf SERVO_WAIT_TIME
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x0A
    movwf STATE
    
move_down_servo:
    
    banksel STATE
    movlw 0x0A
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto goto_water
    
    banksel TASK_READY
    btfss TASK_READY, 3
    goto goto_water
    
    ; clear the flag of the servo
    bcf TASK_READY, 3
    
    ;clear the output for the servo
    banksel LATC
    bcf LATC, 1
    
    ;Turn on the third LED for the dowload bar
    banksel LATC
    bsf LATC, 7
    
    banksel STEPPER_OR_SERVO
    bsf STEPPER_OR_SERVO, 0
    
    banksel POSITION
    movlw LOW_POSITION
    movwf POSITION
    
    ; clear the wait of the servo
    clrf WAIT_SERVO
    
    movlw SERVO_WAIT_DOWN
    movwf SERVO_WAIT_TIME
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x0B
    movwf STATE
    
goto_water:
    
    banksel STATE
    movlw 0x0B
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto pump_1
    
    banksel TASK_READY
    btfss TASK_READY, 3
    goto pump_1
    
    ; clear the flag of the servo
    bcf TASK_READY, 3
    
    ;clear the output for the servo
    banksel LATC
    bcf LATC, 1
    
    ;turn off all the servos
    banksel LATA
    bcf LATA, 2
    bcf LATA, 3
    bcf LATA, 4
    
    banksel CHOICE
    btfss CHOICE, 4
    goto we_want_water
    
    banksel TASK_READY
    bsf TASK_READY, 5
    
    banksel STATE
    movlw 0x0E
    movwf STATE
    
    goto goto_end_destination
    
we_want_water:  
    ; launch the stepper motor
    banksel LATD
    bsf LATD, 0 ; choose the direction of the stepper
    bcf LATD, 3 ; enable the stepper
    
    banksel STEPPER_TARGET
    movlw PUMP_POSITION
    movwf STEPPER_TARGET
    
    banksel STEPPER_OR_SERVO
    clrf STEPPER_OR_SERVO
    
    banksel T1CON
    bsf T1CON, 0
  
    ; Configure Pump
    banksel CCP1CON
    movlw 00111111B ; le 2 est pour le duty cycle
    movwf CCP1CON
    
    banksel STATE
    movlw 0x0C
    movwf STATE
    
pump_1:
    banksel STATE
    movlw 0x0C
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto pump_2
    
    banksel TASK_READY
    btfss TASK_READY, 2
    goto pump_2
    
    call read_weight_1
   
    ;######################## 
    ; 20 ms delay before asking the answer
    
    banksel STEPPER_OR_SERVO
    bsf	    STEPPER_OR_SERVO, 1
    
    banksel T1CON
    movlw 00110000B
    movwf T1CON
    
    banksel TMR1H
    movlw 11100000B
    movwf TMR1L
    movlw 10110001B
    movwf TMR1H
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x0D
    movwf STATE
    
    
pump_2:
    banksel STATE
    movlw 0x0D
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto goto_end_destination
    
    banksel TASK_READY
    btfss TASK_READY, 4
    goto goto_end_destination
    
    banksel TASK_READY
    bcf TASK_READY, 4
    
    banksel T1CON
    clrf    T1CON
    
    banksel STEPPER_OR_SERVO
    bcf	    STEPPER_OR_SERVO, 1
    
    call read_weight_2

    ;Put the 8 interesting bits into DATA_MSB
    banksel DATA_LSB
    rlf	    DATA_LSB, 1
    rlf	    DATA_MSB, 1
    rlf	    DATA_LSB, 1
    rlf	    DATA_MSB, 1
    rlf	    DATA_LSB, 1
    rlf	    DATA_MSB, 1
    rlf	    DATA_LSB, 1
    rlf	    DATA_MSB, 1
    
    ;move it into DATA_MUL
    banksel DATA_MSB
    movf    DATA_MSB, W
    banksel DATA_MUL
    movwf   DATA_MUL
    banksel DATA_MSB
    clrf    DATA_MSB
    clrf    DATA_LSB
    
    ;check if control is finished : 
    ;if(data_mul >= goal_weight - tolerance)
    
    
    banksel DATA_MUL	; DATA_MUL -> TEMP_VAR
    movf    DATA_MUL, W
    banksel TEMP_VAR
    movwf   TEMP_VAR
    
    movlw   TOLERANCE
    sublw   GOAL_WEIGHT	; GOAL_WEIGHT - TOLERANCE -> w
    
    banksel TEMP_VAR	; DATA_MUL - (GOAL_WEIGHT - TOLERANCE) -> TEMP_VAR
    subwf   TEMP_VAR
    banksel STATUS
    btfss   STATUS, 0
    goto    pid
    goto    control_finished
    
    ;It means PID is finished, stop pump 
    control_finished:

	banksel INTCON
	bsf	INTCON, 7

	;set pwm in "setup" mode
	banksel TRISC
	bsf	TRISC, 2
	banksel T2CON
	bcf	T2CON, 2
	
	banksel CCPR1L
	clrf    CCPR1L
	
	 ;re-enable pwm
	banksel PIR1
	bcf PIR1, 1
	banksel TRISC
	bcf TRISC, 2
	
	banksel	CCP1CON
	clrf	CCP1CON

	banksel	TMR2
	clrf	TMR2
	
	banksel LATC
	bcf	LATC, 2
	
	banksel STEPPER_OR_SERVO
	bsf STEPPER_OR_SERVO, 2

	banksel T1CON
	movlw 00110000B
	movwf T1CON

	banksel TMR1H
	clrf TMR1H
	clrf TMR1L

	banksel T1CON
	bsf T1CON, 0
	
	banksel	STATE
	movlw	0x0E
	movwf	STATE
	
	goto    goto_end_destination
    
    
    
    ;##################################################################
    ;#			P(ID) CONTROLLER			      #
    ;##################################################################
    
    pid:
    
    ; re-activate global interrupt, since i2c is finished 
    banksel INTCON
    bsf	    INTCON, 7

    ;compute e_k by substracting the reading from goal weight
    banksel E_K
    movlw   GOAL_WEIGHT
    movwf   E_K
    
    banksel DATA_MUL
    movf    DATA_MUL, W
    clrf    DATA_MUL
    banksel E_K
    subwf   E_K, 1  ; E_K <-- GOAL_WEIGHT - DATA_MUL
    
    ;MULTIPLY E_K BY K_P
    
    ;initialize registers
    banksel KP_REG
    movlw   KP
    movwf   KP_REG
    banksel SHIFT_COUNTER
    movlw   NB_BITS_REG_B
    movwf   SHIFT_COUNTER
    banksel UP
    clrf    UP
    
    
    multiplication_loop:
	;check if counter is equal to zero
	banksel	SHIFT_COUNTER
	movf    SHIFT_COUNTER, W
	xorlw   0x00
	banksel STATUS
	btfsc   STATUS, 2       
	goto    end_multiplication

	; decrease index
	banksel SHIFT_COUNTER
	decf    SHIFT_COUNTER		
	; check if last bit of KP_REG is equal to 1
	banksel KP_REG
	btfss   KP_REG, 0      
	goto    SHIFT_A

	; Ajouter REG_A à REG_RESULT
	goto    ADD_REG_A_REG_RESULT

    SHIFT_A:
	;clear the carry flag
	banksel STATUS
	bcf STATUS, 0

	; Shift REG_A to the right
	banksel E_K
	rlf     E_K, F	   

	;check if overflow happened
	banksel STATUS
	btfss	STATUS, 0
	goto	not_overflow
	
	banksel KP_REG
	movlw	0x00
	xorwf	KP_REG, w
	banksel	STATUS
	btfsc   STATUS, 2
	goto	not_overflow

	goto	overflow
	
    not_overflow:
    
	;clear carry flag
	banksel	STATUS
	bcf	STATUS, 0
	
	; Shift REG_B to the right
	banksel KP_REG
	rrf     KP_REG, F

	; loop
	goto   multiplication_loop
	
    ADD_REG_A_REG_RESULT:
	; Add REG_A (E_K) to REG_RESULT
	banksel E_K
	movf    E_K, W
	banksel UP
	addwf   UP, F

	goto SHIFT_A
	
    overflow:
	banksel	UP
	movlw	11111111B
	movwf	UP
	goto	end_multiplication
	
    end_multiplication:
	
    ;Now that we got u_p, we should map it to the duty cycle of the pump 

    banksel UP
    movf    UP, W
    call    map
    banksel PUMP_VALUE
    movwf   PUMP_VALUE
    
    ;################################################################
    ;#			 SEND PWM VALUE TO PUMP			    #
    ;################################################################
    
    ;set pwm in "setup" mode
    banksel TRISC
    bsf TRISC, 2
    banksel T2CON
    bcf	    T2CON, 2
    
    ;Change value of the duty cycle 
    banksel PUMP_VALUE
    movf    PUMP_VALUE, W
    banksel CCPR1L
    movwf   CCPR1L
    
    ;re-enable pwm
    banksel PIR1
    bcf PIR1, 1
    banksel T2CON
    bsf T2CON, 2
    banksel TRISC
    bcf TRISC, 2
    
    banksel STATE
    movlw 0x0C
    movwf STATE
    
    banksel TASK_READY
    bsf TASK_READY, 2

goto_end_destination:
    banksel STATE
    movlw 0x0E
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto arrive_to_end_destination
    
    banksel TASK_READY
    btfss TASK_READY, 5
    goto arrive_to_end_destination
    
    banksel T1CON
    clrf    T1CON
    
    banksel TASK_READY
    bcf TASK_READY, 5
    
    banksel STEPPER_OR_SERVO
    bcf	    STEPPER_OR_SERVO, 2
    
    ;clear the flag of end of destination
    bcf TASK_READY, 2
    
    ; launch the stepper
    banksel LATD
    bsf LATD, 0 ; choose the direction of the stepper
    bcf LATD, 3 ; enable the stepper
    
    banksel STEPPER_TARGET
    movlw END_DESTINATION
    movwf STEPPER_TARGET
    
    banksel STEPPER_OR_SERVO
    clrf STEPPER_OR_SERVO
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x0F
    movwf STATE
    
arrive_to_end_destination:
    ; le projet est d'attendre que le weigth sensor d tecte l'enl vement du verre, ensuite attende quelques secondes
    ; puis reset la position de la boite.
    
    banksel STATE
    movlw 0x0F
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto buttons
    
    banksel TASK_READY
    btfss TASK_READY, 2
    goto buttons
    
    ; Turn on the last LED of the download bar
    banksel LATD
    bsf LATD, 4
    
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto buttons
    
    ; reset the use of the buttons
    bcf BUTTONS_USE, 1
    
    ;clear the flag of end of destination
    banksel TASK_READY
    bcf TASK_READY, 2
    
    ;allow to enter in the init_stepper state
    bsf TASK_READY, 3
    
    movlw 0x05
    movwf STATE
    
buttons:
    
    banksel BUTTONS_USE
    clrf BUTTONS_USE
    
    banksel TASK_READY
    btfss TASK_READY, 0
    goto main_loop
    
    banksel BUTTONS_USE
    clrf BUTTONS_USE
    
    ;disable global interrupts
    banksel INTCON
    bcf INTCON, 7
    
    banksel BUTTONS_VALUES
    movf BUTTONS_VALUES_INT, w
    movwf BUTTONS_VALUES
    movf OLD_BUTTONS_VALUES_INT, w
    movwf OLD_BUTTONS_VALUES
    
    bcf TASK_READY, 0
    
    ; enable global interrupts
    banksel INTCON
    bsf INTCON, 7
    
    banksel SAME_BUTTONS_0
    clrf SAME_BUTTONS_0
    clrf SAME_BUTTONS_1
    
    call check_if_buttons_0
    
    call check_if_buttons_1

    banksel SAME_BUTTONS_0
    btfss SAME_BUTTONS_0, 5
    goto off_1
    
    banksel BUTTONS_PRESS
    btfsc BUTTONS_PRESS, 5
    goto next_button_1
    
    banksel BUTTONS_USE
    bsf BUTTONS_USE, 0
    
    banksel BUTTONS_PRESS
    bsf BUTTONS_PRESS, 5
    goto next_button_1
    
    off_1:
    
	banksel SAME_BUTTONS_1
	btfss SAME_BUTTONS_1, 5
	goto next_button_1
	
	bcf BUTTONS_PRESS, 5
    
next_button_1:
    
    banksel SAME_BUTTONS_0
    btfss SAME_BUTTONS_0, 4
    goto off_2
    
    banksel BUTTONS_PRESS
    btfsc BUTTONS_PRESS, 4
    goto next_button_2
    
    banksel BUTTONS_USE
    bsf BUTTONS_USE, 1
    
    banksel BUTTONS_PRESS
    bsf BUTTONS_PRESS, 4
    goto next_button_2
    
    off_2:
    
	banksel SAME_BUTTONS_1
	btfss SAME_BUTTONS_1, 4
	goto next_button_2
	
	bcf BUTTONS_PRESS, 4
    
    next_button_2:
    
	; the tension becomes a logic 1 when the sensor is out of the water
	banksel SAME_BUTTONS_1
	btfss SAME_BUTTONS_1, 0
	goto next_wls
	
	banksel STATE
	movlw 0x03
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto next_wls
	
	; Turn on LED 5
	banksel LATD
	bsf LATD, 5
	
	;discard the information if the machine is in reset state
	; go to off_3 because we only allow the check of the release of the button
	; it is unuseful to check if the button is press
	banksel STATE
	movlw 0x01
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto off_3
	
	banksel STATE
	movlw 0x02
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto off_3
	
	banksel STATE
	movlw 0x0C
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto off_3
	
	banksel STATE
	movlw 0x0D
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto off_3
	
	banksel STATE
	movf STATE, w
	movwf PREVIOUS_STATE
	
	movlw 0x03
	movwf STATE

    next_wls:
    
	banksel SAME_BUTTONS_0
	btfss SAME_BUTTONS_0, 3
	goto off_3

	banksel BUTTONS_PRESS
	btfsc BUTTONS_PRESS, 3
	goto next_button_3

	;check if the machine is not already in reset state
	banksel STATE
	movlw 0x01
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto next_button_3
	
	banksel STATE
	movlw 0x02
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto next_button_3
	
	banksel STATE
	movlw 0x01
	movwf STATE
	
	banksel BUTTONS_PRESS
	bsf BUTTONS_PRESS, 3
	goto next_button_3
	
    off_3:
    
	banksel SAME_BUTTONS_1
	btfss SAME_BUTTONS_1, 3
	goto next_button_3	
	bcf BUTTONS_PRESS, 3
    
    next_button_3:
	goto    main_loop
    
check_if_buttons_0:
    
    banksel BUTTONS_VALUES
    
    btfsc BUTTONS_VALUES, 5
    goto second_button_0
    
    btfss OLD_BUTTONS_VALUES, 5
    bsf SAME_BUTTONS_0, 5
    
    second_button_0:
    
	btfsc BUTTONS_VALUES, 4
	goto third_button_0
    
	btfss OLD_BUTTONS_VALUES, 4
	bsf SAME_BUTTONS_0, 4
	
    third_button_0:
	
	btfsc BUTTONS_VALUES, 3
	goto wls_0
    
	btfss OLD_BUTTONS_VALUES, 3
	bsf SAME_BUTTONS_0, 3
	
    wls_0:
	
	btfsc BUTTONS_VALUES, 0
	goto end_0
    
	btfss OLD_BUTTONS_VALUES, 0
	bsf SAME_BUTTONS_0, 0
    
    end_0:

	return

check_if_buttons_1:
    
    banksel BUTTONS_VALUES
    
    btfss BUTTONS_VALUES, 5
    goto second_button_1
    
    btfsc OLD_BUTTONS_VALUES, 5
    bsf SAME_BUTTONS_1, 5
    
    second_button_1:
    
	btfss BUTTONS_VALUES, 4
	goto third_button_1
    
	btfsc OLD_BUTTONS_VALUES, 4
	bsf SAME_BUTTONS_1, 4
	
    third_button_1: 
    
	btfss BUTTONS_VALUES, 3
	goto wls_1
    
	btfsc OLD_BUTTONS_VALUES, 3
	bsf SAME_BUTTONS_1, 3
	
    wls_1:
	
	btfss BUTTONS_VALUES, 0
	goto end_1
    
	btfsc OLD_BUTTONS_VALUES, 0
	bsf SAME_BUTTONS_1, 0
    
    end_1:
	return

    

wait_SSPIF:
    banksel PIR1
    btfss   PIR1, SSP1IF
    goto    $-1
    bcf	    PIR1, SSP1IF
    return    
    
i2c_fail:

    banksel INTCON
    bsf INTCON, 7
    
    banksel STEPPER_OR_SERVO
    bsf	    STEPPER_OR_SERVO, 1

    banksel T1CON
    movlw 00110000B
    movwf T1CON
    
    banksel TMR1H
    movlw 11100000B
    movwf TMR1L
    movlw 10110001B
    movwf TMR1H
    
    banksel T1CON
    bsf T1CON, 0
    
    banksel TASK_READY
    btfss   TASK_READY, 4
    goto    $-1
    
    bcf TASK_READY, 4

    banksel T1CON
    clrf    T1CON
    
    banksel STEPPER_OR_SERVO
    bcf	    STEPPER_OR_SERVO, 1
    
    banksel STATE
    movlw 0x01
    movwf STATE
    goto main_loop
    
    
;#############################################################################
;  PROCESS_DATA : function responsible of :				     #
;	-> checking if first byte is well equal to 0x12 (otherwise, return 0)#
;	-> moving the second and third bytes into DATA registers	     #
;	-> if second byte equal to 11111111, move 0 into DATA registers	     #
;#############################################################################
    
process_data:
    ;Check if first byte is equal to 0x12 (meaning the sensor is ready)
    banksel RAW_DATA
    movf    RAW_DATA, W		     ; Load the first byte of data into W
    xorlw   0x12
    movlw   0x01
    banksel STATUS
    btfss   STATUS, 2		    ; check if result of the xor is zero	    
    movlw   0x00		    ; if not, return 0 meaning that data is corrupted
    banksel INDEX
    movwf   INDEX
    btfss   INDEX, 0
    return
    
    ;move second byte of RAW DATA into w
    banksel RAW_DATA
    movlw   HIGH(RAW_DATA)	 ; Load the high byte of the address
    movwf   FSR0H                ; Move it to FSR0H
    movlw   LOW(RAW_DATA)	 ; Load the low byte of the address
    movwf   FSR0L                ; Move it to FSR0L
    incf    FSR0L
    movf    INDF0, W

    ;check if it is equal to 11111111 
    xorlw	0xff
    banksel	STATUS
    btfsc	STATUS, 2
    goto	data_zero
    
    
    ;move it into DATA_MSB
    banksel RAW_DATA
    movlw   HIGH(RAW_DATA)	; Load the high byte of the address
    movwf   FSR0H               ; Move it to FSR0H
    movlw   LOW(RAW_DATA)	; Load the low byte of the address
    movwf   FSR0L               ; Move it to FSR0L
    incf    FSR0L
    movf    INDF0, W
    banksel DATA_MSB		;Move the content into DATA_MSB
    movwf   DATA_MSB


    ;move third byte of RAW DATA into DATA_LSB
    banksel	RAW_DATA
    movlw	HIGH(RAW_DATA)	     ; Load the high byte of the address
    movwf	FSR0H                ; Move it to FSR0H
    movlw	LOW(RAW_DATA)	     ; Load the low byte of the address
    movwf	FSR0L                ; Move it to FSR0L
    incf	FSR0L
    incf	FSR0L
    movf	INDF0, W
    banksel	DATA_LSB
    movwf	DATA_LSB
    
    ;check if offset is already set or not 
    banksel OFFSET_FLAG
    btfss   OFFSET_FLAG, 0
    goto    offset
    
    ;Remove offset
    banksel OFFSET_LSB
    movf    OFFSET_LSB, W
    banksel DATA_LSB
    subwf   DATA_LSB, 1
    
    banksel STATUS
    btfss   STATUS, 0
    decf    DATA_MSB, 1
    
    banksel OFFSET_MSB
    movf    OFFSET_MSB, W
    banksel DATA_MSB
    subwf   DATA_MSB, 1
    
    banksel STATUS	; if negative value, return 0
    btfss   STATUS, 0
    goto    data_zero
    
    movlw	0x01
    return
	
data_zero:
    clrw
    banksel	DATA_MSB 
    movwf	DATA_MSB
    banksel	DATA_LSB
    movwf	DATA_LSB
    movlw	0x01
    return
    
offset:
    ;move the content of DATA_MSB and DATA_LSB into OFFSET
    banksel DATA_MSB
    movf    DATA_MSB, W
    banksel OFFSET_MSB
    movwf   OFFSET_MSB
    
    banksel DATA_LSB
    movf    DATA_LSB, W
    banksel OFFSET_LSB
    movwf   OFFSET_LSB
    
    ;set the offset flag
    bsf	    OFFSET_FLAG, 0
    ; return 0
    goto    data_zero
    
;###################################################
;#    MAP : lookup table responsible for	   #
;#	    converting the pid value into	   #
;#	    duty cycle using a lookup table	   #
;###################################################
    
map:
    brw
    retlw 01100000B
    retlw 01100000B
    retlw 01100001B
    retlw 01100001B
    retlw 01100010B
    retlw 01100011B
    retlw 01100011B
    retlw 01100100B
    retlw 01100100B
    retlw 01100101B
    retlw 01100110B
    retlw 01100110B
    retlw 01100111B
    retlw 01101000B
    retlw 01101000B
    retlw 01101001B
    retlw 01101001B
    retlw 01101010B
    retlw 01101011B
    retlw 01101011B
    retlw 01101100B
    retlw 01101101B
    retlw 01101101B
    retlw 01101110B
    retlw 01101110B
    retlw 01101111B
    retlw 01110000B
    retlw 01110000B
    retlw 01110001B
    retlw 01110010B
    retlw 01110010B
    retlw 01110011B
    retlw 01110011B
    retlw 01110100B
    retlw 01110101B
    retlw 01110101B
    retlw 01110110B
    retlw 01110111B
    retlw 01110111B
    retlw 01111000B
    retlw 01111000B
    retlw 01111001B
    retlw 01111010B
    retlw 01111010B
    retlw 01111011B
    retlw 01111100B
    retlw 01111100B
    retlw 01111101B
    retlw 01111101B
    retlw 01111110B
    retlw 01111111B
    retlw 01111111B
    retlw 10000000B
    retlw 10000001B
    retlw 10000001B
    retlw 10000010B
    retlw 10000010B
    retlw 10000011B
    retlw 10000100B
    retlw 10000100B
    retlw 10000101B
    retlw 10000110B
    retlw 10000110B
    retlw 10000111B
    retlw 10000111B
    retlw 10001000B
    retlw 10001001B
    retlw 10001001B
    retlw 10001010B
    retlw 10001011B
    retlw 10001011B
    retlw 10001100B
    retlw 10001100B
    retlw 10001101B
    retlw 10001110B
    retlw 10001110B
    retlw 10001111B
    retlw 10010000B
    retlw 10010000B
    retlw 10010001B
    retlw 10010001B
    retlw 10010010B
    retlw 10010011B
    retlw 10010011B
    retlw 10010100B
    retlw 10010101B
    retlw 10010101B
    retlw 10010110B
    retlw 10010110B
    retlw 10010111B
    retlw 10011000B
    retlw 10011000B
    retlw 10011001B
    retlw 10011001B
    retlw 10011010B
    retlw 10011011B
    retlw 10011011B
    retlw 10011100B
    retlw 10011101B
    retlw 10011101B
    retlw 10011110B
    retlw 10011110B
    retlw 10011111B
    retlw 10100000B
    retlw 10100000B
    retlw 10100001B
    retlw 10100010B
    retlw 10100010B
    retlw 10100011B
    retlw 10100011B
    retlw 10100100B
    retlw 10100101B
    retlw 10100101B
    retlw 10100110B
    retlw 10100111B
    retlw 10100111B
    retlw 10101000B
    retlw 10101000B
    retlw 10101001B
    retlw 10101010B
    retlw 10101010B
    retlw 10101011B
    retlw 10101100B
    retlw 10101100B
    retlw 10101101B
    retlw 10101101B
    retlw 10101110B
    retlw 10101111B
    retlw 10101111B
    retlw 10110000B
    retlw 10110001B
    retlw 10110001B
    retlw 10110010B
    retlw 10110010B
    retlw 10110011B
    retlw 10110100B
    retlw 10110100B
    retlw 10110101B
    retlw 10110110B
    retlw 10110110B
    retlw 10110111B
    retlw 10110111B
    retlw 10111000B
    retlw 10111001B
    retlw 10111001B
    retlw 10111010B
    retlw 10111011B
    retlw 10111011B
    retlw 10111100B
    retlw 10111100B
    retlw 10111101B
    retlw 10111110B
    retlw 10111110B
    retlw 10111111B
    retlw 11000000B
    retlw 11000000B
    retlw 11000001B
    retlw 11000001B
    retlw 11000010B
    retlw 11000011B
    retlw 11000011B
    retlw 11000100B
    retlw 11000101B
    retlw 11000101B
    retlw 11000110B
    retlw 11000110B
    retlw 11000111B
    retlw 11001000B
    retlw 11001000B
    retlw 11001001B
    retlw 11001010B
    retlw 11001010B
    retlw 11001011B
    retlw 11001011B
    retlw 11001100B
    retlw 11001101B
    retlw 11001101B
    retlw 11001110B
    retlw 11001110B
    retlw 11001111B
    retlw 11010000B
    retlw 11010000B
    retlw 11010001B
    retlw 11010010B
    retlw 11010010B
    retlw 11010011B
    retlw 11010011B
    retlw 11010100B
    retlw 11010101B
    retlw 11010101B
    retlw 11010110B
    retlw 11010111B
    retlw 11010111B
    retlw 11011000B
    retlw 11011000B
    retlw 11011001B
    retlw 11011010B
    retlw 11011010B
    retlw 11011011B
    retlw 11011100B
    retlw 11011100B
    retlw 11011101B
    retlw 11011101B
    retlw 11011110B
    retlw 11011111B
    retlw 11011111B
    retlw 11100000B
    retlw 11100001B
    retlw 11100001B
    retlw 11100010B
    retlw 11100010B
    retlw 11100011B
    retlw 11100100B
    retlw 11100100B
    retlw 11100101B
    retlw 11100110B
    retlw 11100110B
    retlw 11100111B
    retlw 11100111B
    retlw 11101000B
    retlw 11101001B
    retlw 11101001B
    retlw 11101010B
    retlw 11101011B
    retlw 11101011B
    retlw 11101100B
    retlw 11101100B
    retlw 11101101B
    retlw 11101110B
    retlw 11101110B
    retlw 11101111B
    retlw 11110000B
    retlw 11110000B
    retlw 11110001B
    retlw 11110001B
    retlw 11110010B
    retlw 11110011B
    retlw 11110011B
    retlw 11110100B
    retlw 11110101B
    retlw 11110101B
    retlw 11110110B
    retlw 11110110B
    retlw 11110111B
    retlw 11111000B
    retlw 11111000B
    retlw 11111001B
    retlw 11111010B
    retlw 11111010B
    retlw 11111011B
    retlw 11111011B
    retlw 11111100B
    retlw 11111101B
    retlw 11111101B
    retlw 11111110B
    retlw 11111111B