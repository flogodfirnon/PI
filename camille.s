; ********************************************************************************
; CAMILLE : this is the final code of the project Camille (the cocktail machine) *
; ********************************************************************************

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

; I2C adresses
#define	    ADDRESS_TRANSMITTER	    11001000B	; adress of the weight sensor in transmitter mode
#define	    ADDRESS_RECEIVER	    11001001B	; adress of the weight sensor in receiver mode
#define	    REG_DATA_INIT_SENSOR    01110000B	; initialization register
#define	    REG_CLEAR_REG_STATE	    01100101B	; message to be sent for clearing the registers 
#define	    WEIGHT_DATA_REGISTER    01100110B	; register containing the reading
    
#define	    RAW_DATA_SIZE		    4	; number of bytes sent by the weight sensor

#define	    GOAL_WEIGHT		    01110011B	; weight of the full glass
#define	    TOLERANCE		    00000011B	; tolerance w. r. t. the weight
#define	    KP			    00000011B	; proportional constant for the control
#define	    NB_BITS_REG_B		    8	; number of bits for the multiplication
    

#define STEP_COUNT 7	    ; constant that define the max value of COUNTER_STEP
#define HIGH_POSITION 10    ; define the duty cycle to put the servo in high position
#define LOW_POSITION 16	    ; define the duty cycle to put the servo in low position
#define SERVO_STEP 160	    ; define the period of the PWM for the servo

; each of the waitings below are relative to a 20 ms counter
#define SERVO_WAIT_UP 190       ; waiting when the servos is up
#define SERVO_WAIT_DOWN 20      ; waiting when the servos is down
#define SERVO_WAIT_TIME_PROG 6  ; constant for the movement of the servos progressively up 
			        ; or down and to wait before intermediate positions
				
;each of the positions are for the sliding box 
#define END_DESTINATION 5 ; position of the end destination
#define PUMP_POSITION 32  ; position of the pump
#define ALCOHOL_1_POS 113 ; position of the first alcohol dispenser
#define ALCOHOL_2_POS 180 ; position of the second alcohol dispenser
#define ALCOHOL_3_POS 250 ; position of the third alcohol dispenser
				
#define SEC_WAITING 35  ; constant to compare with the counter of the timer that must wait 
			; around 2 sec
      
    
PSECT udata_bank2
;###################################################
; first three bits equals to zero if stepper is on #
; bit 0 equal to 1 if servo is on                  #
; bit 1 equal to 1 if timer of 10 ms is on         #
; bit 2 equal to 1 if timer of around 2 sec is on  #
;###################################################
TIMER_1_CHOICE: DS 1
SEC_WAITING_COUNTER: DS 1 ; counter for the timer of around 2 sec
			  ; to use this timer, the prescaler must be eigth

; For the weight sensor 
RAW_DATA:	  DS	    RAW_DATA_SIZE   ; array receiving the raw data 
					    ; of the weight sensor
DATA_MSB:	  DS	    1		    ; contains the two interesting byte of the sensor reading (2nd and 3rd)
DATA_LSB:	  DS	    1		  
OFFSET_MSB:	  DS	    1		    ; weight offset that is removed from each reading
OFFSET_LSB:	  DS	    1
DATA_MUL:	  DS	    1		    ; 8 final bits that are kept from the reading of the weight sensor
INDEX:		  DS	    1		    ; index variable for the loops 
	
;Flag register
OFFSET_FLAG:	  DS	    1		    ; flag that indicates wether the offset has been computed or not
    
; Control part
E_K:		  DS	    1		    ; error at time k
SHIFT_COUNTER:	  DS	    1		    ; useful variable for multiplication
TEMP_VAR:	  DS	    1		    ; temporar variable for arithmetics
KP_REG:		  DS	    1		    ; register to store k_p
UP:		  DS	    1		    ; proportional term (control)
PUMP_VALUE:	  DS	    1		    ; PWM value sent to pump 
    

; variables for the stepper
COUNTER_STEP: DS 1  ; counter to increase the spatial distance between two values of COUNTER_TURN
COUNTER_TURN: DS 1  ; store the position of the box on the machine
STEPPER_TARGET: DS 1 ; store the target of the stepper

; variables for the buttons
BUTTONS_VALUES_INT: DS 1 ; variables to store the values of the buttons in the interrupt routine
OLD_BUTTONS_VALUES_INT: DS 1 ; store the old values of the buttons in the interrupt routine
BUTTONS_VALUES: DS 1 ; store the values of the buttons in the main loop
OLD_BUTTONS_VALUES: DS 1 ; store the old values of the buttons in the main loop
SAME_BUTTONS_1: DS 1 ; if bit equal to one, the corresponding button is in a 1 state
SAME_BUTTONS_0: DS 1 ; if bit equal to one, the corresponding button is in a 0 state
BUTTONS_PRESS: DS 1 ; tell when the button was already press at the previous loop iteration
BUTTONS_USE: DS 1 ; tell if the button must be use by setting the value to one
READING: DS 1 ; store a reading on a specific port

;variables for the servos
POSITION: DS 1  ; specify the position wanted for the servo
CURRENT_POSITION: DS 1 ;current position of the servos (useful for the progressive movement)
COUNTER_SERVO: DS 1 ;counter to generate the PWM of the servos
WAIT_SERVO: DS 1 ; Count to be sure that the dispenser has completed its operation
SERVO_WAIT_TIME: DS 1 ; store the time to wait after the movement of the servo

; variables for the processing of the machine
 
; Bit 0 is set: task button is ready
; Bit 1 is set: the stop switch has been pushed
; Bit 2 is set: the stepper is arrived at destination
; Bit 3 is set: the servo has finished its operation
; Bit 4 is set: the timer of 10ms has finished
; Bit 5 is set: the timer of 1s has finished
TASK_READY: DS 1
; Bit 0 is set: the first alcohol has been choosen
; Bit 1 is set: the second alcohol has been choosen
; Bit 2 is set: the third alcohol has been choosen
; Bit 4 is set: not water is choosen
CHOICE: DS 1
STATE: DS 1 ; state of the finite-state machine (the state 0 is used for debugging)
PREVIOUS_STATE: DS 1 ; previous state when moving to not enough water state

    
PSECT rst_vec, abs, class = CODE, delta=2
ORG 0x00h
    goto    initialisation
    
PSECT isr_vec, class = CODE, delta = 2
ORG 0x04h
    goto interrupt
    
PSECT code
initialisation:
    
    ;########################################
    ; initialisation of device frequency    #
    ;########################################
    
    ; Set the device frequency to 32 MHz (the highest frequency)
    banksel OSCCON
    movlw   11111100B
    movwf   OSCCON
    banksel ADCON1
    movlw 01000000B
    movwf ADCON1
    
    ;################################
    ; initialisation of timer 1     #
    ;################################
    
    ; Set the configuration of the timer 1 in order to use it in the main code
    ; but it is not yet enabled (TMR1ON (bit 0 of T1CON) = 0)
    banksel T1CON
    movlw   00000000B ; Prescale value of 1 (bit 5-4 = 00)
		      ; Frequancy set to 8 Mz (bit 7-6 = 00)
    movwf   T1CON
    ;initialise the value of the 16 bits counter to zero
    banksel TMR1L
    clrf TMR1L
    clrf TMR1H
    ;clear the interrupt flag of the timer 1
    banksel PIR1
    bcf PIR1, 0
    
    ;Allow timer1 interrupts
    banksel PIE1
    bsf PIE1, 0
    
    ;################################
    ; initialisation of timer 0     #
    ;################################
    
    ; set the timer 0 in timer mode (bit 5 = 0)
    banksel OPTION_REG
    bcf OPTION_REG, 5
    
    ; initialisation of timer 0 interrupts
    banksel INTCON
    bsf INTCON, 5  ; enables timer0 interrupts
    bcf INTCON, 2  ; clear interrupt flag of timer 0
    
    ; Allow the prescaler of the timer0
    banksel OPTION_REG
    bcf OPTION_REG, 3
    
    ; Set the prescaler to 256
    movlw 07h
    iorwf OPTION_REG, f
    
    ;##############################################
    ; initialisation of input and output pins     #
    ;##############################################
    
    ; Disable all the weak pull-up (needed to use IOC)
    bsf OPTION_REG, 7
    
    ;###################################################
    ; Port A pins                                      #
    ; -------------------------------------------------#
    ; RA7 : nothing                                    #
    ; RA6 : nothing                                    #
    ; RA5 : nothing                                    #
    ; RA4 : output enable servo 3                      #
    ; RA3 : output enable servo 2                      #
    ; RA2 : output enable servo 1                      #
    ; RA1 : IOC input (stop switch)                    #
    ; RA0 : Water level sensor input (Schmidt-Trigger) #
    ;###################################################
    
    banksel TRISA
    movlw 00000011B
    movwf TRISA
    
    ;disable all the servos
    banksel LATA
    clrf LATA
    
    ; put the two inputs in digital mode
    banksel ANSELA
    clrf ANSELA
    
    ; Put RA0 (input of the water level sensor) and RA1 in Schmidt-Trigger mode
    banksel INLVLA
    bsf INLVLA, 0
    bsf INLVLA, 1
    
    ; enable IOC
    banksel INTCON
    bsf INTCON, 3
    
    ; enable the IOC for a rising edge on RA1
    banksel IOCAN
    bsf IOCAN, 1
    
    ;###################################################
    ; Port B pins                                      #
    ; -------------------------------------------------#
    ; RB7 : programmer                                 #
    ; RB6 : programmer                                 #
    ; RB5 : input button (selection)                   #
    ; RB4 : input button (validation)                  #
    ; RB3 : input button (reset)                       #
    ; RB2 : nothing                                    #
    ; RB1 : nothing                                    #
    ; RB0 : nothing                                    #
    ;###################################################
    
    banksel TRISB
    movlw 00111000B
    movwf TRISB
    
    banksel LATB
    clrf LATB
    
    banksel ANSELB
    clrf ANSELB
    
    
    ;###################################################
    ; Port C pins                                      #
    ; -------------------------------------------------#
    ; RC7 : output LED 3                               #
    ; RC6 : nothing                                    #
    ; RC5 : nothing                                    #
    ; RC4 : I2C - SDA                                  #
    ; RC3 : I2C - SCLK                                 #
    ; RC2 : output PWM of the pump                     #
    ; RC1 : output PWM of the servos                   #
    ; RC0 : nothing                                    #
    ;###################################################
    
    banksel TRISC
    clrf    TRISC
    bsf	    TRISC, RC3
    bsf	    TRISC, RC4
    banksel LATC
    clrf    LATC
    banksel ANSELC
    clrf    ANSELC
    
    ;#############################################################
    ; Port D pins                                                #
    ; -----------------------------------------------------------#
    ; RD7 : output LED 1                                         #
    ; RD6 : output LED 2                                         #
    ; RD5 : output LED 5                                         #
    ; RD4 : outpur LED 4                                         #
    ; RD3 : output for the EN (enable) pin of the stepper        #
    ; RD2 : output for the CLK (PWM) pin of the stepper          #
    ; RD1 : nothing                                              #
    ; RD0 : output for the CW (direction) pin of the stepper     #
    ;############################################################# 
    
    banksel TRISD
    clrf    TRISD
    banksel LATD
    movlw   00001000B ; disable the stepper (EN = 1)
    movwf   LATD

    ;######################
    ; I2C CONFIGURATION   #
    ;######################
    
    banksel SSP1CON1	; Serial Communication COnfiguration Register
    movlw   00101000B	; bit 3-0 configures the type of communication. 
			; 1000 corresponds to I2C Master mode 
    movwf   SSP1CON1 
    
    banksel SSP1ADD	; modify the clock frequency
    ;movlw   00100111B	; SSP1ADD = 39 to have 200kHz f clock and so a 100kHz pulse 
    movlw   00011011B	;seems to be closer to 100kHz with that value 
    movwf   SSP1ADD
    
    banksel PIR1	; clear I2C flag
    bcf	    PIR1, SSP1IF
    
    ;############################################
    ;PWM configuration for the pump with CCP1   #
    ;############################################
    
    ; Step 1 : Disable the output RC2
    banksel TRISC
    bsf TRISC, 2
    
    ; Step 2 : set the frequency of the PWM but keep the timer 2 disable (bit 2 of T2CON = 0)
    banksel PR2
    movlw 0xFF ; minimal frequency
    movwf PR2
    movlw 03h
    banksel T2CON
    movwf T2CON ;set the prescale value to 64 (bit 1-0 = 11)
    
    ; Step 3 : set the CCP module in PWM mode (bit 3-2 = 11)
    ; bit 5-4 = 11 : two least significant bits of the duty cycle (not very useful)
    banksel CCP1CON
    movlw 00111111B
    movwf CCP1CON
    
    ; Step 4 : initial value of the duty cycle (will change during the control)
    banksel CCPR1L
    movlw 01111111B
    movwf CCPR1L
    
    ; Step 5
    ; clear the interrupt flag of timer 2
    banksel PIR1
    bcf PIR1, 1

    ; enable the pin RC2 as an output
    banksel TRISC
    bcf TRISC, 2
    
    ;##################################################
    ; Initialization of the values of the variables   #
    ;##################################################
    
    ;clear the flags
    banksel TASK_READY
    clrf    TASK_READY
    banksel OFFSET_FLAG
    clrf    OFFSET_FLAG
    clrf    TIMER_1_CHOICE
    bsf TIMER_1_CHOICE, 0 ; the operation begin with the reset of the servos
    clrf CHOICE
    
    ;initialize the variables of the stepper
    banksel COUNTER_STEP
    clrf COUNTER_STEP
    ; in order to be sure that the box is able to move until the stop switch before stopping
    ; because of reaching the end destination : the maximal destination has been put in
    ; STEPPER_TARGET and the value just below for COUNTER_TURN (these two positions are in fact
    ; not reachable on the machine)
    movlw 0xFE
    movwf COUNTER_TURN
    movlw 0xFF
    movwf STEPPER_TARGET
    
    ;clear the variables of the buttons
    clrf SAME_BUTTONS_0
    clrf SAME_BUTTONS_1
    clrf BUTTONS_PRESS
    clrf BUTTONS_USE
    clrf WAIT_SERVO
    
    ;initialize the variables of the servos
    ; the operation begin with the reset of the servos to put them in low position
    movlw LOW_POSITION
    movwf POSITION
    movlw SERVO_WAIT_DOWN
    movwf SERVO_WAIT_TIME
    movlw LOW_POSITION
    movwf CURRENT_POSITION
    
    ; initialize the variables for the state of the machine
    ; the operation begin with the reset of the servos
    movlw 0x04
    movwf STATE
    movwf PREVIOUS_STATE ; in case of not enough water
    
    ;initialize the variable for the around 2 sec waiting counter
    clrf SEC_WAITING_COUNTER
    
    ;force the machine to check first if there is water before starting
    bsf TASK_READY, 1
    
    ;############################################
    ;Allow global and peripheral interrupts    #
    ;############################################
    
    banksel INTCON
    bsf INTCON, 6
    bsf	INTCON, 7

    goto sensor_init
    
interrupt:
    ;#########################################################
    ; function called each time an interrupt flag is raised  #
    ;#########################################################
    
    ; timer 1 interrupt
    banksel PIR1
    btfsc PIR1, 0
    goto timer1_selection
    
    ; timer 0 interrupt
    banksel INTCON
    btfsc   INTCON, 2
    goto buttons_int
    
    ; IOC on RA1 interrupt
    banksel IOCAF
    btfsc IOCAF, 1
    goto stop_switch
    
    retfie

buttons_int:
    ;##############################################################
    ; store the values of the input and raise a flag to enable    #
    ; the buttons task.                                           #
    ;##############################################################
    
    ; enable the task for the buttons
    banksel TASK_READY
    bsf TASK_READY, 0
    
    ;store the old buttons values
    banksel BUTTONS_VALUES_INT
    movf BUTTONS_VALUES_INT, w
    movwf OLD_BUTTONS_VALUES_INT
    
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
    ; take only care of bit 0
    movlw 00000001B
    andwf READING, w
    
    ; update the variable with this new input
    banksel BUTTONS_VALUES_INT
    iorwf BUTTONS_VALUES_INT, f
    
    banksel INTCON
    bcf INTCON, 5 ; disable timer 0
    bcf INTCON, 2 ; clear interrupt flag
    bsf INTCON, 5 ; enable timer 0
    
    retfie
    
timer1_selection:
    ;######################################################
    ; choice of the type of timer 1 interrupt             #
    ;######################################################
    
    ;option waiting 2 sec is chosen
    banksel TIMER_1_CHOICE
    btfsc   TIMER_1_CHOICE, 2
    goto    timer1_waiting_s
    
    ;option waiting 10 ms is chosen
    banksel TIMER_1_CHOICE
    btfsc   TIMER_1_CHOICE, 1
    goto    timer1_waiting_10ms
    
    ;option stepper is chosen
    btfss   TIMER_1_CHOICE, 0
    goto    stepper
    
    ; else : option servo is chosen
    goto    servo

timer1_waiting_s:
    ;######################################################
    ; raise a flag after a wait of around 2 sec           #
    ;######################################################
    
    ; clear the interrupt flag
    banksel PIR1
    bcf PIR1, 0
    
    ;reset the 16 bit counter to have the maximal delay between interrupts
    banksel T1CON
    bcf T1CON, 0
    clrf TMR1L
    clrf TMR1H
    bsf T1CON, 0
    
    ;check if the value is equal to the max value of the waiting and raise a flag if it is
    banksel SEC_WAITING_COUNTER
    incf SEC_WAITING_COUNTER
    movlw SEC_WAITING
    xorwf SEC_WAITING_COUNTER, 0
    banksel STATUS
    btfss STATUS, 2
    retfie
    
    banksel TASK_READY
    bsf TASK_READY, 5
    
    clrf SEC_WAITING_COUNTER ; clear the counter of the waiting
    
    retfie
    
    
timer1_waiting_10ms:
    ;######################################################
    ; raise a flag after waiting 10 ms                    #
    ;######################################################
    
    ; clear the interrupt flag
    banksel PIR1
    bcf PIR1, 0
    
    ; raise a flag to say that the waiting is finished
    banksel TASK_READY
    bsf TASK_READY, 4
    
    retfie
    
stepper:
    ;######################################################################
    ; generate the PWM and the position tracking of the stepper           #
    ;######################################################################
    
    ; clear the interrupt flag
    banksel PIR1
    bcf PIR1, 0
    
    ; inverse the value on RD2 (CLK+ for the stepper)
    banksel LATD	
    movlw   04h
    xorwf   LATD, 1
    
    ; load the value of the counter to reach a frequency of 1 kHz (prescale value of 1
    ; for the timer)
    banksel T1CON
    bcf T1CON, 0
    movlw 11000000B
    movwf TMR1L
    movlw 11100000B
    movwf TMR1H
    bsf T1CON, 0
    
    ; increment the counter if two variations of the output has occured (creating a PWM
    ; of frequency 500 Hz)
    banksel LATD
    btfsc LATD, 2
    call increment_counter_step
    
    ; change the COUNTER_TURN if COUNTER_STEP has reached its maximal value
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
    ; moving to the rigth
    banksel COUNTER_TURN
    incf COUNTER_TURN, 1
    return
    
decrementation:
    ; moving to the left
    banksel COUNTER_TURN
    decf COUNTER_TURN, 1
    return

change:
    
    ; all the information about the direction is stored in CW+ (RD0)
    ; increment if we are going to the rigth
    banksel LATD
    btfss LATD, 0
    call incrementation
    
    ; decrement if we are going to the left
    btfsc LATD, 0
    call decrementation
    
    banksel COUNTER_STEP
    clrf COUNTER_STEP
    
    ;check if the box has arrived at destination
    movf STEPPER_TARGET, w
    xorwf COUNTER_TURN, w
    banksel STATUS
    btfss STATUS, 2
    return
    
    ; cut the stepper : set EN pin (RD3) 
    banksel LATD
    bsf LATD, 3
    
    ; cut the timer
    banksel T1CON
    bcf T1CON, 0
    
    ; raise the flag to tell it to the main code 
    banksel TASK_READY
    bsf TASK_READY, 2
    
    return

servo:
    ;######################################################
    ; generate the PWM for the servo                      #
    ;######################################################
    
    ; clear the interrupt flag
    banksel PIR1
    bcf PIR1, 0
    
    ;value of the 16 bits counter to reach a frequency of 8kHz
    banksel T1CON
    bcf T1CON, 0
    movlw 00110000B
    movwf TMR1L
    movlw 11111100B
    movwf TMR1H
    bsf T1CON, 0

    ; check the value of COUNTER_SERVO to know if the PWM need to be reset
    ; the value when the reset is needed is contained in CURRENT_POSITION
    banksel COUNTER_SERVO
    incf COUNTER_SERVO
    movf CURRENT_POSITION, w
    xorwf COUNTER_SERVO, 0
    banksel STATUS
    btfsc STATUS, 2
    call clear_output
    
    ; check if the counter reaches its maximal value and restart the PWM if it is
    banksel COUNTER_SERVO
    movlw SERVO_STEP
    xorwf COUNTER_SERVO, 0
    banksel STATUS
    btfsc STATUS, 2
    call reset_servo_counter
    
    retfie
    
    
reset_servo_counter:
    
    ;reset the counter
    banksel COUNTER_SERVO
    clrf COUNTER_SERVO
    
    ; set again RC1 (PWM servo) to 1
    banksel LATC
    bsf LATC, 1
    
    ; check if the current position is equal to the final position wanted and if not
    ; change the current position
    banksel POSITION
    movf POSITION, w
    banksel CURRENT_POSITION
    xorwf CURRENT_POSITION, 0
    banksel STATUS
    btfss STATUS, 2
    goto progressive
    
    ; if the servo is at its final position, wait until the counter reaches its maximal value
    ; before stopping the servo
    banksel WAIT_SERVO
    incf WAIT_SERVO
    movf SERVO_WAIT_TIME, w
    banksel WAIT_SERVO
    xorwf WAIT_SERVO, 0
    banksel STATUS
    btfss STATUS, 2
    return
    
    ; tell to the main code that the servo has finished
    banksel TASK_READY
    bsf TASK_READY, 3
    
    ; stop timer 1
    banksel T1CON
    bcf T1CON, 0
    
    return

clear_output:
    
    ; clear the output of the PWM servo (RC1)
    banksel LATC
    bcf LATC, 1
    
    return
    
progressive:
    
    ; wait until the counter reaches its maximal value before moving further
    banksel WAIT_SERVO
    incf WAIT_SERVO
    movlw SERVO_WAIT_TIME_PROG
    banksel WAIT_SERVO
    xorwf WAIT_SERVO, 0
    banksel STATUS
    btfss STATUS, 2
    return
    
    ; reste the waiting counter
    banksel WAIT_SERVO
    clrf WAIT_SERVO
    
    ; check wich of position and current_position is the highest and change current_position
    ; towards position
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
    ;######################################################
    ; handle the press of the stop switch positioned      #
    ; at the left limit of the sliding belt               #
    ;######################################################
    
    ; clear the interrupt flag
    banksel IOCAF
    bcf IOCAF, 1
    
    ; discard if the stepper is not moving (RD3 = 1)
    banksel LATD
    btfsc LATD, 3
    retfie
    
    banksel LATD
    bcf LATD, 0 ; the direction of the platform is forced to the rigth
    bcf LATD, 2 ; clear the output of the PWM for the stepper
    
    ; reset the counter because the box is at the left limit and the left limit 
    ; correspond to counters equal to zero
    banksel COUNTER_STEP
    clrf COUNTER_STEP
    clrf COUNTER_TURN
    
    ; in this code, TASK_READY is left to 1 if the stop_switch was not waited
    banksel TASK_READY
    btfsc TASK_READY, 1
    retfie
    
    ; If the stop switch is waited, then it means that the box need to be stopped
    ; ==> RD3 = 1
    banksel LATD
    bsf LATD, 3
    
    ; stop timer 1
    banksel T1CON
    bcf T1CON, 0
    
    ; clear the interrupt flag if the box had to move
    banksel PIR1
    bcf PIR1, 0
    
    ; tell to the main code that the box arrived at the left limit
    banksel TASK_READY
    bsf TASK_READY, 1
    
    retfie
    
sensor_init:
    ;#####################################################
    ;# initialization sequence of the sensor :           #
    ;#   - 10 ms delay					 #
    ;#   - Sending sensor adress			 #
    ;#	 - Sending init register			 #
    ;#	 - Clear sensor registers		         #
    ;#	 - Make a first reading to compute the offset	 #
    ;#####################################################
    
    ;######################## 
    
    ; 10 ms delay before asking anything : busy waiting 
    
    ;choice of the role of timer1
    banksel TIMER_1_CHOICE
    bsf	    TIMER_1_CHOICE, 1
    
    ; setup timer 1 frequency
    banksel T1CON 
    movlw 00110000B
    movwf T1CON
    
    ; initial value of timer
    banksel TMR1H
    movlw 11100000B
    movwf TMR1L
    movlw 10110001B
    movwf TMR1H
    
    ; start timer
    banksel T1CON
    bsf T1CON, 0
    
    ; wait until flag is set
    banksel TASK_READY
    btfss   TASK_READY, 4
    goto    $-1
    
    ; clear flagg
    banksel TASK_READY
    bcf TASK_READY, 4
    
    ; clear timer register 
    banksel T1CON
    bcf T1CON, 0
    clrf    T1CON
    
    banksel TIMER_1_CHOICE
    bcf	    TIMER_1_CHOICE, 1
    
    ; ##################################################
    
    ; Senging different initialization messages (adress, init_reg, clear_reg) : 
    
    ; Disable interrupts since I2C is working
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
    
	
    ;send initialization register adress
    banksel  SSP1BUF
    movlw    REG_DATA_INIT_SENSOR  ; Adress of the sensor init register
    movwf    SSP1BUF
    
    call    wait_SSPIF
    
    banksel SSP1CON2
    btfsc   SSP1CON2, ACKSTAT  ; check if ACKSTAT has been received
    goto    i2c_fail

    
    ;send clear register adress
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
    
    ; Reset interrupts since I2C communication stopped
    banksel INTCON
    bsf	    INTCON, 7
    
    ;#####################################################
    
    ;make a first reading to compute the offset,
    ;we use busy waiting but only for the first time
    
    ; First part of the reading : send the question (the weight)
    call read_weight_1
    
    ; 10 ms delay before asking the answer 
    
    
    ; choice of the role of timer1
    banksel TIMER_1_CHOICE
    bsf	    TIMER_1_CHOICE, 1
    
    ; configure timer1
    banksel T1CON
    movlw 00110000B
    movwf T1CON
    
    ;initialize timer1
    banksel TMR1H
    movlw 11100000B
    movwf TMR1L
    movlw 10110001B
    movwf TMR1H
    
    ;start timer1
    banksel T1CON
    bsf T1CON, 0
    
    ; wait until flag is set
    banksel TASK_READY
    btfss   TASK_READY, 4
    goto    $-1
    
    ; clear flag
    banksel TASK_READY
    bcf TASK_READY, 4
    
    ; clear timer1 configuration, stop timer1
    banksel T1CON
    clrf    T1CON
    bcf T1CON, 0
    
    banksel TIMER_1_CHOICE
    bcf	    TIMER_1_CHOICE, 1
    
    ; Second part of the reading : ask for an answer after waiting 10ms
    call read_weight_2

    ; then check the buttons a first time
    goto    buttons

;###################################################
;# READ_WEIGHT_1 : First part of the I2C sequence  #
;#		  that sends the question	   #
;#						   # 
;#   - Send sensor adress			   #
;#   - Ask for data register			   #
;###################################################
    
read_weight_1:
    
    ;Disable interrupts for I2C communication
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
    
    ; Re-enable interrupts
    banksel INTCON
    bsf INTCON, 7
    
    return 

;##########################################################
;# READ_WEIGHT_2 : Second part of the I2C sequence,	  #
;#		   ask for the answer after a 10ms delay  #
;#							  #
;#	- Send sensor adress in receiver mode		  #
;#	- Loop to store the answer			  #
;#	- Process the data				  #
;##########################################################
    
read_weight_2:
    ;###########################
    ; Data has been required, we now ask for an answer
    

    ; Disable interrupts for I2C communication
    banksel INTCON
    bcf INTCON, 7

    ;initialize index and memory 
    banksel INDEX
    movlw   RAW_DATA_SIZE
    movwf   INDEX
    
    ;Move RAW_DATA adress into FSR0 for indirect memory adressing
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
    
    ;Loop on the answer to store all arriving bytes
    acquire_data:
	
	; Send a Restart condition 
	banksel SSP1CON2
	bsf	SSP1CON2, RCEN	
	
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
	
	; Send NACK
	BANKSEL SSP1CON2
	bsf	SSP1CON2, ACKDT ; ACK DATA to send is 1, which is NACK.
	bsf	SSP1CON2, ACKEN ; Send ACK DATA now.
	
	call	wait_SSPIF
	
	; Stop communication
	banksel SSP1CON2	
	bsf	SSP1CON2,PEN ; set the PEN bit which stops the communication

	call    wait_SSPIF

    ; Process the data
    call    process_data
    
    ;if first bit of w is equal to 0, it means the data is corrupted
    ;---> come back to main loop
    ;movwf   INDEX
    ;btfss   INDEX , 0
    ;goto    main_loop
    
    ; Re-enable interrupts
    banksel INTCON
    bsf	    INTCON, 7
    return

main_loop:
    
reset_state:
    
    ;#######################################################################
    ; first state of the reset (01) :                                      #
    ;           this state stop the pump, the stepper motor and launch      #
    ;           the positioning in down position of all servos             #
    ;#######################################################################
    
    banksel STATE
    movlw 0x01
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto reset_wait
    
    ; stop the timer and set a prescale value of 0
    banksel T1CON
    clrf    T1CON
    
    ;#################
    ; cut the pump   #
    ;#################
    
    ;set pwm in "setup" mode
    banksel TRISC
    bsf	TRISC, 2
    banksel T2CON
    bcf	T2CON, 2

    ; set a duty cycle of zero
    banksel CCPR1L
    clrf    CCPR1L

    ; enable again the output of the PWM
    banksel TRISC
    bcf TRISC, 2

    ; cut completely all the CCP1 module (bit 3-0 = 0000)
    banksel	CCP1CON
    clrf	CCP1CON

    ; reset the counter of the Timer 2
    banksel	TMR2
    clrf	TMR2

    ; clear the output of the Timer 2
    banksel LATC
    bcf	LATC, 2
    
    ;####################
    ; cut the stepper   #
    ;####################
    banksel LATD
    bsf LATD, 3
    
    ; reset the different tasks because we don't know when the reset button is pushed
    banksel TASK_READY
    bcf TASK_READY, 2
    bcf TASK_READY, 3
    bcf TASK_READY, 4
    bcf TASK_READY, 5
    
    ;#####################
    ; cut all the LEDS   #
    ;#####################
    bcf LATC, 7
    movlw 00101111B
    andwf LATD, f
    
    ;#############################
    ; move all the servos down   #
    ;#############################
    ; all the LATx are in the same bank
    banksel LATA
    bsf LATA, 2
    bsf LATA, 3
    bsf LATA, 4
    
    ;clear the output for the servo (RC1)
    banksel LATC
    bcf LATC, 1
    
    ; chose servo option
    banksel TIMER_1_CHOICE
    bsf TIMER_1_CHOICE, 0
    
    ; bypass the progressive movement of the servos
    banksel POSITION
    movlw LOW_POSITION
    movwf POSITION
    movwf CURRENT_POSITION
    
    ; clear the wait of the servo
    clrf WAIT_SERVO
    
    ; wait of the servo when it is going down
    movlw SERVO_WAIT_DOWN
    movwf SERVO_WAIT_TIME
    
    ; enable timer 1
    banksel T1CON
    bsf T1CON, 0
    
    ; move to the next state
    banksel STATE
    movlw 0x02
    movwf STATE
    
reset_wait:
    ;#######################################################################
    ; second state of the reset (02) :                                     #
    ;           this state wait until the end of the servos operation      #
    ;           to begin and then wait until the button VAL is press       #
    ;           to move on                                                 #
    ;#######################################################################

    banksel STATE
    movlw 0x02
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto not_enough_water
    
    ; wait that the servos have finished their operation
    banksel TASK_READY
    btfss TASK_READY, 3
    goto not_enough_water
    
    ; wait until the buttons VAL is press
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto not_enough_water
    
    ; clear the flag of the button and then reset the position of the box on the left
    bcf BUTTONS_USE, 1
    movlw 0x05
    movwf STATE
    
    ;reset the LED of the wls if now there is water
    banksel SAME_BUTTONS_0
    btfss SAME_BUTTONS_0, 0
    goto not_enough_water
    
    ; clear the LED of the wls (RD5)
    banksel LATD
    bcf LATD, 5
      
not_enough_water:
    ;#######################################################################
    ; state when there is not enough water (03) :                          #
    ;           this state is called when there is not enough water.       #
    ;           It mainly do nothing except waiting until the button VAL   #
    ;           is press with water filled in the tank                     #
    ;#######################################################################
    
    banksel STATE
    movlw 0x03
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto init_servo
    
    ; wait that water is put back in the tank
    banksel SAME_BUTTONS_0
    btfss SAME_BUTTONS_0, 0
    goto init_servo
    
    ; wait that the button VAL is pressed
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto init_servo
    
    ; put back the machine in the previous state
    bcf BUTTONS_USE, 1
    movf PREVIOUS_STATE, w
    movwf STATE
    
    ; clear the LED of the wls
    banksel LATD
    bcf LATD, 5
    
init_servo:
    ;#######################################################################
    ; state for the initialization of the servos (04) :                    #
    ;           this state is called only once at the launching of         #
    ;           the machine and is used to put all the servos in down      #
    ;           position (security)                                        #
    ;#######################################################################
    
    banksel STATE
    movlw 0x04
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto init_platform
    
    ; enable all the servos at the same time
    banksel LATA
    bsf LATA, 2
    bsf LATA, 3
    bsf LATA, 4
    
    ; chose the servo mode
    banksel TIMER_1_CHOICE
    bsf TIMER_1_CHOICE, 0
    
    ; bypass the progressive decrease of the position
    banksel POSITION
    movlw LOW_POSITION
    movwf POSITION
    movwf CURRENT_POSITION
    
    movlw SERVO_WAIT_DOWN
    movwf SERVO_WAIT_TIME
    
    ; clear the wait of the servo
    clrf WAIT_SERVO
    
    ; enable timer 1
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x05
    movwf STATE
    
init_platform:
    ;#######################################################################
    ; state for the initialization of the stepper (05) :                   #
    ;           this state is called at the beginning of each              #
    ;           creation of a cocktail and after a reset to put the box    #
    ;           at the left limit position                                 #
    ;#######################################################################
    
    banksel STATE
    movlw 0x05
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto alcohol_selection
    
    ; wait until the servos are down for the first operation
    ; and raised by hand if not
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
    ; begin with choice 1 for the alcohol and with water option
    bsf LATD, 7
    bsf LATD, 4
    
    ; clear the choice of the alcohol and set the first bit for alcohol 1
    banksel CHOICE
    clrf CHOICE
    bsf CHOICE, 0
    
    ; skip the movement if the stop switch is already press
    banksel PORTA
    btfss PORTA, 1
    goto move_to_alcohol_selection
    
    ; clear the flag of the stop switch saying that we are waiting for it
    banksel TASK_READY
    bcf TASK_READY, 1
    
    ; enable the stepper motor and make it move to the left
    banksel LATD
    bsf LATD, 0 ; the direction of the stepper
    bcf LATD, 3 ; Enable the stepper
    
    ; reset the position and the target of the stepper to be sure that they are never 
    ; going to match
    banksel STEPPER_TARGET
    movlw 0xFF
    movwf STEPPER_TARGET
    
    clrf COUNTER_STEP
    movlw 0xFE
    movwf COUNTER_TURN
    
    ; chose the stepper
    banksel TIMER_1_CHOICE
    clrf TIMER_1_CHOICE
    
    ; enable Timer 1
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x06
    movwf STATE
    
    goto alcohol_selection
    
move_to_alcohol_selection:
    ; this function make in the main code what the stop switch do in the interrupt routine
    banksel STATE
    movlw 0x06
    movwf STATE
    
    banksel TASK_READY
    bsf TASK_READY, 1
    
    clrf COUNTER_STEP
    clrf COUNTER_TURN
    
alcohol_selection:
    ;#######################################################################
    ; state for the selection of the alcohol (06) :                        #
    ;           this state is used to choose the alcohol wanted            #
    ;           using the buttons SEL and VAL                              #
    ;#######################################################################
    
    banksel STATE
    movlw 0x06
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto water_selection
    
    ; wait that the stepper encounters the stop switch
    banksel TASK_READY
    btfss TASK_READY, 1
    goto water_selection
    
    ; if the button SEL is press, then shift the desired alcohol by 1
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 0
    goto validation_alcohol
    
    bcf BUTTONS_USE, 0
    
    ;clear the carry flag for the rlf instruction
    banksel STATUS
    bcf STATUS, 0
    
    banksel CHOICE
    rlf CHOICE, f
    
    ; come back to choice 1
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
    ; if the VAL button is press, then move to the next state
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto water_selection
    
    ; be sure to begin with a choice including water
    banksel CHOICE
    bcf CHOICE, 4
    
    banksel STATE
    movlw 0x07
    movwf STATE
    
    bcf BUTTONS_USE, 1
    
water_selection:
    ;#######################################################################
    ; state for the selection of the alcohol (07) :                        #
    ;           this state is used to choose if water is wanted            #
    ;           using the buttons SEL and VAL                              #
    ;#######################################################################
    
    banksel STATE
    movlw 0x07
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto displacement_to_alcohol
    
    ; if the button SEL is press, xor the previous decision about water
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
    ; if the button VAL is press, then move to the next state
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto displacement_to_alcohol
    
    ; cut all the LED except the water level sensor led
    banksel LATC
    bcf LATC, 7
    movlw 00101111B
    andwf LATD, f
    
    banksel STATE
    movlw 0x08
    movwf STATE
    bcf BUTTONS_USE, 1
    
displacement_to_alcohol:
    ;#######################################################################
    ; state for the displacement under the alcohol dispenser (08) :        #
    ;           this state is used to check the alcohol chosen             #
    ;           and launch the displacement of the box under this one      #
    ;#######################################################################
    
    banksel STATE
    movlw 0x08
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto move_up_servo
    
    banksel LATD
    bcf LATD, 0 ; choose the direction of the stepper (rigth)
    bcf LATD, 3 ; turn on the stepper
    
    ;Turn on the first LED for the dowload bar
    banksel LATD
    bsf LATD, 7
    
    banksel STEPPER_TARGET
    ; alcohol 1 is chosen
    btfsc CHOICE, 0
    movlw ALCOHOL_1_POS
    
    ; alcohol 2 is chosen
    btfsc CHOICE, 1
    movlw ALCOHOL_2_POS
    
    ; alcohol 3 is chosen
    btfsc CHOICE, 2
    movlw ALCOHOL_3_POS
    
    movwf STEPPER_TARGET
    
    ; chose stepper mode
    banksel TIMER_1_CHOICE
    clrf TIMER_1_CHOICE
    
    ; enable timer 1
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x09
    movwf STATE
    
move_up_servo:
    ;#######################################################################
    ; state for the movement of the servo to the high position (09) :      #
    ;           this state is used to check the alcohol chosen             #
    ;           and launch the movement to the up position of the          #
    ;           according servo                                            #
    ;#######################################################################
    
    banksel STATE
    movlw 0x09
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto move_down_servo
    
    ; wait that the stepper arrived to destination
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
    ; chose servo mode
    banksel TIMER_1_CHOICE
    bsf TIMER_1_CHOICE, 0
    
    ; send the servo to high position
    banksel POSITION
    movlw HIGH_POSITION
    movwf POSITION
    
    ; wait enough to be sure that all the alcohol leaved the dispenser
    movlw SERVO_WAIT_UP
    movwf SERVO_WAIT_TIME
    
    ; clear the wait of the servo
    clrf WAIT_SERVO
    
    ; enable timer 1
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x0A
    movwf STATE
    
move_down_servo:
    ;#######################################################################
    ; state for the movement of the servo to the low position (10) :       #
    ;           this state is used to  move down the servos that           #
    ;           is in up position                                          #
    ;#######################################################################
    
    banksel STATE
    movlw 0x0A
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto goto_water
    
    ; wait until the previous operation of the servos is finished
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
    
    ; chose servo option
    banksel TIMER_1_CHOICE
    bsf TIMER_1_CHOICE, 0
    
    ; send the servo to low position
    banksel POSITION
    movlw LOW_POSITION
    movwf POSITION
    
    movlw SERVO_WAIT_DOWN
    movwf SERVO_WAIT_TIME
    
    ; clear the wait of the servo
    clrf WAIT_SERVO
    
    ; enable the timer 1
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x0B
    movwf STATE
    
goto_water:
    ;#######################################################################
    ; state for the movement of the stepper under the water (11) :         #
    ;           this state is used to launch the movement of the box       #
    ;           to make it move under the water or to bypass the water     #
    ;           if the water option was not selected                       #
    ;#######################################################################
    
    banksel STATE
    movlw 0x0B
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto pump_1
    
    ; wait that the operation of the servo is finished
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
    
    ; skip the movement to the water if the without water option was chosen
    
    ; set to fill the conditions in order to launch the state to go to the final position
    banksel TASK_READY
    bsf TASK_READY, 5
    
    ; state to go to the final position
    banksel STATE
    movlw 0x0E
    movwf STATE
    
    goto goto_end_destination
    
we_want_water:  
    ; launch the stepper motor
    banksel LATD
    bsf LATD, 0 ; choose the direction of the stepper (left)
    bcf LATD, 3 ; enable the stepper
    
    banksel STEPPER_TARGET
    movlw PUMP_POSITION
    movwf STEPPER_TARGET
    
    ; chose stepper option
    banksel TIMER_1_CHOICE
    clrf TIMER_1_CHOICE
    
    ; enable timer 1
    banksel T1CON
    bsf T1CON, 0
  
    ; Configure CCP1 in PWM mode
    banksel CCP1CON
    movlw 00111111B
    movwf CCP1CON
    
    banksel STATE
    movlw 0x0C
    movwf STATE
    
pump_1:
    ;#######################################################################
    ; first state of the filling of the glass (12) :                       #
    ;           this state is used to launch the communication with the    #
    ;           weigth sensor (saying that we want data)                   #
    ;#######################################################################
    
    banksel STATE
    movlw 0x0C
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto pump_2
    
    banksel TASK_READY
    btfss TASK_READY, 2
    goto pump_2
    
    ;make the first part of the reading sequence
    call read_weight_1
   
    ;########################################## 
    ; 10 ms delay before asking the answer    #
    ;##########################################
    
    ;ccoice of the role of the timer1
    banksel TIMER_1_CHOICE
    bsf	    TIMER_1_CHOICE, 1
    
    ;configure timer1
    banksel T1CON
    movlw 00110000B
    movwf T1CON
    
    ;intialize timer1
    banksel TMR1H
    movlw 11100000B
    movwf TMR1L
    movlw 10110001B
    movwf TMR1H
    
    ;start timer1
    banksel T1CON
    bsf T1CON, 0
    
    ; change state accordingly
    banksel STATE
    movlw 0x0D
    movwf STATE
    
    
pump_2:
    ;#######################################################################
    ; second state of the filling of the glass (13) :                      #
    ;           this state is used to communicate with the weigth sensor   #
    ;           and to recover data. It then process these datas to        #
    ;           to control the duty cycle of the pump                      #
    ;#######################################################################
    
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
    
    ;Since the 10ms delay is done, answer can be asked
    
    ; clear timer1 configuration
    banksel T1CON
    clrf    T1CON
    
    ; clear timer1 role
    banksel TIMER_1_CHOICE
    bcf	    TIMER_1_CHOICE, 1
    
    ;call the second part of the reading sequence 
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
    
	; Re-enable interrupts since control is finished 
	banksel INTCON
	bsf	INTCON, 7

	; STOP PUMP: several steps:
	
	; set pwm in "setup" mode
	banksel TRISC
	bsf	TRISC, 2
	banksel T2CON
	bcf	T2CON, 2
	
	;set duty cycle to 0
	banksel CCPR1L
	clrf    CCPR1L
	
	 ;re-enable pwm
	banksel PIR1
	bcf PIR1, 1
	banksel TRISC
	bcf TRISC, 2
	
	; clear PWM configuration register
	banksel	CCP1CON
	clrf	CCP1CON

	; Clear timer2
	banksel	TMR2
	clrf	TMR2
	
	; Clear value of the output
	banksel LATC
	bcf	LATC, 2
	
	; Pump is stopped, stop timer1
	
	; clear timer1 role
	banksel TIMER_1_CHOICE
	bsf TIMER_1_CHOICE, 2

	; clear timer1 register
	banksel T1CON
	movlw 00110000B
	movwf T1CON

	; clear timer1 value
	banksel TMR1H
	clrf TMR1H
	clrf TMR1L
	
	; stop timer1
	banksel T1CON
	bsf T1CON, 0
	
	; change state
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
    
    ;###################
    ;MULTIPLY E_K BY K_P
    ;###################
    
    ;initialize multiplication registers
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
	; if overflow, return 11111111
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
    
    ; change state 
    banksel STATE
    movlw 0x0C
    movwf STATE
    
    banksel TASK_READY
    bsf TASK_READY, 2

goto_end_destination:
    ;#######################################################################
    ; send the box the end destination (14) :                              #
    ;           this state is used to send the box to the end position,    #
    ;           a position nearly on the left limit where the glass        #
    ;           wil stop because the creation is finished                  #
    ;#######################################################################
    
    banksel STATE
    movlw 0x0E
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto arrive_to_end_destination
    
    ; wait that the timer of 2 seconds has finished
    banksel TASK_READY
    btfss TASK_READY, 5
    goto arrive_to_end_destination
    
    ; reset the characteristics of timer 1
    banksel T1CON
    clrf    T1CON
    
    banksel TASK_READY
    bcf TASK_READY, 5
    
    ;clear the flag of end of destination
    bcf TASK_READY, 2
    
    ; launch the stepper
    banksel LATD
    bsf LATD, 0 ; choose the direction of the stepper (left)
    bcf LATD, 3 ; enable the stepper
    
    banksel STEPPER_TARGET
    movlw END_DESTINATION
    movwf STEPPER_TARGET
    
    ; choose stepper option
    banksel TIMER_1_CHOICE
    clrf TIMER_1_CHOICE
    
    ; enable timer 1
    banksel T1CON
    bsf T1CON, 0
    
    banksel STATE
    movlw 0x0F
    movwf STATE
    
arrive_to_end_destination:
    ;#######################################################################
    ; end of creation (15) :                                               #
    ;           the stepper has arrived to its destination and will wait   #
    ;           that the VAL button is pressed to move to another creation #
    ;#######################################################################
    
    banksel STATE
    movlw 0x0F
    xorwf STATE, w
    banksel STATUS
    btfss STATUS, 2
    goto buttons
    
    ; wait until the stepper arrived
    banksel TASK_READY
    btfss TASK_READY, 2
    goto buttons
    
    ; Turn on the last LED of the download bar
    banksel LATD
    bsf LATD, 4
    
    ; wait until the VAL button is pressed
    banksel BUTTONS_USE
    btfss BUTTONS_USE, 1
    goto buttons
    
    ; reset the use of the VAL button
    bcf BUTTONS_USE, 1
    
    ;clear the flag of end of destination
    banksel TASK_READY
    bcf TASK_READY, 2
    
    ;allow to enter in the init_stepper state
    bsf TASK_READY, 3
    
    ; reset the cocktail creation
    movlw 0x05
    movwf STATE
    
buttons:
    ;#################################################################
    ; This task is enable by the Timer 0 when its counter overflow   #
    ; and check the state of the buttons and for some of them,       #
    ; make operations if required                                    #
    ;#################################################################
    
    banksel TASK_READY
    btfss TASK_READY, 0
    goto main_loop
    
    ; clear the state of the buttons each time a new value will be loaded
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
    
    ; clear the flag of the button
    bcf TASK_READY, 0
    
    ; enable global interrupts
    banksel INTCON
    bsf INTCON, 7
    
    ; clear the comparisons of the buttons
    banksel SAME_BUTTONS_0
    clrf SAME_BUTTONS_0
    clrf SAME_BUTTONS_1
    
    call check_if_buttons_0
    
    call check_if_buttons_1

    ;#########################
    ; SEL button             #
    ;#########################
    ; Note : the buttons are in pull down configuration, so the button is press
    ; when its value is zero
    banksel SAME_BUTTONS_0
    btfss SAME_BUTTONS_0, 5
    goto off_1
    
    ; skip if button was already press
    banksel BUTTONS_PRESS
    btfsc BUTTONS_PRESS, 5
    goto next_button_1
    
    banksel BUTTONS_USE
    bsf BUTTONS_USE, 0
    
    banksel BUTTONS_PRESS
    bsf BUTTONS_PRESS, 5
    goto next_button_1
    
    off_1:
    
	; clear the BUTTONS_PRESS flag if the button is released
	banksel SAME_BUTTONS_1
	btfss SAME_BUTTONS_1, 5
	goto next_button_1
	
	bcf BUTTONS_PRESS, 5
    
next_button_1:
    
    ;#########################
    ; VAL button             #
    ;#########################
    banksel SAME_BUTTONS_0
    btfss SAME_BUTTONS_0, 4
    goto off_2
    
    ; skip if button was already press
    banksel BUTTONS_PRESS
    btfsc BUTTONS_PRESS, 4
    goto next_button_2
    
    banksel BUTTONS_USE
    bsf BUTTONS_USE, 1
    
    banksel BUTTONS_PRESS
    bsf BUTTONS_PRESS, 4
    goto next_button_2
    
    off_2:
    
	; clear the BUTTONS_PRESS flag if the button is released
	banksel SAME_BUTTONS_1
	btfss SAME_BUTTONS_1, 4
	goto next_button_2
	
	bcf BUTTONS_PRESS, 4
    
    next_button_2:
    
	; the tension becomes a logic 1 when the sensor is out of the water
	banksel SAME_BUTTONS_1
	btfss SAME_BUTTONS_1, 0
	goto next_wls
	
	; discard if we are already in the state 3
	banksel STATE
	movlw 0x03
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto next_wls
	
	; Turn on LED 5
	banksel LATD
	bsf LATD, 5
	
	; discard the information if the machine is in reset state
	; go to off_3 because we only allow the check if the button has been released
	; it is unuseful to check if the button is press because we are already in the reset state
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
	
	; let the fill of the glass process because if at the beginning of the filling
	; the tank had enough water, then there is enough water to fill a complete glass
	banksel STATE
	movlw 0x0C
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto next_wls
	
	banksel STATE
	movlw 0x0D
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto next_wls
	
	; Once the glass is filled, it is useless to stop the creation, this can wait until the 
	; glass reaches its end position
	banksel STATE
	movlw 0x0E
	xorwf STATE, w
	banksel STATUS
	btfsc STATUS, 2
	goto next_wls
	
	; store the previous state before entering the stop state (3)
	banksel STATE
	movf STATE, w
	movwf PREVIOUS_STATE
	
	movlw 0x03
	movwf STATE

    next_wls:
	;#########################
	; RESET button           #
	;#########################
	banksel SAME_BUTTONS_0
	btfss SAME_BUTTONS_0, 3
	goto off_3

	; discard if button was already press
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
	
	; move in reset state
	banksel STATE
	movlw 0x01
	movwf STATE
	
	banksel BUTTONS_PRESS
	bsf BUTTONS_PRESS, 3
	goto next_button_3
	
    off_3:
    
	; clear the flag of the button press
	banksel SAME_BUTTONS_1
	btfss SAME_BUTTONS_1, 3
	goto next_button_3
	
	bcf BUTTONS_PRESS, 3
    
    next_button_3:
	goto    main_loop
    
check_if_buttons_0:
    ;###################################################################
    ; This function check for each button if the current value and     #
    ; the previous value are both equals to zero                       #       
    ;###################################################################
    
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
    ;###################################################################
    ; This function check for each button if the current value and     #
    ; the previous value are both equals to one                       #       
    ;###################################################################
    
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

;################################################
;#  WAIT_SSPIF : Function that performs busy    #
;#		 waiting before the SSP1IF flag #
;#		 is raised			#
;################################################
	
wait_SSPIF:
    banksel PIR1
    btfss   PIR1, SSP1IF
    goto    $-1
    bcf	    PIR1, SSP1IF
    return    
   
;#####################################################
;#  I2C_FAIL:  happens if no ACK/NACK if received    #
;#	       while it should			     #
;#	       wait 10ms and then go to reset state  #
;#####################################################
    
i2c_fail:
    
    ; Re-enable interrupts
    banksel INTCON
    bsf INTCON, 7
    
    ; WAIT 10MS
    
    ;Change timer1 role
    banksel TIMER_1_CHOICE
    bsf	    TIMER_1_CHOICE, 1

    ; re-configure timer1
    banksel T1CON
    movlw 00110000B
    movwf T1CON
    
    ; Intiialize timer
    banksel TMR1H
    movlw 11100000B
    movwf TMR1L
    movlw 10110001B
    movwf TMR1H
    
    ;Start 10ms
    banksel T1CON
    bsf T1CON, 0
    
    ; Wait until flag is raised
    banksel TASK_READY
    btfss   TASK_READY, 4
    goto    $-1
    
    ; Clear flag
    bcf TASK_READY, 4

    ; clear timer configuration
    banksel T1CON
    clrf    T1CON
    
    ;clear timer1 role
    banksel TIMER_1_CHOICE
    bcf	    TIMER_1_CHOICE, 1
    
    ; set the machine in reset state when an I2C-fail occured
    banksel STATE
    movlw 0x01
    movwf STATE
    goto main_loop
    
    
;#############################################################################
;  PROCESS_DATA : function responsible of :				     #
;	-> checking if first byte is well equal to 0x12 (otherwise, return 0)#
;	-> moving the second and third bytes into DATA registers	     #
;	-> if second byte equal to 11111111 (negative reading),              #
;	    move 0 into DATA registers				             #
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
    
    ;Remove offset : 16bits substraction 
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