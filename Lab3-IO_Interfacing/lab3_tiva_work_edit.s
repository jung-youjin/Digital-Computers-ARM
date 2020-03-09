; This code is based upon InputOutput's from the book:
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
;
; The code provided initializes all 3 ports (A,B,E) with the ECE Shield plugged into the Tiva board
; Port F with the Tiva 3 LEDs (Red, Green, Blue) and two buttons is also initialized
; Then the LEDs on each port are turned off and on with time delays - while the Tiva board R, G, B LEDs are turned on and off
;
; Dec 2017

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start

; These equates allow one to associate a name with a value to make the code more readable

RED       EQU 0x02		; These are the values (bit locations) for various LEDs on the Tiva (Port F)
BLUE      EQU 0x04
GREEN     EQU 0x08
;SW1       EQU 0x10                 ; on the left side of the Tiva board
;SW2       EQU 0x01                 ; on the right side of the Tiva board



GPIO_PORTA_DIR_R   EQU 0x40004400
GPIO_PORTA_AFSEL_R EQU 0x40004420
GPIO_PORTA_PUR_R   EQU 0x40004510
GPIO_PORTA_DEN_R   EQU 0x4000451C
GPIO_PORTA_LOCK_R  EQU 0x40004520
GPIO_PORTA_CR_R    EQU 0x40004524
GPIO_PORTA_AMSEL_R EQU 0x40004528
GPIO_PORTA_PCTL_R  EQU 0x4000452C


LEDS      EQU 0x40025038
SWITCHES  EQU 0x40025044
SW1       EQU 0x10                 ; on the left side of the Launchpad board
SW2       EQU 0x01                 ; on the right side of the Launchpad board
PA5       EQU 0x40004080


Start
	BL  Port_Init                  ; initialize input and output pins of Ports A to F
	BL Switch_Init
	MOV R11, #0xf3c0	; initialize the Random Number routine - NEVER WRITE TO R11 after this!! or you break how it works
	MOV R3, #0
	LDR R3, =LEDS
	BL Switch_Input    ;?
	BL Switch_Input		;?

	
	MOV R5, #0					; initialize a counter to display on a 7-segment digit
keep_looping;Just ignore all the commented out stuff
	MOV R4, #0
	BL DelayReaction	;The rand 'timer'	

	MOV R6, #GREEN					;Set R6 as 0
	LDR R8, =GPIO_PORTF_DATA_R  ;Set R8 as port
	STR R6, [R8]	;Store R6 in R8
	
	BL SWITCH ;need the increment value from this
	
	BL LEDBLINK ;R7 is value displayed
	
;	BL SIMPLECOUNTER   ;Works //Deliverable 1

			

	
	
	MOV R1, #10000
	BL Delay

	B keep_looping
	
	
	
	
	
	
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

SWITCH

loopA
	MOV R8, #RED
	MOV R9, #0
	BL Board_Input

	
	CMP R0, #0x01
	;BEQ loopA
	BEQ sw1pressed
	
	CMP R0, #0x11
	BEQ nopressed
	
	STR R7, [R3]
	
	;SUBS R2, R2, #1
	B loopA

	BX LR
	
Board_Input
	LDR R1, =SWITCHES
	LDR R0, [R1]
	BX LR
	
sw1pressed
	STR R8, [R3]
;	MOV R2, #1
	;CBZ R8, #0
	MOV R4, #0
	ADD R4, #1
	B loopA

	
nopressed
	STR R9, [R3]
	CMP R4, #1
	BEQ keep_looping
	B loopA
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;------------Switch_Input------------
; Read and return the status of PA5
; Input: none
; Output: R0  0x20 if PA5 high
;         R0  0x00 if PA5 low
; Modifies: R1

Switch_Input
    LDR R1, =PA5      ; pointer to PA5
    LDR R0, [R1]      ; read just PA5
    BX  LR            ; 0x20 or 0x00
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	
Switch_Init
    LDR R1, =SYSCTL_RCGCGPIO_R         ; 1) activate clock for Port A
    LDR R0, [R1]                 
    ORR R0, R0, #0x01               ; set bit 0 to turn on clock
    STR R0, [R1]                  
    NOP
    NOP                             ; allow time for clock to finish
                                    ; 2) no need to unlock Port A                 
    LDR R1, =GPIO_PORTA_AMSEL_R     ; 3) disable analog functionality
    LDR R0, [R1]                    
    BIC R0, #0x20                   ; 0 means analog is off
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTA_PCTL_R      ; 4) configure as GPIO
    LDR R0, [R1]                    
    BIC R0, #0x00F00000             ; 0 means configure PA5 as GPIO
    STR R0, [R1]                  
    LDR R1, =GPIO_PORTA_DIR_R       ; 5) set direction register
    LDR R0, [R1]                    
    BIC R0, #0x20                   ; PA5 input
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTA_AFSEL_R     ; 6) regular port function
    LDR R0, [R1]                    
    BIC R0, #0x20                   ; 0 means disable alternate function 
    STR R0, [R1]                                 
    LDR R1, =GPIO_PORTA_DEN_R       ; 7) enable Port A digital port
    LDR R0, [R1]                    
    ORR R0, #0x20                   ; 1 means enable digital I/O
    STR R0, [R1]                   
    BX  LR    

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	

LEDBLINK		STMFD		R13!,{R14}
	;willdisplay value in R7
	LDR R3, =GPIO_PORTB + (PORT_B_MASK << 2)		; generate the base address for port B
	MOV R1, R7		; set lowest 8 bits except for B1 to 1
    STR R1, [R3, #GPIO_DATA_OFFSET]
	
	BL Delay
	
	LDMFD		R13!,{R15} 
	BLX 			LR
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

DelayReaction	STMFD		R13!,{R1, R14}
	
	LDRH R1, [R11]		;changes R11 to 16 bit for 2-10 second counter
	MOV R0, #1   ;For debbuging with 0 delay
loopwaitforlight
	BL Delay
	SUBS R1, R1, #1
	BNE loopwaitforlight
	
	LDMFD		R13!,{R1, R15}
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
SwitchCount		STMFD		R13!,{R2, R14}
loopuntilpress	
	BL Delay
	ADD R9, R9, #1
	STR R1, [R1]
	BEQ loopuntilpress
	
	LDMFD		R13!,{R2, R9, R15}
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
;POLL 		;	STMFD		R13!,{R3, R9}  ;need loop?
;	LDR			R6, =GPIO_PORTF_DATA_R  			; check input
;	LDR			R6, [R6]				
;	
;	LSR			R6, #10					; put 10th bit of R6 into R8
;	BFI			R8, R6, #0, #1				
;	
;	MOV			R0, #1					; 0.1 ms delay
;	BL			DELAY
;	ADD			R9, #1					; increase counter
	
;	TEQ			R8, #0					; when INT0 is pressed, 10th bit becomes 0
;	BNE			POLL
	
	
	
	
	
;looppoll
;	MOV R6, #SW1
;	LDR R3, =             ;r6 is the pointer pointing to the input pin
;	STR R6, [R3] 
;	MOV R0, #1                      ;Only one delay loop each time
;	BL Delay                        ;Go for a 0.1 ms delay loop
;	ADD R9, R9, #1                  ;counter increments by 1
			
			
			   ;Now r6 contains the data in p2. The 10th bit of r6 should be the input
		   ; MOV R7, R6, LSR #10             ;Right shift R6 by 10 bits to check the input
			;ANDS R7, R7, #1                 ;Take the bit out
			;MOV R8, R9 

;	MOV R6, #SW1				;Set R6 to RED
;	LDR R3, =GPIO_PORTF_DATA_R  ;Set R8 to Port
;	STR R6, [R3]
;	CMP R3, #0

;	BEQ looppoll
			
;	LDMFD		R13!,{R3, R9}
	;			BEQ POLL
	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

SHORTDELAYCOUNT             EQU 400    ; faction of a second delay

		; alternate name for this subroutine
ShortDelay 
Delay 				STMFD		R13!,{R0, R1, R14}		; push the LR or return address

loopnumtimes
	LDR R8, =SHORTDELAYCOUNT			;Sets R8 as Delay value
loop
		SUBS R8, R8, #1				;Delay Loop
		BNE loop
		
		
		SUBS R0, R0, #1
		BNE loopnumtimes
exitdelay     		LDMFD		R13!,{R0, R1, R15}		

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

SIMPLECOUNTER		STMFD		R13!,{R0, R2, R14}	
	MOV R2, #255  ; counter down to 0
	MOV R7, #0		;Count up for LED display
	
loopsimplecounter			
	MOV R0, #1000 ;100ms delay

	BL LEDBLINK
	ADD R7, #1
	SUBS R2, #1	
exit	BNE loopsimplecounter

				LDMFD		R13!,{R0, R2, R15}	









































































; Driving the 7-segment outputs is harder because they're spread over multiple ports
; Naming of the 7-segment segments is clockwise from the top:  A, B, C, D, E, F, G, DP (decimal point) as per Appendix H
; DO NOT USE THE DP (decimal point) - they are not included below

; left 7-segment bits are: F2, F3, D2, D3, D6, D7, E0   [ports F, D, E are used]
; right 7-segment bits are: A2, A3, E2, A5, A6, A7, A4 [ ports A, E are used]

; Display numbers to the right 7-segment as an example of read-modify-write data manipulation on I/O ports
; 
; INPUT: R5 - The 4 LSB of R5 will be displayed
; OUTPUT: none
;
display_right_7seg		STMFD		R13!,{R0, R1, R2, R3, R4, R5, R14}

	PUSH {R5}	; save R5 so that it can be used for the other 7-segment display
; First take the 4 LSB and display that to HEX #2
; Convert the binary number to the bit pattern required by the 7-segment display

	LDR R3, =Seven_Seg_Table	; get the base address of the 7-segment pattern table
	AND	R5, #0xf		; mask the number to display to the range 0 to 0xf
	LSL R5, #1			; multiply by two, to get an address offset in bytes, as the lookup table holds an array of 16-bit data
	LDRH R0, [R3, R5]		; read the 7-segment pattern - table base address + offset in R5
	; DP is Bit 0, then A, B, C, D, E, F, G (bit 7)

; brute force data manipulation - read bit by bit and conditionally set the appropiate bit in a register which gets written to the port
; use right shift - the LSB that falls off goes into the Carry flag and is used to conditionally set

	MOV	R1, #0		; R1 will hold the data written to port A
	MOV R2, #0		; R1 will hold the data written to port E

; right 7-segment bits are: A2, A3, E2, A5, A6, A7, A4 [ ports A, E are used]
; NOTE: a "1" in the 7-seg pattern should write a 0 because the 7-seg display is active low - so the code below inverts

	LSRS R0, #1		; shift LSB into Carry flag - this goes to dp (decimal point) - but is not used
	LSRS R0, #1		; shift next LSB - this is segment "G" and goes to port A bit 2 (A2)
	ORRCC R1, #0x04		; conditionally set A2 if Carry is Equal to 0

	LSRS R0, #1
	ORRCC R1, #0x08		; conditionally set A3 if Carry is Equal to 0
	
	LSRS R0, #1
	ORRCC R2, #0x04		; conditionally set E2 if Carry is Equal to 0
	
	LSRS R0, #1
	ORRCC R1, #0x20		; conditionally set A5 if Carry is Equal to 0
	
	LSRS R0, #1
	ORRCC R1, #0x40		; conditionally set A6 if Carry is Equal to 0
	
	LSRS R0, #1
	ORRCC R1, #0x80		; conditionally set A7 - bit 7 if Carry is Equal to 0
	
	LSRS R0, #1
	ORRCC R1, #0x10		; conditionally set A4 - bit 4 if Carry is Equal to 0
	
; now read-modify-write port A
	LDR R0, =GPIO_PORTA + (PORT_A_MASK << 2)
	LDR R4, [R0, #GPIO_DATA_OFFSET]		; read port A 
	AND R4, #0x03		; clear all bits being overwritten from the data manipulation above
	ORR R4, R1			; ORR in the data manipulation done - R1 which is port A data
	STR R4, [R0, #GPIO_DATA_OFFSET]		; write port A back out
	
; now read-modify-write port E
	LDR R0, =GPIO_PORTE + (PORT_E_MASK << 2)
	LDR R4, [R0, #GPIO_DATA_OFFSET]		; read port A 
	AND R4, #0xfb		; clear all bits being overwritten from the data manipulation above
	ORR R4, R2			; ORR in the data manipulation done - R2 which is port E data
	STR R4, [R0, #GPIO_DATA_OFFSET]		; write port A back out

; In order to drive the Left 7-segment:
; Ports D, E and F are used - so a register is needed to read-modify-write each of those

	POP {R5}			; retrieve the original value passed in
	LSR R5, #4			; shift right to access the upper 4 bits of the provided # which will be displayed to the 7-seg
	AND	R5, #0xf		; mask the number to display to the range 0 to 0xf - this is unnecessary 
	LSL R5, #1			; multiply by two, to get an address offset in bytes, as the lookup table holds an array of 16-bit data
	LDR R3, =Seven_Seg_Table	; get the base address of the 7-segment pattern table
	LDRH R0, [R3, R5]		; read the 7-segment pattern - table base address + offset in R5
	; DP is Bit 0, then A, B, C, D, E, F, G (bit 7)

	MOV R1, #0		; R1 will hold the data written to port D
	MOV R2, #0		; R1 will hold the data written to port E
	MOV R3, #0		; R1 will hold the data written to port F

; right 7-segment bits are: A2, A3, E2, A5, A6, A7, A4 [ ports A, E are used]
; NOTE: a "1" in the 7-seg pattern should write a 0 because the 7-seg display is active low - so the code below inverts

	LSRS R0, #1		; shift LSB into Carry flag - this goes to dp (decimal point) - but is not used

	LSRS R0, #1		; shift next LSB - this is segment "G" and goes to port F2
;	ORRCC R1, #0x??		; conditionally set F2 if Carry is Equal to 0

	LSRS R0, #1
;	ORRCC R1, #0x??		; conditionally set F3 if Carry is Equal to 0
	
	LSRS R0, #1
;	ORRCC R2, #0x??		; conditionally set D2 if Carry is Equal to 0
	
	LSRS R0, #1
;	ORRCC R1, #0x??		; conditionally set D3 if Carry is Equal to 0
	
	LSRS R0, #1
;	ORRCC R1, #0x??		; conditionally set D6 if Carry is Equal to 0
	
	LSRS R0, #1
;	ORRCC R1, #0x??		; conditionally set D7 if Carry is Equal to 0
	
	LSRS R0, #1
;	ORRCC R1, #0x??		; conditionally set E0 if Carry is Equal to 0
	
; now read-modify-write port D as was above
; now read-modify-write port E as was above
; now read-modify-write port F as was above

	LDMFD		R13!,{R0, R1, R2, R3, R4, R5, R15}

	ALIGN
; This table is the binary pattern of 8 bits required to drive the 7 LEDs of a 7-segment display to display a hex. number
; The LSB is the lowest bit and is not to be used
;
Seven_Seg_Table
	DCW 0x7e, 0x0c, 0xb6, 0x9e, 0xcc, 0xda, 0xfa, 0x0e, 0xfe, 0xce	; 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
	DCW 0xee, 0xf8, 0xb0, 0xbc, 0xf2, 0xe2		; A, B, C, D, E, F with LSB being 0 for the DP
	ALIGN
	
; ShortDelay subroutine - delay for a fixed amount of time
;
; This is a bad piece of code - only use it as an example for what never to do!!
;
; Input: R1 - how many times do we loop (multiply) the fixed delay - If R1 = 0 then there is no time delay
; Output: none


;	TEQ R1, #0
;	BNE not_done_delay
;	B	done_delay
;not_done_delay
;	SUB R1, #1
;	LDR R0, =SHORTDELAYCOUNT   	       ; R0 = a value to get about a second delay
;delay_loop
 ;   SUBS R0, R0, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
				; Note: For SUBs the "s" suffix means to set the status bits, without this the loops would not exit
;	CBNZ R0, delay_back			; compare if not zero then keep counting down
;	B delay_outer_loop
;delay_back
;	B delay_loop
;done_delay
;	LDMFD		R13!,{R0, R1, R15}		; pull the LR or return address and return


; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
;   DO NOT EDIT CODE BELOW THIS LINE
; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 

;------------RandomNum------------
; R11 holds a 16-bit "random" number via a pseudo-random sequence as per the Linear feedback shift register (Fibonacci) on WikiPedia
; Take as many bits of R11 as you need.
;
; R11 can be read anywhere in the user code but must only be written to by this subroutine
;
; INPUT: R11 - before calling this for the FIRST time R11 must be initialized to a large 16-bit non-zero value
;      if R11 is ever set to 0 then R11 will stay stuck at zero
; OUTPUT: R11 - random number is the lowest 16 bits of R11 which is between 1 and 0xffff
;
RandomNum		STMFD		R13!,{R1, R2, R3, R14}

				AND			R1, R11, #0x8000
				AND			R2, R11, #0x2000
				LSL			R2, #2
				EOR			R3, R1, R2
				AND			R1, R11, #0x1000
				LSL			R1, #3
				EOR			R3, R3, R1
				AND			R1, R11, #0x0400
				LSL			R1, #5
				EOR			R3, R3, R1		; the new bit to go into the LSB is present
				LSR			R3, #15
				LSL			R11, #1
				ORR			R11, R11, R3
				MOV			R1, #0xFFFF
				AND			R11, R1			; clear the upper 16 bits of R11 as they're not part of the random #
				
				LDMFD		R13!,{R1, R2, R3, R15}


; Tons of initialization to be done in order to use the I/O ports as they're off by default.
;
; Define the addresses and provide functions to initialize everything.

GPIO_PORTF_DIR_R   EQU 0x40025400		; Port F Data Direction Register setting pins as input or output
GPIO_PORTF_DATA_R  EQU 0x400253FC		; address for reading button inputs and writing to LEDs
GPIO_PORTF_AFSEL_R EQU 0x40025420
GPIO_PORTF_PUR_R   EQU 0x40025510
GPIO_PORTF_DEN_R   EQU 0x4002551C
GPIO_PORTF_LOCK_R  EQU 0x40025520
GPIO_PORTF_CR_R    EQU 0x40025524
GPIO_PORTF_AMSEL_R EQU 0x40025528
GPIO_PORTF_PCTL_R  EQU 0x4002552C

;Section 3.1.2 Nested Vector Interrupt Controller

;The Cortex-M4F processor supports interrupts and system exceptions. The processor and the
;Nested Vectored Interrupt Controller (NVIC) prioritize and handle all exceptions. An exception
;changes the normal flow of software control. The processor uses Handler mode to handle all
;exceptions except for reset. See ?xception Entry and Return?on page 108 for more information.
;The NVIC registers control interrupt handling. See ?ested Vectored Interrupt Controller
;(NVIC)?on page 124 for more information.

;Table 3-8 on page 134 details interrupt Set / Clear 
; they allow one to enable individual interrupts and DIS? lets one disable individual interrupt numbers

; Table 2-9 Interrupts on page 104 details interrupt number / bit assignments
; Port F - Bit 30
; Timer 0A Bit 19
; Timer 0B Bit 20
 
;For edge-triggered interrupts, software must clear the interrupt to enable any further interrupts.

; NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
; the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
; and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
; is written to the Port F GPIO Lock Register.  After Port F is
; unlocked, bit 0 of the Port F GPIO Commit Register must be set to
; allow access to PF0's control registers.  On the LM4F120, the other
; bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
; that the rest of Port F can always be freely re-configured at any
; time.  Requiring this procedure makes it unlikely to accidentally
; re-configure the JTAG and NMI pins as GPIO, which can lock the
; debugger out of the processor and make it permanently unable to be
; debugged or re-programmed.
	
; These are the configuration registers which should not be touched
; Port Base addresses for the legacy (not high-performance) interface to I/O ports
GPIO_PORTA			EQU 0x40004000
GPIO_PORTB			EQU 0x40005000
GPIO_PORTC			EQU 0x40006000
GPIO_PORTD			EQU 0x40007000
GPIO_PORTE			EQU 0x40024000
GPIO_PORTF			EQU 0x40025000

; WARNING outputs PD0 & PD1 are shorted to PB6 and PB7 - one pair MUST BE INPUTS!! - we disable D0, D1

; These are the masks for pins which are outputs, setting a bit to 1 makes the pin an output, 0 is input
PORT_A_MASK			EQU 0xfc	; PA7,6,5,4,3,2 are outputs for 7-segment LEDs
PORT_B_MASK			EQU 0xfd	; PB7,6,5,4,3,2,0 are LEDs
PORT_C_MASK			EQU 0x30	; this breaks programming the CPU - DO NOT ENABLE	
PORT_D_MASK			EQU 0xcc	; PD7,6,3,2  Disable D0, D1 due to short with B6, B7
PORT_E_MASK			EQU 0x37	; PE0,1,2,4,5 are used for 7-segment, LED & speaker
PORT_F_MASK			EQU 0x0e	; PF has LEDs on PF1,2,3 and buttons PF0, PF4 (don't enable buttons as outputs)


; Offsets are from table 10-6 on page 660
GPIO_DATA_OFFSET	EQU 0x000		; Data address is the base address - YOU HAVE TO ADD AN ADDRESS MASK TOO to read or write this!!
GPIO_DIR_OFFSET		EQU 0x400		; Direction register
GPIO_AFSEL_OFFSET EQU 0x420			; Alternate Function SELection
GPIO_PUR_OFFSET   EQU 0x510			; Pull Up Resistors
GPIO_DEN_OFFSET   EQU 0x51C			; Digital ENable
GPIO_LOCK_OFFSET  EQU 0x520
GPIO_CR_OFFSET    EQU 0x524
GPIO_AMSEL_OFFSET EQU 0x528			; Analog Mode SELect
GPIO_PCTL_OFFSET  EQU 0x52C

SYSCTL_HBCTL  EQU   0x400FE06C		; high performance bus control for ports A to F

GPIO_LOCK_KEY      EQU 0x4C4F434B  ; Unlocks the GPIO_CR register
SYSCTL_RCGCGPIO_R  EQU   0x400FE608		; Register to enable clocks to the I/O port hardware

;------------Port_Init------------
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2, R3
Port_Init
	STMFD		R13!,{R14}		; push the LR or return address

; First enable the clock to the I/O ports, by default the clocks are off to save power
; If a clock is not enabled to a port and you access it - then the processor hard faults
	LDR R1, =SYSCTL_RCGCGPIO_R      ; activate clock for Ports (see page 340)
    LDR R0, [R1]                 
    ORR R0, R0, #0x3F               ; turn on clock to all 6 ports (A to F, bits 0 to 5)
    STR R0, [R1]                  
    NOP
    NOP                             ; allow time for clock to finish
	
; Set all ports to APB bus instead of AHB - this should be unnecessary
;	LDR R1, =SYSCTL_HBCTL
;	LDR R0, [R1]
;	AND R0, #0xFFFFFFE0		; set Ports A thru F to APB (0) and leave the rest at their default
;	STR R0, [R1]

; Page 650, Table 10-1 GPIO Pins with Special Considerations.
; These pins must be left as configured after reset:
;  PA[5:0] (UART0 and SSIO), PB[3:2] (I2C), PC[3:0] (JTAG)

; Initialize the I/O ports A, B, E, F via a common subroutine Port_Init_Individual
; Call Port_Init_Individual with the following paramaters passed:
; R1 is the base port address
; R2 is the output pin mask (which bits are outputs)
; R3 is the input pin mask  (which bits get configured as inputs)

	MOV R3, #0x00				; Select no pins as input (unless it's changed as for port F)
	
; Init Port A, B, E are by default GPIO - set all output pins used to a 1 to enable them
;   and leave all of the other pins as previously configured!
    LDR R1, =GPIO_PORTA
    MOV R2, #PORT_A_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port B
    LDR R1, =GPIO_PORTB
    MOV R2, #PORT_B_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port C
    LDR R1, =GPIO_PORTC
    MOV R2, #PORT_C_MASK
	;BL Port_Init_Individual		; Do not initialize Port C as it renders the Tiva board unprogramable !

; Init Port D
    LDR R1, =GPIO_PORTD
    MOV R2, #PORT_D_MASK
	BL Port_Init_Individual

; Init Port E
	LDR R1, =GPIO_PORTE
    MOV R2, #PORT_E_MASK			; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port F
	LDR R1, =GPIO_PORTF
    MOV R2, #PORT_F_MASK		; enable commit for Port, 1 means allow access
	MOV R3, #0x11				; enable weak pull-up on PF0 and PF4 (buttons)
	BL Port_Init_Individual

	LDMFD		R13!,{R15}		; pull the LR or return address from the stack and return


;------------Port_Init_Individual------------
; Initialize one GPIO Port with select bits as inputs and outputs
; Output: none
; Input: R1, R2, R3
; R1 has to be the port address
; R2 has to hold the mask for output pins
; R3 has to be the mask for input pins
; Modifies: R0

Port_Init_Individual
	STMFD		R13!,{R14}		; push the LR or return address
    LDR R0, =0x4C4F434B             ; unlock GPIO Port F Commit Register
    STR R0, [R1, #GPIO_LOCK_OFFSET]	; 2) unlock the lock register
	ORR R0, R2, R3					; all access to inputs and outputs as masked in R2 and R3
    STR R0, [R1, #GPIO_CR_OFFSET]	; enable commit for Port F
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1, #GPIO_AMSEL_OFFSET]	; 3) disable analog functionality
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1, #GPIO_PCTL_OFFSET]	; 4) configure as GPIO
    LDR R0, [R1, #GPIO_DIR_OFFSET]	; 5) read default direction register configuration
    ORR R0, R2						; ORR in only the bits we want as outputs
    STR R0, [R1, #GPIO_DIR_OFFSET]	; 5) set direction register
    MOV R0, #0                      ; 0 means disable alternate function 
    STR R0, [R1, #GPIO_AFSEL_OFFSET]	; 6) regular port function
    STR R3, [R1, #GPIO_PUR_OFFSET]	; pull-up resistors for PF4,PF0
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1, #GPIO_DEN_OFFSET]
	LDMFD		R13!,{R15}		; pull the LR or return address and return

; Beep the speaker on the ECE Shield using port E4 and E5
; The speaker is conencted to two pins - toggle each end for more volume than a singled ended drive
;
; Before exiting ensure that both wires to the speaker are 0 to prevent it from being heated up
;
; Each beep, sounded or not, is about the same length - 0x300 loops of delay loop
;
; Input: R1 sets the tone - 0 is NO BEEP,
;                           1 is a high pitch beep and larger numbers are lower pitched
; Output: none

SpeakerBeepLength	EQU 0x300			; the length of the speaker beep
	
SpeakerBeep
	STMFD		R13!,{R1-R4, R11, R14}		; push the LR or return address

	MOV R4, #0x30		; This xor'd with the port holding the speaker pins will create a beep
	TEQ R1, #0		; If R1 = 0 then just delay but do not beep
	BNE make_a_sound
	MOV R1, #1		; stick a valid value into R1 as 0 will divide by 0
	MOV R4, #0x0		; setting R4 to 0 will ensure that no beep will sound
make_a_sound
	MOV R3, #SpeakerBeepLength	; how many loops of the beep delay do we do?  Ie how long is the beep
	UDIV R3, R1		; loop the tone R1 / R3 times to ensure a total of 0x100 delays for all tones
	
	LDR R2, =GPIO_PORTE + (PORT_E_MASK << 2)
	LDR R11, [R2, #GPIO_DATA_OFFSET]		; get the initial value - read-modify-write to only change 2 bits
	AND R11, #0xcf							; clear two bits that the speaker is on
	ORR R11, #0x10		; initial speaker output (one side high 0x10, the other low 0x20)
buzz_loop
	BL ShortDelay			; delay
	EOR R11, R4			; xor to toggle the two speaker pins to create a beep
	STR R11, [R2, #GPIO_DATA_OFFSET]
	SUBS R3, #1
	BNE buzz_loop
; now power down the speaker
	LDR R11, [R2, #GPIO_DATA_OFFSET]		; restore the speaker pins to 0V on each side
	AND R11, #0xcf							; clear two bits that the speaker is on
	STR R11, [R2, #GPIO_DATA_OFFSET]

	LDMFD		R13!,{R1-R4, R11, R15}		; pull the LR or return address and return


	ALIGN

Port_Table
	DCD	GPIO_PORTA + (PORT_A_MASK << 2)		; DCD - Define Constant Double Word (32-bits)
	DCD	GPIO_PORTB + (PORT_B_MASK << 2), GPIO_PORTC + (PORT_C_MASK << 2)
	DCD	GPIO_PORTD + (PORT_D_MASK << 2), GPIO_PORTE + (PORT_E_MASK << 2)
	DCD	GPIO_PORTF + (PORT_F_MASK << 2), 0

    ALIGN                           ; make sure the end of this section is aligned
    END                             ; end of file - nothing after this is assembled
