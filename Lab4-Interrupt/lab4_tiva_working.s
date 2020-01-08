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
        EXPORT  GPIOPortF_Handler

; These equates allow one to associate a name with a value to make the code more readable

RED       EQU 0x02		; These are the values (bit locations) for various LEDs on the Tiva (Port F)
BLUE      EQU 0x04
GREEN     EQU 0x08
SW1       EQU 0x10                 ; on the left side of the Tiva board
SW2       EQU 0x01                 ; on the right side of the Tiva board

Start
	BL  Port_Init                  ; initialize input and output pins of Ports A to F

	BL Interrupt_Init		; initialize an interrupt for the button

	MOV R11, #0xe1f0		; seed the pseudo random number generator

; each of the Tiva ports is 8 bits wide and connect to wires which can be inputs or outputs

; to read the buttons on port F use the address GPIO_PORTF_DATA_R  EQU 0x400253FC

; feel free to reuse code from Lab #3 to drive the 7-segment display, speaker, ....
	
	
	BL RandomNum
	MOV R7, R11
	STR R5, [R7]
	MOV R4, #10
	UDIV R5, R4
	BL LEDBLINK
	
keep_flashing
	MOV R6, #0		; initialize R6 to zero to detect an interrupt / button press

	; flash the LEDs on and off at a 1 to 10Hz rate
	; be sure that the RandomNum routine is called in the time delay routine
looprand
	SUBS R7, R7, #1
	MOV R0, #100
	BL LEDBLINK
	
	; check for R6 being non-zero as an exit contition
	MOV R0, #100
	BL DELAY
	
	BL RandomNum

	;LDR R7,[R0]
;	SUBS R5, R5, #1
	BEQ looprand
	
	BNE keep_flashing
	BNE.W KeepDisplaying
	
	B keep_flashing

count_down

	BL LEDBLINK
	
	; display the number in R6 on the eight LEDs and optionally on the 7-segment displays

	; delay 1 second
	; be sure that the RandomNum routine is called in the time delay routine

	; if R6 will go negative go back to flashing
	; subtract 10 from R6
		
	B count_down

; This ISR is called when any interrupt associated with a wire on Port F is generated
; A careful programmer would verify which of the 8 possible interrupt sources (pins) generated the input
; If there is only one interrupt and the program is simple then one doesn't have to be so paranoid
;
; On ARM processors R0 .. R3 are automatically saved - you must preserve all other registers you change

GPIOPortF_Handler	; an interrupt due to any pin on Port F calls this interrupt handler

	STMFD		R13!,{R7, R14}		; I'm paranoid - save R1 and R3 since I'm using them
	
	; some code to toggle an LED so I know I made it in here - or use a break point ...

	BL RandomNum	
	MOV R7, R11

	STR R5, [R7]
	MOV R4, #10
	UDIV R5, R4
	MOV R0, #1000
	
	;BL LEDBLINK
	
;	LDR R1, =GPIO_PORTF_DATA_R ; pointer to Port F data register where the LEDs are
;	LDR R2, [R1, #GPIO_DATA_OFFSET]
;	EOR R2, #RED	; toggle the RED LED
;	STR R2, [R1, #GPIO_DATA_OFFSET]		; read-modify-write opertion complete

; scale the random number in R11 to generate a number in R6 between 50 and 250
	LDR R2, [R0]
	STR R2, [R1, #GPIO_DATA_OFFSET]
	LDR R6, [R11]
	MOV R9, #20
	UDIV R6, R6, R9
	MOV R9, #50
	ADD R6, R6, R9
	;MOV R6, ????
	
; Before exiting, the interrupt must be acknowledged by clearing the appropiate bit in the Port F ICR register
;   If this is not done all future interrupts from this Port will be blocked
;   There is no harm in clearing extra interrupt sources (ie using either button to generate an interrupt)

	LDR R1, =GPIO_PORTF + GPIO_ICR_OFFSET
	MOV R0, #0x01            		; clear the interrupt by having a one match the correct bit where the switch is
	STR R0, [R1]	; acknowledge the interrupt
	
	B looprand

	LDMFD		R13!,{R7, R15}		; restore the saved registers and return

GPIO_IS_OFFSET  EQU 0x404	;GPIOIS - Interrupt Sense : 0 = edge, 1 = level interrupt
GPIO_IBE_OFFSET  EQU 0x408	;GPIOIBE - Interrupt Both Edges : 0 = single edge, 1 = both edges
GPIO_IEV_OFFSET  EQU 0x40c	;GPIOIEV - Interrupt Event : 0 = low level or falling edge, 1= high level or rising edge
GPIO_IM_OFFSET  EQU 0x410	;GPIOIM - Interrupt Mask : 0 = masked, 1 = unmasked
GPIO_RIS_OFFSET  EQU 0x414		; Raw Interrupt Status - READ ONLY
GPIO_MIS_OFFSET  EQU 0x418		; Masked Interrupt Status - READ ONLY
GPIO_ICR_OFFSET  EQU 0x41c		; Interrupt Clear - writing a 1 clears the RIS and MIS registers
	
;Program the GPIOIS, GPIOIBE, GPIOEV, and GPIOIM registers to configure the type, event,
;and mask of the interrupts for each port.
;Note: To prevent false interrupts, the following steps should be taken when re-configuring
;GPIO edge and interrupt sense registers:
Interrupt_Init
	STMFD		R13!,{R14}		; push the LR or return address

;a. Mask the corresponding port by clearing the IME field in the GPIOIM register.
	
	LDR R1, =GPIO_PORTF
    MOV R0, #0x00             ; 0 means mask or block interrupts
    STR R0, [R1, #GPIO_IM_OFFSET]	; mask interrupts from happening

; b. Configure the IS field in the GPIOIS register and the IBE field in the GPIOIBE register.

    MOV R0, #0x00             ; 0 means edge detecting interrupt
    STR R0, [R1, #GPIO_IS_OFFSET]	; 

    MOV R0, #0x00             ; 0 means falling edge detecting
    STR R0, [R1, #GPIO_IEV_OFFSET]	; 

    MOV R0, #0x00             ; 0 means single edge detection
    STR R0, [R1, #GPIO_IBE_OFFSET]	; 

;c. Clear the GPIORIS register using the ICR register to clear any pending interrupts.
; The switches are bits 0 and 4 on Port F and a 1 must be written to the bit / switch used.
    MOV R0, #0x10             ; 0 means mask or block interrupts
    STR R0, [R1, #GPIO_ICR_OFFSET]	; clear any interrupts recieved

;d. Unmask the port by setting the IME field in the GPIOIM register.
; Set the appropiate bit to 1 to enable interrupts for only the one switch required
; This register only uses the lowest 8 bits - one for each wire on the port.
    MOV R0, #0x10             ; 0 means mask or block interrupts
	STR R0, [R1, #GPIO_IM_OFFSET]	; mask interrupts from happening

;Looking in the Startup.s file one will find an EXPORT of the address for interrupt handlers, one for each GPIO port

; Interrupt Enable Registers
CORE_PERIPHERALS 		EQU 0xe000e000
INTERRUPT_EN0_OFFSET 	EQU 0x100

; The Interrupt Number (Bit in Interrupt Registers) value written to the EN0 register to enable port F interrupts can be found in Table 2-9 (page 104)
	MOV R0,#0x40000000	; this 32-bit value enables GPIO Port F Interrupts - by setting only the appropiate single bit to one

	LDR R1, =CORE_PERIPHERALS
	STR R0, [R1, #INTERRUPT_EN0_OFFSET]		; GPIO Interrupts require this enable

	LDMFD		R13!,{R15}		; push the LR or return address
	
	
; Driving the 7-segment outputs is harder because they're spread over multiple ports
; Naming of the 7-segment segments is clockwise from the top:  A, B, C, D, E, F, G, DP (decimal point) as per Appendix H
; DO NOT USE THE DP (decimal point) - they are not included below

; left 7-segment bits are: F2, F3, D2, D3, D6, D7, E0   [ports F, D, E are used]
; a code template for driving the left 7-segment is in Lab 3

; right 7-segment bits are: A2, A3, E2, A5, A6, A7, A4 [ ports A, E are used]

; Display numbers to the right 7-segment as an example of read-modify-write data manipulation on I/O ports
; 
; INPUT: R5 - The 4 LSB of R5 will be displayed
; OUTPUT: none
;
display_right_7seg		STMFD		R13!,{R0, R1, R2, R3, R4, R5, R14}

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

	LDMFD		R13!,{R0, R1, R2, R3, R4, R5, R15}

	ALIGN
; This table is the binary pattern of 8 bits required to drive the 7 LEDs of a 7-segment display to display a hex. number
; The LSB is the lowest bit and is not to be used
;
Seven_Seg_Table
	DCW 0x7e, 0x0c, 0xb6, 0x9e, 0xcc, 0xda, 0xfa, 0x0e, 0xfe, 0xce	; 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
	DCW 0xee, 0xf8, 0xb0, 0xbc, 0xf2, 0xe2		; A, B, C, D, E, F with LSB being 0 for the DP
	ALIGN
		
		
		
		
SHORTDELAYCOUNT1             EQU 400   ; faction of a second delay

		; alternate name for this subroutine
;ShortDelay 
DELAY 				STMFD		R13!,{R0, R1, R14}		; push the LR or return address

loopnumtimes
	LDR R8, =SHORTDELAYCOUNT1			;Sets R8 as Delay value
loop
	SUBS R8, R8, #1				;Delay Loop
	BNE loop
		
		
	SUBS R0, R0, #1
	BNE loopnumtimes
exitdelay     		LDMFD		R13!,{R0, R1, R15}	
		
;DELAY
;	MOV R5, #10
;decrement
;	SUB R5, R5, #1
;	BEQ decrement
;	B DELAY
	
		
		
		
		
KeepDisplaying					;Keep looping the reaction time
Displayloop						;Display the bit(s) loop
	BL LEDBLINK 				;Displays byte on the 8LEDS
;	LSR R7, R7, #8				;Shifts to next 'set' of 8 Bites
;	
;	MOV R0, #20000				;2 Seconds
;	BL Delay
;	
;	SUBS R2, R2, #1				;Decrease counter
;	BNE Displayloop	
;								;All bits have been displayed
;	
;	MOV R7, #0
;	MOV R6, #0					;Turn Off all LEDS
;	LDR R8, =GPIO_PORTF_DATA_R  
;	STR R6, [R8]
;	
;	
;	MOV R1, #0					
;	
;	LDR R3, =GPIO_PORTB + (PORT_B_MASK << 2)		
;	LDR R5, =GPIO_PORTE + (PORT_E_MASK << 2)

;	MOV R1, R7		
;    STR R1, [R3, #GPIO_DATA_OFFSET] 
;	STR R1, [R5, #GPIO_DATA_OFFSET]

;	
;	MOV R7, R4					;'Reset' R7 back to reaction time counter
;	MOV R2, #4					;'Reset' 4 set of 8 counter
;	
;	MOV R0, #50000				;5sec Wait
	
	BL DELAY
	
	B KeepDisplaying			;Continue Looping
	




LEDBLINK		STMFD		R13!,{R14}

	MOV R1, #4
	
	LDR R3, =GPIO_PORTB + (PORT_B_MASK << 2)		; generate the base address for port B
	LDR R5, =GPIO_PORTE + (PORT_E_MASK << 2)

	MOV R1, R7		; set lowest 8 bits except for B1 to 1
    STR R1, [R3, #GPIO_DATA_OFFSET] 
	STR R1, [R5, #GPIO_DATA_OFFSET]

;	MOV R0, #10
	BL DELAY

	
	LDMFD		R13!,{R15} 
	BLX 			LR
	
	
	
	

DelayReaction	STMFD		R13!,{R1, R14} 

;	;To get to rand value between 2-10 sec, must scale by a factor of about 1.2 (6/5) and shift 20 (20000) Right
;	MOV R9, #6
;	MUL R1, R11, R9			;Mul by 6
;	
;	MOV R9, #5
;	UDIV R1, R1, R9			;Divide by 5
;	
;	;Output (R1) is now a rand num between 0-80
;	
;	LSR R1, R1, #20			;Shift Right by 20
;	
;	;Output (R1) is now a rand num between 20-100
;	
;	LDRH R1, [R11]		;changes R11 to 16 bit for 2-10 second counter
;	
;	;Scale to 50-250(5-25seconds) then save to R6
	LDR R6, [R11]
	MOV R9, #20
	UDIV R6, R6, R9
	MOV R9, #50
	ADD R6, R6, R9 ; R6 has 50-250 value
	LDR R1, [R6]











; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
;   DO NOT EDIT CODE BELOW THIS LINE
; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 

;------------RandomNum------------
; R11 holds a 16-bit "random" number via a pseudo-random sequence as per the Linear feedback shift register (Fibonacci) on WikiPedia
; R11 holds a non-zero 16-bit number.  If a zero is fed in, as the seed, the pseudo-random sequence will stay stuck at 0
; Take as many bits of R11 as you need.  If you take the lowest 4 bits then you get a number between 0 and 15 while 16 bits gives you a value between 1 and 0xffff.
;
; R11 can be read anywhere in the user code but must only be written to by this subroutine
;
; INPUT: R11 - before calling this for the FIRST time R11 must be initialized to a large 16-bit non-zero value or else it will stay stuck at 0
; OUTPUT: R11 - random number is the lowest 16 bits of R11, the upper 16 bits are cleared
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
				AND			R11, R1		; zero all bits above the pseudorandom 16 bits
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
;exceptions except for reset. See Exception Entry and Return on page 108 for more information.
;The NVIC registers control interrupt handling. See Nested Vectored Interrupt Controller
;(NVIC) on page 124 for more information.

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

; These are the masks for pins which are outputs
PORT_A_MASK			EQU 0xfc	;0xE0		; PA7,6,5 are outputs for LEDs
PORT_B_MASK			EQU 0xff	;3f	; exclude B2:3 0xff	;33		; PB5,4,1,0 are outputs %0011 0011 
PORT_C_MASK			EQU 0x30	; this hangs the CPU 0xf0	
PORT_D_MASK			EQU 0xcc	;exclude d7 0xcf	Disable D0, D1 due to short with B6, B7
PORT_E_MASK			EQU 0x3f	;0x30		; PE5,4 are outputs %0011 0000
PORT_F_MASK			EQU 0x0e		; PF has LEDs on PF 1,2,3 and buttons PF0, PF4 (don't enable buttons as outputs)
	
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
; The speaker is connected to two pins - toggle each end for more volume than a singled ended drive
; before exiting ensure that both wires to the speaker are 0 to prevent it from being heated up
; Ensure that each beep is about the same length - 0x300 loops of delay loop
;
; Input: R1 sets the tone - 2 is a high pitch, and pitch 0 is NO SOUND with the time delay of a beep
; Output: none

SpeakerBeep
	STMFD		R13!,{R1-R4, R11, R14}		; push the LR or return address
	
	MOV R4, #0x30		; a value of 0x30 will toggle the voltage to the speaker
	TEQ R1, #0			; if the user inputs R1 = 0 then keep the speaker silent and just delay for how long a beep would last
	BNE make_beeps
	MOV R1, #1			; define a reasonable value so that the UDIV instruction does not divide by 0
	MOV R4, #0x0		; a value of 0x0 will keep the speaker silent
	
make_beeps
	MOV R3, #0x300
	UDIV R3, R1		; loop the tone R1 / R3 times to ensure a total of 0x100 delays for all tones
	
	LDR R2, =GPIO_PORTE + (PORT_E_MASK << 2)
	LDR R11, [R2, #GPIO_DATA_OFFSET]		; get the initial value of the port E outputs - read-modify-write to only change 2 bits going to the speaker
	AND R11, #0xcf							; clear two bits that the speaker is on  %1100 1111
	ORR R11, #0x10		; initial speaker output (one side high 0x10, the other low 0x20)
buzz_loop
	BL ShortDelay			; delay
	EOR R11, R4			; toggle the voltage to the speaker if R4 ix 0x30 or not if it is 0x0
	STR R11, [R2, #GPIO_DATA_OFFSET]
	SUBS R3, #1
	BNE buzz_loop
	
	AND R11, #0xcf							; before exiting power down the speaker by having 0 on both of it's wires
	STR R11, [R2, #GPIO_DATA_OFFSET]		; this doesn't work perfectly - there is a bit of sound created

	LDMFD		R13!,{R1-R4, R11, R15}		; pull the LR or return address and return
	
; ShortDelay subroutine - delay for a fixed amount of time
;
; This is a bad piece of code - only use it as an example for what never to do!!
;
; Input: R1 - how many times do we loop (multiply) the fixed delay - If R1 = 0 then there is no time delay
; Output: none

SHORTDELAYCOUNT             EQU 2      ; faction of a second delay

Delay		; alternate name for this subroutine
ShortDelay
	STMFD		R13!,{R0, R1, R14}		; push the LR or return address

delay_outer_loop
	TEQ R1, #0
	BNE not_done_delay
	B	done_delay
not_done_delay
	SUB R1, #1
	LDR R0, =SHORTDELAYCOUNT   	       ; R0 = a value to get about a second delay
delay_loop
    SUBS R0, R0, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
				; Note: For SUBs the "s" suffix means to set the status bits, without this the loops would not exit
	CBNZ R0, delay_back			; compare if not zero then keep counting down
	B delay_outer_loop
delay_back
	B delay_loop
done_delay
	LDMFD		R13!,{R0, R1, R15}		; pull the LR or return address and return

	ALIGN

Port_Table
	DCD	GPIO_PORTA + (PORT_A_MASK << 2)		; DCD - Define Constant Double Word (32-bits)
	DCD	GPIO_PORTB + (PORT_B_MASK << 2), GPIO_PORTC + (PORT_C_MASK << 2)
	DCD	GPIO_PORTD + (PORT_D_MASK << 2), GPIO_PORTE + (PORT_E_MASK << 2)
	DCD	GPIO_PORTF + (PORT_F_MASK << 2), 0

    ALIGN                           ; make sure the end of this section is aligned
    END                             ; end of file - nothing after this is assembled
