;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2015 hapcan.com
;
;   This program is free software: you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation, either version 3 of the License, or
;   (at your option) any later version.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program.  If not, see <http://www.gnu.org/licenses/>.
;==============================================================================
;   Filename:              univ_3-6-0-1.asm
;   Associated diagram:    univ_3-6-0-x.sch
;   Author:                Jacek Siwilo
;   Note:                  Trailing edge dimmer for resistive and capacitive
;                          loads
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     05.2015   Original version
;   1     07.2015   Updated with "univ3-routines-rev4.inc"
;==============================================================================
;===  FIRMWARE DEFINITIONS  ===================================================
;==============================================================================
    #define    ATYPE    .6                            ;application type [0-255]
    #define    AVERS    .0                         ;application version [0-255]
    #define    FVERS    .1                            ;firmware version [0-255]

    #define    FREV     .1                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include "univ_3-6-0-1-rev1.inc"                         ;project variables
INCLUDEDFILES   code
    #include "univ3-routines-rev4.inc"                     ;UNIV 3 CPU routines

;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x5F, 0xC1, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
;==============================================================================
;===  FIRMWARE ID  ============================================================
;==============================================================================
FIRMID      code    0x001010
    DB      0x30, 0x00, 0x03,ATYPE,AVERS,FVERS,FREV>>8,FREV
;            |     |     |     |     |     |     |_____|_____ firmware revision
;            |     |     |     |     |     |__________________ firmware version
;            |     |     |     |     |_____________________ application version
;            |     |     |     |______________________________ application type
;            |     |     |________________________________ hardware version '3'
;            |_____|______________________________________ hardware type 'UNIV'
;==============================================================================
;===  MOVED VECTORS  ==========================================================
;==============================================================================
;PROGRAM RESET VECTOR
FIRMRESET   code    0x1020
        goto    Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x1030
        call    HighInterrupt
        retfie
;PROGRAM LOW PRIORITY INTERRUPT VECTOR
FIRMLOWINT  code    0x1040
        call    LowInterrupt
        retfie

;==============================================================================
;===  FIRMWARE STARTS  ========================================================
;==============================================================================
FIRMSTART   code    0x001050
;------------------------------------------------------------------------------
;---  LOW PRIORITY INTERRUPT  -------------------------------------------------
;------------------------------------------------------------------------------
LowInterrupt
        movff   STATUS,STATUS_LOW           ;save STATUS register
        movff   WREG,WREG_LOW               ;save working register
        movff   BSR,BSR_LOW                 ;save BSR register
        movff   FSR0L,FSR0L_LOW             ;save other registers used in low int
        movff   FSR0H,FSR0H_LOW
        movff   FSR1L,FSR1L_LOW
        movff   FSR1H,FSR1H_LOW

    ;main firmware ready flag
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitLowInterrupt            ;main firmware is not ready yet
    ;CAN buffer
        banksel CANFULL
        btfsc   CANFULL,0                   ;check if CAN received anything
        call    CANInterrupt                ;proceed with CAN interrupt

ExitLowInterrupt
        movff   BSR_LOW,BSR                 ;restore BSR register
        movff   WREG_LOW,WREG               ;restore working register
        movff   STATUS_LOW,STATUS           ;restore STATUS register
        movff   FSR0L_LOW,FSR0L             ;restore other registers used in low int
        movff   FSR0H_LOW,FSR0H
        movff   FSR1L_LOW,FSR1L
        movff   FSR1H_LOW,FSR1H
    return

;------------------------------------------------------------------------------
;---  HIGH PRIORITY INTERRUPT  ------------------------------------------------
;------------------------------------------------------------------------------
HighInterrupt
        movff   STATUS,STATUS_HIGH          ;save STATUS register
        movff   WREG,WREG_HIGH              ;save working register
        movff   BSR,BSR_HIGH                ;save BSR register
        movff   FSR0L,FSR0L_HIGH            ;save other registers used in high int
        movff   FSR0H,FSR0H_HIGH
        movff   FSR1L,FSR1L_HIGH
        movff   FSR1H,FSR1H_HIGH

    ;main firmware ready flag
        banksel FIRMREADY
        btfss   FIRMREADY,0
        bra     ExitHighInterrupt           ;main firmware is not ready yet
    ;Timer0
        btfsc   INTCON,TMR0IF               ;Timer0 interrupt? (1000ms)
        rcall   Timer0Interrupt
    ;Timer2
        btfsc   PIR1,TMR2IF                 ;Timer2 interrupt? (20ms)
        rcall   Timer2Interrupt
    ;Timer3
        btfsc   PIR2,TMR3IF                 ;Timer3 interrupt? (phase edge)
        rcall   Timer3Interrupt
    ;INT1
        btfsc   INTCON3,INT1IF              ;INT1 interrupt? (mains zero cross)
        rcall   INT1Interrupt

ExitHighInterrupt
        movff   BSR_HIGH,BSR                ;restore BSR register
        movff   WREG_HIGH,WREG              ;restore working register
        movff   STATUS_HIGH,STATUS          ;restore STATUS register
        movff   FSR0L_HIGH,FSR0L            ;restore other registers used in high int
        movff   FSR0H_HIGH,FSR0H
        movff   FSR1L_HIGH,FSR1L
        movff   FSR1H_HIGH,FSR1H
    return

;------------------------------------------------------------------------------
; Routine:          CAN INTERRUPT
;------------------------------------------------------------------------------
; Overview:         Checks CAN message for response and RTR and saves to FIFO
;------------------------------------------------------------------------------
CANInterrupt
        banksel CANFRAME2
        btfsc   CANFRAME2,0                 ;response message?
    return                                  ;yes, so ignore it and exit
        btfsc   CANFRAME2,1                 ;RTR (Remote Transmit Request)?
    return                                  ;yes, so ignore it and exit
        call    Copy_RXB_RXFIFOIN           ;copies received message to CAN RX FIFO input buffer
        call    WriteToCanRxFIFO            ;saves message to FIFO
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 0 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         1000ms periodical interrupt
;------------------------------------------------------------------------------
Timer0Interrupt:
        call    Timer0Initialization8MHz    ;restart 1000ms Timer
        call    UpdateUpTime                ;counts time from restart
        call    UpdateTransmitTimer         ;increment transmit timer (seconds after last transmission)
        rcall   MainsPresence               ;check if mains connected
        banksel TIMER0_1000ms
        setf    TIMER0_1000ms               ;timer 0 interrupt occurred flag
    return
;-------------------------------
MainsPresence                               ;indicates mains error after 2 seconds
        tstfsz  MAINS_TIMER                 ;decrement MAINS_TIMER
        bra     $ + .4
        bra     $ + .8
        bcf     DIMMER_ERROR,NoMainsErr     ;clear error "no mains"
        decfsz  MAINS_TIMER
        bra     EndMainsPresence
        clrf    PERIOD_H                    ;set error "no mains"
        clrf    PERIOD_L
        bsf     DIMMER_ERROR,NoMainsErr
EndMainsPresence
    return

;------------------------------------------------------------------------------
; Routine:          TIMER 2 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         20ms periodical interrupt
;------------------------------------------------------------------------------
Timer2Interrupt
        rcall   Timer2Initialization8MHz    ;restart timer
        banksel TIMER2_20ms
        setf    TIMER2_20ms                 ;timer 2 interrupt occurred flag
    return
;-------------------------------
Timer2Initialization8MHz
        movlb   0xF
        bcf     PMD1,TMR2MD                 ;enable timer 2
        movlw   0x3F
        movwf   TMR2                        ;set 20ms (19.999500)
        movlw   b'01001111'                 ;start timer, prescaler=16, postscaler=10
        movwf   T2CON
        bsf     IPR1,TMR2IP                 ;high priority for interrupt
        bcf     PIR1,TMR2IF                 ;clear timer's flag
        bsf     PIE1,TMR2IE                 ;interrupt on
    return

;------------------------------------------------------------------------------
; Routine:			TIMER 3 INTERRUPT
;------------------------------------------------------------------------------
; Overview:			Phase cut off beginning
;------------------------------------------------------------------------------
Timer3Interrupt
        bcf 	PIR2,TMR3IF                 ;clear flag
        movlw   0x23                        ;do not turn off transistor
        cpfsgt  PHASE_ON_H,W                ;if PHASE_ON > 0x23FF=9215us
        bcf     MOSFET_PORT,MOSFET_PIN      ;transistor off
    return

;------------------------------------------------------------------------------
; Routine:          EXTERNAL 1 INTERRUPT
;------------------------------------------------------------------------------
; Overview:         Zero Cross Detection
;------------------------------------------------------------------------------
INT1Interrupt
        bcf     INTCON3,INT1IF              ;clear INT1 flag INT1IF
;check if it is close to 10ms period
        movf    TMR1L,W                     ;read timer 1 value (H value will be saved to TMR1H)
        movlw   0x24                        ;0x24FF = 9471 us
        cpfsgt  TMR1H                       ;is f>WREG
        bra     ExitINT1Interrupt           ;no, interrupt occured ealier than 9472ms
;save period value
        movlw   0x02                        ;keep mains timer equal 0x02
        movwf 	MAINS_TIMER
        ;calculate arithmetic mean of two values
        movf    TMR1L,W                     ;add current to previous value
        addwf   PERIOD_L
        movf    TMR1H, W
        addwfc  PERIOD_H
        bcf     STATUS,C                    ;divide by 2
        rrcf    PERIOD_H
        rrcf    PERIOD_L
        ;restart period timer
        clrf    TMR1H
        clrf    TMR1L
;load timer
        comf    PHASE_ON_H,W                ;complement PHASE_H_ON value to get timer value
        movwf   TMR3H
        comf    PHASE_ON_L,W                ;complement PHASE_L_ON value to get timer value
        movwf   TMR3L
;check overheating
        btfsc   DIMMER_ERROR,OvrhErr        ;overheating?
        bra     ExitINT1Interrupt           ;yes
;start transistor
        tstfsz  PHASE1_WORK                 ;don't start if PHASE = 0
        bsf     MOSFET_PORT,MOSFET_PIN
ExitINT1Interrupt
    return

;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Main:
    ;disable global interrupts for startup
        call    DisAllInt                   ;disable all interrupt
    ;firmware initialization
        rcall   PortInitialization          ;prepare processor ports
        call    GeneralInitialization       ;read eeprom config, clear other registers
        call    FIFOInitialization          ;prepare FIFO buffers
        call    Timer0Initialization8MHz    ;Timer 0 initialization for 1s periodical interrupt
        call    Timer2Initialization8MHz    ;Timer 2 initialization for 20ms periodical interrupt
        call    DimmerInitialization        ;init settings
        call    DimmerPowerUpStates         ;sets power up states
    ;firmware ready
        banksel FIRMREADY
        bsf     FIRMREADY,0                 ;set flag "firmware started and ready for interrupts"
    ;enable global interrupts
        call    EnAllInt                    ;enable all interrupts

;-------------------------------
Loop:                                       ;main loop
        clrwdt                              ;clear Watchdog timer
        call    ReceiveProcedure            ;check if any msg in RX FIFO and if so - process the msg
        call    TransmitProcedure           ;check if any msg in TX FIFO and if so - transmit it
        rcall   OnceA20ms                   ;do routines only after 20ms interrupt
        rcall   OnceA1000ms                 ;do routines only after 1000ms interrupt
    bra     Loop

;-------------------------------
OnceA20ms                                   ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER2_20ms
        tstfsz  TIMER2_20ms                 ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    DimmerControl_20ms          ;do all dimmer procedures
        banksel TIMER2_20ms
        clrf    TIMER2_20ms
    return
;-------------------------------
OnceA1000ms                                 ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER0_1000ms
        tstfsz  TIMER0_1000ms               ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    UpdateDelayTimers           ;updates channel timers
        call    UpdateHealthRegs            ;saves health maximums to eeprom
        call    DimmerSaveSateToEeprom      ;save dimmer states into eeprom memory if needed
        banksel TIMER0_1000ms
        clrf    TIMER0_1000ms
    return


;==============================================================================
;===  FIRMWARE ROUTINES  ======================================================
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          PORT INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         It sets processor pins. All unused pins should be set as
;                   outputs and driven low
;------------------------------------------------------------------------------
PortInitialization                          ;default all pins set as analog (portA,B) or digital (portB,C) inputs
    ;PORT A
        banksel ANCON0                      ;select memory bank
        ;0-digital, 1-analog input
        movlw   b'00000011'                 ;(x,x,x,AN4,AN3,AN2,AN1-boot_mode,AN0-volt)
        movwf   ANCON0
        ;output level
        clrf    LATA                        ;all low
        ;0-output, 1-input
        movlw   b'00000011'                 ;all outputs except, bit<1>-boot_mode, bit<0>-volt
        movwf   TRISA
    ;PORT B
        ;0-digital, 1-analog input
        movlw   b'00000000'                 ;(x,x,x,x,x,AN10,AN9,AN8)
        movwf   ANCON1
        ;output level
        clrf    LATB                        ;all low
        ;0-output, 1-input
        movlw   b'00001011'                 ;all output except CANRX, INT0, INT1
        movwf   TRISB
    ;PORT C
        ;output level
        clrf    LATC                        ;all low
        ;0-output, 1-input
        movlw   b'00000000'                 ;all output
        movwf   TRISC
    return

;------------------------------------------------------------------------------
; Routine:          NODE STATUS
;------------------------------------------------------------------------------
; Overview:         It prepares status messages when status request was
;                   received
;------------------------------------------------------------------------------
NodeStatusRequest
        banksel TXFIFOIN0
        ;send status
        movlw   0x01                        ;this is channel 1
        movwf   TXFIFOIN6
        movff   PHASE1_WORK,TXFIFOIN7       ;send PHASE_WORK status
        setf    TXFIFOIN8                   ;unused
        movff   Instr1Ch1,TXFIFOIN9         ;info what instruction is waiting for execution
        movff   Instr2Ch1,TXFIFOIN10
        movff   TimerCh1,TXFIFOIN11         ;value of channel timer
        rcall   SendStatus
        ;send error
        movlw   0xF0                        ;error frame
        movwf   TXFIFOIN6
        movff   DIMMER_ERROR,TXFIFOIN7      ;send PHASE_WORK status
        setf    TXFIFOIN8                   ;unused
        setf    TXFIFOIN9
        setf    TXFIFOIN10
        setf    TXFIFOIN11
        rcall   SendStatus
        ;send MOSFET conduction time
        movlw   0xFE                        ;mosfet conduction time frame                   
        movwf   TXFIFOIN6                   ;channel 1
        movlw   0x01
        movwf   TXFIFOIN7
        movff   PHASE_ON_H,TXFIFOIN8        ;time in us
        movff   PHASE_ON_L,TXFIFOIN9   
        setf    TXFIFOIN10
        setf    TXFIFOIN11
        rcall   SendStatus
		;send period value
        movlw   0xFF                        ;period frame
        movwf   TXFIFOIN6
        movff   PERIOD_H,TXFIFOIN7          ;period value in us
        movff   PERIOD_L,TXFIFOIN8
        setf    TXFIFOIN9                   ;unused
        setf    TXFIFOIN10                  ;unused
        setf    TXFIFOIN11                  ;unused
        rcall	SendStatus
    return
;-------------------------------
SendStatus
        movlw   0x30                        ;dimmer frame
        movwf   TXFIFOIN0
        movlw   0x60
        movwf   TXFIFOIN1
        bsf     TXFIFOIN1,0                 ;response bit
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        setf    TXFIFOIN5                   ;unused
        call    WriteToCanTxFIFO
    return

;------------------------------------------------------------------------------
; Routine:          DO INSTRUCTION
;------------------------------------------------------------------------------
; Overview:         Executes instruction immediately or sets timer for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionRequest
        ;allow only known values
        banksel INSTR1
        movlw   0x0A                        ;INSTR less than?
        cpfslt  INSTR1
    bra     ExitDoInstructionRequest        ;no, so exit
        ;Recognize instruction
        movf    INSTR1,W
        call    ComputedGoto                ;modify PC according to WREG value
        bra     Instr00                     ;instruction 00
        bra     Instr01                     ;instruction 01
        bra     Instr02                     ;instruction 02
        bra     Instr03                     ;instruction 03
        bra     Instr04                     ;instruction 04
        bra     Instr05                     ;instruction 05
        bra     Instr06                     ;instruction 06
        bra     Instr07                     ;instruction 07
        bra     Instr08                     ;instruction 08
        bra     Instr09                     ;instruction 09

;-------------------------------
;Instruction execution
Instr00                                     ;SET DIMMER VALUE to INSTR2
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest
        movff   INSTR2,PHASE1               ;move desired value
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        call    DoInstLater_ClearCh1        ;clear any waiting instruction
        bra     ExitDoInstructionRequest
;---------------
Instr01                                     ;TOGGLE DIMMER VALUE
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest
        clrf    PHASE1_STARTCNT             ;clear START instr counter
        clrf    PHASE1                      ;turn off
        tstfsz  PHASE1_WORK                 ;test current value
        bra     $ + .16
        ;memory
        banksel E_PHASE1_SOURCE
        btfss   E_PHASE1_SOURCE,E_MEM_BIT   ;memory set?
        bra     $ + .8                      ;no
        movff   PHASE1_MEM,PHASE1           ;yes, so take value from last used
        bra     $ + .4
        setf    PHASE1                      ;change to max value
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        call    DoInstLater_ClearCh1
        bra     ExitDoInstructionRequest
;---------------
Instr02                                     ;INSTR2 STEPS DOWN
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest
        tstfsz  INSTR2                      ;is counter = 0?
        bra     $ + .4
        bra     ExitDoInstructionRequest    ;yes, so exit
        clrf    WREG                        ;exit when PHASE = zero
        xorwf   PHASE1,W
        bz      $ + .8
        decf    PHASE1
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        call    DoInstLater_ClearCh1
        bra     ExitDoInstructionRequest
;---------------
Instr03                                     ;INSTR2 STEPS UP
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest
        tstfsz  INSTR2                      ;is counter = 0?
        bra     $ + .4
        bra     ExitDoInstructionRequest    ;yes, so exit
        setf    WREG                        ;exit when max value
        xorwf   PHASE1,W
        bz      $ + .8
        incf    PHASE1
        decfsz  INSTR2
        bra     $ - .10
        bcf     PHASE_FLAG,Ch1ChangeSlow
        call    DoInstLater_ClearCh1
        bra     ExitDoInstructionRequest
;---------------
Instr04                                     ;STOP DIMMER
        movlw   0x00                        ;stop after START? (400ms passed?)
        xorwf   PHASE1_STARTCNT,W
        bz      $ + .6                      ;yes, so stop dimming/brightening
        clrf    INSTR3                      ;no, clear TIMER for toggle instruction
        bra     Instr01                     ;go to toggle channel state
        movff   PHASE1_WORK,PHASE1          ;stop phase
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately flag
        call    DoInstLater_ClearCh1
        bra     ExitDoInstructionRequest
;---------------
Instr05                                     ;START DIMMER
        movlw   .20                         ;(move 20x 20ms = 400ms to counter)
        movwf   PHASE1_STARTCNT
        clrf    TIMER                       ;instruction will be visible in status msg
        call    DoInstLater_SetCh1
        bra     ExitDoInstructionRequest
;---------------
Instr06                                     ;SET DIMMER VALUE SOFTLY to INSTR2
        tstfsz  INSTR3                      ;is timer = 0?
        bra     $ + .4                      ;no, so do instruction later
        bra     $ + .12                     ;yes, so do instruction now
        movff   INSTR3,TIMER                ;move timer from INSTR3
        call    DoInstLater_SetCh1          ;save instruction for later execution
        bra     ExitDoInstructionRequest
        movff   INSTR2,PHASE1               ;move desired value
        bsf     PHASE_FLAG,Ch1ChangeSlow    ;change softly flag
        call    DoInstLater_ClearCh1        ;clear any waiting instruction
        bra     ExitDoInstructionRequest
;---------------
Instr07                                     ;SET MINIMUM to INSTR2
        movff   INSTR2,PHASE1_MIN
        bra     ExitDoInstructionRequest
;---------------
Instr08                                     ;SET MAXIMUM to INSTR2
        movff   INSTR2,PHASE1_MAX
        bra     ExitDoInstructionRequest
;---------------
Instr09                                     ;SET DIMMING SPEED to INSTR2
        movff   INSTR2,PHASE1_SPEED
        bra     ExitDoInstructionRequest
;---------------
ExitDoInstructionRequest
        call    EepromToSave                ;save new states to eeprom in a few seconds
    return

;------------------------------------------------------------------------------
; Routine:          DO INSTRUCTION LATER
;------------------------------------------------------------------------------
; Overview:         It saves instruction for particular channel for later
;                   execution
;------------------------------------------------------------------------------
DoInstLater_SetCh1
        call    SetTimer                    ;update SUBTIMER1 & SUBTIMER2 registers
        movff   INSTR1,Instr1Ch1            ;copy registers
        movff   INSTR2,Instr2Ch1
        movff   TIMER,TimerCh1
        movff   SUBTIMER1,SubTmr1Ch1
        movff   SUBTIMER2,SubTmr2Ch1
    return

DoInstLater_ClearCh1
        banksel Instr1Ch1
        setf    Instr1Ch1
        setf    Instr2Ch1
        clrf    TimerCh1
        clrf    SubTmr1Ch1
        clrf    SubTmr2Ch1
    return

;==============================================================================
;                   DIMMER PROCEDURES
;==============================================================================
;------------------------------------------------------------------------------
; Routine:          DIMMER INITIALIZATION
;------------------------------------------------------------------------------
; Overview:         Sets external and Timer3 interupts
;------------------------------------------------------------------------------
DimmerInitialization
    ;external interupt 1                    ;zero cross detection
        bcf     INTCON3,INT1IF              ;clear INT1 flag INT1IF
        bsf     INTCON2,INTEDG1             ;interrupt on rising edge INTEDG1 (just before mains zero cross)
        bsf     INTCON3,INT1IP              ;high priority
        bsf     INTCON3,INT1IE              ;enable INT1 interrupt INT1IE
    ;initiate timer 1                       ;for mains period counting
        movlb   0xF
        bcf     PMD1,TMR1MD                 ;enable timer 1
        movlw   b'00011011'                 ;FOSC/4, prescaler 1:2, internal clock, 16bit, start
        movwf   T1CON
    ;initiate timer 3                       ;for phase time control
        movlb   0xF
        bcf     PMD1,TMR3MD                 ;enable timer 3
        movlw   b'00011011'                 ;FOSC/4, prescaler 1:2, internal clock, 16bit, start
        movwf   T3CON
        bsf     IPR2,TMR3IP                 ;high priority
        bcf     PIR2,TMR3IF                 ;clear flag
        bsf     PIE2,TMR3IE                 ;enable interrupt

    return

;------------------------------------------------------------------------------
; Routine:          DIMMER POWER UP STATES
;------------------------------------------------------------------------------
; Overview:         Sets power up states according to configuration
;------------------------------------------------------------------------------
DimmerPowerUpStates
    ;CONFIG
        banksel E_PHASE1_SOURCE
        ;dimmer value
        movff   E_PHASE1_VAL_FIXED,PHASE1   ;take value from fixed state reg
        btfsc   E_PHASE1_SOURCE,E_VAL_BIT   ;set 1?
        movff   E_PHASE1_VAL_SAVED,PHASE1   ;bit set, so take value from last saved
        ;dimmer min
        movff   E_PHASE1_MIN_FIXED,PHASE1_MIN
        btfsc   E_PHASE1_SOURCE,E_MIN_BIT
        movff   E_PHASE1_MIN_SAVED,PHASE1_MIN
        ;dimmer max
        movff   E_PHASE1_MAX_FIXED,PHASE1_MAX
        btfsc   E_PHASE1_SOURCE,E_MAX_BIT
        movff   E_PHASE1_MAX_SAVED,PHASE1_MAX
        ;dimming speed
        movff   E_PHASE1_SPEED_FIXED,PHASE1_SPEED
        btfsc   E_PHASE1_SOURCE,E_SPEED_BIT
        movff   E_PHASE1_SPEED_SAVED,PHASE1_SPEED

    ;OTHER REGISTERS
        clrf    PERIOD_H
        clrf    PERIOD_L
        clrf    PHASE1_WORK
        bcf     PHASE_FLAG,Ch1ChangeSlow    ;change immediately
        setf    PHASE1_MEM                  ;remembered value equals 255 at the begining
        clrf    PHASE1_STARTCNT             ;START instruction timers
        clrf    DIMMER_ERROR
    return

;------------------------------------------------------------------------------
; Routine:          DIMMER CONTROL
;------------------------------------------------------------------------------
; Overview:         It periodically (on every 20ms timer interrupt) updates
;                   all Dimmer control settings
;------------------------------------------------------------------------------
DimmerControl_20ms
        rcall   DimmerOverheating           ;indicate overheating
        rcall   DimmerDSCPhase1             ;count how long the START instruction is present
        rcall   DimmerUpdateState           ;update PHASE reg
        rcall   DimmerStates                ;send new states if changed
DimmerControlExit
    return

;------------------------------------------------------------------------------
; Routine:          DIMMER OVERHEATING
;------------------------------------------------------------------------------
; Overview:         Checks if dimmer is overheating and indicates with 2 flashes
;------------------------------------------------------------------------------
DimmerOverheating
    ;check if overheating
        btfss   OVRH_PORT,OVRH_PIN          ;overheating?
        bra     $ + .6                      ;yes      
        bcf     DIMMER_ERROR,OvrhErr        ;clear error
    return                                  ;no
        tstfsz  PHASE1_WORK                      ;dimmer turned on?
        bra     $ + .4
    return                                  ;no
    ;indicate overheating
        bcf     DIMMER_ERROR,OvrhErr        ;clear error for indication
        clrf    PHASE_ON_H                  ;turn off
        clrf    PHASE_ON_L
        rcall   Wait500ms
        movlw   0x25                        ;turn on
        movwf   PHASE_ON_H
        rcall   Wait500ms
        clrf    PHASE_ON_H                  ;turn off
        rcall   Wait500ms
        movlw   0x25                        ;turn on
        movwf   PHASE_ON_H
        rcall   Wait500ms
        clrf    PHASE_ON_H                  ;turn off
    ;set other regs
        bsf     DIMMER_ERROR,OvrhErr        ;set error
        clrf    PHASE1                      ;clear phase regs
        clrf    PHASE1_WORK
    return        

;-------------------------------
Wait100ms
        setf    R1	
T20L2:  setf    R0
T20L1:  decfsz  R0
        bra     T20L1
        decfsz  R1
        bra     T20L2
    return
;-------------------------------
Wait500ms
        rcall   Wait100ms
        rcall   Wait100ms
        rcall   Wait100ms
        rcall   Wait100ms
        rcall   Wait100ms
    return

;------------------------------------------------------------------------------
; Routine:          DIMMER DECREMENT START COUNTERS
;------------------------------------------------------------------------------
; Overview:         Decrements counters of START instruction. If START
;                   instruction is present for longer than 400ms then
;                   dimming/brightening begins.
;------------------------------------------------------------------------------
DimmerDSCPhase1
        tstfsz  PHASE1_STARTCNT             ;zero already?
        bra     $ + .4
        bra     DimmerDSCPhase1Exit         ;yes, so exit
        decfsz  PHASE1_STARTCNT             ;decrement counter
        bra     DimmerDSCPhase1Exit         ;and exit if not zero
        ;if counter zero, start dim/bright
        bsf     PHASE_FLAG,Ch1ChangeSlow    ;change softly flag
        movf    PHASE1_WORK,W               ;if channel is at maximum then start dimming
        cpfseq  PHASE1_MAX
        bra     $ + .4
        bra     Phase1Dim
        movf    PHASE1_WORK,W               ;if channel is at minimum then start brightening
        cpfseq  PHASE1_MIN
        bra     $ + .4
        bra     Phase1Bright
        tstfsz  PHASE1_WORK                 ;if channel is turned off start brightening
        bra     $ + .4
        bra     Phase1Bright
        btfss   PHASE_FLAG,0                ;if none of them, then check what was last 0-dimming 1-brightening?
        bra     $ + .6
        bra     Phase1Dim
        bra     $ + .4
        bra     Phase1Bright
Phase1Bright
        movff   PHASE1_MAX,PHASE1           ;brightening - set maximum as target
        bra     DimmerDSCPhase1Exit
Phase1Dim
        movff   PHASE1_MIN,PHASE1           ;dimming - set mminimum as target
        bra     DimmerDSCPhase1Exit
DimmerDSCPhase1Exit
    return

;------------------------------------------------------------------------------
; Routine:          DIMMER UPDATE STATE
;------------------------------------------------------------------------------
; Overview:         It updates immediately or slowly dimmer state acording to
;                   new settings
;------------------------------------------------------------------------------
DimmerUpdateState
        btfsc   PHASE_FLAG,Ch1ChangeSlow    ;ch1 change slowly?
        rcall   DimmerUpdateStateSoftlyCh1  ;yes
        btfss   PHASE_FLAG,Ch1ChangeSlow    ;ch1 change immediately?
        rcall   DimmerUpdateStateImmediatelyCh1 ;yes
    ;memory                                 ;remember last value
        movf    PHASE1_WORK,W               ;check if PHASE=PHASE_WORK (PHASE_WORK is stable?)
        cpfseq  PHASE1
        bra     $ + .8                      ;no, so exit
        tstfsz  PHASE1_WORK
        movff   PHASE1_WORK,PHASE1_MEM      ;save phase in mem reg
    ;take dimmer value from characteristic
        banksel E_PHASE1_CURVE              ;make sure that requested curve exists
        movlw   0x06                        ;curve number less than?
        cpfslt  E_PHASE1_CURVE
        bra     Characteristic_00           ;no, so take default curve

        movf    E_PHASE1_CURVE,W
        call    ComputedGoto                ;modify PC according to WREG value
        bra     Characteristic_00           ;curve 00 - default
        bra     Characteristic_01           ;curve 01 - user defined
        bra     Characteristic_02           ;curve 02 - linear
        bra     Characteristic_03           ;curve 03 - square
        bra     Characteristic_04           ;curve 04 - incandescent
        bra     Characteristic_05           ;curve 05 - LED

Characteristic_00                           ;default
        ;calculate Timer PHASE_ON value from actual PHASE
		movf	PHASE1_WORK,W               ;multiply phase register with constant
	    mullw	.37					        ;37x255=9435 cycles = 9,45ms
	    movff	PRODH,PHASE_ON_H            ;move 16bit result to PHASE_ON regs
	    movff	PRODL,PHASE_ON_L
        bra     DimmerUpdateStateExit
Characteristic_01                           ;user defined
        movlw   upper CHAR_USER             ;access beginning of table
        movwf   TBLPTRU
        movlw   high CHAR_USER
        movwf   TBLPTRH
        movlw   low CHAR_USER
        movwf   TBLPTRL
        rcall   CharacteristicGetValue      ;read value from curve
        bra     DimmerUpdateStateExit
Characteristic_02                           ;linear
        movlw   upper CHAR_02
        movwf   TBLPTRU
        movlw   high CHAR_02
        movwf   TBLPTRH
        movlw   low CHAR_02
        movwf   TBLPTRL
        rcall   CharacteristicGetValue
        bra     DimmerUpdateStateExit
Characteristic_03                           ;square
        movlw   upper CHAR_03
        movwf   TBLPTRU
        movlw   high CHAR_03
        movwf   TBLPTRH
        movlw   low CHAR_03
        movwf   TBLPTRL
        rcall   CharacteristicGetValue
        bra     DimmerUpdateStateExit
Characteristic_04                           ;incandescent
        movlw   upper CHAR_04
        movwf   TBLPTRU
        movlw   high CHAR_04
        movwf   TBLPTRH
        movlw   low CHAR_04
        movwf   TBLPTRL
        rcall   CharacteristicGetValue
        bra     DimmerUpdateStateExit
Characteristic_05                           ;LED
        movlw   upper CHAR_05
        movwf   TBLPTRU
        movlw   high CHAR_05
        movwf   TBLPTRH
        movlw   low CHAR_05
        movwf   TBLPTRL
        rcall   CharacteristicGetValue
        bra     DimmerUpdateStateExit

DimmerUpdateStateExit
    return

CharacteristicGetValue
        movf    PHASE1_WORK,W               ;add twice PHASE1_WORK value to get table specific address
        addwf   TBLPTRL,F
        clrf    WREG
        addwfc  TBLPTRH
        movf    PHASE1_WORK,W
        addwf   TBLPTRL,F
        clrf    WREG
        addwfc  TBLPTRH
        tblrd*+                             ;read table and increment table address
        movff   TABLAT,PHASE_ON_L           ;move value from table
        tblrd*
        movff   TABLAT,PHASE_ON_H
    return

;------------------------------------------------------------------------------
DimmerUpdateStateImmediatelyCh1
        movf    PHASE1,W                    ;check if PHASE=PHASE_WORK
        cpfseq  PHASE1_WORK
        bra     $ + .4                      ;not equal
    return
        call    EepromToSave                ;pospone saving to eeprom
        movf    PHASE1,W                    ;check if PHASE<PHASE_WORK
        cpfslt  PHASE1_WORK
        bra     DimImmediatelyCh1           ;not less
        bra     BrightImmediatelyCh1        ;less
DimImmediatelyCh1
		rcall	DecImmediatelyPHASE1
        call    MAMMPhase1                  ;match against MIN & MAX
        bcf     PHASE_FLAG,Ch1LastWasBright ;set mark=0 - last was dimming
    return
BrightImmediatelyCh1
        rcall   IncImmediatelyPHASE1
        call    MAMMPhase1                  ;match against MIN & MAX
        bsf     PHASE_FLAG,Ch1LastWasBright ;set mark=1 - last was brightening
    return
;-------------------------------
DecImmediatelyPHASE1                        ;decrements PHASE regs immediately
        movlw   0x05                        ;dimming time=1s, so decrement 5x (means 255/5=51steps 20ms each)
        movwf   LoopCnt                     ;loop counter
        movf    PHASE1,W                    ;check if PHASE<PHASE_WORK
        cpfseq  PHASE1_WORK
        bra     $ + 4                       ;not equal
    return                                  ;equal, so exit
        rcall   DecImmediatPHASE1Once
        decfsz  LoopCnt
        bra $ - .10                         ;go to line 'cpfseq	PHASE1_WORK' to compare
    return
DecImmediatPHASE1Once
        tstfsz  PHASE1_WORK                 ;do not dim if already zero
        decf    PHASE1_WORK
    return
IncImmediatelyPHASE1
        movlw   0x05                        ;dimming time=1s
        movwf   LoopCnt                     ;loop counter
        movf    PHASE1,W                    ;check if PHASE<PHASE_WORK
        cpfseq  PHASE1_WORK
        bra $ + 4                           ;not equal
    return                                  ;equal, so exit
        rcall   IncImmediatPHASE1Once
        decfsz  LoopCnt
        bra     $ - .10
    return
IncImmediatPHASE1Once
        infsnz  PHASE1_WORK                 ;skip if not zero after increment
        decf    PHASE1_WORK                 ;if PHASE_WORK euqals 0x00 step back to 0xFF
    return

;------------------------------------------------------------------------------
DimmerUpdateStateSoftlyCh1
        movf    PHASE1,W                    ;check if PHASE=PHASE_WORK
        cpfseq  PHASE1_WORK
        bra     $ + .4                      ;not equal
    return
        call    EepromToSave                ;pospone saving to eeprom
        movf    PHASE1,W                    ;check if PHASE<PHASE_WORK
        cpfslt  PHASE1_WORK
        bra     GoToDim1                    ;not less
        bra     GoToBright1                 ;less
GoToDim1
        rcall   DecPHASE1
        call    MAMMPhase1                  ;match against MIN & MAX
        bcf     PHASE_FLAG,Ch1LastWasBright ;set mark=0 - last was dimming
    return
GoToBright1
        rcall   IncPHASE1
        call    MAMMPhase1                  ;match against MIN & MAX
        bsf     PHASE_FLAG,Ch1LastWasBright ;set mark=1 - last was brightening
    return
;-------------------------------
EepromToSave                                ;indicate that save to eeprom nedded
        banksel EEPROMTIMER
        movlw   0x06                        ;wait 6s before saving to eeprom
        movwf   EEPROMTIMER
    return

;------------------------------------------------------------------------------
; Decrement or Increment PHASE per 20ms
;------------------------------------------------------------------------------
Get20msStep     ;calculates PHASE delay step based on WREG reg (0-255) (1s-256s).
                ;gives value which has to be added to PHASE reg on every 20ms
                ;24 bit result (8bit int, 16bit fraction) in PHASE_STEP reg
                ;PHASE_STEP=255/(WREG*50+50)
        mullw   .50                         ;step 50 times per second for 20ms interrupt
        movff   PRODL,DIVISOR_L             ;WREG*50
        movff   PRODH,DIVISOR_H
        movlw   .50                         ;WREG*50+50
        addwf   DIVISOR_L
        clrf    WREG                        ;check if carry
        addwfc  DIVISOR_H
        setf    DIVIDEND_U                  ;set divident 255.00 = 0xFF.0000 (max PHASE value)
        clrf    DIVIDEND_H
        clrf    DIVIDEND_L
        clrf    PHASE_STEP_U                ;clear result
        clrf    PHASE_STEP_H
        clrf    PHASE_STEP_L
        clrf    REMAINDER_H                 ;clear reminder
        clrf    REMAINDER_L

        movlw   .24                         ;255/(WREG*50+50)
        movwf   BIT_COUNTER                 ;shift 24 bits
DPG20S_Loop
        ;rotate bit by bit
        rlcf    DIVIDEND_L                  ;rotate divident
        rlcf    DIVIDEND_H
        rlcf    DIVIDEND_U
        rlcf    REMAINDER_L,F               ;rotate remainder through MSBit of dividend
        rlcf    REMAINDER_H,F
        ;check if divisor can be substracted from remainder
        movf    DIVISOR_L,W
        subwf   REMAINDER_L,W
        movf    DIVISOR_H,W
        subwfb  REMAINDER_H,W
        btfss   STATUS,C
        bra     DPG20S_ShiftResult          ;C=!B=0, borrowing, so do not substruct
        ;substract divisor from remainder
        movf    DIVISOR_L,W
        subwf   REMAINDER_L,F
        movf    DIVISOR_H,W
        subwfb  REMAINDER_H,F
DPG20S_ShiftResult
        rlcf    PHASE_STEP_L                ;rotate result
        rlcf    PHASE_STEP_H
        rlcf    PHASE_STEP_U

        decfsz  BIT_COUNTER                 ;all bits done?
        bra     DPG20S_Loop                 ;not yet
    return

;-------------------------------
PHASE1AddStep
        movf    PHASE1_SPEED,W
        rcall   Get20msStep                 ;calculates 20ms PHASE step based on PHASE_SPEED reg
        movf    PHASE_STEP_L,W              ;add step to PHASE_COUNTER
        addwf   PHASE1_CNT_L,F              ;PHASE_COUNTER fraction
        movf    PHASE_STEP_H,W
        addwfc  PHASE1_CNT_H,F              ;PHASE_COUNTER fraction
        movf    PHASE_STEP_U,W
        addwfc  PHASE1_CNT_U,F              ;PHASE_COUNTER whole
    return
;---------------
DecPHASE1
        rcall   PHASE1AddStep               ;add step into PHASE_COUNTER
        movf    PHASE1_CNT_U,W              ;decrement PHASE only PHASE_COUNTER whole number times
        bz      $ + .8                      ;whole part equals zero, so exit
        rcall   DecPHASE1Once
        decfsz  PHASE1_CNT_U
        bra     $ - .4
    return
DecPHASE1Once
        tstfsz  PHASE1_WORK                 ;do not dim if already zero
        bra     $ + .4
        bra     $ + .4
        decf    PHASE1_WORK
    return
;---------------
IncPHASE1
        rcall   PHASE1AddStep               ;add step into PHASE_COUNTER
        movf    PHASE1_CNT_U,W              ;increment PHASE only PHASE_COUNTER whole number times
        bz      $ + .8                      ;whole part equals zero, so exit
        rcall   IncPHASE1Once
        decfsz  PHASE1_CNT_U
        bra     $ - .4
    return
IncPHASE1Once
        infsnz  PHASE1_WORK                 ;skip if not zero after increment
        decf    PHASE1_WORK                 ;if PHASE_WORK euqals 0x00 step back to 0xFF
    return

;------------------------------------------------------------------------------
;  Match Wworking Phase Against Min & Max
;------------------------------------------------------------------------------
MAMMPhase1                                  ;PHASE 1
        movf    PHASE1_MIN,W                ;MIN
        cpfslt  PHASE1_WORK                 ;is PHASE_WORK < PHASE_MIN ?
        bra     MAMMPhase1_Max              ;no, so keep current value & go to MAX check
        movwf   PHASE1_WORK                 ;yes, so change PHASE_WORK to PHASE_MIN
        tstfsz  PHASE1                      ;is target, value of PHASE=0?
        bra     $ + .6                      ;no
        clrf    PHASE1_WORK                 ;yes, so change to zero
    return
        movwf   PHASE1                      ;so update target as PHASE_MIN value
    return
MAMMPhase1_Max
        movf    PHASE1_MAX,W                ;MAX
        cpfsgt  PHASE1_WORK                 ;is PHASE_WORK > PHASE_MAX ?
        bra     $ + .6                      ;no, so keep current value
        movwf   PHASE1_WORK                 ;yes, so change PHASE_WORK to PHASE_MAX
        movwf   PHASE1                      ;and change PHASE to PHASE_MAX, so will not be compared again
    return

;------------------------------------------------------------------------------
; Routine:          SEND DIMMER STATES
;------------------------------------------------------------------------------
; Overview:         Sends new states if changed
;------------------------------------------------------------------------------
DimmerStates
        rcall   DimmerValue                 ;send dimmer value if changed
        rcall   DimmerError
    return

;-------------------------------
DimmerValue
        movf    PHASE1_WORK,W               ;check if PHASE=PHASE_WORK (PHASE_WORK is stable?)
        cpfseq  PHASE1
    return                                  ;no, so exit
        movf    PHASE1_WORK,W               ;check if PHASE_COPY=PHASE_WORK (previously sent state the same?)
        cpfseq  PHASE1_COPY
        bra     $ + .4
    return                                  ;yes, so exit
        banksel TXFIFOIN0
        movlw   0x01                        ;"ch1"
        movwf   TXFIFOIN6
        movff   PHASE1_WORK,TXFIFOIN7
        setf    TXFIFOIN8                   ;unused
        movff   Instr1Ch1,TXFIFOIN9         ;instruction of channel
        movff   Instr2Ch1,TXFIFOIN10
        movff   TimerCh1,TXFIFOIN11
        movff   PHASE1_WORK,PHASE1_COPY     ;save state
        rcall   DimmerSendState
    return
;-------------------------------
DimmerError
        movf    DIMMER_ERROR,W              ;check if DIMMER_ERROR_COPY=DIMMER_ERROR (previously sent error the same?)
        cpfseq  DIMMER_ERROR_COPY
        bra     $ + .4
    return                                  ;yes, so exit
        banksel TXFIFOIN0
        movlw   0xF0                        ;"error frame"
        movwf   TXFIFOIN6
        movff   DIMMER_ERROR,TXFIFOIN7
        setf    TXFIFOIN8                   ;unused
        setf    TXFIFOIN9 
        setf    TXFIFOIN10
        setf    TXFIFOIN11
        movff   DIMMER_ERROR,DIMMER_ERROR_COPY ;save last sent error
        rcall   DimmerSendState
    return
;-------------------------------
DimmerSendState
        banksel TXFIFOIN0
        movlw   0x30                        ;set dimmer frame
        movwf   TXFIFOIN0
        movlw   0x60
        movwf   TXFIFOIN1
        movff   NODENR,TXFIFOIN2            ;node id
        movff   GROUPNR,TXFIFOIN3
        setf    TXFIFOIN4                   ;unused
        setf    TXFIFOIN5                   ;unused
        call    WriteToCanTxFIFO
    ;node can respond to its own message
        bcf     INTCON,GIEL                 ;disable low priority intr to make sure RXFIFO buffer is not overwritten
        call    Copy_TXFIFOIN_RXFIFOIN
        call    WriteToCanRxFIFO
        bsf     INTCON,GIEL                 ;enable back interrupt
    return

;------------------------------------------------------------------------------
; Routine:          SAVE STATES TO EEPROM
;------------------------------------------------------------------------------
; Overview:         It saves current relay states into EEPROM memory
;------------------------------------------------------------------------------
DimmerSaveSateToEeprom
        banksel EEPROMTIMER
    ;wait 6s before saving
        tstfsz  EEPROMTIMER
        decfsz  EEPROMTIMER
        bra     ExitSaveSateToEeprom
    ;save to eeprom
        banksel E_PHASE1_VAL_SAVED
        clrf    EEADRH                      ;point at high address
        ;dimmer value
        movf    E_PHASE1_VAL_SAVED,W        ;values the same?
        xorwf   PHASE1_WORK,W
        bz      $ + .16                     ;yes, so don't save
        movff   PHASE1_WORK,E_PHASE1_VAL_SAVED
        movlw   low E_PHASE1_VAL_SAVED      ;take eeprom address
        movwf   EEADR
        movf    PHASE1_WORK,W               ;set data for EepromSaveWREG routine
        call    EepromSaveWREG
        ;dimmer min
        movf    E_PHASE1_MIN_SAVED,W
        xorwf   PHASE1_MIN,W
        bz      $ + .16
        movff   PHASE1_MIN,E_PHASE1_MIN_SAVED
        movlw   low E_PHASE1_MIN_SAVED
        movwf   EEADR
        movf    PHASE1_MIN,W
        call    EepromSaveWREG
        ;dimmer max
        movf    E_PHASE1_MAX_SAVED,W
        xorwf   PHASE1_MAX,W
        bz      $ + .16
        movff   PHASE1_MAX,E_PHASE1_MAX_SAVED
        movlw   low E_PHASE1_MAX_SAVED
        movwf   EEADR
        movf    PHASE1_MAX,W
        call    EepromSaveWREG
        ;dimming speed
        movf    E_PHASE1_SPEED_SAVED,W
        xorwf   PHASE1_SPEED,W
        bz      $ + .16
        movff   PHASE1_SPEED,E_PHASE1_SPEED_SAVED
        movlw   low E_PHASE1_SPEED_SAVED
        movwf   EEADR
        movf    PHASE1_SPEED,W
        call    EepromSaveWREG
ExitSaveSateToEeprom
    return



;------------------------------------------------------------------------------
;                   FIRMWARE FIXED DATA
;------------------------------------------------------------------------------
CHAR_02                                         ;dimmer linear characteristic
    DW         .0, .412, .462, .505, .540, .580, .620, .650, .680, .700, .735, .775, .810, .835, .880, .905
    DW       .930, .960, .985,.1005,.1040,.1070,.1095,.1140,.1165,.1195,.1225,.1250,.1265,.1290,.1315,.1345
    DW      .1375,.1405,.1425,.1445,.1480,.1510,.1535,.1550,.1565,.1600,.1640,.1660,.1675,.1715,.1735,.1765
    DW      .1800,.1825,.1855,.1865,.1885,.1925,.1960,.1980,.2000,.2035,.2050,.2080,.2115,.2140,.2165,.2185
    DW      .2215,.2235,.2255,.2265,.2295,.2325,.2350,.2370,.2400,.2415,.2445,.2460,.2485,.2520,.2540,.2575
    DW      .2595,.2615,.2645,.2670,.2695,.2710,.2720,.2755,.2785,.2795,.2825,.2840,.2860,.2885,.2910,.2935
    DW      .2950,.2970,.2990,.3020,.3045,.3070,.3090,.3100,.3130,.3175,.3205,.3215,.3240,.3260,.3275,.3305
    DW      .3330,.3340,.3355,.3380,.3395,.3430,.3455,.3480,.3495,.3535,.3545,.3575,.3595,.3615,.3635,.3660
    DW      .3680,.3710,.3725,.3750,.3770,.3795,.3820,.3855,.3880,.3905,.3935,.3945,.3980,.4010,.4050,.4080
    DW      .4100,.4120,.4150,.4180,.4205,.4225,.4245,.4265,.4290,.4320,.4355,.4380,.4405,.4445,.4465,.4495
    DW      .4520,.4555,.4580,.4600,.4625,.4645,.4665,.4675,.4685,.4700,.4735,.4755,.4775,.4805,.4850,.4880
    DW      .4910,.4930,.4955,.4980,.5000,.5020,.5055,.5075,.5100,.5140,.5170,.5195,.5240,.5255,.5295,.5325
    DW      .5360,.5395,.5415,.5445,.5480,.5505,.5535,.5575,.5635,.5650,.5665,.5695,.5715,.5750,.5785,.5830
    DW      .5870,.5905,.5930,.5980,.6000,.6025,.6050,.6090,.6145,.6175,.6195,.6220,.6255,.6290,.6340,.6380
    DW      .6410,.6450,.6490,.6535,.6595,.6615,.6675,.6720,.6760,.6825,.6885,.6930,.6990,.7030,.7060,.7150
    DW      .7200,.7270,.7340,.7390,.7455,.7540,.7640,.7760,.7840,.7910,.8010,.8170,.8310,.8410,.8550,.9020

CHAR_03                                         ;dimmer square characteristic
    DW         .0,  .50, .100, .200, .210, .220, .230, .240, .260, .280, .300, .310, .320, .330, .340, .360
    DW       .380, .400, .410, .420, .430, .440, .460, .480, .500, .505, .510, .515, .520, .525, .530, .540
    DW       .545, .550, .560, .570, .580, .590, .600, .610, .620, .630, .640, .650, .660, .670, .680, .690
    DW       .700, .715, .730, .745, .760, .765, .780, .800, .815, .830, .845, .860, .870, .880, .890, .900
    DW       .915, .930, .945, .960, .975, .990,.1000,.1020,.1040,.1060,.1080,.1100,.1115,.1130,.1150,.1170
    DW      .1185,.1200,.1220,.1240,.1260,.1280,.1300,.1315,.1330,.1350,.1370,.1385,.1400,.1415,.1430,.1450
    DW      .1470,.1485,.1500,.1525,.1550,.1575,.1600,.1625,.1650,.1675,.1700,.1725,.1750,.1775,.1800,.1820
    DW      .1840,.1860,.1880,.1900,.1920,.1940,.1960,.1980,.2000,.2030,.2070,.2100,.2125,.2150,.2175,.2200
    DW      .2220,.2240,.2260,.2280,.2300,.2325,.2350,.2375,.2400,.2420,.2440,.2460,.2480,.2500,.2550,.2600
    DW      .2630,.2670,.2700,.2725,.2750,.2775,.2800,.2820,.2840,.2860,.2880,.2900,.2930,.2970,.3000,.3030
    DW      .3070,.3100,.3125,.3150,.3175,.3200,.3230,.3270,.3300,.3325,.3350,.3375,.3400,.3425,.3450,.3475
    DW      .3500,.3550,.3600,.3630,.3670,.3700,.3725,.3750,.3775,.3800,.3830,.3870,.3900,.3950,.4000,.4030
    DW      .4070,.4100,.4130,.4170,.4200,.4250,.4300,.4350,.4400,.4430,.4470,.4500,.4550,.4600,.4630,.4670
    DW      .4700,.4730,.4770,.4800,.4850,.4900,.5000,.5030,.5070,.5100,.5150,.5200,.5300,.5350,.5400,.5450
    DW      .5500,.5550,.5600,.5650,.5700,.5800,.5850,.5900,.6000,.6050,.6100,.6200,.6250,.6300,.6400,.6500
    DW      .6550,.6600,.6700,.6800,.6900,.7000,.7100,.7200,.7300,.7500,.7700,.7800,.7900,.8200,.8800,.9300

CHAR_04                                         ;dimmer characteristic for incandescent lamp
    DW         .0, .900, .905, .910, .915, .920, .925, .930, .935, .940, .945, .950, .955, .960, .965, .970
    DW       .975, .980, .985, .990, .995,.1000,.1005,.1010,.1015,.1020,.1025,.1030,.1035,.1040,.1045,.1050
    DW      .1055,.1060,.1065,.1070,.1075,.1080,.1085,.1090,.1095,.1100,.1105,.1110,.1115,.1120,.1125,.1130
    DW      .1135,.1140,.1145,.1150,.1155,.1160,.1165,.1170,.1175,.1180,.1185,.1190,.1195,.1200,.1205,.1210
    DW      .1215,.1220,.1225,.1230,.1235,.1240,.1245,.1250,.1255,.1260,.1265,.1270,.1275,.1280,.1285,.1290
    DW      .1295,.1300,.1305,.1310,.1315,.1320,.1325,.1330,.1340,.1350,.1370,.1385,.1400,.1415,.1430,.1450
    DW      .1470,.1485,.1500,.1525,.1550,.1575,.1600,.1625,.1650,.1675,.1700,.1725,.1750,.1775,.1800,.1820
    DW      .1840,.1860,.1880,.1900,.1920,.1940,.1960,.1980,.2000,.2030,.2070,.2100,.2125,.2150,.2175,.2200
    DW      .2220,.2240,.2260,.2280,.2300,.2325,.2350,.2375,.2400,.2420,.2440,.2460,.2480,.2500,.2550,.2600
    DW      .2630,.2670,.2700,.2725,.2750,.2775,.2800,.2820,.2840,.2860,.2880,.2900,.2930,.2970,.3000,.3030
    DW      .3070,.3100,.3125,.3150,.3175,.3200,.3230,.3270,.3300,.3325,.3350,.3375,.3400,.3425,.3450,.3475
    DW      .3500,.3550,.3600,.3630,.3670,.3700,.3725,.3750,.3775,.3800,.3830,.3870,.3900,.3950,.4000,.4030
    DW      .4070,.4100,.4130,.4170,.4200,.4250,.4300,.4350,.4400,.4430,.4470,.4500,.4550,.4600,.4630,.4670
    DW      .4700,.4730,.4770,.4800,.4850,.4900,.5000,.5030,.5070,.5100,.5150,.5200,.5300,.5350,.5400,.5450
    DW      .5500,.5550,.5600,.5650,.5700,.5800,.5850,.5900,.6000,.6050,.6100,.6200,.6250,.6300,.6400,.6500
    DW      .6550,.6600,.6700,.6800,.6900,.7000,.7100,.7200,.7300,.7500,.7700,.7800,.7900,.8200,.8800,.9300

CHAR_05                                         ;dimmer characteristic for LED
    DW         .0, .600, .603, .606, .609, .612, .615, .618, .621, .624, .627, .630, .633, .636, .639, .642
    DW       .645, .648, .651, .654, .657, .660, .663, .666, .669, .672, .675, .678, .681, .684, .687, .690
    DW       .693, .696, .699, .702, .705, .708, .711, .714, .717, .720, .723, .726, .729, .732, .735, .738
    DW       .741, .744, .747, .750, .753, .756, .759, .762, .765, .768, .771, .774, .777, .780, .783, .786
    DW       .789, .792, .795, .798, .801, .804, .807, .810, .813, .816, .819, .822, .825, .828, .831, .834
    DW       .837, .840, .843, .846, .849, .852, .855, .858, .861, .864, .867, .870, .873, .876, .879, .882
    DW       .885, .888, .891, .894, .897, .900, .903, .906, .909, .912, .915, .918, .921, .924, .927, .930
    DW       .933, .936, .939, .942, .945, .948, .951, .954, .957, .960, .963, .966, .969, .972, .975, .978
    DW       .981, .984, .987, .990, .993, .996, .999,.1002,.1005,.1008,.1011,.1014,.1017,.1020,.1023,.1026
    DW      .1029,.1032,.1035,.1038,.1041,.1044,.1047,.1050,.1053,.1056,.1059,.1062,.1065,.1068,.1071,.1074
    DW      .1077,.1080,.1083,.1086,.1089,.1092,.1095,.1098,.1101,.1104,.1107,.1110,.1115,.1130,.1150,.1170
    DW      .1185,.1200,.1220,.1240,.1260,.1280,.1300,.1315,.1330,.1350,.1370,.1385,.1400,.1415,.1430,.1450
    DW      .1470,.1485,.1500,.1525,.1550,.1575,.1600,.1625,.1650,.1675,.1700,.1725,.1750,.1775,.1800,.1820
    DW      .1840,.1860,.1880,.1900,.1920,.1940,.1960,.1980,.2000,.2030,.2070,.2100,.2125,.2150,.2175,.2200
    DW      .2220,.2240,.2260,.2280,.2300,.2325,.2350,.2375,.2400,.2420,.2440,.2460,.2480,.2500,.2550,.2600
    DW      .2630,.2670,.2700,.2725,.2750,.2775,.2800,.2820,.2840,.2860,.2880,.2900,.3100,.3300,.3500,.9300

;==============================================================================
;===  END  OF  PROGRAM  =======================================================
;==============================================================================
    END


