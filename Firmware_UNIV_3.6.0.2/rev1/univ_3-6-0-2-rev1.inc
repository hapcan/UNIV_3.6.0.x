    NOLIST
;==============================================================================
;   MIT License
;
;   Copyright (c) 2022 HAPCAN Home Automation Project (http://hapcan.com)
;
;   Permission is hereby granted, free of charge, to any person obtaining a copy
;   of this software and associated documentation files (the "Software"), to deal
;   in the Software without restriction, including without limitation the rights
;   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;   copies of the Software, and to permit persons to whom the Software is
;   furnished to do so, subject to the following conditions:
;
;   The above copyright notice and this permission notice shall be included in all
;   copies or substantial portions of the Software.
;
;   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;   SOFTWARE.
;==============================================================================

;==============================================================================
;                         DEFAULT CONFIG DATA
;==============================================================================
EEPROM      code    0xF00008                                    ;default config
    DE      0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x04,0x00,0x02
;            |    |         |         |         |         |____ linear curve
;            |    |         |         |         |______________ dimming speed (5s)
;            |    |         |         |________________________ power up dimmer fixed maximum value (255)
;            |    |         |__________________________________ power up dimmer fixed minimum value (0)
;            |    |____________________________________________ power up dimmer fixed state value (0)
;            |_________________________________________________ no memory, on power up takes values from fixed registers

FLASH       code    0x008600                                    ;user defined dimmer characteristic
CHAR_USER   EQU     0x008600        
    DW      0x0810,0x1820,0x2830,0x3840,0x4850,0x5860,0x6870,0x7880
    DW      0x8890,0x98A0,0xA8B0,0xB8C0,0xC8D0,0xD8E0,0xE8F0,0xF8FF

;==============================================================================
;                         FIRMWARE REGISTER DEFINITIONS
;==============================================================================
;definitions
CHANNELSNR          SET 1           ;number of channels

;eeprom config
E_PHASE1_SOURCE     SET CONFIG0     ;defines settings on power up
E_MEM_BIT            SET 0          ;bit<0> - PHASE1_MEM state memory;    values: '1' - when module turn on it goes to last remembered state, '0' - no memory of last state
E_VAL_BIT            SET 1          ;bit<1> - PHASE1_VAL power up source; values: '1' - value from last saved, '0' - from fixed power up value
E_MIN_BIT            SET 2          ;bit<2> - PHASE1_MIN dimmer minimum;  values: '1' - value from last saved, '0' - from fixed power up value
E_MAX_BIT            SET 3          ;bit<3> - PHASE1_MAX dimmer maximum;  values: '1' - value from last saved, '0' - from fixed power up value
E_SPEED_BIT          SET 4          ;bit<4> - PHASE1_SPEED dimmer speed;  values: '1' - value from last saved, '0' - from fixed power up value
E_PHASE1_VAL_FIXED  SET CONFIG1     ;power up fixed value
E_PHASE1_VAL_SAVED  SET CONFIG2     ;last saved dimmer value in eeprom state
E_PHASE1_MIN_FIXED  SET CONFIG3     ;power up fixed minimum value
E_PHASE1_MIN_SAVED  SET CONFIG4     ;last saved minimum in eeprom value
E_PHASE1_MAX_FIXED  SET CONFIG5     ;power up fixed maximum value
E_PHASE1_MAX_SAVED  SET CONFIG6     ;last saved maximum in eeprom value
E_PHASE1_SPEED_FIXED SET CONFIG7    ;power up fixed dimming speed (dimmer running time from value 0% to 100%), value 0-255 (1s-256s)
E_PHASE1_SPEED_SAVED SET CONFIG8    ;last saved dimming speed
E_PHASE1_CURVE      SET CONFIG9     ;dimmer characteristic curve (00-default, 01-user defined, 02-linear, 03-square, 04-incadescent, 05-LED)

;external signals
MOSFET_PORT         SET PORTC       ;transistor gate pin
MOSFET_PIN           SET 2
OVRH_PORT           SET PORTB       ;overheating signaling pin
OVRH_PIN             SET 0

;dimmer registers
R0                  EQU H'0000'     ;general registers
R1                  EQU H'0001'
PHASE_FLAG          EQU H'0002'     ;FLAG register
Ch1LastWasBright     EQU 0          ;last was brigtening if '1' or dimming if '0'
Ch1ChangeSlow        EQU 4          ;change phase slowly if '1' or immediately if '0'
DIMMER_ERROR        EQU H'0003'     ;holds dimmer errors
NoMainsErr           EQU 0          ;no mains
OvrhErr              EQU 1          ;overheating
DIMMER_ERROR_COPY   EQU H'0004'     ;copy of last sent state of ERROR
MAINS_TIMER         EQU H'0005'     ;time after which no power is indicated (in seconds)
LoopCnt             EQU H'0006'     ;loop counter in local procedures

PERIOD_H            EQU H'0010'		;mains period - reg H (in us)
PERIOD_L            EQU H'0011'		;mains period - reg L
PHASE_ON_H          EQU H'0012'     ;time in us when MOSFET is ON - value H
PHASE_ON_L          EQU H'0013'     ;time in us when MOSFET is ON - value L
;PHASE_ON_H_COPY     EQU H'0014'     ;saved value H
;PHASE_ON_L_COPY     EQU H'0015'     ;saved value L


PHASE1_MIN          EQU H'0016'     ;dimer minimum
PHASE1_MAX          EQU H'0017'     ;dimmer maximum
PHASE1              EQU H'0018'     ;dimmer desired value
PHASE1_WORK         EQU H'0019'     ;dimmer working phase value
PHASE1_COPY         EQU H'001A'     ;copy of last sent state of PHASE
PHASE1_STARTCNT     EQU H'001B'     ;START instr counter
PHASE1_SPEED        EQU H'001C'     ;running time between 0 and 255 of dimmer values
PHASE1_MEM          EQU H'001D'     ;memory value register

PHASE1_CNT_U        EQU H'0020'     ;PHASE1 step counter (for smooth dimming)
PHASE1_CNT_H        EQU H'0021'
PHASE1_CNT_L        EQU H'0022'
PHASE_STEP_U        EQU H'0023'     ;calculated PHASE step that must be added on each 20ms
PHASE_STEP_H        EQU H'0024'
PHASE_STEP_L        EQU H'0025'

DIVIDEND_U          EQU H'0034'     ;working regs for DecPhaseGet20msStep
DIVIDEND_H          EQU H'0035'
DIVIDEND_L          EQU H'0036'
DIVISOR_H           EQU H'0037'
DIVISOR_L           EQU H'0038'
REMAINDER_H         EQU H'0039'
REMAINDER_L         EQU H'003A'
BIT_COUNTER         EQU H'003B'
       

;==============================================================================
;                         ROUTINES REGISTERS
;==============================================================================

;------------------------------
;   WORKING INSTRUCTION REGS
;------------------------------
INSTR1      EQU     H'0050'
INSTR2      EQU     H'0051'
INSTR3      EQU     H'0052'
INSTR4      EQU     H'0053'
INSTR5      EQU     H'0054'
INSTR6      EQU     H'0055'
INSTR7      EQU     H'0056'
INSTR8      EQU     H'0057'
TIMER       EQU     H'0058'
SUBTIMER1   EQU     H'0059'
SUBTIMER2   EQU     H'005A'

;------------------------------
;   INTERRUPT SHADOW REGS
;------------------------------
STATUS_HIGH EQU     H'0060'
WREG_HIGH   EQU     H'0061'
BSR_HIGH    EQU     H'0062'
FSR0L_HIGH  EQU     H'0063'
FSR0H_HIGH  EQU     H'0064'
FSR1L_HIGH  EQU     H'0065'
FSR1H_HIGH  EQU     H'0066'
STATUS_LOW  EQU     H'0067'
WREG_LOW    EQU     H'0068'
BSR_LOW     EQU     H'0069'
FSR0L_LOW   EQU     H'006A'
FSR0H_LOW   EQU     H'006B'
FSR1L_LOW   EQU     H'006C'
FSR1H_LOW   EQU     H'006D'
INTCON_SHD  EQU     H'006E'

;------------------------------
;   FIFO REGISTERS
;------------------------------
RXFIFOSIZE  EQU     .42                   ;size of CAN RX FIFO buffer
TXFIFOSIZE  EQU     .42                   ;size of CAN TX FIFO buffer
RXFIFORAM   EQU     0x300                 ;begining of RX FIFO in RAM
TXFIFORAM   EQU     0x500                 ;begining of TX FIFO in RAM
;CAN RX FIFO input buffer
RXFIFOIN0   EQU     H'0070'     
RXFIFOIN1   EQU     H'0071'
RXFIFOIN2   EQU     H'0072'
RXFIFOIN3   EQU     H'0073'
RXFIFOIN4   EQU     H'0074'
RXFIFOIN5   EQU     H'0075'
RXFIFOIN6   EQU     H'0076'
RXFIFOIN7   EQU     H'0077'
RXFIFOIN8   EQU     H'0078'
RXFIFOIN9   EQU     H'0079'
RXFIFOIN10  EQU     H'007A'
RXFIFOIN11  EQU     H'007B'
;CAN RX FIFO output buffer
RXFIFO0     EQU     H'007C'     
RXFIFO1     EQU     H'007D'
RXFIFO2     EQU     H'007E'
RXFIFO3     EQU     H'007F'
RXFIFO4     EQU     H'0080'
RXFIFO5     EQU     H'0081'
RXFIFO6     EQU     H'0082'
RXFIFO7     EQU     H'0083'
RXFIFO8     EQU     H'0084'
RXFIFO9     EQU     H'0085'
RXFIFO10    EQU     H'0086'
RXFIFO11    EQU     H'0087'
;CAN TX FIFO input buffer
TXFIFOIN0   EQU     H'0088'     
TXFIFOIN1   EQU     H'0089'
TXFIFOIN2   EQU     H'008A'
TXFIFOIN3   EQU     H'008B'
TXFIFOIN4   EQU     H'008C'
TXFIFOIN5   EQU     H'008D'
TXFIFOIN6   EQU     H'008E'
TXFIFOIN7   EQU     H'008F'
TXFIFOIN8   EQU     H'0090'
TXFIFOIN9   EQU     H'0091'
TXFIFOIN10  EQU     H'0092'
TXFIFOIN11  EQU     H'0093'
;CAN TX FIFO output buffer
TXFIFO0     EQU     H'0094'     
TXFIFO1     EQU     H'0095'
TXFIFO2     EQU     H'0096'
TXFIFO3     EQU     H'0097'
TXFIFO4     EQU     H'0098'
TXFIFO5     EQU     H'0099'
TXFIFO6     EQU     H'009A'
TXFIFO7     EQU     H'009B'
TXFIFO8     EQU     H'009C'
TXFIFO9     EQU     H'009D'
TXFIFO10    EQU     H'009E'
TXFIFO11    EQU     H'009F'
;CAN FIFO RX registers
RXTOP       EQU     H'00A0'     ;FIFO RX stack top
RXCNT       EQU     H'00A1'     ;FIFO RX current stack depth
RXCNTMX     EQU     H'00A2'     ;FIFO RX maximum reached stack depth
RXFIFOR0    EQU     H'00A3'     ;working register
RXFIFOR1    EQU     H'00A4'     ;working register
;CAN FIFO TX registers
TXTOP       EQU     H'00A5'     ;FIFO RX stack top
TXCNT       EQU     H'00A6'     ;FIFO RX current stack depth
TXCNTMX     EQU     H'00A7'     ;FIFO RX maximum reached stack depth
TXFIFOR0    EQU     H'00A8'     ;working register
TXFIFOR1    EQU     H'00A9'     ;working register

;------------------------------
;   HEALTH CHECK REGISTERS
;------------------------------
CANINTCNT   EQU     H'00AA'     ;counts can reinitialization events
TRSMTTIMER  EQU     H'00AB'     ;counts seconds when transmit buffers are busy

;------------------------------
;   UPTIME REGISTERS
;------------------------------
UPTIME0     EQU     H'00AC'
UPTIME1     EQU     H'00AD'
UPTIME2     EQU     H'00AE'
UPTIME3     EQU     H'00AF'

;------------------------------
;   WORKING REGISTERS
;------------------------------
ROUT0       EQU     H'00B0'     ;CanTransmit routine
ROUT1       EQU     H'00B1'     ;ProcessSystemMessage routine (HealthAskClear)
ROUT2       EQU     H'00B2'     ;CheckAgainstBoxes routine
ROUT3       EQU     H'00B3'     ;CheckAgainstBoxes routine
ROUT4       EQU     H'00B4'     ;ProcessSystemMessage routine (ChannelName)
ROUT5       EQU     H'00B5'     ;ProcessSystemMessage routine (ChannelName)
;Flag register
TIMER0_1000ms EQU   H'00BD'     ;Timer 0 interrupt occurred flag
TIMER2_20ms EQU     H'00BE'     ;Timer 2 interrupt occurred flag
EEPROMTIMER EQU     H'00BF'     ;Save node states to EEPROM timer (6s)


;------------------------------
;   BOX REGISTERS
;------------------------------
BOXADR      EQU     H'00C0'
BOXEN       EQU     H'00C1'
SAVEENBITS  EQU     H'00C2'     ;Box enable bits need to be saved into EEPROM flag

BOXMASK1    EQU     H'00D0'     ;enable/disable/toggle box mask
BOXMASK2    EQU     H'00D1'
BOXMASK3    EQU     H'00D2'
BOXMASK4    EQU     H'00D3'
BOXMASK5    EQU     H'00D4'
BOXMASK6    EQU     H'00D5'
BOXMASK7    EQU     H'00D6'
BOXMASK8    EQU     H'00D7'
BOXMASK9    EQU     H'00D8'
BOXMASK10   EQU     H'00D9'
BOXMASK11   EQU     H'00DA'
BOXMASK12   EQU     H'00DB'
BOXMASK13   EQU     H'00DC'
BOXMASK14   EQU     H'00DD'
BOXMASK15   EQU     H'00DE'
BOXMASK16   EQU     H'00DF'

BOXFIL1     EQU     H'00E0'     ;message filter
BOXFIL2     EQU     H'00E1'
BOXFIL3     EQU     H'00E2'
BOXFIL4     EQU     H'00E3'
BOXFIL5     EQU     H'00E4'
BOXFIL6     EQU     H'00E5'
BOXFIL7     EQU     H'00E6'
BOXFIL8     EQU     H'00E7'
BOXFIL9     EQU     H'00E8'
BOXFIL10    EQU     H'00E9'
BOXFIL11    EQU     H'00EA'
BOXFIL12    EQU     H'00EB'
BOXFIL1C    EQU     H'00EC'     ;message filter condition
BOXFIL2C    EQU     H'00ED'
BOXFIL3C    EQU     H'00EE'
BOXFIL4C    EQU     H'00EF'
BOXFIL5C    EQU     H'00F0'
BOXFIL6C    EQU     H'00F1'
BOXFIL7C    EQU     H'00F2'
BOXFIL8C    EQU     H'00F3'
BOXFIL9C    EQU     H'00F4'
BOXFIL10C   EQU     H'00F5'
BOXFIL11C   EQU     H'00F6'
BOXFIL12C   EQU     H'00F7'
BOXINSTR1   EQU     H'00F8'     ;instruction in box
BOXINSTR2   EQU     H'00F9'
BOXINSTR3   EQU     H'00FA'
BOXINSTR4   EQU     H'00FB'
BOXINSTR5   EQU     H'00FC'
BOXINSTR6   EQU     H'00FD'
BOXINSTR7   EQU     H'00FE'
BOXINSTR8   EQU     H'00FF'

;------------------------------
;   CHANNEL REGISTERS
;------------------------------
Instr1Ch1   EQU     H'0700'
Instr2Ch1   EQU     H'0701'
Instr3Ch1   EQU     H'0702'
Instr4Ch1   EQU     H'0703'
Instr5Ch1   EQU     H'0704'
Instr6Ch1   EQU     H'0705'
Instr7Ch1   EQU     H'0706'
Instr8Ch1   EQU     H'0707'
TimerCh1    EQU     H'0708'
SubTmr1Ch1  EQU     H'0709'
SubTmr2Ch1  EQU     H'070A'
Instr1Ch2   EQU     H'070B'
Instr2Ch2   EQU     H'070C'
Instr3Ch2   EQU     H'070D'
Instr4Ch2   EQU     H'070E'
Instr5Ch2   EQU     H'070F'
Instr6Ch2   EQU     H'0710'
Instr7Ch2   EQU     H'0711'
Instr8Ch2   EQU     H'0712'
TimerCh2    EQU     H'0713'
SubTmr1Ch2  EQU     H'0714'
SubTmr2Ch2  EQU     H'0715'
Instr1Ch3   EQU     H'0716'
Instr2Ch3   EQU     H'0717'
Instr3Ch3   EQU     H'0718'
Instr4Ch3   EQU     H'0719'
Instr5Ch3   EQU     H'071A'
Instr6Ch3   EQU     H'071B'
Instr7Ch3   EQU     H'071C'
Instr8Ch3   EQU     H'071D'
TimerCh3    EQU     H'071E'
SubTmr1Ch3  EQU     H'071F'
SubTmr2Ch3  EQU     H'0720'
Instr1Ch4   EQU     H'0721'
Instr2Ch4   EQU     H'0722'
Instr3Ch4   EQU     H'0723'
Instr4Ch4   EQU     H'0724'
Instr5Ch4   EQU     H'0725'
Instr6Ch4   EQU     H'0726'
Instr7Ch4   EQU     H'0727'
Instr8Ch4   EQU     H'0728'
TimerCh4    EQU     H'0729'
SubTmr1Ch4  EQU     H'072A'
SubTmr2Ch4  EQU     H'072B'
Instr1Ch5   EQU     H'072C'
Instr2Ch5   EQU     H'072D'
Instr3Ch5   EQU     H'072E'
Instr4Ch5   EQU     H'072F'
Instr5Ch5   EQU     H'0730'
Instr6Ch5   EQU     H'0731'
Instr7Ch5   EQU     H'0732'
Instr8Ch5   EQU     H'0733'
TimerCh5    EQU     H'0734'
SubTmr1Ch5  EQU     H'0735'
SubTmr2Ch5  EQU     H'0736'
Instr1Ch6   EQU     H'0737'
Instr2Ch6   EQU     H'0738'
Instr3Ch6   EQU     H'0739'
Instr4Ch6   EQU     H'073A'
Instr5Ch6   EQU     H'073B'
Instr6Ch6   EQU     H'073C'
Instr7Ch6   EQU     H'073D'
Instr8Ch6   EQU     H'073E'
TimerCh6    EQU     H'073F'
SubTmr1Ch6  EQU     H'0740'
SubTmr2Ch6  EQU     H'0741'
Instr1Ch7   EQU     H'0742'
Instr2Ch7   EQU     H'0743'
Instr3Ch7   EQU     H'0744'
Instr4Ch7   EQU     H'0745'
Instr5Ch7   EQU     H'0746'
Instr6Ch7   EQU     H'0747'
Instr7Ch7   EQU     H'0748'
Instr8Ch7   EQU     H'0749'
TimerCh7    EQU     H'074A'
SubTmr1Ch7  EQU     H'074B'
SubTmr2Ch7  EQU     H'074C'
Instr1Ch8   EQU     H'074D'
Instr2Ch8   EQU     H'074E'
Instr3Ch8   EQU     H'074F'
Instr4Ch8   EQU     H'0750'
Instr5Ch8   EQU     H'0751'
Instr6Ch8   EQU     H'0752'
Instr7Ch8   EQU     H'0753'
Instr8Ch8   EQU     H'0754'
TimerCh8    EQU     H'0755'
SubTmr1Ch8  EQU     H'0756'
SubTmr2Ch8  EQU     H'0757'
Instr1Ch9   EQU     H'0758'
Instr2Ch9   EQU     H'0759'
Instr3Ch9   EQU     H'075A'
Instr4Ch9   EQU     H'075B'
Instr5Ch9   EQU     H'075C'
Instr6Ch9   EQU     H'075D'
Instr7Ch9   EQU     H'075E'
Instr8Ch9   EQU     H'075F'
TimerCh9    EQU     H'0760'
SubTmr1Ch9  EQU     H'0761'
SubTmr2Ch9  EQU     H'0762'
Instr1Ch10  EQU     H'0763'
Instr2Ch10  EQU     H'0764'
Instr3Ch10  EQU     H'0765'
Instr4Ch10  EQU     H'0766'
Instr5Ch10  EQU     H'0767'
Instr6Ch10  EQU     H'0768'
Instr7Ch10  EQU     H'0769'
Instr8Ch10  EQU     H'076A'
TimerCh10   EQU     H'076B'
SubTmr1Ch10 EQU     H'076C'
SubTmr2Ch10 EQU     H'076D'

;==============================================================================
;                             COPPIED EEPROM
;==============================================================================
;eeprom last used address
EEPROM_U    EQU     H'0202'
EEPROM_H    EQU     H'0203'
EEPROM_L    EQU     H'0204'
;flash last used address
FLASH_U     EQU     H'0205'
FLASH_H     EQU     H'0206'
FLASH_L     EQU     H'0207'
;eeprom module config
CONFIG0     EQU     H'0208'
CONFIG1     EQU     H'0209'
CONFIG2     EQU     H'020A'
CONFIG3     EQU     H'020B'
CONFIG4     EQU     H'020C'
CONFIG5     EQU     H'020D'
CONFIG6     EQU     H'020E'
CONFIG7     EQU     H'020F'
CONFIG8     EQU     H'0210'
CONFIG9     EQU     H'0211'
CONFIG10    EQU     H'0212'
CONFIG11    EQU     H'0213'
CONFIG12    EQU     H'0214'
CONFIG13    EQU     H'0215'
CONFIG14    EQU     H'0216'
CONFIG15    EQU     H'0217'
CONFIG16    EQU     H'0218'
CONFIG17    EQU     H'0219'
CONFIG18    EQU     H'021A'
CONFIG19    EQU     H'021B'
CONFIG20    EQU     H'021C'
CONFIG21    EQU     H'021D'
CONFIG22    EQU     H'021E'
CONFIG23    EQU     H'021F'
;module id
NODENR      EQU     H'0226'     ;node number
GROUPNR     EQU     H'0227'     ;node group number
;health check regs
;res        EQU     H'0228'
;res        EQU     H'0229'
;res        EQU     H'022A'
RXCNTMXEE   EQU     H'022B'     ;maximum achieved CAN RX FIFO depth
TXCNTMXEE   EQU     H'022C'     ;maximum achieved CAN TX FIFO depth
CANINTCNTEE EQU     H'022D'     ;can engine reinitialization number
RXERRCNTEE  EQU     H'022E'     ;CAN receive errors
TXERRCNTEE  EQU     H'022F'     ;CAN transmit errors
;enable box registers
ENBOX08     EQU     H'0240'     ;boxes 1-8
ENBOX16     EQU     H'0241'
ENBOX24     EQU     H'0242'
ENBOX32     EQU     H'0243'
ENBOX40     EQU     H'0244'
ENBOX48     EQU     H'0245'
ENBOX56     EQU     H'0246'
ENBOX64     EQU     H'0247'
ENBOX72     EQU     H'0248'
ENBOX80     EQU     H'0249'
ENBOX88     EQU     H'024A'
ENBOX96     EQU     H'024B'
ENBOX104    EQU     H'024C'
ENBOX112    EQU     H'024D'
ENBOX120    EQU     H'024E'
ENBOX128    EQU     H'024F'     ;boxes 121-128


;==============================================================================
;                   BOOTLOADER REGISTERS - DO NOT CHANGE
;==============================================================================
;CAN receive buffer
RxBCON      EQU     H'0100'
CANFRAME1   EQU     H'0101'
CANFRAME2   EQU     H'0102'
CANNODE     EQU     H'0103'
CANGROUP    EQU     H'0104'
CANDLC      EQU     H'0105'
CAND0       EQU     H'0106'
CAND1       EQU     H'0107'
CAND2       EQU     H'0108'
CAND3       EQU     H'0109'
CAND4       EQU     H'010A'
CAND5       EQU     H'010B'
CAND6       EQU     H'010C'
CAND7       EQU     H'010D'
CANFULL     EQU     H'010E'     ;CAN buffer full flag
FIRMREADY   EQU     H'010F'     ;firmware started completely flag
;------------------------------
;UART receive buffer
UART0       EQU     H'0110'
UART1       EQU     H'0111'
UART2       EQU     H'0112'
UART3       EQU     H'0113'
UART4       EQU     H'0114'
UART5       EQU     H'0115'
UART6       EQU     H'0116'
UART7       EQU     H'0117'
UART8       EQU     H'0118'
UART9       EQU     H'0119'
UART10      EQU     H'011A'
UART11      EQU     H'011B'
UART12      EQU     H'011C'
UART13      EQU     H'011D'
UART14      EQU     H'011E'
UART15      EQU     H'011F'
UART16      EQU     H'0120'
UART17      EQU     H'0121'
UART18      EQU     H'0122'
UART19      EQU     H'0123'
UART20      EQU     H'0124'
UART21      EQU     H'0125'
UART22      EQU     H'0126'
UART23      EQU     H'0127'
UART24      EQU     H'0128'
UART25      EQU     H'0129'
UART26      EQU     H'012A'
UART27      EQU     H'012B'
UART28      EQU     H'012C'
UART29      EQU     H'012D'
UARTOVF     EQU     H'012E'     ;overflow indicator
UARTCNT     EQU     H'012F'     ;uart buffer counter
;------------------------------
;bootloader other regs
UARTON      EQU     H'016F'     ;indicates that UART bootloader is on when 0xFF
;------------------------------
;bootloader shadow registers for high interrupt
STATUS_H    EQU     H'0170'
WREG_H      EQU     H'0171'
BSR_H       EQU     H'0172'
PIR1_H      EQU     H'0173'
PIR4_H      EQU     H'0174'
PIR5_H      EQU     H'0175'
CANSTAT_H   EQU     H'0176'
CANCON_H    EQU     H'0177'
FSR0L_H     EQU     H'0178'
FSR0H_H     EQU     H'0179'
FSR1L_H     EQU     H'017A'
FSR1H_H     EQU     H'017B'
TABLAT_H    EQU     H'017C'
TBLPTRL_H   EQU     H'017D'
TBLPTRH_H   EQU     H'017E'
TBLPTRU_H   EQU     H'017F'
INTCON_H    EQU     H'0180'
EECON1_H    EQU     H'0181'
EEDATA_H    EQU     H'0182'
EEADRH_H    EQU     H'0183'
EEADR_H     EQU     H'0184'
TRISA_H     EQU     H'0185'
ADCON2_H    EQU     H'0186'
ADCON1_H    EQU     H'0187'
ADCON0_H    EQU     H'0188'
ANCON0_H    EQU     H'0189'
ADRESL_H    EQU     H'018A'
ADRESH_H    EQU     H'018B'
RCSTA1_H    EQU     H'018C'
TXSTA1_H    EQU     H'018D'
PMD1_H      EQU     H'018E'
COMSTAT_H   EQU     H'018F'
;------------------------------
;bootloader shadow registers for low interrupt
STATUS_L    EQU     H'0190'
WREG_L      EQU     H'0191'
BSR_L       EQU     H'0192'
PIR1_L      EQU     H'0193'
PIR4_L      EQU     H'0194'
PIR5_L      EQU     H'0195'
CANSTAT_L   EQU     H'0196'
CANCON_L    EQU     H'0197'
FSR0L_L     EQU     H'0198'
FSR0H_L     EQU     H'0199'
FSR1L_L     EQU     H'019A'
FSR1H_L     EQU     H'019B'
TABLAT_L    EQU     H'019C'
TBLPTRL_L   EQU     H'019D'
TBLPTRH_L   EQU     H'019E'
TBLPTRU_L   EQU     H'019F'
INTCON_L    EQU     H'01A0'
EECON1_L    EQU     H'01A1'
EEDATA_L    EQU     H'01A2'
EEADRH_L    EQU     H'01A3'
EEADR_L     EQU     H'01A4'
TRISA_L     EQU     H'01A5'
ADCON2_L    EQU     H'01A6'
ADCON1_L    EQU     H'01A7'
ADCON0_L    EQU     H'01A8'
ANCON0_L    EQU     H'01A9'
ADRESL_L    EQU     H'01AA'
ADRESH_L    EQU     H'01AB'
RCSTA1_L    EQU     H'01AC'
TXSTA1_L    EQU     H'01AD'
PMD1_L      EQU     H'01AE'
COMSTAT_L   EQU     H'01AF'
;==============================================================================
;==============================================================================
;==============================================================================
    LIST