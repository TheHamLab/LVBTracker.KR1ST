
; CC5X Version 3.8, Copyright (c) B Knudsen Data
; C compiler for the PICmicro family
; ************   1. Feb 2023  22:22  *************

        processor  16F876A
        radix  DEC

        __config 0x3F71

INDF        EQU   0x00
TMR0        EQU   0x01
STATUS      EQU   0x03
FSR         EQU   0x04
PORTA       EQU   0x05
TRISA       EQU   0x85
PORTB       EQU   0x06
TRISB       EQU   0x86
PCLATH      EQU   0x0A
PA0         EQU   3
PA1         EQU   4
PS0         EQU   0
PS1         EQU   1
PS2         EQU   2
PSA         EQU   3
T0CS        EQU   5
Carry       EQU   0
Zero_       EQU   2
RP0         EQU   5
RP1         EQU   6
IRP         EQU   7
T0IF        EQU   2
T0IE        EQU   5
GIE         EQU   7
PORTC       EQU   0x07
TXREG       EQU   0x19
RCREG       EQU   0x1A
ADRESH      EQU   0x1E
TRISC       EQU   0x87
SPBRG       EQU   0x99
ADRESL      EQU   0x9E
EEDATA      EQU   0x10C
EEADR       EQU   0x10D
EEDATH      EQU   0x10E
EEADRH      EQU   0x10F
EECON1      EQU   0x18C
EECON2      EQU   0x18D
PEIE        EQU   6
TXIF        EQU   4
RCIF        EQU   5
OERR        EQU   1
CREN        EQU   4
RX9         EQU   6
SPEN        EQU   7
ADON        EQU   0
GO          EQU   2
CHS0        EQU   3
CHS1        EQU   4
CHS2        EQU   5
ADCS0       EQU   6
ADCS1       EQU   7
TXIE        EQU   4
RCIE        EQU   5
BRGH        EQU   2
SYNC        EQU   4
TXEN        EQU   5
TX9         EQU   6
PCFG0       EQU   0
PCFG1       EQU   1
PCFG2       EQU   2
PCFG3       EQU   3
ADFM        EQU   7
RD          EQU   0
WR          EQU   1
WREN        EQU   2
EEPGD       EQU   7
_u8Buttons  EQU   0x4B
_u8ButtonsLast EQU   0x4C
_cRS232RxIn EQU   0xA0
_cRS232RxOut EQU   0xA1
_cRS232TxIn EQU   0xB6
_cRS232TxOut EQU   0xB7
_u8LinePos  EQU   0x4D
_bAzTrack   EQU   0
_bElTrack   EQU   1
_s16AzTarget EQU   0x4F
_s16ElTarget EQU   0x51
_fAzMul     EQU   0x53
_u16AzOff   EQU   0x56
_fElMul     EQU   0x58
_u16ElOff   EQU   0x5B
_u16Flags   EQU   0x5D
_u16AzOffset EQU   0x5F
_fAzMult    EQU   0x71
_u16Timer   EQU   0x69
_bLCDActive EQU   2
_u16LCDTimer EQU   0x6B
svrWREG     EQU   0x70
svrSTATUS   EQU   0x20
svrPCLATH   EQU   0x21
sv_FSR      EQU   0x22
c           EQU   0x23
FpOverflow  EQU   1
FpUnderFlow EQU   2
FpDiv0      EQU   3
FpRounding  EQU   6
arg1f24     EQU   0x41
arg2f24     EQU   0x44
aarg        EQU   0x47
sign        EQU   0x49
counter     EQU   0x4A
aarg_2      EQU   0x47
sign_2      EQU   0x49
counter_2   EQU   0x4A
xtra        EQU   0x47
temp        EQU   0x48
expo        EQU   0x49
sign_3      EQU   0x4A
expo_2      EQU   0x47
xtra_2      EQU   0x48
sign_4      EQU   0x49
rval        EQU   0x41
sign_6      EQU   0x47
expo_4      EQU   0x48
xtra_4      EQU   0x49
rval_3      EQU   0x41
c_2         EQU   0x2F
uc          EQU   0x39
uc2         EQU   0x3A
u16         EQU   0x3D
pu8Pos      EQU   0x31
bFinished   EQU   0
bWhiteFound EQU   1
u8Pos       EQU   0x32
c_3         EQU   0x33
pu8Pos_2    EQU   0x3C
pu16        EQU   0x3D
bFinished_2 EQU   0
bNumFound   EQU   1
u8Pos_2     EQU   0x3E
c_4         EQU   0x3F
u16_2       EQU   0x40
C1cnt       EQU   0x43
C2tmp       EQU   0x44
pu8Pos_3    EQU   0x31
pf          EQU   0x32
u16Int      EQU   0x33
u16Frac     EQU   0x35
f           EQU   0x37
u8          EQU   0x3A
u8Pos_3     EQU   0x3B
fFrac       EQU   0x3C
u8Channel   EQU   0x33
n           EQU   0x34
u16Sum      EQU   0x35
u16Result   EQU   0x37
C3cnt       EQU   0x37
C4tmp       EQU   0x38
C5rem       EQU   0x3A
c_5         EQU   0x27
c_6         EQU   0x24
cNextIdx    EQU   0x25
c_7         EQU   0x23
c_8         EQU   0x3A
cNextIdx_2  EQU   0x3B
u16_3       EQU   0x34
u16a        EQU   0x36
u16b        EQU   0x38
C6cnt       EQU   0x3A
C7tmp       EQU   0x3B
C8rem       EQU   0x3D
C9cnt       EQU   0x3A
C10tmp      EQU   0x3B
C11cnt      EQU   0x3A
C12tmp      EQU   0x3B
C13rem      EQU   0x3D
C14cnt      EQU   0x3A
C15tmp      EQU   0x3B
C16cnt      EQU   0x3A
C17tmp      EQU   0x3B
C18rem      EQU   0x3D
C19cnt      EQU   0x3A
s16         EQU   0x2D
c_9         EQU   0x2F
f_2         EQU   0x2E
f2          EQU   0x31
psz         EQU   0x2E
b           EQU   0
u16ADC      EQU   0x33
u8Index     EQU   0x35
u16Index    EQU   0x36
u16Offset   EQU   0x38
fMult       EQU   0x3A
u16DEG      EQU   0x3D
u16DEGOffset EQU   0x3F
C21cnt      EQU   0x41
C22tmp      EQU   0x42
uc_2        EQU   0x39
uc_3        EQU   0x3B
uc2_2       EQU   0x3C
u16_4       EQU   0x39
u16Diff     EQU   0x3B
uc_4        EQU   0x38
uc_5        EQU   0x31
ucPos       EQU   0x30
c_10        EQU   0x37
u16_5       EQU   0x30
u8NumDigs   EQU   0x32
u16a_2      EQU   0x33
u16b_2      EQU   0x35
C23cnt      EQU   0x37
C24tmp      EQU   0x38
C25rem      EQU   0x3A
C26cnt      EQU   0x37
C27tmp      EQU   0x38
C28cnt      EQU   0x37
C29tmp      EQU   0x38
C30rem      EQU   0x3A
C31cnt      EQU   0x37
C32tmp      EQU   0x38
C33cnt      EQU   0x37
C34tmp      EQU   0x38
C35rem      EQU   0x3A
C36cnt      EQU   0x37
C37tmp      EQU   0x38
C38cnt      EQU   0x37
C39tmp      EQU   0x38
C40rem      EQU   0x3A
C41cnt      EQU   0x37
s16_2       EQU   0x2C
u8NumDigs_2 EQU   0x2E
c_11        EQU   0x2F
psz_2       EQU   0x30
s16_3       EQU   0x2A
u16_6       EQU   0x26
u16Timer    EQU   0x26
s16_4       EQU   0x28
u8Addr      EQU   0x38
u8Addr_2    EQU   0x35
u16_7       EQU   0x36
u8Addr_3    EQU   0x26
fu          EQU   0x27
C43tmp      EQU   0x2A
u8Addr_4    EQU   0x35
u8Data      EQU   0x36
u8Addr_5    EQU   0x32
u16Data     EQU   0x33
u8Addr_6    EQU   0x30
f_3         EQU   0x31
pfu         EQU   0x34
u8_2        EQU   0x35
u16Sum_2    EQU   0x36
u16_8       EQU   0x30
u16SumRead  EQU   0x30
u16SumCalc  EQU   0x32
u16_9       EQU   0x26
u8_3        EQU   0x3C
u8Org       EQU   0x34
u16Org      EQU   0x35
u8New       EQU   0x37
u16New      EQU   0x38
u16Diff_2   EQU   0x3A
u8Mask      EQU   0x32
u8_4        EQU   0x33
s16Az       EQU   0x26
s16El       EQU   0x26
u8Mask_2    EQU   0x30
u8Button    EQU   0x31
u8Button_2  EQU   0x26
u16AzMin    EQU   0x27
u16AzMax    EQU   0x29
u16ElMin    EQU   0x2B
u16ElMax    EQU   0x2D
bAz450      EQU   1
bEl90       EQU   2
bSNS        EQU   3
pu8Pos_4    EQU   0x2A
c1          EQU   0x2B
c2          EQU   0x2C
u8Pos_4     EQU   0x2D
u8Command   EQU   0x2E
s16Az_2     EQU   0x2F
s16El_2     EQU   0x31
c_12        EQU   0x26
u8Pos_5     EQU   0x27
u8EasyCommCommand EQU   0x28
bError      EQU   0
f_4         EQU   0x2A
s16_5       EQU   0x2D
u8_5        EQU   0x30
c_13        EQU   0x31
cAzEl       EQU   0x29
cEndStart   EQU   0x2A
u16_10      EQU   0x2B
f_6         EQU   0x2B
u16_11      EQU   0x2B
u16_12      EQU   0x2B
u16_13      EQU   0x2B
f_7         EQU   0x2B
u16_14      EQU   0x2B
u16_15      EQU   0x2B
s16_6       EQU   0x29
s16_7       EQU   0x2B
s16Az_3     EQU   0x29
s16El_3     EQU   0x2B
ci          EQU   0x32

        GOTO main

  ; FILE C:/Data/MPLABXProjects/LVBTracker.X/LVBTrack.c
                        ;//From PicProg.c
                        ;// Osc=XT, WDT off, PUT on, BOD off, LVP off, FPW on, DBG off, data EE prot off, CodeProt off
                        ;//au16Data[0x2007]=0x3F31;
                        ;//
                        ;//#pragma chip PIC16F876
                        ;//#pragma config WDTE = OFF
                        ;//#pragma config FOSC = XT
                        ;//#pragma config BOREN = OFF
                        ;//#pragma config PWRTE = ON
                        ;//
                        ;//AK 02/02/2023 Changed config bits to turn on BOREN
                        ;//#pragma config = 0x3F31
                        ;#pragma config = 0x3F71
                        ;//
                        ;// Todo:
                        ;// * Make work without LCD - OK 0.2
                        ;// * Test -180 - 0 - +180 rotator. OK 0.2
                        ;// * Test 0 - 450 rotator ("FAF" command) OK 0.2 but no with S-N-S rotator
                        ;// * Add Easycomm interface - fixed 0.7
                        ;// * Add more GS-232 commands - 'M###' added 0.7
                        ;// * Fix 16F876A and combined right/up button push problem. OK 0.2
                        ;// * Allow calibration from buttons - switch on with one of the front panel buttons pressed OK 0.2
                        ;// * Random Az/El target displayed on boot up OK 0.2
                        ;// * Random Az/El target displayed after calibration OK 0.3
                        ;// * Floating point OK 0.3
                        ;// * Include speed control
                        ;// * Fix overshoot
                        ;// * Fix SatPC32 problem of repeatedly rewriting - fixed 0.5
                        ;// * Bug when Elevation not moving in certain scenarios - fixed 0.6
                        ;// * Reduce LCD flicker by only updating occasionally - fixed 0.6
                        ;// * Smooth ADC readings by taking an averaged number - fixed 0.6
                        ;// * Change defualt ADC constants for my rotator ;-) - fixed 0.6
                        ;// * Add 'M' GS-232 command to set Az only - fixed 0.7
                        ;// * Multi-platform build for 16F876 and 18F2620 - 0.8
                        ;// * Predict compatibility - ignore nuls on RS232 - 0.8
                        ;// * Adjust BRG slightly to cope with strange Nova behaviour - 0.9
                        ;//
                        ;// Modifications by Alex KR1ST. March 2021
                        ;// Version 1.2
                        ;// Added calibration points for 0, 90, 180, 270, and 360 degrees to support
                        ;// rotators with a non-linear azimuth feedback voltage.
                        ;
                        ;
                        ;#ifdef __CC5X__
                        ;#include "int16CXX.h"
                        ;#endif
                        ;
                        ;#ifdef __CC8E__
                        ;
                        ;bit PCFG0 @ ADCON1.0;
                        ;bit PCFG1 @ ADCON1.1;
                        ;bit PCFG2 @ ADCON1.2;
                        ;bit PCFG3 @ ADCON1.3;
                        ;
                        ;bit T0PS0 @ T0CON.0;
                        ;bit T0PS1 @ T0CON.1;
                        ;bit T0PS2 @ T0CON.2;
                        ;
                        ;bit ACQT0 @ ADCON2.3;
                        ;bit ACQT1 @ ADCON2.4;
                        ;bit ACQT2 @ ADCON2.5;
                        ;
                        ;#include "int18XXX.h"
                        ;
                        ;#define T0IF TMR0IF
                        ;#define T0IE TMR0IE
                        ;#define TMR0 TMR0L
                        ;
                        ;#define PS0 T0PS0
                        ;#define PS1 T0PS1
                        ;#define PS2 T0PS2
                        ;
                        ;#endif
                        ;
                        ;
                        ;#define FALSE 0
                        ;#define TRUE 1
                        ;
                        ;#define DELAY_CONSTANT 13  /* Software delay constant: 16 for 4.9152MHz, 13 for 4MHz */
                        ;
                        ;#define LCD_RS PORTB.0
                        ;#define LCD_RW PORTB.1
                        ;#define LCD_E PORTB.2
                        ;#define LCD_TIMERWAIT 200 // Time to wait in ms to update LCD to stop flickering
                        ;
                        ;#define ROT_LEFT PORTC.0
                        ;#define ROT_RIGHT PORTC.1
                        ;#define ROT_DOWN PORTC.2
                        ;#define ROT_UP PORTC.3
                        ;
                        ;#define BTN_LEFT 1
                        ;#define BTN_RIGHT 2
                        ;#define BTN_DOWN 4
                        ;#define BTN_UP 8
                        ;
                        ;#define FLAG_AZ450 (_u16Flags.0) // 450 degree azimuth flag
                        ;#define FLAG_SNS (_u16Flags.1) // -180 - 0 - +180 flag
                        ;
                        ;#define EC_NONE 0 // EasyComm command decode numbers
                        ;#define EC_AZ 1
                        ;#define EC_EL 2
                        ;#define EC_UP 3
                        ;#define EC_DN 4
                        ;
                        ;//#define RS232_BRG 26 /* Baud rate generator BR=FOSC/(16(SPBRG+1)) 9600 bps 26 for 4MHz, 31 for 4.9152MHz */
                        ;// BRG adjustment for 0.9
                        ;#define RS232_BRG 25 /* Baud rate generator BR=FOSC/(16(SPBRG+1)) 9600 bps 26 for 4MHz, 31 for 4.9152MHz */
                        ;#define RS232_RXBUFFERSIZE 60
                        ;#define RS232_TXBUFFERSIZE 20
                        ;#define RS232_LINESIZE 60
                        ;
                        ;#define ADC_NUMSAMPLES 25 // number of ADC samples to take to try to smooth things
                        ;
                        ;#define EE_AZMUL 0
                        ;#define EE_AZOFF 3
                        ;#define EE_ELMUL 5
                        ;#define EE_ELOFF 8
                        ;#define EE_FLAGS 10
                        ;//AK
                        ;//#define EE_MAX 11 /* Total number of EE bytes used -1 for checksum purposes */
                        ;#define EE_AZOFF_0   11
                        ;#define EE_AZOFF_90  13
                        ;#define EE_AZOFF_180 15
                        ;#define EE_AZOFF_270 17
                        ;#define EE_AZOFF_360 19
                        ;#define EE_AZMUL_0   21
                        ;#define EE_AZMUL_90  24
                        ;#define EE_AZMUL_180 27
                        ;#define EE_AZMUL_270 30
                        ;#define EE_AZMUL_360 33
                        ;#define EE_MAX       36 /* Total number of EE bytes used -1 for checksum purposes */
                        ;
                        ;typedef uns32 U32;
                        ;typedef uns16 U16;
                        ;typedef int16 S16;
                        ;typedef uns8 U8;
                        ;typedef bit BOOL;
                        ;
                        ;typedef union
                        ;{
                        ;  float f;
                        ;  struct
                        ;  {
                        ;    U8 u8Lo;
                        ;    U8 u8Mid;
                        ;    U8 u8Hi;
                        ;  } fs;
                        ;} FLOATUNION;
                        ;
                        ;static U8 _u8Buttons;
                        ;static U8 _u8ButtonsLast;
                        ;
                        ;// RS232 buffers etc
                        ;#ifdef __CC5X__
                        ;#pragma rambank 2
                        ;#endif
                        ;static char _acRS232RxData[RS232_RXBUFFERSIZE]; /* Rx Buffer */
                        ;#ifdef __CC5X__
                        ;#pragma rambank 1
                        ;#endif
                        ;static char _cRS232RxIn;
                        ;static char _cRS232RxOut;
                        ;static char _acRS232TxData[RS232_TXBUFFERSIZE]; /* Tx Buffer */
                        ;static char _cRS232TxIn;
                        ;static char _cRS232TxOut;
                        ;#ifdef __CC5X__
                        ;#pragma rambank 0
                        ;#endif
                        ;
                        ;static BOOL RS232RxCharReady(void);
                        ;static char RS232RxGetChar(void);
                        ;static BOOL RS232RxPutChar(char c);
                        ;static BOOL RS232TxCharReady(void);
                        ;static char RS232TxGetChar(void);
                        ;static BOOL RS232TxPutChar(char c);
                        ;
                        ;/* Line input parser */
                        ;#ifdef __CC5X__
                        ;#pragma rambank 3
                        ;#endif
                        ;static char _acLine[RS232_LINESIZE];
                        ;#ifdef __CC5X__
                        ;#pragma rambank 0
                        ;#endif
                        ;static U8 _u8LinePos;
                        ;
                        ;// Are we moving? 
                        ;static BOOL _bAzTrack;
                        ;static BOOL _bElTrack;
                        ;
                        ;// Target Az/El
                        ;static S16 _s16AzTarget;
                        ;static S16 _s16ElTarget;
                        ;
                        ;// EEPROM stored values
                        ;static float _fAzMul;
                        ;static U16 _u16AzOff; // Raw ADC offset for zero Az
                        ;static float _fElMul;
                        ;static U16 _u16ElOff; // Raw ADC offset for zero El
                        ;static U16 _u16Flags; // Flags
                        ;
                        ;//AK
                        ;static U16 _u16AzOffset[5];
                        ;static float _fAzMult[5];
                        ;
                        ;// Incremented by timer 0 interrupt
                        ;static U16 _u16Timer; 
                        ;
                        ;static BOOL _bLCDActive;
                        ;static U16 _u16LCDTimer; // Timer setting last time LCD was updated to stop flicker
                        ;
                        ;#ifdef __CC5X__
                        ;#pragma origin 4
        ORG 0x0004
                        ;#endif
                        ;
                        ;#ifdef __CC8E__
                        ;#pragma origin 8
                        ;#endif
                        ;
                        ;interrupt serverX(void)
                        ;{
serverX
                        ;  // W and STATUS are saved by the next macro.
                        ;  // PCLATH is also saved if necessary.
                        ;  // The code produced is strongly CPU-dependent.
                        ;
                        ;#ifdef __CC5X__
                        ;  int_save_registers    // W, STATUS (and PCLATH)
        MOVWF svrWREG
        SWAPF STATUS,W
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF svrSTATUS
        MOVF  PCLATH,W
        MOVWF svrPCLATH
        CLRF  PCLATH
                        ;  char sv_FSR; sv_FSR = FSR;  // if required
        MOVF  FSR,W
        MOVWF sv_FSR
                        ;#endif
                        ;
                        ;#ifdef __CC8E__
                        ;	uns16 sv_FSR0=FSR0;
                        ;#endif
                        ;
                        ;  // handle the timer interrupt
                        ;  if (T0IF && T0IE)
        BTFSS 0x0B,T0IF
        GOTO  m001
        BTFSS 0x0B,T0IE
        GOTO  m001
                        ;  {
                        ;    _u16Timer++;
        INCF  _u16Timer,1
        BTFSC 0x03,Zero_
        INCF  _u16Timer+1,1
                        ;    TMR0=250; /* Increments and overflows when rolls over past 255 */
        MOVLW 250
        MOVWF TMR0
                        ;    T0IF=0; /* Must remember to clear overflow or will continuously interrupt! */
        BCF   0x0B,T0IF
                        ;  }
                        ;
                        ;  // handle the tx serial interrupt
                        ;  if (TXIF && TXIE)
m001    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x0C,TXIF
        GOTO  m003
        BSF   0x03,RP0
        BTFSS 0x8C,TXIE
        GOTO  m003
                        ;  {
                        ;    if (RS232TxCharReady())
        BSF   0x0A,PA1
        CALL  RS232TxCharReady
        BCF   0x0A,PA1
        BTFSS 0x03,Carry
        GOTO  m002
                        ;    {
                        ;      TXREG=RS232TxGetChar();
        BSF   0x0A,PA1
        CALL  RS232TxGetChar
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF TXREG
                        ;    }
                        ;    else
        GOTO  m003
                        ;    {
                        ;      TXIE=0;
m002    BSF   0x03,RP0
        BCF   0x03,RP1
        BCF   0x8C,TXIE
                        ;    }
                        ;  }
                        ;
                        ;  // handle the rx serial interrupt
                        ;  if (RCIF && RCIE)
m003    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x0C,RCIF
        GOTO  m005
        BSF   0x03,RP0
        BTFSS 0x8C,RCIE
        GOTO  m005
                        ;  {
                        ;    if (OERR)
        BCF   0x03,RP0
        BTFSS 0x18,OERR
        GOTO  m004
                        ;    {
                        ;      CREN=0; /* Must reset Rx logic on overrun */
        BCF   0x18,CREN
                        ;      CREN=1;
        BSF   0x18,CREN
                        ;    }
                        ;    else
        GOTO  m005
                        ;    {
                        ;			char c=RCREG; /* New for v0.8 for Predict compatibility */
m004    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  RCREG,W
        MOVWF c
                        ;
                        ;			if (c!='\0')
        MOVF  c,1
        BTFSC 0x03,Zero_
        GOTO  m005
                        ;			{
                        ;	      RS232RxPutChar(c);
        MOVF  c,W
        BSF   0x0A,PA1
        CALL  RS232RxPutChar
        BCF   0x0A,PA1
                        ;			}
                        ;    }
                        ;  }
                        ;
                        ;#ifdef __CC5X__
                        ;  FSR = sv_FSR;               // if required
m005    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  sv_FSR,W
        MOVWF FSR
                        ;  int_restore_registers // W, STATUS (and PCLATH)
        MOVF  svrPCLATH,W
        MOVWF PCLATH
        SWAPF svrSTATUS,W
        MOVWF STATUS
        SWAPF svrWREG,1
        SWAPF svrWREG,W
                        ;#endif
                        ;#ifdef __CC8E__
                        ;	FSR0=sv_FSR0;
                        ;#pragma fastMode
                        ;#endif
                        ;}
        RETFIE
                        ;
                        ;
                        ;#ifdef __CC5X__
                        ;#pragma codepage 3
        ORG 0x1800

  ; FILE C:\Program Files (x86)\bknd\CC5X\math24f.h
                        ;// *************************************************
                        ;// 24 bit basic floating point math operations
                        ;// Copyright (c) B Knudsen Data, Norway, 2000 - 2009
                        ;// *************************************************
                        ;
                        ;#pragma library 1
                        ;/* PROTOTYPES for page definition in application header file:
                        ;float24 operator* _fmul24( float24 arg1f24, float24 arg2f24);
                        ;float24 operator/ _fdiv24( float24 arg1f24, float24 arg2f24);
                        ;float24 operator+ _fadd24( float24 arg1f24, float24 arg2f24);
                        ;float24 operator- _fsub24( float24 arg1f24, float24 arg2f24);
                        ;float24 operator= _int24ToFloat24( int24 arg1f24);
                        ;float24 operator= _int32ToFloat24( int32 arg32);
                        ;int24 operator= _float24ToInt24( float24 arg1f24);
                        ;bit operator< _f24_LT_f24( float24 arg1f24, float24 arg2f24);
                        ;bit operator>= _f24_GE_f24( float24 arg1f24, float24 arg2f24);
                        ;bit operator> _f24_GT_f24( float24 arg1f24, float24 arg2f24);
                        ;bit operator<= _f24_LE_f24( float24 arg1f24, float24 arg2f24);
                        ;*/
                        ;
                        ;// DEFINABLE SYMBOLS (in the application code):
                        ;//#define FP_OPTIM_SPEED  // optimize for SPEED: default
                        ;//#define FP_OPTIM_SIZE   // optimize for SIZE
                        ;//#define DISABLE_ROUNDING   // disable rounding and save code space
                        ;
                        ;#define float24ToIEEE754(a) { a.mid8=rl(a.mid8); a.high8=rr(a.high8);\
                        ;                              a.mid8=rr(a.mid8); }
                        ;#define IEEE754ToFloat24(a) { a.mid8=rl(a.mid8); a.high8=rl(a.high8);\
                        ;                              a.mid8=rr(a.mid8); }
                        ;
                        ;
                        ;/*  24 bit floating point format:
                        ;
                        ;  address  ID
                        ;    X      a.low8  : LSB, bit 0-7 of mantissa
                        ;    X+1    a.mid8  : bit 8-14 of mantissa, bit 15 is the sign bit
                        ;    X+2    a.high8 : MSB, bit 0-7 of exponent, with bias 0x7F
                        ;
                        ;    bit 15 of mantissa is a hidden bit, always equal to 1
                        ;    zero (0.0) :  a.high8 = 0 (mantissa & sign ignored)
                        ;
                        ;   MSB    LSB
                        ;    7F 00 00  : 1.0   =  1.0  * 2**(0x7F-0x7F) = 1.0 * 1
                        ;    7F 80 00  : -1.0  = -1.0  * 2**(0x7F-0x7F) = -1.0 * 1
                        ;    80 00 00  : 2.0   =  1.0  * 2**(0x80-0x7F) = 1.0 * 2
                        ;    80 40 00  : 3.0   =  1.5  * 2**(0x80-0x7F) = 1.5 * 2
                        ;    7E 60 00  : 0.875 =  1.75 * 2**(0x7E-0x7F) = 1.75 * 0.5
                        ;    7F 60 00  : 1.75  =  1.75 * 2**(0x7E-0x7F) = 1.75 * 1
                        ;    7F 7F FF  : 1.999969482
                        ;    00 7C 5A  : 0.0 (mantissa & sign ignored)
                        ;    01 00 00  : 1.17549435e-38 =  1.0 * 2**(0x01-0x7F)
                        ;    FE 7F FF  : 3.40277175e+38 =  1.999969482 * 2**(0xFE-0x7F)
                        ;    FF 00 00  : +INF : positive infinity
                        ;    FF 80 00  : -INF : negative infinity
                        ;*/                 
                        ;
                        ;#define  FpBIAS  0x7F
                        ;
                        ;#ifndef FpFlags_defined
                        ; #define FpFlags_defined
                        ;
                        ; char FpFlags;
                        ; //bit IOV         @ FpFlags.0; // integer overflow flag: NOT USED
                        ; bit FpOverflow    @ FpFlags.1; // floating point overflow flag
                        ; bit FpUnderFlow   @ FpFlags.2; // floating point underflow flag
                        ; bit FpDiv0        @ FpFlags.3; // floating point divide by zero flag
                        ; //bit FpNAN       @ FpFlags.4; // not-a-number exception flag: NOT USED
                        ; bit FpDomainError @ FpFlags.5; // domain error exception flag
                        ; bit FpRounding    @ FpFlags.6; // floating point rounding flag, 0=truncation
                        ;                                // 1 = unbiased rounding to nearest LSB
                        ; //bit FpSaturate  @ FpFlags.7; // floating point saturate flag: NOT USED
                        ;
                        ; #pragma floatOverflow FpOverflow
                        ; #pragma floatUnderflow FpUnderFlow
                        ;
                        ; #define InitFpFlags()  FpFlags = 0x40 /* enable rounding as default */
                        ;#endif
                        ;
                        ;#ifdef DISABLE_ROUNDING
                        ; #pragma floatRounding 0
                        ;#endif
                        ;
                        ;
                        ;#if __CoreSet__ < 1410
                        ; #define genAdd(r,a) W=a; btsc(Carry); W=incsz(a); r+=W
                        ; #define genSub(r,a) W=a; btss(Carry); W=incsz(a); r-=W
                        ; #define genAddW(r,a) W=a; btsc(Carry); W=incsz(a); W=r+W
                        ; #define genSubW(r,a) W=a; btss(Carry); W=incsz(a); W=r-W
                        ;#else
                        ; #define genAdd(r,a) W=a; r=addWFC(r)
                        ; #define genSub(r,a) W=a; r=subWFB(r)
                        ; #define genAddW(r,a) W=a; W=addWFC(r)
                        ; #define genSubW(r,a) W=a; W=subWFB(r)
                        ;#endif
                        ;
                        ;
                        ;
                        ;float24 operator* _fmul24( sharedM float24 arg1f24, sharedM float24 arg2f24)
                        ;{
_fmul24
                        ;    uns16 aarg;
                        ;    W = arg1f24.mid8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24+1,W
                        ;    aarg.high8 = W;
        MOVWF aarg+1
                        ;
                        ;    // save sign
                        ;    char sign = arg2f24.mid8 ^ W;  // before first overflow test
        XORWF arg2f24+1,W
        MOVWF sign
                        ;
                        ;    W = arg1f24.high8;
        MOVF  arg1f24+2,W
                        ;    if (!Zero_)
        BTFSS 0x03,Zero_
                        ;        W = arg2f24.high8;
        MOVF  arg2f24+2,W
                        ;    if (Zero_)
        BTFSC 0x03,Zero_
                        ;        goto RES0;
        GOTO  m012
                        ;
                        ;    arg1f24.high8 += W /* arg2f24.high8 */;
        BCF   0x03,RP0
        BCF   0x03,RP1
        ADDWF arg1f24+2,1
                        ;    W = FpBIAS-1;
        MOVLW 126
                        ;    if (Carry)  {
        BTFSS 0x03,Carry
        GOTO  m006
                        ;        arg1f24.high8 -= W;
        SUBWF arg1f24+2,1
                        ;        if (Carry)
        BTFSS 0x03,Carry
        GOTO  m007
                        ;            goto OVERFLOW;
        GOTO  m013
                        ;    }
                        ;    else  {
                        ;        arg1f24.high8 -= W;
m006    BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF arg1f24+2,1
                        ;        if (!Carry)
        BTFSS 0x03,Carry
                        ;            goto UNDERFLOW;
        GOTO  m011
                        ;    }
                        ;    aarg.low8 = arg1f24.low8;
m007    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF aarg
                        ;
                        ;    aarg.15 = 1;
        BSF   aarg+1,7
                        ;    arg2f24.15 = 1;
        BSF   arg2f24+1,7
                        ;
                        ;    arg1f24.low16 = 0;
        CLRF  arg1f24
        CLRF  arg1f24+1
                        ;
                        ;    char counter = sizeof(aarg)*8;
        MOVLW 16
        MOVWF counter
                        ;
                        ;    do  {
                        ;        aarg = rr( aarg);
m008    BCF   0x03,RP0
        BCF   0x03,RP1
        RRF   aarg+1,1
        RRF   aarg,1
                        ;        if (Carry)  {
        BTFSS 0x03,Carry
        GOTO  m009
                        ;            arg1f24.low8 += arg2f24.low8;
        MOVF  arg2f24,W
        ADDWF arg1f24,1
                        ;            genAdd( arg1f24.mid8, arg2f24.mid8);
        MOVF  arg2f24+1,W
        BTFSC 0x03,Carry
        INCFSZ arg2f24+1,W
        ADDWF arg1f24+1,1
                        ;        }
                        ;        arg1f24.low16 = rr( arg1f24.low16);
m009    BCF   0x03,RP0
        BCF   0x03,RP1
        RRF   arg1f24+1,1
        RRF   arg1f24,1
                        ;    } while (-- counter > 0);
        DECFSZ counter,1
        GOTO  m008
                        ;
                        ;    if (!arg1f24.15)  {
        BTFSC arg1f24+1,7
        GOTO  m010
                        ;        // catch Carry bit that was shifted out previously
                        ;        arg1f24.low16 = rl( arg1f24.low16);
        RLF   arg1f24,1
        RLF   arg1f24+1,1
                        ;        if (arg1f24.high8 == 0)
        MOVF  arg1f24+2,1
        BTFSC 0x03,Zero_
                        ;            goto UNDERFLOW;
        GOTO  m011
                        ;        arg1f24.high8 -= 1;
        BCF   0x03,RP0
        BCF   0x03,RP1
        DECF  arg1f24+2,1
                        ;        W = rl( aarg.high8);
        RLF   aarg+1,W
                        ;        // restore bit behind LSB in Carry
                        ;    }
                        ;
                        ;   #ifndef DISABLE_ROUNDING
                        ;    if (FpRounding  &&  Carry)  {
m010    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x6D,FpRounding
        GOTO  m015
        BTFSS 0x03,Carry
        GOTO  m015
                        ;        arg1f24.low8 += 1;
        INCFSZ arg1f24,1
                        ;        if (!arg1f24.low8)  {
        GOTO  m015
                        ;            arg1f24.mid8 += 1;
        INCFSZ arg1f24+1,1
                        ;            if (!arg1f24.mid8)  {
        GOTO  m015
                        ;                // Carry = 1; //OK
                        ;                arg1f24.low16 = rr( arg1f24.low16);
        RRF   arg1f24+1,1
        RRF   arg1f24,1
                        ;                arg1f24.high8 += 1;
        INCFSZ arg1f24+2,1
                        ;                if (Zero_)
        GOTO  m015
                        ;                    goto OVERFLOW;
        GOTO  m013
                        ;            }
                        ;        }
                        ;    }
                        ;   #endif
                        ;    goto SET_SIGN;
                        ;
                        ;  UNDERFLOW:
                        ;    FpUnderFlow = 1;
m011    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x6D,FpUnderFlow
                        ;  RES0:
                        ;    arg1f24.high8 = 0;
m012    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg1f24+2
                        ;    goto MANTISSA;
        GOTO  m014
                        ;
                        ;  OVERFLOW:
                        ;    FpOverflow = 1;
m013    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x6D,FpOverflow
                        ;    arg1f24.high8 = 0xFF;
        MOVLW 255
        MOVWF arg1f24+2
                        ;  MANTISSA:
                        ;    arg1f24.low16 = 0x8000;
m014    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg1f24
        MOVLW 128
        MOVWF arg1f24+1
                        ;
                        ;  SET_SIGN:
                        ;    if (!(sign & 0x80))
m015    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS sign,7
                        ;        arg1f24.15 = 0;
        BCF   arg1f24+1,7
                        ;    return arg1f24;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        RETURN
                        ;}
                        ;
                        ;
                        ;
                        ;float24 operator/ _fdiv24( sharedM float24 arg1f24, sharedM float24 arg2f24)
                        ;{
_fdiv24
                        ;    uns16 aarg;
                        ;    W = arg1f24.mid8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24+1,W
                        ;    aarg.high8 = W;
        MOVWF aarg_2+1
                        ;
                        ;    // save sign
                        ;    char sign = arg2f24.mid8 ^ W;  // before first overflow test
        XORWF arg2f24+1,W
        MOVWF sign_2
                        ;
                        ;    W = arg2f24.high8;
        MOVF  arg2f24+2,W
                        ;    if (Zero_)
        BTFSC 0x03,Zero_
                        ;        goto Div0;
        GOTO  m026
                        ;    if (!arg1f24.high8)
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24+2,1
        BTFSC 0x03,Zero_
                        ;        goto RES0;
        GOTO  m028
                        ;
                        ;    arg1f24.high8 -= arg2f24.high8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24+2,W
        SUBWF arg1f24+2,1
                        ;    W = FpBIAS;
        MOVLW 127
                        ;    if (!Carry)  {
        BTFSC 0x03,Carry
        GOTO  m016
                        ;        arg1f24.high8 += W;
        ADDWF arg1f24+2,1
                        ;        if (!Carry)
        BTFSC 0x03,Carry
        GOTO  m017
                        ;            goto UNDERFLOW;
        GOTO  m027
                        ;    }
                        ;    else  {
                        ;        arg1f24.high8 += W;
m016    BCF   0x03,RP0
        BCF   0x03,RP1
        ADDWF arg1f24+2,1
                        ;        if (Carry)
        BTFSC 0x03,Carry
                        ;            goto OVERFLOW;
        GOTO  m029
                        ;    }
                        ;
                        ;    aarg.low8 = arg1f24.low8;
m017    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF aarg_2
                        ;    aarg.15 = 1;
        BSF   aarg_2+1,7
                        ;    arg2f24.15 = 1;
        BSF   arg2f24+1,7
                        ;
                        ;    // division: shift & add
                        ;    char counter = 16;
        MOVLW 16
        MOVWF counter_2
                        ;    arg1f24.low16 = 0;  // speedup
        CLRF  arg1f24
        CLRF  arg1f24+1
                        ;
                        ;#if defined FP_OPTIM_SPEED || !defined FP_OPTIM_SIZE  // SPEED
                        ;
                        ;    goto START_ML;
        GOTO  m020
                        ;
                        ;  TEST_ZERO_L:
                        ;    W = aarg.low8 - arg2f24.low8;
m018    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24,W
        SUBWF aarg_2,W
                        ;    if (!Carry)
        BTFSS 0x03,Carry
                        ;        goto SHIFT_IN_CARRY;
        GOTO  m023
                        ;    aarg.low8 = W;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF aarg_2
                        ;    aarg.high8 = 0;
        CLRF  aarg_2+1
                        ;    goto SET_AND_SHIFT_IN_CARRY;
        GOTO  m022
                        ;
                        ;// MAIN LOOP
                        ;    do  {
                        ;      LOOP_ML:
                        ;        if (!Carry)  {
m019    BTFSC 0x03,Carry
        GOTO  m021
                        ;           START_ML:
                        ;            W = aarg.high8 - arg2f24.mid8;
m020    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24+1,W
        SUBWF aarg_2+1,W
                        ;            if (Zero_)
        BTFSC 0x03,Zero_
                        ;                goto TEST_ZERO_L;
        GOTO  m018
                        ;            if (!Carry)
        BTFSS 0x03,Carry
                        ;                goto SHIFT_IN_CARRY;
        GOTO  m023
                        ;        }
                        ;        aarg.low8 -= arg2f24.low8;
m021    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24,W
        SUBWF aarg_2,1
                        ;        genSub( aarg.high8, arg2f24.mid8);
        MOVF  arg2f24+1,W
        BTFSS 0x03,Carry
        INCFSZ arg2f24+1,W
        SUBWF aarg_2+1,1
                        ;      SET_AND_SHIFT_IN_CARRY:
                        ;        Carry = 1;
m022    BSF   0x03,Carry
                        ;      SHIFT_IN_CARRY:
                        ;        arg1f24.low16 = rl( arg1f24.low16);
m023    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   arg1f24,1
        RLF   arg1f24+1,1
                        ;        // Carry = 0;  // ok, speedup
                        ;        aarg = rl( aarg);
        RLF   aarg_2,1
        RLF   aarg_2+1,1
                        ;    } while (-- counter > 0);
        DECFSZ counter_2,1
        GOTO  m019
                        ;
                        ;
                        ;
                        ;#else  // SIZE
                        ;
                        ;    goto START_ML;
                        ;
                        ;// MAIN LOOP
                        ;    do  {
                        ;      LOOP_ML:
                        ;        if (Carry)
                        ;            goto SUBTRACT;
                        ;      START_ML:
                        ;        W = aarg.low8 - arg2f24.low8;
                        ;        genSubW( aarg.high8, arg2f24.mid8);
                        ;        if (!Carry)
                        ;            goto SKIP_SUB;
                        ;       SUBTRACT:
                        ;        aarg.low8 -= arg2f24.low8;
                        ;        genSub( aarg.high8, arg2f24.mid8);
                        ;        Carry = 1;
                        ;       SKIP_SUB:
                        ;        arg1f24.low16 = rl( arg1f24.low16);
                        ;        // Carry = 0;  // ok
                        ;        aarg = rl( aarg);
                        ;    } while (-- counter > 0);
                        ;
                        ;#endif
                        ;
                        ;    if (!arg1f24.15)  {
        BTFSC arg1f24+1,7
        GOTO  m024
                        ;        if (!arg1f24.high8)
        MOVF  arg1f24+2,1
        BTFSC 0x03,Zero_
                        ;            goto UNDERFLOW;
        GOTO  m027
                        ;        arg1f24.high8 --;
        BCF   0x03,RP0
        BCF   0x03,RP1
        DECF  arg1f24+2,1
                        ;        counter ++;
        INCF  counter_2,1
                        ;        goto LOOP_ML;
        GOTO  m019
                        ;    }
                        ;
                        ;   #ifndef DISABLE_ROUNDING
                        ;    if (FpRounding)  {
m024    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x6D,FpRounding
        GOTO  m032
                        ;        if (Carry)
        BTFSC 0x03,Carry
                        ;            goto ADD_1;
        GOTO  m025
                        ;        aarg.low8 -= arg2f24.low8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24,W
        SUBWF aarg_2,1
                        ;        genSub( aarg.high8, arg2f24.mid8);
        MOVF  arg2f24+1,W
        BTFSS 0x03,Carry
        INCFSZ arg2f24+1,W
        SUBWF aarg_2+1,1
                        ;        if (Carry)  {
        BTFSS 0x03,Carry
        GOTO  m032
                        ;          ADD_1:
                        ;            arg1f24.low8 += 1;
m025    BCF   0x03,RP0
        BCF   0x03,RP1
        INCFSZ arg1f24,1
                        ;            if (!arg1f24.low8)  {
        GOTO  m032
                        ;                arg1f24.mid8 ++;
        INCFSZ arg1f24+1,1
                        ;                if (!arg1f24.mid8)  {
        GOTO  m032
                        ;                    arg1f24.low16 = rr( arg1f24.low16);
        RRF   arg1f24+1,1
        RRF   arg1f24,1
                        ;                    arg1f24.high8 ++;
        INCFSZ arg1f24+2,1
                        ;                    if (!arg1f24.high8)
        GOTO  m032
                        ;                        goto OVERFLOW;
        GOTO  m029
                        ;                }
                        ;            }
                        ;        }
                        ;    }
                        ;   #endif
                        ;    goto SET_SIGN;
                        ;
                        ;  Div0:
                        ;    FpDiv0 = 1;
m026    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x6D,FpDiv0
                        ;    goto SATURATE;
        GOTO  m030
                        ;
                        ;  UNDERFLOW:
                        ;    FpUnderFlow = 1;
m027    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x6D,FpUnderFlow
                        ;  RES0:
                        ;    arg1f24.high8 = 0;
m028    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg1f24+2
                        ;    goto MANTISSA;
        GOTO  m031
                        ;
                        ;  OVERFLOW:
                        ;    FpOverflow = 1;
m029    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x6D,FpOverflow
                        ;  SATURATE:
                        ;    arg1f24.high8 = 0xFF;
m030    MOVLW 255
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF arg1f24+2
                        ;  MANTISSA:
                        ;    arg1f24.low16 = 0x8000;
m031    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg1f24
        MOVLW 128
        MOVWF arg1f24+1
                        ;
                        ;  SET_SIGN:
                        ;    if (!(sign & 0x80))
m032    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS sign_2,7
                        ;        arg1f24.15 = 0;
        BCF   arg1f24+1,7
                        ;    return arg1f24;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        RETURN
                        ;}
                        ;
                        ;
                        ;float24 operator+ _fadd24( sharedM float24 arg1f24, sharedM float24 arg2f24)
                        ;{
_fadd24
                        ;    char xtra, temp;
                        ;    char expo = arg1f24.high8 - arg2f24.high8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24+2,W
        SUBWF arg1f24+2,W
        MOVWF expo
                        ;    if (!Carry)  {
        BTFSC 0x03,Carry
        GOTO  m033
                        ;        expo = -expo;
        COMF  expo,1
        INCF  expo,1
                        ;        temp = arg1f24.high8;
        MOVF  arg1f24+2,W
        MOVWF temp
                        ;        arg1f24.high8 = arg2f24.high8;
        MOVF  arg2f24+2,W
        MOVWF arg1f24+2
                        ;        arg2f24.high8 = temp;
        MOVF  temp,W
        MOVWF arg2f24+2
                        ;        temp = arg1f24.mid8;
        MOVF  arg1f24+1,W
        MOVWF temp
                        ;        arg1f24.mid8 = arg2f24.mid8;
        MOVF  arg2f24+1,W
        MOVWF arg1f24+1
                        ;        arg2f24.mid8 = temp;
        MOVF  temp,W
        MOVWF arg2f24+1
                        ;        temp = arg1f24.low8;
        MOVF  arg1f24,W
        MOVWF temp
                        ;        arg1f24.low8 = arg2f24.low8;
        MOVF  arg2f24,W
        MOVWF arg1f24
                        ;        arg2f24.low8 = temp;
        MOVF  temp,W
        MOVWF arg2f24
                        ;    }
                        ;    if (expo > sizeof(arg1f24)*8-7)
m033    MOVLW 18
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF expo,W
        BTFSC 0x03,Carry
                        ;        goto _RETURN_MF;
        GOTO  m051
                        ;    if (!arg2f24.high8)
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24+2,1
        BTFSC 0x03,Zero_
                        ;        goto _RETURN_MF;   // result is arg1f24
        GOTO  m051
                        ;
                        ;    xtra = 0;
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  xtra
                        ;
                        ;    temp = arg1f24.mid8;
        MOVF  arg1f24+1,W
        MOVWF temp
                        ;    char sign = arg2f24.mid8 ^ arg1f24.mid8;
        MOVF  arg1f24+1,W
        XORWF arg2f24+1,W
        MOVWF sign_3
                        ;    arg1f24.15 = 1;
        BSF   arg1f24+1,7
                        ;    arg2f24.15 = 1;
        BSF   arg2f24+1,7
                        ;
                        ;    while (1)  {
                        ;        W = 8;
m034    MOVLW 8
                        ;        expo -= W;
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF expo,1
                        ;        if (!Carry)
        BTFSS 0x03,Carry
                        ;            break;
        GOTO  m035
                        ;        xtra = arg2f24.low8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24,W
        MOVWF xtra
                        ;        arg2f24.low8 = arg2f24.mid8;
        MOVF  arg2f24+1,W
        MOVWF arg2f24
                        ;        arg2f24.mid8 = 0;
        CLRF  arg2f24+1
                        ;    }
        GOTO  m034
                        ;    expo += W;
m035    BCF   0x03,RP0
        BCF   0x03,RP1
        ADDWF expo,1
                        ;    if (expo)  {
        BTFSC 0x03,Zero_
        GOTO  m037
                        ;        do  {
                        ;            Carry = 0;
m036    BCF   0x03,Carry
                        ;            arg2f24.low16 = rr( arg2f24.low16);
        BCF   0x03,RP0
        BCF   0x03,RP1
        RRF   arg2f24+1,1
        RRF   arg2f24,1
                        ;            xtra = rr( xtra);
        RRF   xtra,1
                        ;        } while (--expo > 0);
        DECFSZ expo,1
        GOTO  m036
                        ;    }
                        ;
                        ;
                        ;    if (sign & 0x80)  {
m037    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS sign_3,7
        GOTO  m043
                        ;        // SUBTRACT
                        ;        arg1f24.low8 -= arg2f24.low8;
        MOVF  arg2f24,W
        SUBWF arg1f24,1
                        ;        genSub( arg1f24.mid8, arg2f24.mid8);
        MOVF  arg2f24+1,W
        BTFSS 0x03,Carry
        INCFSZ arg2f24+1,W
        SUBWF arg1f24+1,1
                        ;        if (!Carry)  {  // arg2f24 > arg1f24
        BTFSC 0x03,Carry
        GOTO  m038
                        ;            arg1f24.low16 = -arg1f24.low16;
        COMF  arg1f24+1,1
        COMF  arg1f24,1
        INCF  arg1f24,1
        BTFSC 0x03,Zero_
        INCF  arg1f24+1,1
                        ;            // xtra == 0 because arg1f24.exp == arg2f24.exp
                        ;            temp ^= 0x80;  // invert sign
        MOVLW 128
        XORWF temp,1
                        ;        }
                        ;        xtra = -xtra;
m038    BCF   0x03,RP0
        BCF   0x03,RP1
        COMF  xtra,1
        INCF  xtra,1
                        ;        if (xtra)
        BTFSC 0x03,Zero_
        GOTO  m039
                        ;            arg1f24.low16 --;
        DECF  arg1f24,1
        INCF  arg1f24,W
        BTFSC 0x03,Zero_
        DECF  arg1f24+1,1
                        ;        // adjust result left
                        ;       #define counter expo
                        ;        counter = 3;
m039    MOVLW 3
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF expo
                        ;        while (!arg1f24.mid8)  {
m040    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24+1,1
        BTFSS 0x03,Zero_
        GOTO  m041
                        ;            arg1f24.mid8 = arg1f24.low8;
        MOVF  arg1f24,W
        MOVWF arg1f24+1
                        ;            arg1f24.low8 = xtra;
        MOVF  xtra,W
        MOVWF arg1f24
                        ;            xtra = 0;
        CLRF  xtra
                        ;            arg1f24.high8 -= 8;
        MOVLW 8
        SUBWF arg1f24+2,1
                        ;            if (!Carry)
        BTFSS 0x03,Carry
                        ;                goto RES0;
        GOTO  m047
                        ;            if (--counter == 0)  // max 2 iterations
        BCF   0x03,RP0
        BCF   0x03,RP1
        DECFSZ expo,1
        GOTO  m040
                        ;                goto RES0;
        GOTO  m047
                        ;        }
                        ;       #undef counter
                        ;        while (!arg1f24.15)  {
m041    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC arg1f24+1,7
        GOTO  m042
                        ;            Carry = 0;
        BCF   0x03,Carry
                        ;            xtra = rl( xtra);
        RLF   xtra,1
                        ;            arg1f24.low16 = rl( arg1f24.low16);
        RLF   arg1f24,1
        RLF   arg1f24+1,1
                        ;            arg1f24.high8 --;
        DECFSZ arg1f24+2,1
                        ;            if (!arg1f24.high8)
        GOTO  m041
                        ;                goto RES0;   // UNDERFLOW?
        GOTO  m047
                        ;        }
                        ;       #ifndef DISABLE_ROUNDING
                        ;        if (FpRounding  &&  (xtra & 0x80))  {
m042    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x6D,FpRounding
        GOTO  m050
        BTFSS xtra,7
        GOTO  m050
                        ;            xtra = 0; // disable recursion
        CLRF  xtra
                        ;            goto INCREMENT;
        GOTO  m046
                        ;        }
                        ;       #endif
                        ;    }
                        ;    else  {
                        ;        // ADD arg1f24 and arg2f24
                        ;        arg1f24.low8 += arg2f24.low8;
m043    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg2f24,W
        ADDWF arg1f24,1
                        ;        genAdd( arg1f24.mid8, arg2f24.mid8);
        MOVF  arg2f24+1,W
        BTFSC 0x03,Carry
        INCFSZ arg2f24+1,W
        ADDWF arg1f24+1,1
                        ;        if (Carry)  {
        BTFSS 0x03,Carry
        GOTO  m045
                        ;          ADJUST_RIGHT:
                        ;            arg1f24.low16 = rr( arg1f24.low16);
m044    BCF   0x03,RP0
        BCF   0x03,RP1
        RRF   arg1f24+1,1
        RRF   arg1f24,1
                        ;            xtra = rr( xtra);
        RRF   xtra,1
                        ;            arg1f24.high8 += 1;  // exp
        INCF  arg1f24+2,1
                        ;            if (!arg1f24.high8)
        BTFSC 0x03,Zero_
                        ;                goto OVERFLOW;
        GOTO  m048
                        ;        }
                        ;       #ifndef DISABLE_ROUNDING
                        ;        if (FpRounding  &&  (xtra & 0x80))  {
m045    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x6D,FpRounding
        GOTO  m050
        BTFSS xtra,7
        GOTO  m050
                        ;          INCREMENT:
                        ;            arg1f24.low8 += 1;
m046    BCF   0x03,RP0
        BCF   0x03,RP1
        INCFSZ arg1f24,1
                        ;            if (!arg1f24.low8)  {
        GOTO  m050
                        ;                arg1f24.mid8 += 1;
        INCFSZ arg1f24+1,1
                        ;                if (!arg1f24.mid8)  {
        GOTO  m050
                        ;                    Carry = 1; // prepare for shift
        BSF   0x03,Carry
                        ;                    arg1f24.0 = 0;  // disable recursion
        BCF   arg1f24,0
                        ;                    goto ADJUST_RIGHT;
        GOTO  m044
                        ;                }
                        ;            }
                        ;        }
                        ;       #endif
                        ;    }
                        ;    goto SET_SIGN;
                        ;
                        ;//  UNDERFLOW:
                        ;//    FpUnderFlow = 1;
                        ;  RES0:
                        ;    arg1f24.high8 = 0;
m047    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg1f24+2
                        ;    goto MANTISSA;
        GOTO  m049
                        ;
                        ;  OVERFLOW:
                        ;    FpOverflow = 1;
m048    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x6D,FpOverflow
                        ;    arg1f24.high8 = 0xFF;
        MOVLW 255
        MOVWF arg1f24+2
                        ;  MANTISSA:
                        ;    arg1f24.low16 = 0x8000;
m049    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg1f24
        MOVLW 128
        MOVWF arg1f24+1
                        ;
                        ;  SET_SIGN:
                        ;    if (!(temp & 0x80))
m050    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS temp,7
                        ;        arg1f24.15 = 0;
        BCF   arg1f24+1,7
                        ;
                        ;  _RETURN_MF:
                        ;    return arg1f24;
m051    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        RETURN
                        ;}
                        ;
                        ;
                        ;// SUBTRACTION
                        ;
                        ;float24 operator- _fsub24( sharedM float24 arg1f24, sharedM float24 arg2f24)
                        ;{
_fsub24
                        ;    arg2f24.mid8 ^= 0x80;
        MOVLW 128
        BCF   0x03,RP0
        BCF   0x03,RP1
        XORWF arg2f24+1,1
                        ;    arg1f24 += arg2f24;
        CALL  _fadd24
                        ;    return arg1f24;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        RETURN
                        ;}
                        ;
                        ;
                        ;float24 operator=( int8 arg) @
                        ;float24 operator=( uns8 arg) @
                        ;float24 operator=( int16 arg) @
                        ;float24 operator=( uns16 arg) @
                        ;float24 operator= _int24ToFloat24( sharedM int24 arg1f24)
                        ;{
_int24ToFloat24
                        ;    sharedM float24 arg2f24;   // unused, but required
                        ;    char expo = FpBIAS + 16 - 1;
        MOVLW 142
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF expo_2
                        ;    char xtra = 0;
        CLRF  xtra_2
                        ;    char sign = 0;
        CLRF  sign_4
                        ;    if (arg1f24 < 0)  {
        BTFSS arg1f24+2,7
        GOTO  m053
                        ;        arg1f24 = -arg1f24;
        COMF  arg1f24+2,1
        COMF  arg1f24+1,1
        COMF  arg1f24,1
        INCFSZ arg1f24,1
        GOTO  m052
        INCF  arg1f24+1,1
        BTFSC 0x03,Zero_
        INCF  arg1f24+2,1
                        ;        sign |= 0x80;
m052    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   sign_4,7
                        ;    }
                        ;    if (arg1f24.high8)  {
m053    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24+2,1
        BTFSC 0x03,Zero_
        GOTO  m054
                        ;        expo += 8;
        MOVLW 8
        ADDWF expo_2,1
                        ;        xtra = arg1f24.low8;
        MOVF  arg1f24,W
        MOVWF xtra_2
                        ;        arg1f24.low8 = arg1f24.mid8;
        MOVF  arg1f24+1,W
        MOVWF arg1f24
                        ;        arg1f24.mid8 = arg1f24.high8;
        MOVF  arg1f24+2,W
        MOVWF arg1f24+1
                        ;    }
                        ;    else if (!arg1f24.mid8)  {
        GOTO  m056
m054    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24+1,1
        BTFSS 0x03,Zero_
        GOTO  m056
                        ;        expo -= 8;
        MOVLW 8
        SUBWF expo_2,1
                        ;        W = arg1f24.low8;
        MOVF  arg1f24,W
                        ;        if (!W)
        BTFSC 0x03,Zero_
                        ;            goto _RETURN_MF;
        GOTO  m058
                        ;        arg1f24.mid8 = W;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF arg1f24+1
                        ;        arg1f24.low8 = 0;
        CLRF  arg1f24
                        ;    }
                        ;
                        ;    // arg1f24.mid8 != 0
                        ;    goto TEST_ARG1_B15;
        GOTO  m056
                        ;    do  {
                        ;        xtra = rl( xtra);
m055    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   xtra_2,1
                        ;        arg1f24.low16 = rl( arg1f24.low16);
        RLF   arg1f24,1
        RLF   arg1f24+1,1
                        ;        expo --;
        DECF  expo_2,1
                        ;      TEST_ARG1_B15:
                        ;    } while (!arg1f24.15);
m056    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS arg1f24+1,7
        GOTO  m055
                        ;
                        ;   #ifndef DISABLE_ROUNDING
                        ;    if (FpRounding && (xtra & 0x80))  {
        BTFSS 0x6D,FpRounding
        GOTO  m057
        BTFSS xtra_2,7
        GOTO  m057
                        ;        arg1f24.low8 += 1;
        INCFSZ arg1f24,1
                        ;        if (!arg1f24.low8)  {
        GOTO  m057
                        ;            arg1f24.mid8 += 1;
        INCFSZ arg1f24+1,1
                        ;            if (!arg1f24.mid8)  {
        GOTO  m057
                        ;                Carry = 1;
        BSF   0x03,Carry
                        ;                arg1f24.low16 = rr( arg1f24.low16);
        RRF   arg1f24+1,1
        RRF   arg1f24,1
                        ;                expo ++;
        INCF  expo_2,1
                        ;            }
                        ;        }
                        ;    }
                        ;   #endif
                        ;
                        ;    arg1f24.high8 = expo;
m057    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  expo_2,W
        MOVWF arg1f24+2
                        ;    if (!(sign & 0x80))
        BTFSS sign_4,7
                        ;        arg1f24.15 = 0;
        BCF   arg1f24+1,7
                        ;
                        ;  _RETURN_MF:
                        ;    float24 rval @ arg1f24;
                        ;    rval.low24 = arg1f24.low24;
                        ;    return rval;
m058    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval,W
        RETURN
                        ;}
                        ;
                        ;
                        ;float24 operator=( uns24 arg) @
                        ;float24 operator= _int32ToFloat24( int32 arg32)
                        ;{
_int32ToFloat24
                        ;    char expo = FpBIAS + 16 - 1;
                        ;    char xtra @ arg32.high8;
                        ;    char sign = 0;
                        ;    if (arg32 < 0)  {
                        ;        arg32 = -arg32;
                        ;        sign |= 0x80;
                        ;    }
                        ;    if (arg32.high8)  {
                        ;        expo += 8;
                        ;        arg32.low8 = arg32.midL8;
                        ;        arg32.midL8 = arg32.midH8;
                        ;        arg32.midH8 = arg32.high8;
                        ;        arg32.high8 = 0;
                        ;    }
                        ;    if (arg32.midH8)  {
                        ;        expo += 8;
                        ;        xtra = arg32.low8;
                        ;        arg32.low8 = arg32.midL8;
                        ;        arg32.midL8 = arg32.midH8;
                        ;    }
                        ;    else if (!arg32.midL8)  {
                        ;        expo -= 8;
                        ;        W = arg32.low8;
                        ;        if (!W)
                        ;            goto _RETURN_MF;
                        ;        arg32.midL8 = W;
                        ;        arg32.low8 = 0;
                        ;    }
                        ;
                        ;    // arg32.midL8 != 0
                        ;    goto TEST_ARG_B15;
                        ;    do  {
                        ;        xtra = rl( xtra);
                        ;        arg32.low16 = rl( arg32.low16);
                        ;        expo --;
                        ;      TEST_ARG_B15:
                        ;    } while (!arg32.15);
                        ;
                        ;   #ifndef DISABLE_ROUNDING
                        ;    if (FpRounding && (xtra & 0x80))  {
                        ;        arg32.low8 += 1;
                        ;        if (!arg32.low8)  {
                        ;            arg32.midL8 += 1;
                        ;            if (!arg32.midL8)  {
                        ;                Carry = 1;
                        ;                arg32.low16 = rr( arg32.low16);
                        ;                expo ++;
                        ;            }
                        ;        }
                        ;    }
                        ;   #endif
                        ;
                        ;    arg32.midH8 = expo;
                        ;    if (!(sign & 0x80))
                        ;        arg32.15 = 0;
                        ;
                        ;  _RETURN_MF:
                        ;    float24 rval @ arg32;
                        ;    rval.low24 = arg32.low24;
                        ;    return rval;
                        ;}
                        ;
                        ;
                        ;uns8 operator=( sharedM float24 arg1f24) @
                        ;int8 operator=( sharedM float24 arg1f24) @
                        ;uns16 operator=( sharedM float24 arg1f24) @
                        ;int16 operator=( sharedM float24 arg1f24) @
                        ;int24 operator= _float24ToInt24( sharedM float24 arg1f24)
                        ;{
_float24ToInt24
                        ;    sharedM float24 arg2f24;   // unused, but required
                        ;    char sign = arg1f24.mid8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24+1,W
        MOVWF sign_6
                        ;    char expo = arg1f24.high8 - (FpBIAS-1);
        MOVLW 126
        SUBWF arg1f24+2,W
        MOVWF expo_4
                        ;    if (!Carry)
        BTFSS 0x03,Carry
                        ;        goto RES0;
        GOTO  m064
                        ;    arg1f24.15 = 1;
        BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   arg1f24+1,7
                        ;
                        ;    arg1f24.high8 = 0;
        CLRF  arg1f24+2
                        ;   #ifndef DISABLE_ROUNDING
                        ;    char xtra = 0;
        CLRF  xtra_4
                        ;   #endif
                        ;
                        ;    // (a): expo = 0..8 : shift 1 byte to the right
                        ;    // (b): expo = 9..16: shift 0 byte
                        ;    // (c): expo = 17..24: shift 1 byte to the left
                        ;   #if __CoreSet__ / 100 == 12
                        ;    expo -= 17;
                        ;    expo = 0xFF - expo;  // COMF (Carry unchanged)
                        ;    if (Carry)  {  // (c)
                        ;   #else
                        ;    expo = 16 - expo;
        MOVF  expo_4,W
        SUBLW 16
        MOVWF expo_4
                        ;    if (!Carry)  {  // (c)
        BTFSC 0x03,Carry
        GOTO  m059
                        ;   #endif
                        ;        expo += 8;
        MOVLW 8
        ADDWF expo_4,1
                        ;        if (!Carry)
        BTFSS 0x03,Carry
                        ;            goto OVERFLOW;
        GOTO  m063
                        ;        arg1f24.high8 = arg1f24.mid8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24+1,W
        MOVWF arg1f24+2
                        ;        arg1f24.mid8 = arg1f24.low8;
        MOVF  arg1f24,W
        MOVWF arg1f24+1
                        ;        arg1f24.low8 = 0;
        CLRF  arg1f24
                        ;    }
                        ;    else  {  // (a) (b)
        GOTO  m060
                        ;        // expo = 0 .. 16
                        ;        W = expo - 8;
m059    MOVLW 8
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF expo_4,W
                        ;        if (Carry)  {  // (a)
        BTFSS 0x03,Carry
        GOTO  m060
                        ;            expo = W;
        MOVWF expo_4
                        ;           #ifndef DISABLE_ROUNDING
                        ;            xtra = arg1f24.low8;
        MOVF  arg1f24,W
        MOVWF xtra_4
                        ;           #endif
                        ;            arg1f24.low8 = arg1f24.mid8;
        MOVF  arg1f24+1,W
        MOVWF arg1f24
                        ;            arg1f24.mid8 = 0;
        CLRF  arg1f24+1
                        ;        }
                        ;    }
                        ;    if (expo)  {
m060    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  expo_4,1
        BTFSC 0x03,Zero_
        GOTO  m062
                        ;        do  {
                        ;            Carry = 0;
m061    BCF   0x03,Carry
                        ;            arg1f24.high8 = rr( arg1f24.high8);
        BCF   0x03,RP0
        BCF   0x03,RP1
        RRF   arg1f24+2,1
                        ;            arg1f24.low16 = rr( arg1f24.low16);
        RRF   arg1f24+1,1
        RRF   arg1f24,1
                        ;           #ifndef DISABLE_ROUNDING
                        ;            xtra = rr( xtra);
        RRF   xtra_4,1
                        ;           #endif
                        ;        } while (--expo);
        DECFSZ expo_4,1
        GOTO  m061
                        ;    }
                        ;    if (arg1f24.23)  {
m062    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS arg1f24+2,7
        GOTO  m066
                        ;       OVERFLOW:
                        ;        FpOverflow = 1;
m063    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x6D,FpOverflow
                        ;        W = 0xFF;
        MOVLW 255
                        ;        goto ASSIGNW;
        GOTO  m065
                        ;       RES0:
                        ;        W = 0;
m064    CLRW 
                        ;       ASSIGNW:
                        ;        arg1f24.low8 = W;
m065    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF arg1f24
                        ;        arg1f24.mid8 = W;
        MOVWF arg1f24+1
                        ;        arg1f24.high8 = W;
        MOVWF arg1f24+2
                        ;        arg1f24.23 = 0;
        BCF   arg1f24+2,7
                        ;    }
                        ;    else  {
        GOTO  m068
                        ;       #ifndef DISABLE_ROUNDING
                        ;        if (FpRounding && (xtra & 0x80))  {
m066    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x6D,FpRounding
        GOTO  m067
        BTFSS xtra_4,7
        GOTO  m067
                        ;            arg1f24.low8 += 1;
        INCF  arg1f24,1
                        ;            if (!arg1f24.low8)
        BTFSC 0x03,Zero_
                        ;                arg1f24.mid8 += 1;
        INCF  arg1f24+1,1
                        ;        }
                        ;       #endif
                        ;        if (sign & 0x80)
m067    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS sign_6,7
        GOTO  m068
                        ;            arg1f24.low24 = -arg1f24.low24;
        COMF  arg1f24+2,1
        COMF  arg1f24+1,1
        COMF  arg1f24,1
        INCFSZ arg1f24,1
        GOTO  m068
        INCF  arg1f24+1,1
        BTFSC 0x03,Zero_
        INCF  arg1f24+2,1
                        ;    }
                        ;    int24 rval @ arg1f24;
                        ;    rval = arg1f24.low24;
                        ;    return rval;
m068    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        RETURN
                        ;}
                        ;
                        ;
                        ;bit operator< _f24_LT_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
                        ;{
_f24_LT_f24
                        ;    Carry = 0;
                        ;    if (!(arg1f24.high8 | arg2f24.high8))
                        ;        return Carry;
                        ;    if (!arg1f24.15)  {
                        ;        if (arg2f24.15)
                        ;            return Carry;
                        ;        W = arg1f24.low8 - arg2f24.low8;
                        ;        genSubW( arg1f24.mid8, arg2f24.mid8);
                        ;        genSubW( arg1f24.high8, arg2f24.high8);
                        ;        goto _RETURN_MF;
                        ;    }
                        ;    if (!arg2f24.15)
                        ;        goto _RETURN_MF;
                        ;    W = arg2f24.low8 - arg1f24.low8;
                        ;    genSubW( arg2f24.mid8, arg1f24.mid8);
                        ;    genSubW( arg2f24.high8, arg1f24.high8);
                        ;  _RETURN_MF:
                        ;    if (Carry)
                        ;        return 0;
                        ;    return 1;
                        ;}
                        ;
                        ;
                        ;bit operator>= _f24_GE_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
                        ;{
_f24_GE_f24
                        ;    Carry = 1;
                        ;    if (!(arg1f24.high8 | arg2f24.high8))
                        ;        return Carry;
                        ;    if (!arg1f24.15)  {
                        ;        if (arg2f24.15)
                        ;            return Carry;
                        ;        W = arg1f24.low8 - arg2f24.low8;
                        ;        genSubW( arg1f24.mid8, arg2f24.mid8);
                        ;        genSubW( arg1f24.high8, arg2f24.high8);
                        ;        return Carry;
                        ;    }
                        ;    Carry = 0;
                        ;    if (!arg2f24.15)
                        ;        return Carry;
                        ;    W = arg2f24.low8 - arg1f24.low8;
                        ;    genSubW( arg2f24.mid8, arg1f24.mid8);
                        ;    genSubW( arg2f24.high8, arg1f24.high8);
                        ;    return Carry;
                        ;}
                        ;
                        ;
                        ;
                        ;bit operator> _f24_GT_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
                        ;{
_f24_GT_f24
                        ;    Carry = 0;
                        ;    if (!(arg1f24.high8 | arg2f24.high8))
                        ;        return Carry;
                        ;    if (!arg1f24.15)  {
                        ;        if (arg2f24.15)
                        ;            goto _RETURN_MF;
                        ;        W = arg2f24.low8 - arg1f24.low8;
                        ;        genSubW( arg2f24.mid8, arg1f24.mid8);
                        ;        genSubW( arg2f24.high8, arg1f24.high8);
                        ;        goto _RETURN_MF;
                        ;    }
                        ;    if (!arg2f24.15)
                        ;        return Carry;
                        ;    W = arg1f24.low8 - arg2f24.low8;
                        ;    genSubW( arg1f24.mid8, arg2f24.mid8);
                        ;    genSubW( arg1f24.high8, arg2f24.high8);
                        ;  _RETURN_MF:
                        ;    if (Carry)
                        ;        return 0;
                        ;    return 1;
                        ;}
                        ;
                        ;
                        ;
                        ;bit operator<= _f24_LE_f24( sharedM float24 arg1f24, sharedM float24 arg2f24)
                        ;{
_f24_LE_f24
                        ;    Carry = 1;
                        ;    if (!(arg1f24.high8 | arg2f24.high8))
                        ;        return Carry;
                        ;    if (!arg1f24.15)  {
                        ;        Carry = 0;
                        ;        if (arg2f24.15)
                        ;            return Carry;
                        ;        W = arg2f24.low8 - arg1f24.low8;
                        ;        genSubW( arg2f24.mid8, arg1f24.mid8);
                        ;        genSubW( arg2f24.high8, arg1f24.high8);
                        ;        return Carry;
                        ;    }
                        ;    if (!arg2f24.15)
                        ;        return Carry;
                        ;    W = arg1f24.low8 - arg2f24.low8;
                        ;    genSubW( arg1f24.mid8, arg2f24.mid8);
                        ;    genSubW( arg1f24.high8, arg2f24.high8);
                        ;    return Carry;

  ; FILE C:/Data/MPLABXProjects/LVBTracker.X/LVBTrack.c
                        ;#endif
                        ;
                        ;#include "math24f.h"
                        ;
                        ;#ifdef __CC5X__
                        ;#pragma codepage 2
        ORG 0x1000
                        ;#endif
                        ;
                        ;// Some glue functions...
                        ;
                        ;//static U16 ByteToWord(U8 u8Lo,U8 u8Hi)
                        ;//{
                        ;//  U16 u16=((U16)u8Hi)<<8;
                        ;//  return u16+u8Lo;
                        ;//}
                        ;
                        ;static char toupper(char c)
                        ;{
_const1
        BCF   0x03,RP0
        BCF   0x03,RP1
        RRF   ci+1,W
        ADDLW 16
        BSF   0x03,RP1
        MOVWF EEADRH
        BCF   0x03,RP1
        RRF   ci+1,W
        RRF   ci,W
        ADDLW 33
        BSF   0x03,RP1
        MOVWF EEADR
        BTFSC 0x03,Carry
        INCF  EEADRH,1
        BSF   0x03,RP0
        BSF   0x03,RP1
        BSF   0x18C,EEPGD
        BSF   0x18C,RD
        NOP  
        NOP  
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC ci,0
        GOTO  m069
        BSF   0x03,RP1
        MOVF  EEDATA,W
        ANDLW 127
        RETURN
m069    BCF   0x03,RP0
        BSF   0x03,RP1
        RLF   EEDATA,W
        RLF   EEDATH,W
        RETURN
        DW    0x2B4C
        DW    0x1042
        DW    0x3954
        DW    0x31E1
        DW    0x32EB
        DW    0x72
        DW    0x34C6
        DW    0x36F2
        DW    0x30F7
        DW    0x32F2
        DW    0x3B20
        DW    0x1731
        DW    0x32
        DW    0x3D41
        DW    0x20
        DW    0x3645
        DW    0x20
        DW    0x1420
        DW    0x2B80
        DW    0x3961
        DW    0x34EE
        DW    0x33EE
        DW    0x103A
        DW    0x30E2
        DW    0x64
        DW    0x22C5
        DW    0x2950
        DW    0x26CF
        DW    0x3220
        DW    0x3A61
        DW    0x1761
        DW    0x2980
        DW    0x3A65
        DW    0x20A0
        DW    0x1EFA
        DW    0x1030
        DW    0x26
        DW    0x32D3
        DW    0x1074
        DW    0x3D41
        DW    0x1CBD
        DW    0x1030
        DW    0x26
        DW    0x32D3
        DW    0x1074
        DW    0x3D41
        DW    0x18BD
        DW    0x1838
        DW    0x1320
        DW    0x2980
        DW    0x3A65
        DW    0x20A0
        DW    0x1EFA
        DW    0x1BB2
        DW    0x1030
        DW    0x26
        DW    0x32D3
        DW    0x1074
        DW    0x3D41
        DW    0x19BD
        DW    0x1836
        DW    0x1320
        DW    0x3800
        DW    0x32F2
        DW    0x39F3
        DW    0x2220
        DW    0x1420
        DW    0x1ED5
        DW    0x39E5
        DW    0x14E3
        DW    0x2980
        DW    0x3A65
        DW    0x22A0
        DW    0x1EEC
        DW    0x34ED
        DW    0x106E
        DW    0x26
        DW    0x32D3
        DW    0x1074
        DW    0x3645
        DW    0x36BD
        DW    0x3C61
        DW    0x1320
        DW    0x3800
        DW    0x32F2
        DW    0x39F3
        DW    0x2620
        DW    0x1420
        DW    0x1ED2
        DW    0x39E5
        DW    0x14E3
        DW    0x2080
        DW    0x107A
        DW    0x30F2
        DW    0x33EE
        DW    0x1D65
        DW    0x1A20
        DW    0x1835
        DW    0x2AA0
        DW    0x1980
        DW    0x1836
        DW    0x2220
        DW    0x1420
        DW    0x1ED2
        DW    0x39E5
        DW    0x14E3
        DW    0x2280
        DW    0x106C
        DW    0x30F2
        DW    0x33EE
        DW    0x1D65
        DW    0x1820
        DW    0x1CAD
        DW    0x1030
        DW    0x55
        DW    0x16B0
        DW    0x1C31
        DW    0x1030
        DW    0x1044
        DW    0x2928
        DW    0x32BD
        DW    0x31F3
        DW    0x29
        DW    0x21C3
        DW    0x1057
        DW    0x3A73
        DW    0x386F
        DW    0x2720
        DW    0x396F
        DW    0x3474
        DW    0x2ABD
        DW    0x2980
        DW    0x3AEF
        DW    0x3474
        DW    0x223D
        DW    0x1420
        DW    0x1ED2
        DW    0x39E5
        DW    0x14E3
        DW    0x2280
        DW    0x1045
        DW    0x3957
        DW    0x3A69
        DW    0x1D65
        DW    0x2AA0
        DW    0x2280
        DW    0x2845
        DW    0x27D2
        DW    0x104D
        DW    0x3977
        DW    0x3A69
        DW    0x1065
        DW    0x25CF
        DW    0x2280
        DW    0x2845
        DW    0x27D2
        DW    0x104D
        DW    0x30E6
        DW    0x3669
        DW    0x3265
        DW    0x21
        DW    0x3AD0
        DW    0x3473
        DW    0x3120
        DW    0x3774
        DW    0x3A20
        DW    0x106F
        DW    0x3C65
        DW    0x3A69
        DW    0x2280
        DW    0x2845
        DW    0x27D2
        DW    0x104D
        DW    0x3977
        DW    0x3A69
        DW    0x1065
        DW    0x25CF
        DW    0x50D
        DW    0x2280
        DW    0x2845
        DW    0x27D2
        DW    0x104D
        DW    0x30E6
        DW    0x3669
        DW    0x3265
        DW    0x50D
        DW    0x2980
        DW    0x3AEF
        DW    0x3474
        DW    0x39A0
        DW    0x37F4
        DW    0x6F0
        DW    0xA
        DW    0x37CE
        DW    0x3A72
        DW    0x1068
        DW    0x3A73
        DW    0x386F
        DW    0x50D
        DW    0x2080
        DW    0x107A
        DW    0x336F
        DW    0x1EE6
        DW    0x2080
        DW    0x107A
        DW    0x334F
        DW    0x39E6
        DW    0x3A65
        DW    0x3D
        DW    0x3D41
        DW    0x36A0
        DW    0x3675
        DW    0x3D
        DW    0x3645
        DW    0x37A0
        DW    0x3366
        DW    0x3D
        DW    0x3645
        DW    0x27A0
        DW    0x3366
        DW    0x32F3
        DW    0x1EF4
        DW    0x2280
        DW    0x106C
        DW    0x3AED
        DW    0x1EEC
        DW    0x0
toupper
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c_2
                        ;  if (c>='a' && c<='z')
        MOVLW 97
        SUBWF c_2,W
        BTFSS 0x03,Carry
        GOTO  m070
        MOVLW 123
        SUBWF c_2,W
        BTFSC 0x03,Carry
        GOTO  m070
                        ;  {
                        ;    c-='a'-'A';
        MOVLW 32
        SUBWF c_2,1
                        ;  }
                        ;  return c;
m070    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c_2,W
        RETURN
                        ;}
                        ;
                        ;static void Delay(unsigned char uc) /* software delay uc * 100us */
                        ;{
Delay
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF uc
                        ;  while (uc)
m071    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  uc,1
        BTFSC 0x03,Zero_
        GOTO  m073
                        ;  {
                        ;    char uc2;
                        ;
                        ;    uc--;
        DECF  uc,1
                        ;
                        ;    for (uc2=0;uc2<DELAY_CONSTANT;uc2++)
        CLRF  uc2
m072    MOVLW 13
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF uc2,W
        BTFSC 0x03,Carry
        GOTO  m071
                        ;    {
                        ;    }
        INCF  uc2,1
        GOTO  m072
                        ;  }
                        ;}
m073    RETURN
                        ;
                        ;static U16 TimerGet(void)
                        ;{
TimerGet
                        ;  U16 u16;
                        ;
                        ;  GIE=0;
        BCF   0x0B,GIE
                        ;  u16=_u16Timer;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u16Timer,W
        MOVWF u16
        MOVF  _u16Timer+1,W
        MOVWF u16+1
                        ;  GIE=1;
        BSF   0x0B,GIE
                        ;
                        ;  return u16;
        MOVF  u16,W
        RETURN
                        ;}
                        ;
                        ;static BOOL ParseWhite(U8 *pu8Pos)
                        ;{
ParseWhite
                        ;  // Ignore white space in _acLine. Return FALSE if no white space...
                        ;  BOOL bFinished=FALSE;
        BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   0x34,bFinished
                        ;  BOOL bWhiteFound=FALSE; // at end of string
        BCF   0x34,bWhiteFound
                        ;  U8 u8Pos=*pu8Pos;
        BCF   0x03,IRP
        MOVF  pu8Pos,W
        MOVWF FSR
        MOVF  INDF,W
        MOVWF u8Pos
                        ;  char c;
                        ;
                        ;  while (!bFinished)
m074    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC 0x34,bFinished
        GOTO  m077
                        ;  {
                        ;    if (u8Pos<_u8LinePos)
        MOVF  _u8LinePos,W
        SUBWF u8Pos,W
        BTFSC 0x03,Carry
        GOTO  m076
                        ;    {
                        ;      c=_acLine[u8Pos];
        MOVLW 144
        ADDWF u8Pos,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        MOVWF c_3
                        ;      if (c!=' ' && c!='\t')
        MOVF  c_3,W
        XORLW 32
        BTFSC 0x03,Zero_
        GOTO  m075
        MOVF  c_3,W
        XORLW 9
        BTFSC 0x03,Zero_
        GOTO  m075
                        ;      {
                        ;        bFinished=TRUE;
        BSF   0x34,bFinished
                        ;      }
                        ;      else
        GOTO  m074
                        ;      {
                        ;        bWhiteFound=TRUE;
m075    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x34,bWhiteFound
                        ;        u8Pos++;
        INCF  u8Pos,1
                        ;      }
                        ;    }
                        ;    else
        GOTO  m074
                        ;    {
                        ;      bFinished=TRUE;
m076    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x34,bFinished
                        ;    }
                        ;  }
        GOTO  m074
                        ;  *pu8Pos=u8Pos;
m077    BCF   0x03,IRP
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  pu8Pos,W
        MOVWF FSR
        MOVF  u8Pos,W
        MOVWF INDF
                        ;  return bWhiteFound;  
        BCF   0x03,Carry
        BTFSC 0x34,bWhiteFound
        BSF   0x03,Carry
        RETURN
                        ;}
                        ;
                        ;static BOOL ParseU16(U8 *pu8Pos,U16 *pu16)
                        ;{
ParseU16
                        ;  // Parse _acLine from *pu8Pos for a number. Return FALSE if error...
                        ;  BOOL bFinished=FALSE;
        BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   0x42,bFinished_2
                        ;  BOOL bNumFound=FALSE; // at end of string
        BCF   0x42,bNumFound
                        ;  U8 u8Pos=*pu8Pos;
        BCF   0x03,IRP
        MOVF  pu8Pos_2,W
        MOVWF FSR
        MOVF  INDF,W
        MOVWF u8Pos_2
                        ;  char c;
                        ;  U16 u16=0;
        CLRF  u16_2
        CLRF  u16_2+1
                        ;
                        ;  while (!bFinished)
m078    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC 0x42,bFinished_2
        GOTO  m083
                        ;  {
                        ;    if (u8Pos<_u8LinePos)
        MOVF  _u8LinePos,W
        SUBWF u8Pos_2,W
        BTFSC 0x03,Carry
        GOTO  m082
                        ;    {
                        ;      c=_acLine[u8Pos];
        MOVLW 144
        ADDWF u8Pos_2,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        MOVWF c_4
                        ;      if (c<'0' || c>'9')
        MOVLW 48
        SUBWF c_4,W
        BTFSS 0x03,Carry
        GOTO  m079
        MOVLW 58
        SUBWF c_4,W
        BTFSS 0x03,Carry
        GOTO  m080
                        ;      {
                        ;        bFinished=TRUE;
m079    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x42,bFinished_2
                        ;      }
                        ;      else
        GOTO  m078
                        ;      {
                        ;        bNumFound=TRUE;
m080    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x42,bNumFound
                        ;        u16*=10;
        BCF   0x03,Carry
        RLF   u16_2,W
        MOVWF C2tmp
        RLF   u16_2+1,W
        MOVWF C2tmp+1
        CLRF  u16_2
        CLRF  u16_2+1
        MOVLW 5
        MOVWF C1cnt
m081    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  C2tmp+1,W
        ADDWF u16_2+1,1
        MOVF  C2tmp,W
        ADDWF u16_2,1
        BTFSC 0x03,Carry
        INCF  u16_2+1,1
        DECFSZ C1cnt,1
        GOTO  m081
                        ;        c-='0';
        MOVLW 48
        SUBWF c_4,1
                        ;        u16+=c;
        MOVF  c_4,W
        ADDWF u16_2,1
        BTFSC 0x03,Carry
        INCF  u16_2+1,1
                        ;        u8Pos++;
        INCF  u8Pos_2,1
                        ;      }
                        ;    }
                        ;    else
        GOTO  m078
                        ;    {
                        ;      bFinished=TRUE;
m082    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x42,bFinished_2
                        ;    }
                        ;  }
        GOTO  m078
                        ;  *pu8Pos=u8Pos;
m083    BCF   0x03,IRP
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  pu8Pos_2,W
        MOVWF FSR
        MOVF  u8Pos_2,W
        MOVWF INDF
                        ;  if (bNumFound)
        BTFSS 0x42,bNumFound
        GOTO  m084
                        ;  {
                        ;    *pu16=u16;
        BCF   0x03,IRP
        MOVF  pu16,W
        MOVWF FSR
        MOVF  u16_2,W
        MOVWF INDF
        INCF  FSR,1
        MOVF  u16_2+1,W
        MOVWF INDF
                        ;  }
                        ;  return bNumFound;  
m084    BCF   0x03,Carry
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC 0x42,bNumFound
        BSF   0x03,Carry
        RETURN
                        ;}
                        ;
                        ;static BOOL ParseFloat(U8 *pu8Pos,float *pf)
                        ;{
ParseFloat
                        ;  U16 u16Int;
                        ;  U16 u16Frac=0;
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u16Frac
        CLRF  u16Frac+1
                        ;  float f;
                        ;  U8 u8;
                        ;
                        ;  if (!ParseU16(pu8Pos,&u16Int))
        MOVF  pu8Pos_3,W
        MOVWF pu8Pos_2
        MOVLW 51
        MOVWF pu16
        CALL  ParseU16
        BTFSC 0x03,Carry
        GOTO  m085
                        ;  {
                        ;    return FALSE;
        BCF   0x03,Carry
        RETURN
                        ;  }
                        ;  f=u16Int;
m085    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Int,W
        MOVWF arg1f24
        MOVF  u16Int+1,W
        MOVWF arg1f24+1
        CLRF  arg1f24+2
        BSF   0x0A,PA0
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval,W
        MOVWF f
        MOVF  rval+1,W
        MOVWF f+1
        MOVF  rval+2,W
        MOVWF f+2
                        ;  u8=*pu8Pos;
        BCF   0x03,IRP
        MOVF  pu8Pos_3,W
        MOVWF FSR
        MOVF  INDF,W
        MOVWF u8
                        ;  if (_acLine[u8]=='.')
        MOVLW 144
        ADDWF u8,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        XORLW 46
        BTFSS 0x03,Zero_
        GOTO  m088
                        ;  {
                        ;    U8 u8Pos;
                        ;
                        ;    (*pu8Pos)++;
        BCF   0x03,IRP
        MOVF  pu8Pos_3,W
        MOVWF FSR
        INCF  INDF,1
                        ;
                        ;    u8Pos=*pu8Pos; // So we know how many digits in the fractional part
        BCF   0x03,IRP
        MOVF  pu8Pos_3,W
        MOVWF FSR
        MOVF  INDF,W
        MOVWF u8Pos_3
                        ;
                        ;    if (ParseU16(pu8Pos,&u16Frac))
        MOVF  pu8Pos_3,W
        MOVWF pu8Pos_2
        MOVLW 53
        MOVWF pu16
        CALL  ParseU16
        BTFSS 0x03,Carry
        GOTO  m088
                        ;    {
                        ;      float fFrac=u16Frac;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Frac,W
        MOVWF arg1f24
        MOVF  u16Frac+1,W
        MOVWF arg1f24+1
        CLRF  arg1f24+2
        BSF   0x0A,PA0
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval,W
        MOVWF fFrac
        MOVF  rval+1,W
        MOVWF fFrac+1
        MOVF  rval+2,W
        MOVWF fFrac+2
                        ;
                        ;      u8Pos=*pu8Pos-u8Pos;
        BCF   0x03,IRP
        MOVF  pu8Pos_3,W
        MOVWF FSR
        MOVF  u8Pos_3,W
        SUBWF INDF,W
        MOVWF u8Pos_3
                        ;      while (u8Pos)
m086    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8Pos_3,1
        BTFSC 0x03,Zero_
        GOTO  m087
                        ;      {
                        ;        fFrac/=10;
        MOVF  fFrac,W
        MOVWF arg1f24
        MOVF  fFrac+1,W
        MOVWF arg1f24+1
        MOVF  fFrac+2,W
        MOVWF arg1f24+2
        CLRF  arg2f24
        MOVLW 32
        MOVWF arg2f24+1
        MOVLW 130
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        CALL  _fdiv24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF fFrac
        MOVF  arg1f24+1,W
        MOVWF fFrac+1
        MOVF  arg1f24+2,W
        MOVWF fFrac+2
                        ;        u8Pos--;
        DECF  u8Pos_3,1
                        ;      }
        GOTO  m086
                        ;      f+=fFrac;
m087    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  f,W
        MOVWF arg1f24
        MOVF  f+1,W
        MOVWF arg1f24+1
        MOVF  f+2,W
        MOVWF arg1f24+2
        MOVF  fFrac,W
        MOVWF arg2f24
        MOVF  fFrac+1,W
        MOVWF arg2f24+1
        MOVF  fFrac+2,W
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        CALL  _fadd24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF f
        MOVF  arg1f24+1,W
        MOVWF f+1
        MOVF  arg1f24+2,W
        MOVWF f+2
                        ;    }
                        ;  }
                        ;  *pf=f;
m088    BCF   0x03,IRP
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  pf,W
        MOVWF FSR
        MOVF  f,W
        MOVWF INDF
        INCF  FSR,1
        MOVF  f+1,W
        MOVWF INDF
        INCF  FSR,1
        MOVF  f+2,W
        MOVWF INDF
                        ;  return TRUE;
        BSF   0x03,Carry
        RETURN
                        ;}
                        ;
                        ;/*************** ADC functions ****************/
                        ;
                        ;static U16 ADCGet(U8 u8Channel)
                        ;{
ADCGet
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Channel
                        ;  int n;
                        ;  U16 u16Sum=0;
        CLRF  u16Sum
        CLRF  u16Sum+1
                        ;
                        ;  for (n=0;n<ADC_NUMSAMPLES;n++)
        CLRF  n
m089    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC n,7
        GOTO  m090
        MOVLW 25
        SUBWF n,W
        BTFSC 0x03,Carry
        GOTO  m092
                        ;  {
                        ;    U16 u16Result=0;
m090    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u16Result
        CLRF  u16Result+1
                        ;
                        ;    ADFM=1; // Right Justified
        BSF   0x03,RP0
        BSF   0x9F,ADFM
                        ;
                        ;#ifdef __CC5X__
                        ;    PCFG0=0; // VDD/VSS ref, three channels
        BCF   0x9F,PCFG0
                        ;    PCFG1=0;
        BCF   0x9F,PCFG1
                        ;    PCFG2=1;
        BSF   0x9F,PCFG2
                        ;    PCFG3=0;
        BCF   0x9F,PCFG3
                        ;#endif
                        ;
                        ;#ifdef __CC8E__
                        ;    PCFG0=1; // VDD/VSS ref, two channels
                        ;    PCFG1=0;
                        ;    PCFG2=1;
                        ;    PCFG3=1;
                        ;		VCFG0=0;
                        ;		VCFG1=0;
                        ;#endif
                        ;
                        ;    CHS0=u8Channel.0;
        BCF   0x03,RP0
        BTFSS u8Channel,0
        BCF   0x1F,CHS0
        BTFSC u8Channel,0
        BSF   0x1F,CHS0
                        ;    CHS1=u8Channel.1;
        BTFSS u8Channel,1
        BCF   0x1F,CHS1
        BTFSC u8Channel,1
        BSF   0x1F,CHS1
                        ;    CHS2=u8Channel.2;
        BTFSS u8Channel,2
        BCF   0x1F,CHS2
        BTFSC u8Channel,2
        BSF   0x1F,CHS2
                        ;
                        ;#ifdef __CC8E__
                        ;	  CHS3=u8Channel.3; /********NEW********/
                        ;#endif
                        ;
                        ;
                        ;    ADCS0=0; // FOSC/32
        BCF   0x1F,ADCS0
                        ;    ADCS1=1;
        BSF   0x1F,ADCS1
                        ;
                        ;#ifdef __CC8E__
                        ;	  ADCS2=0; /********NEW********/
                        ;#endif
                        ;
                        ;    ADON=1; // switch on A/D
        BSF   0x1F,ADON
                        ;
                        ;    Delay(1); // Wait for required acquisition time
        MOVLW 1
        CALL  Delay
                        ;
                        ;    GO=1; // Set Go/!Done bit to start conversion
        BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x1F,GO
                        ;
                        ;    while (GO) // Wait for Go/!Done bit to clear
m091    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC 0x1F,GO
                        ;    {
                        ;    }
        GOTO  m091
                        ;
                        ;    u16Result=((U16)(ADRESH))<<8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  ADRESH,W
        MOVWF u16Result+1
        CLRF  u16Result
                        ;    u16Result|=ADRESL;
        BSF   0x03,RP0
        MOVF  ADRESL,W
        BCF   0x03,RP0
        IORWF u16Result,1
                        ;
                        ;    ADON=0; // switch off A/D
        BCF   0x1F,ADON
                        ;
                        ;#ifdef __CC5X__
                        ;    PCFG0=0; // VDD/VSS ref, zero channels
        BSF   0x03,RP0
        BCF   0x9F,PCFG0
                        ;    PCFG1=1;
        BSF   0x9F,PCFG1
                        ;    PCFG2=1;
        BSF   0x9F,PCFG2
                        ;    PCFG3=0;
        BCF   0x9F,PCFG3
                        ;#endif
                        ;
                        ;#ifdef __CC8E__
                        ;    PCFG0=1; // VDD/VSS ref, zero channels
                        ;    PCFG1=1;
                        ;    PCFG2=1;
                        ;    PCFG3=1;
                        ;		VCFG0=0;
                        ;		VCFG1=0;
                        ;#endif
                        ;
                        ;
                        ;    u16Sum+=u16Result;  
        BCF   0x03,RP0
        MOVF  u16Result+1,W
        ADDWF u16Sum+1,1
        MOVF  u16Result,W
        ADDWF u16Sum,1
        BTFSC 0x03,Carry
        INCF  u16Sum+1,1
                        ;  }
        INCF  n,1
        GOTO  m089
                        ;  u16Sum/=ADC_NUMSAMPLES;
m092    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF C4tmp
        MOVF  u16Sum+1,W
        MOVWF C4tmp+1
        CLRF  C5rem
        MOVLW 16
        MOVWF C3cnt
m093    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   C4tmp,1
        RLF   C4tmp+1,1
        RLF   C5rem,1
        BTFSC 0x03,Carry
        GOTO  m094
        MOVLW 25
        SUBWF C5rem,W
        BTFSS 0x03,Carry
        GOTO  m095
m094    MOVLW 25
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF C5rem,1
        BSF   0x03,Carry
m095    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16Sum,1
        RLF   u16Sum+1,1
        DECFSZ C3cnt,1
        GOTO  m093
                        ;  return u16Sum;
        MOVF  u16Sum,W
        RETURN
                        ;}
                        ;
                        ;/********* RS232 Functions *************/
                        ;
                        ;static BOOL RS232RxCharReady(void)
                        ;{
RS232RxCharReady
                        ;  if (_cRS232RxIn==_cRS232RxOut)
        BSF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _cRS232RxIn,W
        XORWF _cRS232RxOut,W
        BTFSS 0x03,Zero_
        GOTO  m096
                        ;  {
                        ;    return FALSE;
        BCF   0x03,Carry
        RETURN
                        ;  }
                        ;  else
                        ;  {
                        ;    return TRUE;
m096    BSF   0x03,Carry
        RETURN
                        ;  }
                        ;}
                        ;
                        ;static char RS232RxGetChar(void)
                        ;{
RS232RxGetChar
                        ;  if (RS232RxCharReady())
        CALL  RS232RxCharReady
        BTFSS 0x03,Carry
        GOTO  m097
                        ;  {
                        ;    char c=_acRS232RxData[_cRS232RxOut];
        MOVLW 16
        BSF   0x03,RP0
        BCF   0x03,RP1
        ADDWF _cRS232RxOut,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        BCF   0x03,RP0
        MOVWF c_5
                        ;    _cRS232RxOut++;
        BSF   0x03,RP0
        INCF  _cRS232RxOut,1
                        ;    if (_cRS232RxOut>=RS232_RXBUFFERSIZE)
        MOVLW 60
        SUBWF _cRS232RxOut,W
        BTFSC 0x03,Carry
                        ;    {
                        ;      _cRS232RxOut=0;
        CLRF  _cRS232RxOut
                        ;    }
                        ;    return c;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c_5,W
        RETURN
                        ;  }
                        ;  else
                        ;  {
                        ;    return FALSE;
m097    RETLW 0
                        ;  }
                        ;}
                        ;
                        ;static BOOL RS232RxPutChar(char c)
                        ;{
RS232RxPutChar
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c_6
                        ;  char cNextIdx=_cRS232RxIn+1;
        BSF   0x03,RP0
        INCF  _cRS232RxIn,W
        BCF   0x03,RP0
        MOVWF cNextIdx
                        ;
                        ;  if (cNextIdx>=RS232_RXBUFFERSIZE)
        MOVLW 60
        SUBWF cNextIdx,W
        BTFSC 0x03,Carry
                        ;  {
                        ;    cNextIdx=0;
        CLRF  cNextIdx
                        ;  }
                        ;
                        ;  if (cNextIdx==_cRS232RxOut)
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cNextIdx,W
        BSF   0x03,RP0
        XORWF _cRS232RxOut,W
        BTFSS 0x03,Zero_
        GOTO  m098
                        ;  {
                        ;    return FALSE;
        BCF   0x03,Carry
        RETURN
                        ;  }
                        ;  _acRS232RxData[_cRS232RxIn]=c;
m098    MOVLW 16
        BSF   0x03,RP0
        BCF   0x03,RP1
        ADDWF _cRS232RxIn,W
        MOVWF FSR
        BSF   0x03,IRP
        BCF   0x03,RP0
        MOVF  c_6,W
        MOVWF INDF
                        ;  _cRS232RxIn=cNextIdx;
        MOVF  cNextIdx,W
        BSF   0x03,RP0
        MOVWF _cRS232RxIn
                        ;  return TRUE;
        BSF   0x03,Carry
        RETURN
                        ;}
                        ;
                        ;static BOOL RS232TxCharReady(void)
                        ;{
RS232TxCharReady
                        ;  if (_cRS232TxIn==_cRS232TxOut)
        BSF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _cRS232TxIn,W
        XORWF _cRS232TxOut,W
        BTFSS 0x03,Zero_
        GOTO  m099
                        ;  {
                        ;    return FALSE;
        BCF   0x03,Carry
        RETURN
                        ;  }
                        ;  else
                        ;  {
                        ;    return TRUE;
m099    BSF   0x03,Carry
        RETURN
                        ;  }
                        ;}
                        ;
                        ;static char RS232TxGetChar(void)
                        ;{
RS232TxGetChar
                        ;  if (RS232TxCharReady())
        CALL  RS232TxCharReady
        BTFSS 0x03,Carry
        GOTO  m100
                        ;  {
                        ;    char c=_acRS232TxData[_cRS232TxOut];
        MOVLW 162
        BSF   0x03,RP0
        BCF   0x03,RP1
        ADDWF _cRS232TxOut,W
        MOVWF FSR
        BCF   0x03,IRP
        MOVF  INDF,W
        BCF   0x03,RP0
        MOVWF c_7
                        ;    _cRS232TxOut++;
        BSF   0x03,RP0
        INCF  _cRS232TxOut,1
                        ;    if (_cRS232TxOut>=RS232_TXBUFFERSIZE)
        MOVLW 20
        SUBWF _cRS232TxOut,W
        BTFSC 0x03,Carry
                        ;    {
                        ;      _cRS232TxOut=0;
        CLRF  _cRS232TxOut
                        ;    }
                        ;    return c;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c_7,W
        RETURN
                        ;  }
                        ;  else
                        ;  {
                        ;    return FALSE;
m100    RETLW 0
                        ;  }
                        ;}
                        ;
                        ;static BOOL RS232TxPutChar(char c)
                        ;{
RS232TxPutChar
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c_8
                        ;  char cNextIdx=_cRS232TxIn+1;
        BSF   0x03,RP0
        INCF  _cRS232TxIn,W
        BCF   0x03,RP0
        MOVWF cNextIdx_2
                        ;
                        ;  if (cNextIdx>=RS232_TXBUFFERSIZE)
        MOVLW 20
        SUBWF cNextIdx_2,W
        BTFSC 0x03,Carry
                        ;  {
                        ;    cNextIdx=0;
        CLRF  cNextIdx_2
                        ;  }
                        ;
                        ;  if (cNextIdx==_cRS232TxOut)
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cNextIdx_2,W
        BSF   0x03,RP0
        XORWF _cRS232TxOut,W
        BTFSS 0x03,Zero_
        GOTO  m101
                        ;  {
                        ;    return FALSE;
        BCF   0x03,Carry
        RETURN
                        ;  }
                        ;  _acRS232TxData[_cRS232TxIn]=c;
m101    MOVLW 162
        BSF   0x03,RP0
        BCF   0x03,RP1
        ADDWF _cRS232TxIn,W
        MOVWF FSR
        BCF   0x03,IRP
        BCF   0x03,RP0
        MOVF  c_8,W
        MOVWF INDF
                        ;  _cRS232TxIn=cNextIdx;
        MOVF  cNextIdx_2,W
        BSF   0x03,RP0
        MOVWF _cRS232TxIn
                        ;
                        ;  if (!TXIE)
        BTFSS 0x8C,TXIE
                        ;  {
                        ;    TXIE=1;
        BSF   0x8C,TXIE
                        ;  }
                        ;  return TRUE;
        BSF   0x03,Carry
        RETURN
                        ;}
                        ;
                        ;static void RS232TxPutU16(U16 u16)
                        ;{
RS232TxPutU16
                        ;  U16 u16a;
                        ;  U16 u16b;
                        ;
                        ;  u16a=u16/1000;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_3,W
        MOVWF C7tmp
        MOVF  u16_3+1,W
        MOVWF C7tmp+1
        CLRF  C8rem
        CLRF  C8rem+1
        MOVLW 16
        MOVWF C6cnt
m102    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   C7tmp,1
        RLF   C7tmp+1,1
        RLF   C8rem,1
        RLF   C8rem+1,1
        MOVLW 3
        SUBWF C8rem+1,W
        BTFSS 0x03,Carry
        GOTO  m104
        BTFSS 0x03,Zero_
        GOTO  m103
        MOVLW 232
        SUBWF C8rem,W
        BTFSS 0x03,Carry
        GOTO  m104
m103    MOVLW 3
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF C8rem+1,1
        MOVLW 232
        SUBWF C8rem,1
        BTFSS 0x03,Carry
        DECF  C8rem+1,1
        BSF   0x03,Carry
m104    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16a,1
        RLF   u16a+1,1
        DECFSZ C6cnt,1
        GOTO  m102
                        ;  u16b=u16a*1000;
        MOVF  u16a,W
        MOVWF C10tmp
        MOVF  u16a+1,W
        MOVWF C10tmp+1
        MOVLW 16
        MOVWF C9cnt
m105    BCF   0x03,Carry
        BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16b,1
        RLF   u16b+1,1
        RLF   C10tmp,1
        RLF   C10tmp+1,1
        BTFSS 0x03,Carry
        GOTO  m106
        MOVLW 3
        ADDWF u16b+1,1
        MOVLW 232
        ADDWF u16b,1
        BTFSC 0x03,Carry
        INCF  u16b+1,1
m106    BCF   0x03,RP0
        BCF   0x03,RP1
        DECFSZ C9cnt,1
        GOTO  m105
                        ;
                        ;  RS232TxPutChar((U8)u16a+'0');
        MOVLW 48
        ADDWF u16a,W
        CALL  RS232TxPutChar
                        ;
                        ;  u16-=u16b;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16b+1,W
        SUBWF u16_3+1,1
        MOVF  u16b,W
        SUBWF u16_3,1
        BTFSS 0x03,Carry
        DECF  u16_3+1,1
                        ;  u16a=u16/100;
        MOVF  u16_3,W
        MOVWF C12tmp
        MOVF  u16_3+1,W
        MOVWF C12tmp+1
        CLRF  C13rem
        MOVLW 16
        MOVWF C11cnt
m107    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   C12tmp,1
        RLF   C12tmp+1,1
        RLF   C13rem,1
        BTFSC 0x03,Carry
        GOTO  m108
        MOVLW 100
        SUBWF C13rem,W
        BTFSS 0x03,Carry
        GOTO  m109
m108    MOVLW 100
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF C13rem,1
        BSF   0x03,Carry
m109    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16a,1
        RLF   u16a+1,1
        DECFSZ C11cnt,1
        GOTO  m107
                        ;  u16b=100*u16a;
        MOVLW 100
        MOVWF C15tmp
        CLRF  u16b
        MOVLW 8
        MOVWF C14cnt
m110    BCF   0x03,Carry
        BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16b,1
        RLF   u16b+1,1
        RLF   C15tmp,1
        BTFSS 0x03,Carry
        GOTO  m111
        MOVF  u16a+1,W
        ADDWF u16b+1,1
        MOVF  u16a,W
        ADDWF u16b,1
        BTFSC 0x03,Carry
        INCF  u16b+1,1
m111    BCF   0x03,RP0
        BCF   0x03,RP1
        DECFSZ C14cnt,1
        GOTO  m110
                        ;  RS232TxPutChar((U8)u16a+'0');
        MOVLW 48
        ADDWF u16a,W
        CALL  RS232TxPutChar
                        ;
                        ;  u16-=u16b;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16b+1,W
        SUBWF u16_3+1,1
        MOVF  u16b,W
        SUBWF u16_3,1
        BTFSS 0x03,Carry
        DECF  u16_3+1,1
                        ;  u16a=u16/10;
        MOVF  u16_3,W
        MOVWF C17tmp
        MOVF  u16_3+1,W
        MOVWF C17tmp+1
        CLRF  C18rem
        MOVLW 16
        MOVWF C16cnt
m112    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   C17tmp,1
        RLF   C17tmp+1,1
        RLF   C18rem,1
        BTFSC 0x03,Carry
        GOTO  m113
        MOVLW 10
        SUBWF C18rem,W
        BTFSS 0x03,Carry
        GOTO  m114
m113    MOVLW 10
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF C18rem,1
        BSF   0x03,Carry
m114    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16a,1
        RLF   u16a+1,1
        DECFSZ C16cnt,1
        GOTO  m112
                        ;  u16b=u16a*10;
        MOVLW 10
        MOVWF C19cnt
        CLRF  u16b
        CLRF  u16b+1
m115    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16a+1,W
        ADDWF u16b+1,1
        MOVF  u16a,W
        ADDWF u16b,1
        BTFSC 0x03,Carry
        INCF  u16b+1,1
        DECFSZ C19cnt,1
        GOTO  m115
                        ;  RS232TxPutChar((U8)u16a+'0');
        MOVLW 48
        ADDWF u16a,W
        CALL  RS232TxPutChar
                        ;
                        ;  u16-=u16b;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16b+1,W
        SUBWF u16_3+1,1
        MOVF  u16b,W
        SUBWF u16_3,1
        BTFSS 0x03,Carry
        DECF  u16_3+1,1
                        ;  RS232TxPutChar((U8)u16+'0');
        MOVLW 48
        ADDWF u16_3,W
        GOTO  RS232TxPutChar
                        ;}
                        ;
                        ;static void RS232TxPutS16(S16 s16)
                        ;{
RS232TxPutS16
                        ;  char c='+';
        MOVLW 43
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c_9
                        ;
                        ;  if (s16<0)
        BTFSS s16+1,7
        GOTO  m116
                        ;  {
                        ;    s16=-s16;
        COMF  s16+1,1
        COMF  s16,1
        INCF  s16,1
        BTFSC 0x03,Zero_
        INCF  s16+1,1
                        ;    c='-';
        MOVLW 45
        MOVWF c_9
                        ;  }
                        ;  RS232TxPutChar(c);
m116    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c_9,W
        CALL  RS232TxPutChar
                        ;  RS232TxPutU16((U16)s16);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16,W
        MOVWF u16_3
        MOVF  s16+1,W
        MOVWF u16_3+1
        GOTO  RS232TxPutU16
                        ;}
                        ;
                        ;static void RS232TxPutFloat(float f)
                        ;{
RS232TxPutFloat
                        ;  float f2;
                        ;
                        ;  if (f<0.0)
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  f_2+2,W
        BTFSC 0x03,Zero_
        GOTO  m117
        BTFSS f_2+1,7
        GOTO  m117
                        ;  {
                        ;    f=0.0;
        CLRF  f_2
        CLRF  f_2+1
        CLRF  f_2+2
                        ;  }
                        ;
                        ;  RS232TxPutU16((U16)f);
m117    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  f_2,W
        MOVWF arg1f24
        MOVF  f_2+1,W
        MOVWF arg1f24+1
        MOVF  f_2+2,W
        MOVWF arg1f24+2
        BSF   0x0A,PA0
        CALL  _float24ToInt24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        MOVWF u16_3
        MOVF  rval_3+1,W
        MOVWF u16_3+1
        CALL  RS232TxPutU16
                        ;  RS232TxPutChar('.');
        MOVLW 46
        CALL  RS232TxPutChar
                        ;  f2=(U16)f; // remove integer part
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  f_2,W
        MOVWF arg1f24
        MOVF  f_2+1,W
        MOVWF arg1f24+1
        MOVF  f_2+2,W
        MOVWF arg1f24+2
        BSF   0x0A,PA0
        CALL  _float24ToInt24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg1f24+2
        BSF   0x0A,PA0
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval,W
        MOVWF f2
        MOVF  rval+1,W
        MOVWF f2+1
        MOVF  rval+2,W
        MOVWF f2+2
                        ;  f-=f2;
        MOVF  f_2,W
        MOVWF arg1f24
        MOVF  f_2+1,W
        MOVWF arg1f24+1
        MOVF  f_2+2,W
        MOVWF arg1f24+2
        MOVF  f2,W
        MOVWF arg2f24
        MOVF  f2+1,W
        MOVWF arg2f24+1
        MOVF  f2+2,W
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        CALL  _fsub24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF f_2
        MOVF  arg1f24+1,W
        MOVWF f_2+1
        MOVF  arg1f24+2,W
        MOVWF f_2+2
                        ;  f*=10000.0;
        MOVF  f_2,W
        MOVWF arg1f24
        MOVF  f_2+1,W
        MOVWF arg1f24+1
        MOVF  f_2+2,W
        MOVWF arg1f24+2
        MOVLW 64
        MOVWF arg2f24
        MOVLW 28
        MOVWF arg2f24+1
        MOVLW 140
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        CALL  _fmul24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF f_2
        MOVF  arg1f24+1,W
        MOVWF f_2+1
        MOVF  arg1f24+2,W
        MOVWF f_2+2
                        ;  RS232TxPutU16((U16)f);
        MOVF  f_2,W
        MOVWF arg1f24
        MOVF  f_2+1,W
        MOVWF arg1f24+1
        MOVF  f_2+2,W
        MOVWF arg1f24+2
        BSF   0x0A,PA0
        CALL  _float24ToInt24
        BCF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        MOVWF u16_3
        MOVF  rval_3+1,W
        MOVWF u16_3+1
        GOTO  RS232TxPutU16
                        ;}
                        ;
                        ;static BOOL RS232TxMsg(const char *psz)
                        ;{
RS232TxMsg
                        ;  while (*psz)
m118    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  psz,W
        MOVWF ci
        MOVF  psz+1,W
        MOVWF ci+1
        CALL  _const1
        XORLW 0
        BTFSC 0x03,Zero_
        GOTO  m119
                        ;  {
                        ;    BOOL b=RS232TxPutChar(*psz++);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  psz,W
        MOVWF ci
        MOVF  psz+1,W
        MOVWF ci+1
        CALL  _const1
        CALL  RS232TxPutChar
        BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   0x30,b
        BTFSC 0x03,Carry
        BSF   0x30,b
        INCF  psz,1
        BTFSC 0x03,Zero_
        INCF  psz+1,1
                        ;
                        ;    if (!b)
        BTFSC 0x30,b
        GOTO  m118
                        ;    {
                        ;      return FALSE;
        BCF   0x03,Carry
        RETURN
                        ;    }
                        ;  }
                        ;  return TRUE;
m119    BSF   0x03,Carry
        RETURN
                        ;}
                        ;
                        ;static void RS232TxPutError(void)
                        ;{
RS232TxPutError
                        ;  RS232TxPutChar('?');
        MOVLW 63
        CALL  RS232TxPutChar
                        ;  RS232TxPutChar('>');
        MOVLW 62
        CALL  RS232TxPutChar
                        ;  RS232TxPutChar('\r');
        MOVLW 13
        CALL  RS232TxPutChar
                        ;  RS232TxPutChar('\n');
        MOVLW 10
        GOTO  RS232TxPutChar
                        ;}
                        ;
                        ;static void RS232TxInit(void)
                        ;{
RS232TxInit
                        ;  _cRS232TxIn=0;
        BSF   0x03,RP0
        BCF   0x03,RP1
        CLRF  _cRS232TxIn
                        ;  _cRS232TxOut=0;
        CLRF  _cRS232TxOut
                        ;}
        RETURN
                        ;
                        ;static void RS232RxInit(void)
                        ;{
RS232RxInit
                        ;  _cRS232RxIn=0;
        BSF   0x03,RP0
        BCF   0x03,RP1
        CLRF  _cRS232RxIn
                        ;  _cRS232RxOut=0;
        CLRF  _cRS232RxOut
                        ;}
        RETURN
                        ;
                        ;#pragma codepage 3
        ORG 0x1A81
                        ;/**************** Degree from ADC conversion functions *******************/
                        ;//AK
                        ;
                        ;//static U16 ADCfromDEG(U16 u16DEG)
                        ;//{
                        ;//    U8    u8Index;
                        ;//    U16   u16Offset;
                        ;//    float fMult;
                        ;//    U16   u16Mod;
                        ;//    U16   u16ADC;
                        ;//    
                        ;//    u8Index   = u16DEG / 90;
                        ;//    u16Offset = _u16AzOffset[u8Index];
                        ;//    fMult     = _fAzMult[u8Index];
                        ;//    u16Mod    = u16DEG % 90;
                        ;//    u16ADC    = u16Mod * fMult;
                        ;//    u16ADC   += u16Offset;
                        ;//    
                        ;//    return u16ADC; 
                        ;//}
                        ;
                        ;static U16 DEGfromADC(U16 u16ADC)
                        ;{
DEGfromADC
                        ;    U8    u8Index;
                        ;    U16   u16Index;
                        ;    U16   u16Offset;
                        ;    float fMult;
                        ;    U16   u16DEG;
                        ;    U16   u16DEGOffset;
                        ;
                        ;    if (u16ADC <= _u16AzOffset[0]) 
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16ADC+1,W
        SUBWF _u16AzOffset+1,W
        BTFSS 0x03,Carry
        GOTO  m121
        BTFSS 0x03,Zero_
        GOTO  m120
        MOVF  u16ADC,W
        SUBWF _u16AzOffset,W
        BTFSS 0x03,Carry
        GOTO  m121
                        ;    {
                        ;        u16DEG = 0;
m120    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u16DEG
        CLRF  u16DEG+1
                        ;    }
                        ;    else
        GOTO  m129
                        ;    {
                        ;        if (u16ADC >= _u16AzOffset[4])
m121    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u16AzOffset+9,W
        SUBWF u16ADC+1,W
        BTFSS 0x03,Carry
        GOTO  m123
        BTFSS 0x03,Zero_
        GOTO  m122
        MOVF  _u16AzOffset+8,W
        SUBWF u16ADC,W
        BTFSS 0x03,Carry
        GOTO  m123
                        ;        {
                        ;            u16DEG = 360;
m122    MOVLW 104
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u16DEG
        MOVLW 1
        MOVWF u16DEG+1
                        ;        }
                        ;        else 
        GOTO  m129
                        ;        {
                        ;            for (u8Index = 0; u8Index < 5; u8Index++) {
m123    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u8Index
m124    MOVLW 5
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF u8Index,W
        BTFSC 0x03,Carry
        GOTO  m126
                        ;                u16Offset = _u16AzOffset[u8Index];
        BCF   0x03,Carry
        RLF   u8Index,W
        ADDLW 95
        MOVWF FSR
        BCF   0x03,IRP
        MOVF  INDF,W
        MOVWF u16Offset
        INCF  FSR,1
        MOVF  INDF,W
        MOVWF u16Offset+1
                        ;                if (u16Offset > u16ADC)
        MOVF  u16Offset+1,W
        SUBWF u16ADC+1,W
        BTFSS 0x03,Carry
        GOTO  m126
        BTFSS 0x03,Zero_
        GOTO  m125
        MOVF  u16Offset,W
        SUBWF u16ADC,W
        BTFSS 0x03,Carry
                        ;                    break;
        GOTO  m126
                        ;            }
m125    BCF   0x03,RP0
        BCF   0x03,RP1
        INCF  u8Index,1
        GOTO  m124
                        ;            u8Index--;
m126    BCF   0x03,RP0
        BCF   0x03,RP1
        DECF  u8Index,1
                        ;            u16Offset    = _u16AzOffset[u8Index];  
        BCF   0x03,Carry
        RLF   u8Index,W
        ADDLW 95
        MOVWF FSR
        BCF   0x03,IRP
        MOVF  INDF,W
        MOVWF u16Offset
        INCF  FSR,1
        MOVF  INDF,W
        MOVWF u16Offset+1
                        ;            fMult        = _fAzMult[u8Index];
        BCF   0x03,Carry
        RLF   u8Index,W
        ADDWF u8Index,W
        ADDLW 113
        MOVWF FSR
        BCF   0x03,IRP
        MOVF  INDF,W
        MOVWF fMult
        INCF  FSR,1
        MOVF  INDF,W
        MOVWF fMult+1
        INCF  FSR,1
        MOVF  INDF,W
        MOVWF fMult+2
                        ;            u16DEG       = u16ADC - u16Offset;
        MOVF  u16Offset+1,W
        SUBWF u16ADC+1,W
        MOVWF u16DEG+1
        MOVF  u16Offset,W
        SUBWF u16ADC,W
        MOVWF u16DEG
        BTFSS 0x03,Carry
        DECF  u16DEG+1,1
                        ;            u16DEG       = u16DEG / fMult;
        MOVF  u16DEG,W
        MOVWF arg1f24
        MOVF  u16DEG+1,W
        MOVWF arg1f24+1
        CLRF  arg1f24+2
        CALL  _int24ToFloat24
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  fMult,W
        MOVWF arg2f24
        MOVF  fMult+1,W
        MOVWF arg2f24+1
        MOVF  fMult+2,W
        MOVWF arg2f24+2
        CALL  _fdiv24
        CALL  _float24ToInt24
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        MOVWF u16DEG
        MOVF  rval_3+1,W
        MOVWF u16DEG+1
                        ;            u16Index     = u8Index;
        MOVF  u8Index,W
        MOVWF u16Index
        CLRF  u16Index+1
                        ;            u16DEGOffset = 90 * u16Index;
        MOVLW 90
        MOVWF C22tmp
        CLRF  u16DEGOffset
        MOVLW 8
        MOVWF C21cnt
m127    BCF   0x03,Carry
        BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16DEGOffset,1
        RLF   u16DEGOffset+1,1
        RLF   C22tmp,1
        BTFSS 0x03,Carry
        GOTO  m128
        MOVF  u16Index+1,W
        ADDWF u16DEGOffset+1,1
        MOVF  u16Index,W
        ADDWF u16DEGOffset,1
        BTFSC 0x03,Carry
        INCF  u16DEGOffset+1,1
m128    BCF   0x03,RP0
        BCF   0x03,RP1
        DECFSZ C21cnt,1
        GOTO  m127
                        ;            u16DEG       = u16DEG + u16DEGOffset;
        MOVF  u16DEGOffset+1,W
        ADDWF u16DEG+1,1
        MOVF  u16DEGOffset,W
        ADDWF u16DEG,1
        BTFSC 0x03,Carry
        INCF  u16DEG+1,1
                        ;        }
                        ;    }
                        ;    return u16DEG;
m129    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16DEG,W
        RETURN
                        ;}
                        ;/***************** LCD functions *****************/
                        ;
                        ;static void LCDWriteNibble(unsigned char uc) /* RS must be set/reset before calling */
                        ;{
LCDWriteNibble
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF uc_2
                        ;  uc<<=2; /* Align with bits 2-5 */
        BCF   0x03,Carry
        RLF   uc_2,1
        BCF   0x03,Carry
        RLF   uc_2,1
                        ;  LCD_RW=0;
        BCF   PORTB,1
                        ;  TRISA=0b.1100.0011; /* Set to output bits 2-5 */
        MOVLW 195
        BSF   0x03,RP0
        MOVWF TRISA
                        ;  PORTA=uc;
        BCF   0x03,RP0
        MOVF  uc_2,W
        MOVWF PORTA
                        ;  LCD_E=1;
        BSF   PORTB,2
                        ;  nop();
        NOP  
                        ;  nop();
        NOP  
                        ;  LCD_E=0;
        BCF   PORTB,2
                        ;  LCD_RW=1;
        BSF   PORTB,1
                        ;  TRISA=0b.1111.1111; /* Set to input bits 2-5 */  
        MOVLW 255
        BSF   0x03,RP0
        MOVWF TRISA
                        ;}
        RETURN
                        ;
                        ;static unsigned char LCDReadByte(void) /* RS must be set/reset before calling */
                        ;{
LCDReadByte
                        ;  unsigned char uc,uc2;
                        ;
                        ;  LCD_RW=1;
        BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   PORTB,1
                        ;  LCD_E=1;
        BSF   PORTB,2
                        ;  nop();
        NOP  
                        ;  nop();
        NOP  
                        ;  uc=PORTA;
        MOVF  PORTA,W
        MOVWF uc_3
                        ;  LCD_E=0;
        BCF   PORTB,2
                        ;  uc<<=2;
        BCF   0x03,Carry
        RLF   uc_3,1
        BCF   0x03,Carry
        RLF   uc_3,1
                        ;  uc&=0xF0;
        MOVLW 240
        ANDWF uc_3,1
                        ;  LCD_E=1;
        BSF   PORTB,2
                        ;  nop();
        NOP  
                        ;  nop();
        NOP  
                        ;  uc2=PORTA;
        MOVF  PORTA,W
        MOVWF uc2_2
                        ;  LCD_E=0;
        BCF   PORTB,2
                        ;  uc2>>=2;
        BCF   0x03,Carry
        RRF   uc2_2,1
        BCF   0x03,Carry
        RRF   uc2_2,1
                        ;  uc2&=0x0F;
        MOVLW 15
        ANDWF uc2_2,1
                        ;  uc|=uc2;
        MOVF  uc2_2,W
        IORWF uc_3,1
                        ;  return uc;
        MOVF  uc_3,W
        RETURN
                        ;}
                        ;
                        ;static BOOL LCDWaitReady(void)
                        ;{
LCDWaitReady
                        ;  if (_bLCDActive)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x4E,_bLCDActive
        GOTO  m132
                        ;  {
                        ;    U16 u16=TimerGet();
        BCF   0x0A,PA0
        CALL  TimerGet
        BSF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16,W
        MOVWF u16_4
        MOVF  u16+1,W
        MOVWF u16_4+1
                        ;
                        ;    while (LCDReadByte() & 0x80)
m130    CALL  LCDReadByte
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m132
                        ;    {
                        ;      U16 u16Diff=TimerGet()-u16;
        BCF   0x0A,PA0
        CALL  TimerGet
        BSF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_4+1,W
        SUBWF u16+1,W
        MOVWF u16Diff+1
        MOVF  u16_4,W
        SUBWF u16,W
        MOVWF u16Diff
        BTFSS 0x03,Carry
        DECF  u16Diff+1,1
                        ;
                        ;      if (u16Diff>5) // Timeout
        MOVF  u16Diff+1,W
        BTFSS 0x03,Zero_
        GOTO  m131
        MOVLW 6
        SUBWF u16Diff,W
        BTFSS 0x03,Carry
        GOTO  m130
                        ;      {
                        ;        _bLCDActive=FALSE;
m131    BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   0x4E,_bLCDActive
                        ;        return FALSE;
        BCF   0x03,Carry
        RETURN
                        ;      }
                        ;    }
                        ;  }
                        ;  return TRUE;
m132    BSF   0x03,Carry
        RETURN
                        ;}
                        ;
                        ;static void LCDWriteData(unsigned char uc)
                        ;{
LCDWriteData
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF uc_4
                        ;  if (_bLCDActive)
        BTFSS 0x4E,_bLCDActive
        GOTO  m133
                        ;  {
                        ;    LCD_RS=0;
        BCF   PORTB,0
                        ;    LCDWaitReady();
        CALL  LCDWaitReady
                        ;    LCD_RS=1;
        BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   PORTB,0
                        ;
                        ;    LCDWriteNibble(uc>>4);
        SWAPF uc_4,W
        ANDLW 15
        CALL  LCDWriteNibble
                        ;    LCDWriteNibble(uc);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  uc_4,W
        GOTO  LCDWriteNibble
                        ;  }
                        ;}
m133    RETURN
                        ;
                        ;//static unsigned char LCDReadData(void)
                        ;//{
                        ;//  if (_bLCDActive)
                        ;//  {
                        ;//    LCD_RS=0;
                        ;//    LCDWaitReady();
                        ;//    LCD_RS=1;
                        ;//    return LCDReadByte();
                        ;//  }
                        ;//  else
                        ;//  {
                        ;//    return 0;
                        ;//  }
                        ;//}
                        ;
                        ;static void LCDCommand(unsigned char uc)
                        ;{
LCDCommand
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF uc_5
                        ;  if (_bLCDActive)
        BTFSS 0x4E,_bLCDActive
        GOTO  m134
                        ;  {
                        ;    LCD_RS=0; /* Instruction mode */
        BCF   PORTB,0
                        ;    LCDWaitReady();
        CALL  LCDWaitReady
                        ;
                        ;    LCDWriteNibble(uc>>4);
        BCF   0x03,RP0
        BCF   0x03,RP1
        SWAPF uc_5,W
        ANDLW 15
        CALL  LCDWriteNibble
                        ;    LCDWriteNibble(uc);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  uc_5,W
        GOTO  LCDWriteNibble
                        ;  }
                        ;}
m134    RETURN
                        ;
                        ;static void LCDSetCursor(unsigned char ucPos)
                        ;{
LCDSetCursor
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF ucPos
                        ;  if (_bLCDActive)
        BTFSS 0x4E,_bLCDActive
        GOTO  m135
                        ;  {
                        ;    LCDCommand(0x80 | ucPos);
        MOVLW 128
        IORWF ucPos,W
        GOTO  LCDCommand
                        ;  }
                        ;}
m135    RETURN
                        ;
                        ;static void LCDClear(void)
                        ;{
LCDClear
                        ;  if (_bLCDActive)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x4E,_bLCDActive
        GOTO  m136
                        ;  {
                        ;    LCDCommand(0x01);
        MOVLW 1
        GOTO  LCDCommand
                        ;  }
                        ;}
m136    RETURN
                        ;
                        ;static void LCDInit(void)
                        ;{
LCDInit
                        ;  _bLCDActive=TRUE; // Let's be optimistic
        BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x4E,_bLCDActive
                        ;  _u16LCDTimer=0; // Initialize timer
        CLRF  _u16LCDTimer
        CLRF  _u16LCDTimer+1
                        ;
                        ;  LCD_E=0;
        BCF   PORTB,2
                        ;  LCD_RS=0;
        BCF   PORTB,0
                        ;  Delay(150);
        MOVLW 150
        BCF   0x0A,PA0
        CALL  Delay
        BSF   0x0A,PA0
                        ;  LCDWriteNibble(3);
        MOVLW 3
        CALL  LCDWriteNibble
                        ;  Delay(50);
        MOVLW 50
        BCF   0x0A,PA0
        CALL  Delay
        BSF   0x0A,PA0
                        ;  LCDWriteNibble(3);
        MOVLW 3
        CALL  LCDWriteNibble
                        ;  Delay(2);
        MOVLW 2
        BCF   0x0A,PA0
        CALL  Delay
        BSF   0x0A,PA0
                        ;  LCDWriteNibble(3);
        MOVLW 3
        CALL  LCDWriteNibble
                        ;  Delay(2);
        MOVLW 2
        BCF   0x0A,PA0
        CALL  Delay
        BSF   0x0A,PA0
                        ;  LCDWriteNibble(2);
        MOVLW 2
        CALL  LCDWriteNibble
                        ;  Delay(2);
        MOVLW 2
        BCF   0x0A,PA0
        CALL  Delay
        BSF   0x0A,PA0
                        ;  LCDCommand(0b.0010.1000);
        MOVLW 40
        CALL  LCDCommand
                        ;  LCDCommand(0b.0000.1000);
        MOVLW 8
        CALL  LCDCommand
                        ;  LCDCommand(0b.0000.0001);
        MOVLW 1
        CALL  LCDCommand
                        ;  LCDCommand(0b.0000.0110);
        MOVLW 6
        CALL  LCDCommand
                        ;  LCDCommand(0b.0000.1111);
        MOVLW 15
        GOTO  LCDCommand
                        ;}
                        ;
                        ;static void LCDPutCh(char c)
                        ;{
LCDPutCh
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c_10
                        ;  LCDWriteData(c);
        MOVF  c_10,W
        GOTO  LCDWriteData
                        ;}
                        ;
                        ;static void LCDPutU16(U16 u16,U8 u8NumDigs)
                        ;{
LCDPutU16
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8NumDigs
                        ;  if (_bLCDActive)
        BTFSS 0x4E,_bLCDActive
        GOTO  m160
                        ;  {
                        ;    U16 u16a;
                        ;    U16 u16b;
                        ;
                        ;    u16a=u16/1000;
        MOVF  u16_5,W
        MOVWF C24tmp
        MOVF  u16_5+1,W
        MOVWF C24tmp+1
        CLRF  C25rem
        CLRF  C25rem+1
        MOVLW 16
        MOVWF C23cnt
m137    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   C24tmp,1
        RLF   C24tmp+1,1
        RLF   C25rem,1
        RLF   C25rem+1,1
        MOVLW 3
        SUBWF C25rem+1,W
        BTFSS 0x03,Carry
        GOTO  m139
        BTFSS 0x03,Zero_
        GOTO  m138
        MOVLW 232
        SUBWF C25rem,W
        BTFSS 0x03,Carry
        GOTO  m139
m138    MOVLW 3
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF C25rem+1,1
        MOVLW 232
        SUBWF C25rem,1
        BTFSS 0x03,Carry
        DECF  C25rem+1,1
        BSF   0x03,Carry
m139    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16a_2,1
        RLF   u16a_2+1,1
        DECFSZ C23cnt,1
        GOTO  m137
                        ;    u16b=u16a*1000;
        MOVF  u16a_2,W
        MOVWF C27tmp
        MOVF  u16a_2+1,W
        MOVWF C27tmp+1
        MOVLW 16
        MOVWF C26cnt
m140    BCF   0x03,Carry
        BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16b_2,1
        RLF   u16b_2+1,1
        RLF   C27tmp,1
        RLF   C27tmp+1,1
        BTFSS 0x03,Carry
        GOTO  m141
        MOVLW 3
        ADDWF u16b_2+1,1
        MOVLW 232
        ADDWF u16b_2,1
        BTFSC 0x03,Carry
        INCF  u16b_2+1,1
m141    BCF   0x03,RP0
        BCF   0x03,RP1
        DECFSZ C26cnt,1
        GOTO  m140
                        ;
                        ;    if (u8NumDigs>=5)
        MOVLW 5
        SUBWF u8NumDigs,W
        BTFSS 0x03,Carry
        GOTO  m142
                        ;    {
                        ;      LCDPutCh((U8)u16a+'0');
        MOVLW 48
        ADDWF u16a_2,W
        CALL  LCDPutCh
                        ;    }
                        ;
                        ;    u16-=u16b;
m142    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16b_2+1,W
        SUBWF u16_5+1,1
        MOVF  u16b_2,W
        SUBWF u16_5,1
        BTFSS 0x03,Carry
        DECF  u16_5+1,1
                        ;    u16a=u16/1000;
        MOVF  u16_5,W
        MOVWF C29tmp
        MOVF  u16_5+1,W
        MOVWF C29tmp+1
        CLRF  C30rem
        CLRF  C30rem+1
        MOVLW 16
        MOVWF C28cnt
m143    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   C29tmp,1
        RLF   C29tmp+1,1
        RLF   C30rem,1
        RLF   C30rem+1,1
        MOVLW 3
        SUBWF C30rem+1,W
        BTFSS 0x03,Carry
        GOTO  m145
        BTFSS 0x03,Zero_
        GOTO  m144
        MOVLW 232
        SUBWF C30rem,W
        BTFSS 0x03,Carry
        GOTO  m145
m144    MOVLW 3
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF C30rem+1,1
        MOVLW 232
        SUBWF C30rem,1
        BTFSS 0x03,Carry
        DECF  C30rem+1,1
        BSF   0x03,Carry
m145    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16a_2,1
        RLF   u16a_2+1,1
        DECFSZ C28cnt,1
        GOTO  m143
                        ;    u16b=u16a*1000;
        MOVF  u16a_2,W
        MOVWF C32tmp
        MOVF  u16a_2+1,W
        MOVWF C32tmp+1
        MOVLW 16
        MOVWF C31cnt
m146    BCF   0x03,Carry
        BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16b_2,1
        RLF   u16b_2+1,1
        RLF   C32tmp,1
        RLF   C32tmp+1,1
        BTFSS 0x03,Carry
        GOTO  m147
        MOVLW 3
        ADDWF u16b_2+1,1
        MOVLW 232
        ADDWF u16b_2,1
        BTFSC 0x03,Carry
        INCF  u16b_2+1,1
m147    BCF   0x03,RP0
        BCF   0x03,RP1
        DECFSZ C31cnt,1
        GOTO  m146
                        ;    if (u8NumDigs>=4)
        MOVLW 4
        SUBWF u8NumDigs,W
        BTFSS 0x03,Carry
        GOTO  m148
                        ;    {
                        ;      LCDPutCh((U8)u16a+'0');
        MOVLW 48
        ADDWF u16a_2,W
        CALL  LCDPutCh
                        ;    }
                        ;
                        ;    u16-=u16b;
m148    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16b_2+1,W
        SUBWF u16_5+1,1
        MOVF  u16b_2,W
        SUBWF u16_5,1
        BTFSS 0x03,Carry
        DECF  u16_5+1,1
                        ;    u16a=u16/100;
        MOVF  u16_5,W
        MOVWF C34tmp
        MOVF  u16_5+1,W
        MOVWF C34tmp+1
        CLRF  C35rem
        MOVLW 16
        MOVWF C33cnt
m149    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   C34tmp,1
        RLF   C34tmp+1,1
        RLF   C35rem,1
        BTFSC 0x03,Carry
        GOTO  m150
        MOVLW 100
        SUBWF C35rem,W
        BTFSS 0x03,Carry
        GOTO  m151
m150    MOVLW 100
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF C35rem,1
        BSF   0x03,Carry
m151    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16a_2,1
        RLF   u16a_2+1,1
        DECFSZ C33cnt,1
        GOTO  m149
                        ;    u16b=100*u16a;
        MOVLW 100
        MOVWF C37tmp
        CLRF  u16b_2
        MOVLW 8
        MOVWF C36cnt
m152    BCF   0x03,Carry
        BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16b_2,1
        RLF   u16b_2+1,1
        RLF   C37tmp,1
        BTFSS 0x03,Carry
        GOTO  m153
        MOVF  u16a_2+1,W
        ADDWF u16b_2+1,1
        MOVF  u16a_2,W
        ADDWF u16b_2,1
        BTFSC 0x03,Carry
        INCF  u16b_2+1,1
m153    BCF   0x03,RP0
        BCF   0x03,RP1
        DECFSZ C36cnt,1
        GOTO  m152
                        ;    if (u8NumDigs>=3)
        MOVLW 3
        SUBWF u8NumDigs,W
        BTFSS 0x03,Carry
        GOTO  m154
                        ;    {
                        ;      LCDPutCh((U8)u16a+'0');
        MOVLW 48
        ADDWF u16a_2,W
        CALL  LCDPutCh
                        ;    }
                        ;
                        ;    u16-=u16b;
m154    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16b_2+1,W
        SUBWF u16_5+1,1
        MOVF  u16b_2,W
        SUBWF u16_5,1
        BTFSS 0x03,Carry
        DECF  u16_5+1,1
                        ;    u16a=u16/10;
        MOVF  u16_5,W
        MOVWF C39tmp
        MOVF  u16_5+1,W
        MOVWF C39tmp+1
        CLRF  C40rem
        MOVLW 16
        MOVWF C38cnt
m155    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   C39tmp,1
        RLF   C39tmp+1,1
        RLF   C40rem,1
        BTFSC 0x03,Carry
        GOTO  m156
        MOVLW 10
        SUBWF C40rem,W
        BTFSS 0x03,Carry
        GOTO  m157
m156    MOVLW 10
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF C40rem,1
        BSF   0x03,Carry
m157    BCF   0x03,RP0
        BCF   0x03,RP1
        RLF   u16a_2,1
        RLF   u16a_2+1,1
        DECFSZ C38cnt,1
        GOTO  m155
                        ;    u16b=u16a*10;
        MOVLW 10
        MOVWF C41cnt
        CLRF  u16b_2
        CLRF  u16b_2+1
m158    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16a_2+1,W
        ADDWF u16b_2+1,1
        MOVF  u16a_2,W
        ADDWF u16b_2,1
        BTFSC 0x03,Carry
        INCF  u16b_2+1,1
        DECFSZ C41cnt,1
        GOTO  m158
                        ;    if (u8NumDigs>=2)
        MOVLW 2
        SUBWF u8NumDigs,W
        BTFSS 0x03,Carry
        GOTO  m159
                        ;    {
                        ;      LCDPutCh((U8)u16a+'0');
        MOVLW 48
        ADDWF u16a_2,W
        CALL  LCDPutCh
                        ;    }
                        ;
                        ;    u16-=u16b;
m159    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16b_2+1,W
        SUBWF u16_5+1,1
        MOVF  u16b_2,W
        SUBWF u16_5,1
        BTFSS 0x03,Carry
        DECF  u16_5+1,1
                        ;    if (u8NumDigs>=1)
        MOVF  u8NumDigs,1
        BTFSC 0x03,Zero_
        GOTO  m160
                        ;    {
                        ;      LCDPutCh((U8)u16+'0');
        MOVLW 48
        ADDWF u16_5,W
        GOTO  LCDPutCh
                        ;    }
                        ;  }
                        ;}
m160    RETURN
                        ;
                        ;static void LCDPutS16(S16 s16,U8 u8NumDigs)
                        ;{
LCDPutS16
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8NumDigs_2
                        ;  char c='+';
        MOVLW 43
        MOVWF c_11
                        ;
                        ;  if (s16<0)
        BTFSS s16_2+1,7
        GOTO  m161
                        ;  {
                        ;    s16=-s16;
        COMF  s16_2+1,1
        COMF  s16_2,1
        INCF  s16_2,1
        BTFSC 0x03,Zero_
        INCF  s16_2+1,1
                        ;    c='-';
        MOVLW 45
        MOVWF c_11
                        ;  }
                        ;  LCDPutCh(c);
m161    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c_11,W
        CALL  LCDPutCh
                        ;  LCDPutU16((U16)s16,u8NumDigs);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16_2,W
        MOVWF u16_5
        MOVF  s16_2+1,W
        MOVWF u16_5+1
        MOVF  u8NumDigs_2,W
        GOTO  LCDPutU16
                        ;}
                        ;
                        ;// Display adhoc string message on the LCD
                        ;static void LCDMsg(const char *psz)
                        ;{
LCDMsg
                        ;  if (_bLCDActive)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x4E,_bLCDActive
        GOTO  m163
                        ;  {
                        ;    while (*psz)
m162    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  psz_2,W
        MOVWF ci
        MOVF  psz_2+1,W
        MOVWF ci+1
        BCF   0x0A,PA0
        CALL  _const1
        BSF   0x0A,PA0
        XORLW 0
        BTFSC 0x03,Zero_
        GOTO  m163
                        ;    {
                        ;      LCDPutCh(*psz++);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  psz_2,W
        MOVWF ci
        MOVF  psz_2+1,W
        MOVWF ci+1
        BCF   0x0A,PA0
        CALL  _const1
        BSF   0x0A,PA0
        CALL  LCDPutCh
        BCF   0x03,RP0
        BCF   0x03,RP1
        INCF  psz_2,1
        BTFSC 0x03,Zero_
        INCF  psz_2+1,1
                        ;    }
        GOTO  m162
                        ;  }
                        ;}
m163    RETURN
                        ;
                        ;// Display signed Azimuth
                        ;static void LCDPutAz(S16 s16)
                        ;{
LCDPutAz
                        ;  if (_bLCDActive)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x4E,_bLCDActive
        GOTO  m167
                        ;  {
                        ;    if (FLAG_SNS)
        BTFSS _u16Flags,1
        GOTO  m166
                        ;    {
                        ;      if (s16<180)
        BTFSC s16_3+1,7
        GOTO  m164
        MOVF  s16_3+1,W
        BTFSS 0x03,Zero_
        GOTO  m165
        MOVLW 180
        SUBWF s16_3,W
        BTFSC 0x03,Carry
        GOTO  m165
                        ;      {
                        ;        LCDPutS16(s16+180,3);
m164    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16_3+1,W
        MOVWF s16_2+1
        MOVLW 180
        ADDWF s16_3,W
        MOVWF s16_2
        BTFSC 0x03,Carry
        INCF  s16_2+1,1
        MOVLW 3
        CALL  LCDPutS16
                        ;      }
                        ;      else
        GOTO  m167
                        ;      {
                        ;        LCDPutS16(s16-180,3);
m165    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16_3+1,W
        MOVWF s16_2+1
        MOVLW 180
        SUBWF s16_3,W
        MOVWF s16_2
        BTFSS 0x03,Carry
        DECF  s16_2+1,1
        MOVLW 3
        CALL  LCDPutS16
                        ;      }
                        ;    }
                        ;    else
        GOTO  m167
                        ;    {
                        ;      LCDPutS16(s16,3);
m166    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16_3,W
        MOVWF s16_2
        MOVF  s16_3+1,W
        MOVWF s16_2+1
        MOVLW 3
        GOTO  LCDPutS16
                        ;    }
                        ;  }
                        ;}
m167    RETURN
                        ;
                        ; static void LCDWelcome(void)
                        ;{
LCDWelcome
                        ;  if (_bLCDActive)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x4E,_bLCDActive
        GOTO  m169
                        ;  {
                        ;    U16 u16;
                        ;    LCDClear();
        CALL  LCDClear
                        ;    LCDMsg("LVB Tracker");
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  psz_2
        CLRF  psz_2+1
        CALL  LCDMsg
                        ;    LCDSetCursor(40);
        MOVLW 40
        CALL  LCDSetCursor
                        ;    LCDMsg("Firmware v1.2");
        MOVLW 12
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        CALL  LCDMsg
                        ;    for (u16=0;u16<200;u16++) // wait 2s
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u16_6
        CLRF  u16_6+1
m168    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_6+1,W
        BTFSS 0x03,Zero_
        GOTO  LCDClear
        MOVLW 200
        SUBWF u16_6,W
        BTFSC 0x03,Carry
        GOTO  LCDClear
                        ;    {
                        ;      Delay(100);
        MOVLW 100
        BCF   0x0A,PA0
        CALL  Delay
        BSF   0x0A,PA0
                        ;    }
        BCF   0x03,RP0
        BCF   0x03,RP1
        INCF  u16_6,1
        BTFSC 0x03,Zero_
        INCF  u16_6+1,1
        GOTO  m168
                        ;    LCDClear();
                        ;  }
                        ;}
m169    RETURN
                        ;
                        ;// Update LCD display
                        ;static void LCDUpdate(void)
                        ;{
LCDUpdate
                        ;  U16 u16Timer=TimerGet();
        BCF   0x0A,PA0
        CALL  TimerGet
        BSF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16,W
        MOVWF u16Timer
        MOVF  u16+1,W
        MOVWF u16Timer+1
                        ;
                        ;  u16Timer-=_u16LCDTimer;
        MOVF  _u16LCDTimer+1,W
        SUBWF u16Timer+1,1
        MOVF  _u16LCDTimer,W
        SUBWF u16Timer,1
        BTFSS 0x03,Carry
        DECF  u16Timer+1,1
                        ;
                        ;  if (_bLCDActive && u16Timer>LCD_TIMERWAIT)
        BTFSS 0x4E,_bLCDActive
        GOTO  m171
        MOVF  u16Timer+1,W
        BTFSS 0x03,Zero_
        GOTO  m170
        MOVLW 201
        SUBWF u16Timer,W
        BTFSS 0x03,Carry
        GOTO  m171
                        ;  {
                        ;    S16 s16;
                        ;
                        ;    _u16LCDTimer+=u16Timer;
m170    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Timer+1,W
        ADDWF _u16LCDTimer+1,1
        MOVF  u16Timer,W
        ADDWF _u16LCDTimer,1
        BTFSC 0x03,Carry
        INCF  _u16LCDTimer+1,1
                        ;
                        ;    LCDSetCursor(0);
        MOVLW 0
        CALL  LCDSetCursor
                        ;
                        ;    LCDMsg("Az ");
        MOVLW 26
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        CALL  LCDMsg
                        ; 
                        ;    //AK
                        ;    s16=(S16)ADCGet(0);
        MOVLW 0
        BCF   0x0A,PA0
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF s16_4
        MOVF  u16Sum+1,W
        MOVWF s16_4+1
                        ;    //s16-=(S16)_u16AzOff;
                        ;    //s16/=_fAzMul;
                        ;    s16 = (S16)DEGfromADC(s16);
        MOVF  s16_4,W
        MOVWF u16ADC
        MOVF  s16_4+1,W
        MOVWF u16ADC+1
        CALL  DEGfromADC
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16DEG,W
        MOVWF s16_4
        MOVF  u16DEG+1,W
        MOVWF s16_4+1
                        ;
                        ;    LCDPutAz(s16);
        MOVF  s16_4,W
        MOVWF s16_3
        MOVF  s16_4+1,W
        MOVWF s16_3+1
        CALL  LCDPutAz
                        ;    LCDPutCh(0xDF);
        MOVLW 223
        CALL  LCDPutCh
                        ;
                        ;    LCDMsg(" (");
        MOVLW 34
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        CALL  LCDMsg
                        ;
                        ;    LCDPutAz(_s16AzTarget);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _s16AzTarget,W
        MOVWF s16_3
        MOVF  _s16AzTarget+1,W
        MOVWF s16_3+1
        CALL  LCDPutAz
                        ;    LCDPutCh(0xDF);
        MOVLW 223
        CALL  LCDPutCh
                        ;    LCDPutCh(')');
        MOVLW 41
        CALL  LCDPutCh
                        ;
                        ;    LCDSetCursor(40);
        MOVLW 40
        CALL  LCDSetCursor
                        ;
                        ;    LCDMsg("El ");
        MOVLW 30
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        CALL  LCDMsg
                        ; 
                        ;    s16=(S16)ADCGet(1);
        MOVLW 1
        BCF   0x0A,PA0
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF s16_4
        MOVF  u16Sum+1,W
        MOVWF s16_4+1
                        ;    s16-=(S16)_u16ElOff;
        MOVF  _u16ElOff+1,W
        SUBWF s16_4+1,1
        MOVF  _u16ElOff,W
        SUBWF s16_4,1
        BTFSS 0x03,Carry
        DECF  s16_4+1,1
                        ;    s16/=_fElMul;
        MOVF  s16_4,W
        MOVWF arg1f24
        MOVF  s16_4+1,W
        MOVWF arg1f24+1
        CLRF  arg1f24+2
        BTFSC arg1f24+1,7
        DECF  arg1f24+2,1
        CALL  _int24ToFloat24
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fElMul,W
        MOVWF arg2f24
        MOVF  _fElMul+1,W
        MOVWF arg2f24+1
        MOVF  _fElMul+2,W
        MOVWF arg2f24+2
        CALL  _fdiv24
        CALL  _float24ToInt24
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        MOVWF s16_4
        MOVF  rval_3+1,W
        MOVWF s16_4+1
                        ;
                        ;    LCDPutS16(s16,3);
        MOVF  s16_4,W
        MOVWF s16_2
        MOVF  s16_4+1,W
        MOVWF s16_2+1
        MOVLW 3
        CALL  LCDPutS16
                        ;    LCDPutCh(0xDF);
        MOVLW 223
        CALL  LCDPutCh
                        ;
                        ;    LCDMsg(" (");
        MOVLW 34
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        CALL  LCDMsg
                        ;
                        ;    LCDPutS16((S16)_s16ElTarget,3);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _s16ElTarget,W
        MOVWF s16_2
        MOVF  _s16ElTarget+1,W
        MOVWF s16_2+1
        MOVLW 3
        CALL  LCDPutS16
                        ;    LCDPutCh(0xDF);
        MOVLW 223
        CALL  LCDPutCh
                        ;    LCDPutCh(')');
        MOVLW 41
        CALL  LCDPutCh
                        ;  }
                        ;}
m171    BCF   0x0A,PA0
        BCF   0x0A,PA1
        GOTO  m282
                        ;
                        ;#ifdef __CC5X__
                        ;//#pragma codepage 2
                        ;#pragma codepage 2
        ORG 0x14D4
                        ;#endif
                        ;
                        ;/***************** EEPROM functions ******************/
                        ;
                        ;// Read a byte of EEPROM data
                        ;static U8 EEReadByte(U8 u8Addr)
                        ;{
EEReadByte
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr
                        ;  EEADR=u8Addr;
        MOVF  u8Addr,W
        BSF   0x03,RP1
        MOVWF EEADR
                        ;  EEPGD=0; // Clear EEPGD
        BSF   0x03,RP0
        BCF   0x18C,EEPGD
                        ;  EECON1.0=1; // Set control bit RD
        BSF   EECON1,0
                        ;  return EEDATA;
        BCF   0x03,RP0
        MOVF  EEDATA,W
        RETURN
                        ;}
                        ;
                        ;// Read a Word of EEPROM data
                        ;static U16 EEReadWord(U8 u8Addr)
                        ;{
EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_2
                        ;  U16 u16;
                        ;
                        ;  u16=EEReadByte(u8Addr+1);
        INCF  u8Addr_2,W
        CALL  EEReadByte
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u16_7
        CLRF  u16_7+1
                        ;  u16<<=8;
        MOVF  u16_7,W
        MOVWF u16_7+1
        CLRF  u16_7
                        ;  u16+=EEReadByte(u8Addr);
        MOVF  u8Addr_2,W
        CALL  EEReadByte
        BCF   0x03,RP0
        BCF   0x03,RP1
        ADDWF u16_7,1
        BTFSC 0x03,Carry
        INCF  u16_7+1,1
                        ;
                        ;  return u16;
        MOVF  u16_7,W
        RETURN
                        ;}
                        ;
                        ;static float EEReadFloat(U8 u8Addr)
                        ;{
EEReadFloat
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_3
                        ;  FLOATUNION fu;
                        ;
                        ;  fu.fs.u8Lo=EEReadByte(u8Addr);
        MOVF  u8Addr_3,W
        CALL  EEReadByte
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF fu
                        ;  fu.fs.u8Mid=EEReadByte(u8Addr+1);
        INCF  u8Addr_3,W
        CALL  EEReadByte
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF fu+1
                        ;  fu.fs.u8Hi=EEReadByte(u8Addr+2);
        MOVLW 2
        ADDWF u8Addr_3,W
        CALL  EEReadByte
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF fu+2
                        ;
                        ;  return fu.f;
        MOVF  fu,W
        MOVWF C43tmp
        MOVF  fu+1,W
        MOVWF C43tmp+1
        MOVF  fu+2,W
        MOVWF C43tmp+2
        MOVF  C43tmp,W
        RETURN
                        ;}
                        ;
                        ;// Write a byte of EEPROM data
                        ;static void EEWriteByte(U8 u8Addr,U8 u8Data)
                        ;{
EEWriteByte
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Data
                        ;  EEADR=u8Addr;
        MOVF  u8Addr_4,W
        BSF   0x03,RP1
        MOVWF EEADR
                        ;  EEDATA=u8Data;
        BCF   0x03,RP1
        MOVF  u8Data,W
        BSF   0x03,RP1
        MOVWF EEDATA
                        ;  EEPGD=0; // Clear EEPGD
        BSF   0x03,RP0
        BCF   0x18C,EEPGD
                        ;  WREN=1;
        BSF   0x18C,WREN
                        ;  GIE=0;
        BCF   0x0B,GIE
                        ;  EECON2=0x55;
        MOVLW 85
        MOVWF EECON2
                        ;  EECON2=0xAA;
        MOVLW 170
        MOVWF EECON2
                        ;  WR=1;
        BSF   0x18C,WR
                        ;  GIE=1;
        BSF   0x0B,GIE
                        ;  WREN=0;  
        BCF   0x18C,WREN
                        ;  while (WR)
m172    BSF   0x03,RP0
        BSF   0x03,RP1
        BTFSC 0x18C,WR
                        ;  {
                        ;  }  
        GOTO  m172
                        ;}
        RETURN
                        ;
                        ;// Write a Word of EEPROM data
                        ;static void EEWriteWord(U8 u8Addr,U16 u16Data)
                        ;{
EEWriteWord
                        ;  EEWriteByte(u8Addr,(U8)u16Data);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8Addr_5,W
        MOVWF u8Addr_4
        MOVF  u16Data,W
        CALL  EEWriteByte
                        ;  u16Data>>=8;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Data+1,W
        MOVWF u16Data
        CLRF  u16Data+1
                        ;  EEWriteByte(u8Addr+1,u16Data);
        INCF  u8Addr_5,W
        MOVWF u8Addr_4
        MOVF  u16Data,W
        GOTO  EEWriteByte
                        ;}
                        ;
                        ;// Write a Float of EEPROM data
                        ;static void EEWriteFloat(U8 u8Addr,float f)
                        ;{
EEWriteFloat
                        ;  FLOATUNION *pfu=&f;
        MOVLW 49
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pfu
                        ;  EEWriteByte(u8Addr,pfu->fs.u8Lo);
        MOVF  u8Addr_6,W
        MOVWF u8Addr_4
        BCF   0x03,IRP
        MOVF  pfu,W
        MOVWF FSR
        MOVF  INDF,W
        CALL  EEWriteByte
                        ;  EEWriteByte(u8Addr+1,pfu->fs.u8Mid);
        BCF   0x03,RP0
        BCF   0x03,RP1
        INCF  u8Addr_6,W
        MOVWF u8Addr_4
        INCF  pfu,W
        MOVWF FSR
        BCF   0x03,IRP
        MOVF  INDF,W
        CALL  EEWriteByte
                        ;  EEWriteByte(u8Addr+2,pfu->fs.u8Hi);
        MOVLW 2
        BCF   0x03,RP0
        BCF   0x03,RP1
        ADDWF u8Addr_6,W
        MOVWF u8Addr_4
        MOVLW 2
        ADDWF pfu,W
        MOVWF FSR
        BCF   0x03,IRP
        MOVF  INDF,W
        GOTO  EEWriteByte
                        ;}
                        ;
                        ;// Calculate the EEPROM checksum
                        ;static U16 EECalcSum(void)
                        ;{
EECalcSum
                        ;  U8 u8;
                        ;  U16 u16Sum=0;
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u16Sum_2
        CLRF  u16Sum_2+1
                        ;
                        ;  for (u8=0;u8<EE_MAX;u8++)
        CLRF  u8_2
m173    MOVLW 36
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF u8_2,W
        BTFSC 0x03,Carry
        GOTO  m174
                        ;  {
                        ;    u16Sum+=(U8)(EEReadByte(u8)+u8);
        MOVF  u8_2,W
        CALL  EEReadByte
        BCF   0x03,RP0
        BCF   0x03,RP1
        ADDWF u8_2,W
        ADDWF u16Sum_2,1
        BTFSC 0x03,Carry
        INCF  u16Sum_2+1,1
                        ;  }
        INCF  u8_2,1
        GOTO  m173
                        ;  return u16Sum;
m174    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum_2,W
        RETURN
                        ;}
                        ;
                        ;// Write the EEPROM checksum
                        ;static void EEWriteSum(void)
                        ;{
EEWriteSum
                        ;  U16 u16=EECalcSum();
        CALL  EECalcSum
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum_2,W
        MOVWF u16_8
        MOVF  u16Sum_2+1,W
        MOVWF u16_8+1
                        ;
                        ;  EEWriteWord(254,u16);
        MOVLW 254
        MOVWF u8Addr_5
        MOVF  u16_8,W
        MOVWF u16Data
        MOVF  u16_8+1,W
        MOVWF u16Data+1
        GOTO  EEWriteWord
                        ;}
                        ;
                        ;// Check the EEPROM checksum is OK
                        ;static BOOL EECheck(void)
                        ;{
EECheck
                        ;  BOOL b;
                        ;
                        ;  U16 u16SumRead=EEReadWord(254);
        MOVLW 254
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF u16SumRead
        MOVF  u16_7+1,W
        MOVWF u16SumRead+1
                        ;  U16 u16SumCalc=EECalcSum();
        CALL  EECalcSum
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum_2,W
        MOVWF u16SumCalc
        MOVF  u16Sum_2+1,W
        MOVWF u16SumCalc+1
                        ;
                        ;  if (u16SumRead==u16SumCalc)
        MOVF  u16SumRead,W
        XORWF u16SumCalc,W
        BTFSS 0x03,Zero_
        GOTO  m175
        MOVF  u16SumRead+1,W
        XORWF u16SumCalc+1,W
        BTFSS 0x03,Zero_
        GOTO  m175
                        ;  {
                        ;    return TRUE;
        BSF   0x03,Carry
        RETURN
                        ;  }
                        ;  return FALSE;
m175    BCF   0x03,Carry
        RETURN
                        ;}
                        ;
                        ;// If EEPROM values are OK, load them up, otherwise choose some default values.
                        ;// Warns on LCD if EEPROM values are wrong.
                        ;static void EEInit()
                        ;{
EEInit
                        ;  if (!EECheck())
        CALL  EECheck
        BTFSC 0x03,Carry
        GOTO  m178
                        ;  {
                        ;    U16 u16;
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA0
        CALL  LCDClear
        BCF   0x0A,PA0
                        ;    LCDMsg("Warning: bad");
        MOVLW 37
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA0
        CALL  LCDMsg
        BCF   0x0A,PA0
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA0
        CALL  LCDSetCursor
        BCF   0x0A,PA0
                        ;    LCDMsg("EEPROM data.");
        MOVLW 50
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA0
        CALL  LCDMsg
        BCF   0x0A,PA0
                        ;    _fAzMul=1.8583;
        MOVLW 221
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF _fAzMul
        MOVLW 109
        MOVWF _fAzMul+1
        MOVLW 127
        MOVWF _fAzMul+2
                        ;    _u16AzOff=8;
        MOVLW 8
        MOVWF _u16AzOff
        CLRF  _u16AzOff+1
                        ;    _fElMul=4.5555;
        MOVLW 199
        MOVWF _fElMul
        MOVLW 17
        MOVWF _fElMul+1
        MOVLW 129
        MOVWF _fElMul+2
                        ;    _u16ElOff=13;
        MOVLW 13
        MOVWF _u16ElOff
        CLRF  _u16ElOff+1
                        ;    _u16Flags=0;
        CLRF  _u16Flags
        CLRF  _u16Flags+1
                        ;    //AK
                        ;    _u16AzOffset[0] = 100;
        MOVLW 100
        MOVWF _u16AzOffset
        CLRF  _u16AzOffset+1
                        ;    _u16AzOffset[1] = 1100;
        MOVLW 76
        MOVWF _u16AzOffset+2
        MOVLW 4
        MOVWF _u16AzOffset+3
                        ;    _u16AzOffset[2] = 1500;
        MOVLW 220
        MOVWF _u16AzOffset+4
        MOVLW 5
        MOVWF _u16AzOffset+5
                        ;    _u16AzOffset[3] = 1800;
        MOVLW 8
        MOVWF _u16AzOffset+6
        MOVLW 7
        MOVWF _u16AzOffset+7
                        ;    _u16AzOffset[4] = 2000;
        MOVLW 208
        MOVWF _u16AzOffset+8
        MOVLW 7
        MOVWF _u16AzOffset+9
                        ;    _fAzMult[0] = 11.11;
        MOVLW 195
        MOVWF _fAzMult
        MOVLW 49
        MOVWF _fAzMult+1
        MOVLW 130
        MOVWF _fAzMult+2
                        ;    _fAzMult[1] = 4.44;
        MOVLW 20
        MOVWF _fAzMult+3
        MOVLW 14
        MOVWF _fAzMult+4
        MOVLW 129
        MOVWF _fAzMult+5
                        ;    _fAzMult[2] = 3.33;
        MOVLW 31
        MOVWF _fAzMult+6
        MOVLW 85
        MOVWF _fAzMult+7
        MOVLW 128
        MOVWF _fAzMult+8
                        ;    _fAzMult[3] = 2.22;
        MOVLW 20
        MOVWF _fAzMult+9
        MOVLW 14
        MOVWF _fAzMult+10
        MOVLW 128
        MOVWF _fAzMult+11
                        ;    _fAzMult[4] = 1.11;
        MOVLW 20
        MOVWF _fAzMult+12
        MOVLW 14
        MOVWF _fAzMult+13
        MOVLW 127
        MOVWF _fAzMult+14
                        ;    
                        ;    for (u16=0;u16<200;u16++) // wait 2s
        CLRF  u16_9
        CLRF  u16_9+1
m176    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_9+1,W
        BTFSS 0x03,Zero_
        GOTO  m177
        MOVLW 200
        SUBWF u16_9,W
        BTFSC 0x03,Carry
        GOTO  m177
                        ;    {
                        ;      Delay(100);
        MOVLW 100
        CALL  Delay
                        ;    }
        BCF   0x03,RP0
        BCF   0x03,RP1
        INCF  u16_9,1
        BTFSC 0x03,Zero_
        INCF  u16_9+1,1
        GOTO  m176
                        ;    LCDClear();
m177    BSF   0x0A,PA0
        CALL  LCDClear
        BCF   0x0A,PA0
                        ;  }
                        ;  else
        GOTO  m179
                        ;  {
                        ;    _fAzMul=EEReadFloat(EE_AZMUL);
m178    MOVLW 0
        CALL  EEReadFloat
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  C43tmp,W
        MOVWF _fAzMul
        MOVF  C43tmp+1,W
        MOVWF _fAzMul+1
        MOVF  C43tmp+2,W
        MOVWF _fAzMul+2
                        ;    _u16AzOff=EEReadWord(EE_AZOFF);
        MOVLW 3
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF _u16AzOff
        MOVF  u16_7+1,W
        MOVWF _u16AzOff+1
                        ;    _fElMul=EEReadFloat(EE_ELMUL);    
        MOVLW 5
        CALL  EEReadFloat
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  C43tmp,W
        MOVWF _fElMul
        MOVF  C43tmp+1,W
        MOVWF _fElMul+1
        MOVF  C43tmp+2,W
        MOVWF _fElMul+2
                        ;    _u16ElOff=EEReadWord(EE_ELOFF);
        MOVLW 8
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF _u16ElOff
        MOVF  u16_7+1,W
        MOVWF _u16ElOff+1
                        ;    _u16Flags=EEReadWord(EE_FLAGS);
        MOVLW 10
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF _u16Flags
        MOVF  u16_7+1,W
        MOVWF _u16Flags+1
                        ;    //AK
                        ;    _u16AzOffset[0] = EEReadWord(EE_AZOFF_0);
        MOVLW 11
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF _u16AzOffset
        MOVF  u16_7+1,W
        MOVWF _u16AzOffset+1
                        ;    _u16AzOffset[1] = EEReadWord(EE_AZOFF_90);
        MOVLW 13
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF _u16AzOffset+2
        MOVF  u16_7+1,W
        MOVWF _u16AzOffset+3
                        ;    _u16AzOffset[2] = EEReadWord(EE_AZOFF_180);
        MOVLW 15
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF _u16AzOffset+4
        MOVF  u16_7+1,W
        MOVWF _u16AzOffset+5
                        ;    _u16AzOffset[3] = EEReadWord(EE_AZOFF_270);
        MOVLW 17
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF _u16AzOffset+6
        MOVF  u16_7+1,W
        MOVWF _u16AzOffset+7
                        ;    _u16AzOffset[4] = EEReadWord(EE_AZOFF_360);
        MOVLW 19
        CALL  EEReadWord
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_7,W
        MOVWF _u16AzOffset+8
        MOVF  u16_7+1,W
        MOVWF _u16AzOffset+9
                        ;    _fAzMult[0]     = EEReadFloat(EE_AZMUL_0);
        MOVLW 21
        CALL  EEReadFloat
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  C43tmp,W
        MOVWF _fAzMult
        MOVF  C43tmp+1,W
        MOVWF _fAzMult+1
        MOVF  C43tmp+2,W
        MOVWF _fAzMult+2
                        ;    _fAzMult[1]     = EEReadFloat(EE_AZMUL_90);
        MOVLW 24
        CALL  EEReadFloat
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  C43tmp,W
        MOVWF _fAzMult+3
        MOVF  C43tmp+1,W
        MOVWF _fAzMult+4
        MOVF  C43tmp+2,W
        MOVWF _fAzMult+5
                        ;    _fAzMult[2]     = EEReadFloat(EE_AZMUL_180);
        MOVLW 27
        CALL  EEReadFloat
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  C43tmp,W
        MOVWF _fAzMult+6
        MOVF  C43tmp+1,W
        MOVWF _fAzMult+7
        MOVF  C43tmp+2,W
        MOVWF _fAzMult+8
                        ;    _fAzMult[3]     = EEReadFloat(EE_AZMUL_270);
        MOVLW 30
        CALL  EEReadFloat
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  C43tmp,W
        MOVWF _fAzMult+9
        MOVF  C43tmp+1,W
        MOVWF _fAzMult+10
        MOVF  C43tmp+2,W
        MOVWF _fAzMult+11
                        ;    _fAzMult[4]     = EEReadFloat(EE_AZMUL_360);
        MOVLW 33
        CALL  EEReadFloat
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  C43tmp,W
        MOVWF _fAzMult+12
        MOVF  C43tmp+1,W
        MOVWF _fAzMult+13
        MOVF  C43tmp+2,W
        MOVWF _fAzMult+14
                        ;  }
                        ;}
m179    RETURN
                        ;
                        ;// Write current RAM values to EEPROM
                        ;static BOOL EEWriteAll(void)
                        ;{
EEWriteAll
                        ;  EEWriteFloat(EE_AZMUL,_fAzMul);
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u8Addr_6
        MOVF  _fAzMul,W
        MOVWF f_3
        MOVF  _fAzMul+1,W
        MOVWF f_3+1
        MOVF  _fAzMul+2,W
        MOVWF f_3+2
        CALL  EEWriteFloat
                        ;  EEWriteWord(EE_AZOFF,_u16AzOff);
        MOVLW 3
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_5
        MOVF  _u16AzOff,W
        MOVWF u16Data
        MOVF  _u16AzOff+1,W
        MOVWF u16Data+1
        CALL  EEWriteWord
                        ;  EEWriteFloat(EE_ELMUL,_fElMul);
        MOVLW 5
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_6
        MOVF  _fElMul,W
        MOVWF f_3
        MOVF  _fElMul+1,W
        MOVWF f_3+1
        MOVF  _fElMul+2,W
        MOVWF f_3+2
        CALL  EEWriteFloat
                        ;  EEWriteWord(EE_ELOFF,_u16ElOff);
        MOVLW 8
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_5
        MOVF  _u16ElOff,W
        MOVWF u16Data
        MOVF  _u16ElOff+1,W
        MOVWF u16Data+1
        CALL  EEWriteWord
                        ;  EEWriteWord(EE_FLAGS,_u16Flags);
        MOVLW 10
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_5
        MOVF  _u16Flags,W
        MOVWF u16Data
        MOVF  _u16Flags+1,W
        MOVWF u16Data+1
        CALL  EEWriteWord
                        ;  //EEWriteSum();
                        ;  //AK
                        ;  EEWriteWord(EE_AZOFF_0,_u16AzOffset[0]);
        MOVLW 11
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_5
        MOVF  _u16AzOffset,W
        MOVWF u16Data
        MOVF  _u16AzOffset+1,W
        MOVWF u16Data+1
        CALL  EEWriteWord
                        ;  EEWriteWord(EE_AZOFF_90,_u16AzOffset[1]);
        MOVLW 13
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_5
        MOVF  _u16AzOffset+2,W
        MOVWF u16Data
        MOVF  _u16AzOffset+3,W
        MOVWF u16Data+1
        CALL  EEWriteWord
                        ;  EEWriteWord(EE_AZOFF_180,_u16AzOffset[2]);
        MOVLW 15
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_5
        MOVF  _u16AzOffset+4,W
        MOVWF u16Data
        MOVF  _u16AzOffset+5,W
        MOVWF u16Data+1
        CALL  EEWriteWord
                        ;  EEWriteWord(EE_AZOFF_270,_u16AzOffset[3]);
        MOVLW 17
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_5
        MOVF  _u16AzOffset+6,W
        MOVWF u16Data
        MOVF  _u16AzOffset+7,W
        MOVWF u16Data+1
        CALL  EEWriteWord
                        ;  EEWriteWord(EE_AZOFF_360,_u16AzOffset[4]);
        MOVLW 19
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_5
        MOVF  _u16AzOffset+8,W
        MOVWF u16Data
        MOVF  _u16AzOffset+9,W
        MOVWF u16Data+1
        CALL  EEWriteWord
                        ;  EEWriteFloat(EE_AZMUL_0,_fAzMult[0]);
        MOVLW 21
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_6
        MOVF  _fAzMult,W
        MOVWF f_3
        MOVF  _fAzMult+1,W
        MOVWF f_3+1
        MOVF  _fAzMult+2,W
        MOVWF f_3+2
        CALL  EEWriteFloat
                        ;  EEWriteFloat(EE_AZMUL_90,_fAzMult[1]);
        MOVLW 24
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_6
        MOVF  _fAzMult+3,W
        MOVWF f_3
        MOVF  _fAzMult+4,W
        MOVWF f_3+1
        MOVF  _fAzMult+5,W
        MOVWF f_3+2
        CALL  EEWriteFloat
                        ;  EEWriteFloat(EE_AZMUL_180,_fAzMult[2]);
        MOVLW 27
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_6
        MOVF  _fAzMult+6,W
        MOVWF f_3
        MOVF  _fAzMult+7,W
        MOVWF f_3+1
        MOVF  _fAzMult+8,W
        MOVWF f_3+2
        CALL  EEWriteFloat
                        ;  EEWriteFloat(EE_AZMUL_270,_fAzMult[3]);
        MOVLW 30
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_6
        MOVF  _fAzMult+9,W
        MOVWF f_3
        MOVF  _fAzMult+10,W
        MOVWF f_3+1
        MOVF  _fAzMult+11,W
        MOVWF f_3+2
        CALL  EEWriteFloat
                        ;  EEWriteFloat(EE_AZMUL_360,_fAzMult[4]);
        MOVLW 33
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Addr_6
        MOVF  _fAzMult+12,W
        MOVWF f_3
        MOVF  _fAzMult+13,W
        MOVWF f_3+1
        MOVF  _fAzMult+14,W
        MOVWF f_3+2
        CALL  EEWriteFloat
                        ;  EEWriteSum();
        CALL  EEWriteSum
                        ;        
                        ;  return EECheck();
        GOTO  EECheck
                        ;}
                        ;
                        ;#ifdef __CC5X__
                        ;//#pragma codepage 0
                        ;#pragma codepage 1
        ORG 0x0800
                        ;#endif
                        ;
                        ;
                        ;/**************** Button functions *******************/
                        ;
                        ;static U8 ButtonGetRaw(void)
                        ;{
ButtonGetRaw
                        ;  // Not debounced.
                        ;  U8 u8;
                        ;
                        ;  PORTB.3=0; // Enable buttons
        BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   PORTB,3
                        ;  nop(); // Need this delay!!! 16F876A does not settle in time otherwise.
        NOP  
                        ;  u8=PORTA; // Read buttons
        MOVF  PORTA,W
        MOVWF u8_3
                        ;  PORTB.3=1; // Disable buttons
        BSF   PORTB,3
                        ;  u8>>=2;
        BCF   0x03,Carry
        RRF   u8_3,1
        BCF   0x03,Carry
        RRF   u8_3,1
                        ;  u8&=0x0F;
        MOVLW 15
        ANDWF u8_3,1
                        ;  u8^=0x0F;
        MOVLW 15
        XORWF u8_3,1
                        ;  return u8;
        MOVF  u8_3,W
        RETURN
                        ;}
                        ;
                        ;static U8 ButtonGet(void)
                        ;{
ButtonGet
                        ;  // Wait for 5ms of completely stable buttons to debounce
                        ;  U8 u8Org=ButtonGetRaw();
        CALL  ButtonGetRaw
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Org
                        ;  U16 u16Org=TimerGet();
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  TimerGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16,W
        MOVWF u16Org
        MOVF  u16+1,W
        MOVWF u16Org+1
                        ;  U8 u8New=u8Org;
        MOVF  u8Org,W
        MOVWF u8New
                        ;  U16 u16New=u16Org;
        MOVF  u16Org,W
        MOVWF u16New
        MOVF  u16Org+1,W
        MOVWF u16New+1
                        ;  U16 u16Diff;
                        ;
                        ;  do
                        ;  {
                        ; 
                        ;    u8New=ButtonGetRaw();
m180    CALL  ButtonGetRaw
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8New
                        ;    u16New=TimerGet();
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  TimerGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16,W
        MOVWF u16New
        MOVF  u16+1,W
        MOVWF u16New+1
                        ;
                        ;    if (u8New!=u8Org)
        MOVF  u8New,W
        XORWF u8Org,W
        BTFSC 0x03,Zero_
        GOTO  m181
                        ;    {
                        ;      u8Org=u8New;
        MOVF  u8New,W
        MOVWF u8Org
                        ;      u16Org=u16New; // reset timer
        MOVF  u16New,W
        MOVWF u16Org
        MOVF  u16New+1,W
        MOVWF u16Org+1
                        ;    }
                        ;    u16Diff=u16New-u16Org;
m181    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Org+1,W
        SUBWF u16New+1,W
        MOVWF u16Diff_2+1
        MOVF  u16Org,W
        SUBWF u16New,W
        MOVWF u16Diff_2
        BTFSS 0x03,Carry
        DECF  u16Diff_2+1,1
                        ;  } while (u16Diff<5);
        MOVF  u16Diff_2+1,W
        BTFSS 0x03,Zero_
        GOTO  m182
        MOVLW 5
        SUBWF u16Diff_2,W
        BTFSS 0x03,Carry
        GOTO  m180
                        ;
                        ;  return u8New;
m182    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8New,W
        RETURN
                        ;}
                        ;
                        ;// Deal with any button pushes and return immediately
                        ;// Only moves rotator according to mask
                        ;// Mask bit 0 Left
                        ;// Mask bit 1 Right
                        ;// Mask bit 2 Down
                        ;// Mask bit 3 Up
                        ;// Returns button state
                        ;static U8 ButtonCheck(U8 u8Mask)
                        ;{
ButtonCheck
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Mask
                        ;  U8 u8;
                        ;
                        ;  // Any change in button status?
                        ;  _u8Buttons=ButtonGet();
        CALL  ButtonGet
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF _u8Buttons
                        ;
                        ;  u8=_u8Buttons^_u8ButtonsLast;
        MOVF  _u8ButtonsLast,W
        XORWF _u8Buttons,W
        MOVWF u8_4
                        ;
                        ;  if (u8!=0)
        MOVF  u8_4,1
        BTFSC 0x03,Zero_
        GOTO  m187
                        ;  {
                        ;    if (u8 & 0x03 & u8Mask)
        MOVLW 3
        ANDWF u8_4,W
        ANDWF u8Mask,W
        BTFSC 0x03,Zero_
        GOTO  m184
                        ;    {
                        ;      ROT_LEFT=0;
        BCF   PORTC,0
                        ;      ROT_RIGHT=0;
        BCF   PORTC,1
                        ;      _bAzTrack=FALSE;
        BCF   0x4E,_bAzTrack
                        ;      if (_u8Buttons & BTN_LEFT)
        BTFSS _u8Buttons,0
        GOTO  m183
                        ;      {
                        ;        ROT_LEFT=1;
        BSF   PORTC,0
                        ;      }
                        ;      else
        GOTO  m184
                        ;      {
                        ;        if (_u8Buttons & BTN_RIGHT)
m183    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC _u8Buttons,1
                        ;        {
                        ;          ROT_RIGHT=1;
        BSF   PORTC,1
                        ;        }
                        ;      }
                        ;    }
                        ;    if (u8 & 0x0C & u8Mask)
m184    MOVLW 12
        BCF   0x03,RP0
        BCF   0x03,RP1
        ANDWF u8_4,W
        ANDWF u8Mask,W
        BTFSC 0x03,Zero_
        GOTO  m186
                        ;    {
                        ;      ROT_DOWN=0;
        BCF   PORTC,2
                        ;      ROT_UP=0;
        BCF   PORTC,3
                        ;      _bElTrack=FALSE;
        BCF   0x4E,_bElTrack
                        ;      if (_u8Buttons & BTN_DOWN)
        BTFSS _u8Buttons,2
        GOTO  m185
                        ;      {
                        ;        ROT_DOWN=1;
        BSF   PORTC,2
                        ;      }
                        ;      else
        GOTO  m186
                        ;      {
                        ;        if (_u8Buttons & BTN_UP)
m185    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC _u8Buttons,3
                        ;        {
                        ;          ROT_UP=1;
        BSF   PORTC,3
                        ;        }
                        ;      }
                        ;    }
                        ;    _u8ButtonsLast=_u8Buttons;
m186    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u8Buttons,W
        MOVWF _u8ButtonsLast
                        ;  }
                        ;  return _u8Buttons;
m187    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u8Buttons,W
        RETURN
                        ;}
                        ;
                        ;// Stop the rotator when it's reached its destination
                        ;static void RotatorUpdate(void)
                        ;{
RotatorUpdate
                        ;  if (_bAzTrack)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x4E,_bAzTrack
        GOTO  m194
                        ;  {
                        ;    S16 s16Az=(S16)ADCGet(0);
        MOVLW 0
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF s16Az
        MOVF  u16Sum+1,W
        MOVWF s16Az+1
                        ;
                        ;    //AK
                        ;    //s16Az-=_u16AzOff;
                        ;    //s16Az/=_fAzMul;
                        ;    s16Az = (S16)DEGfromADC(s16Az);
        MOVF  s16Az,W
        MOVWF u16ADC
        MOVF  s16Az+1,W
        MOVWF u16ADC+1
        BSF   0x0A,PA1
        CALL  DEGfromADC
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16DEG,W
        MOVWF s16Az
        MOVF  u16DEG+1,W
        MOVWF s16Az+1
                        ;    
                        ;    if ((_s16AzTarget>=s16Az && ROT_LEFT) ||
        MOVF  s16Az+1,W
        SUBWF _s16AzTarget+1,W
        BTFSC 0x03,Zero_
        GOTO  m188
        MOVF  _s16AzTarget+1,W
        XORWF s16Az+1,W
        BTFSC 0x03,Carry
        XORLW 128
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m190
        GOTO  m189
m188    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16Az,W
        SUBWF _s16AzTarget,W
        BTFSS 0x03,Carry
        GOTO  m190
m189    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC PORTC,0
        GOTO  m193
                        ;      (_s16AzTarget<=s16Az && ROT_RIGHT))
m190    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _s16AzTarget+1,W
        SUBWF s16Az+1,W
        BTFSC 0x03,Zero_
        GOTO  m191
        MOVF  _s16AzTarget+1,W
        XORWF s16Az+1,W
        BTFSC 0x03,Carry
        XORLW 128
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m194
        GOTO  m192
m191    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _s16AzTarget,W
        SUBWF s16Az,W
        BTFSS 0x03,Carry
        GOTO  m194
m192    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS PORTC,1
        GOTO  m194
                        ;    {
                        ;      ROT_LEFT=0;
m193    BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   PORTC,0
                        ;      ROT_RIGHT=0;
        BCF   PORTC,1
                        ;      _bAzTrack=FALSE;
        BCF   0x4E,_bAzTrack
                        ;    }
                        ;  }
                        ;
                        ;  if (_bElTrack)
m194    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x4E,_bElTrack
        GOTO  m201
                        ;  {
                        ;    S16 s16El=(S16)ADCGet(1);
        MOVLW 1
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF s16El
        MOVF  u16Sum+1,W
        MOVWF s16El+1
                        ;
                        ;    s16El-=_u16ElOff;
        MOVF  _u16ElOff+1,W
        SUBWF s16El+1,1
        MOVF  _u16ElOff,W
        SUBWF s16El,1
        BTFSS 0x03,Carry
        DECF  s16El+1,1
                        ;    s16El/=_fElMul;
        MOVF  s16El,W
        MOVWF arg1f24
        MOVF  s16El+1,W
        MOVWF arg1f24+1
        CLRF  arg1f24+2
        BTFSC arg1f24+1,7
        DECF  arg1f24+2,1
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fElMul,W
        MOVWF arg2f24
        MOVF  _fElMul+1,W
        MOVWF arg2f24+1
        MOVF  _fElMul+2,W
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BSF   0x0A,PA1
        CALL  _float24ToInt24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        MOVWF s16El
        MOVF  rval_3+1,W
        MOVWF s16El+1
                        ;
                        ;    if ((_s16ElTarget>=s16El && ROT_DOWN) ||
        MOVF  s16El+1,W
        SUBWF _s16ElTarget+1,W
        BTFSC 0x03,Zero_
        GOTO  m195
        MOVF  _s16ElTarget+1,W
        XORWF s16El+1,W
        BTFSC 0x03,Carry
        XORLW 128
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m197
        GOTO  m196
m195    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16El,W
        SUBWF _s16ElTarget,W
        BTFSS 0x03,Carry
        GOTO  m197
m196    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC PORTC,2
        GOTO  m200
                        ;      (_s16ElTarget<=s16El && ROT_UP))
m197    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _s16ElTarget+1,W
        SUBWF s16El+1,W
        BTFSC 0x03,Zero_
        GOTO  m198
        MOVF  _s16ElTarget+1,W
        XORWF s16El+1,W
        BTFSC 0x03,Carry
        XORLW 128
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m201
        GOTO  m199
m198    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _s16ElTarget,W
        SUBWF s16El,W
        BTFSS 0x03,Carry
        GOTO  m201
m199    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS PORTC,3
        GOTO  m201
                        ;    {
                        ;      ROT_DOWN=0;
m200    BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   PORTC,2
                        ;      ROT_UP=0;
        BCF   PORTC,3
                        ;      _bElTrack=FALSE;
        BCF   0x4E,_bElTrack
                        ;    }
                        ;  }
                        ;}
m201    RETURN
                        ;
                        ;// Wait for buttons to be pressed not in mask. Allow Rotator to move according to those in mask.
                        ;// Return buttons pressed.
                        ;static U8 ButtonGetPress(U8 u8Mask)
                        ;{
ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Mask_2
                        ;  U8 u8Button;
                        ;
                        ;  do
                        ;  {
                        ;    u8Button=ButtonCheck(u8Mask);
m202    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8Mask_2,W
        CALL  ButtonCheck
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button
                        ;  } while ((u8Button & ~u8Mask)!=0); // wait for buttons to be released first
        COMF  u8Mask_2,W
        ANDWF u8Button,W
        BTFSS 0x03,Zero_
        GOTO  m202
                        ;
                        ;  do
                        ;  {
                        ;    u8Button=ButtonCheck(u8Mask);
m203    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8Mask_2,W
        CALL  ButtonCheck
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button
                        ;  } while ((u8Button & ~u8Mask)==0);
        COMF  u8Mask_2,W
        ANDWF u8Button,W
        BTFSC 0x03,Zero_
        GOTO  m203
                        ;
                        ;  return u8Button;
        MOVF  u8Button,W
        RETURN
                        ;}
                        ;
                        ;static void ButtonConfig(void)
                        ;{
ButtonConfig
                        ;  if (_bLCDActive && ButtonGet())
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x4E,_bLCDActive
        GOTO  m210
        CALL  ButtonGet
        XORLW 0
        BTFSC 0x03,Zero_
        GOTO  m210
                        ;  {
                        ;    U8 u8Button;
                        ;    BOOL bFinished;
                        ;    U16 u16AzMin;
                        ;    U16 u16AzMax;
                        ;    U16 u16ElMin;
                        ;    U16 u16ElMax;
                        ;    BOOL bAz450=FALSE;
        BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   0x2F,bAz450
                        ;    BOOL bEl90=FALSE;
        BCF   0x2F,bEl90
                        ;    BOOL bSNS=FALSE;
        BCF   0x2F,bSNS
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("Set Az=0 &");
        MOVLW 63
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("press D (U=esc)");
        MOVLW 125
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
        MOVLW 3
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_UP)
        BTFSC u8Button_2,3
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    u16AzMin=ADCGet(0);
        MOVLW 0
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF u16AzMin
        MOVF  u16Sum+1,W
        MOVWF u16AzMin+1
                        ;    
                        ;    //AK
                        ;    _u16AzOffset[0]=u16AzMin;
        MOVF  u16AzMin,W
        MOVWF _u16AzOffset
        MOVF  u16AzMin+1,W
        MOVWF _u16AzOffset+1
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("Set Az=90 &");
        MOVLW 74
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("press D (U=esc)");
        MOVLW 125
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
        MOVLW 3
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_UP)
        BTFSC u8Button_2,3
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    _u16AzOffset[1]=ADCGet(0);
        MOVLW 0
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF _u16AzOffset+2
        MOVF  u16Sum+1,W
        MOVWF _u16AzOffset+3
                        ;    _fAzMult[0]=(float)(_u16AzOffset[1]-_u16AzOffset[0])/90;
        CLRF  arg1f24+2
        MOVF  _u16AzOffset+1,W
        SUBWF _u16AzOffset+3,W
        MOVWF arg1f24+1
        MOVF  _u16AzOffset,W
        SUBWF _u16AzOffset+2,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 133
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fAzMult
        MOVF  arg1f24+1,W
        MOVWF _fAzMult+1
        MOVF  arg1f24+2,W
        MOVWF _fAzMult+2
                        ;    
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("Set Az=180 &");
        MOVLW 86
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("press D (U=esc)");
        MOVLW 125
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
        MOVLW 3
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_UP)
        BTFSC u8Button_2,3
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    _u16AzOffset[2]=ADCGet(0);
        MOVLW 0
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF _u16AzOffset+4
        MOVF  u16Sum+1,W
        MOVWF _u16AzOffset+5
                        ;    _fAzMult[1]=(float)(_u16AzOffset[2]-_u16AzOffset[1])/90;
        CLRF  arg1f24+2
        MOVF  _u16AzOffset+3,W
        SUBWF _u16AzOffset+5,W
        MOVWF arg1f24+1
        MOVF  _u16AzOffset+2,W
        SUBWF _u16AzOffset+4,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 133
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fAzMult+3
        MOVF  arg1f24+1,W
        MOVWF _fAzMult+4
        MOVF  arg1f24+2,W
        MOVWF _fAzMult+5
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("Set Az=270 &");
        MOVLW 99
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("press D (U=esc)");
        MOVLW 125
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
        MOVLW 3
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_UP)
        BTFSC u8Button_2,3
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    _u16AzOffset[3]=ADCGet(0);
        MOVLW 0
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF _u16AzOffset+6
        MOVF  u16Sum+1,W
        MOVWF _u16AzOffset+7
                        ;    _fAzMult[2]=(float)(_u16AzOffset[3]-_u16AzOffset[2])/90;
        CLRF  arg1f24+2
        MOVF  _u16AzOffset+5,W
        SUBWF _u16AzOffset+7,W
        MOVWF arg1f24+1
        MOVF  _u16AzOffset+4,W
        SUBWF _u16AzOffset+6,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 133
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fAzMult+6
        MOVF  arg1f24+1,W
        MOVWF _fAzMult+7
        MOVF  arg1f24+2,W
        MOVWF _fAzMult+8
                        ;    
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("Set Az=360 &");
        MOVLW 112
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("press D (U=esc)");
        MOVLW 125
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
        MOVLW 3
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_UP)
        BTFSC u8Button_2,3
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    _u16AzOffset[4]=ADCGet(0);
        MOVLW 0
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF _u16AzOffset+8
        MOVF  u16Sum+1,W
        MOVWF _u16AzOffset+9
                        ;    _fAzMult[3]=(float)(_u16AzOffset[4]-_u16AzOffset[3])/90;
        CLRF  arg1f24+2
        MOVF  _u16AzOffset+7,W
        SUBWF _u16AzOffset+9,W
        MOVWF arg1f24+1
        MOVF  _u16AzOffset+6,W
        SUBWF _u16AzOffset+8,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 133
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fAzMult+9
        MOVF  arg1f24+1,W
        MOVWF _fAzMult+10
        MOVF  arg1f24+2,W
        MOVWF _fAzMult+11
                        ;
                        ;    //AK
                        ;    //LCDClear();
                        ;    //LCDMsg("Set Az=max &");
                        ;    //LCDSetCursor(40);
                        ;    //LCDMsg("press D (U=esc)");
                        ;    //
                        ;    //u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
                        ;    //if (u8Button & BTN_UP)
                        ;    //{
                        ;    //  return;
                        ;    //}
                        ;    //AK
                        ;    //u16AzMax=ADCGet(0);
                        ;    u16AzMax=_u16AzOffset[4]; //make it the same as the 360 degree value
        MOVF  _u16AzOffset+8,W
        MOVWF u16AzMax
        MOVF  _u16AzOffset+9,W
        MOVWF u16AzMax+1
                        ;    //AK
                        ;    //_fAzMult[4]=(float)(u16AzMax-_u16AzOffset[4])/90;
                        ;    _fAzMult[4]=(float)0;
        CLRF  _fAzMult+12
        CLRF  _fAzMult+13
        CLRF  _fAzMult+14
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("Set El=min &");
        MOVLW 141
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("press L (R=esc)");
        MOVLW 167
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(BTN_UP | BTN_DOWN);
        MOVLW 12
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_RIGHT)
        BTFSC u8Button_2,1
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    u16ElMin=ADCGet(1);
        MOVLW 1
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF u16ElMin
        MOVF  u16Sum+1,W
        MOVWF u16ElMin+1
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("Set El=max &");
        MOVLW 154
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("press L (R=esc)");
        MOVLW 167
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(BTN_UP | BTN_DOWN);
        MOVLW 12
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_RIGHT)
        BTFSC u8Button_2,1
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    u16ElMax=ADCGet(1);
        MOVLW 1
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  ADCGet
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF u16ElMax
        MOVF  u16Sum+1,W
        MOVWF u16ElMax+1
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("Az range: 450 U");
        MOVLW 183
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("360 D (R=esc)");
        MOVLW 199
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(0);
        MOVLW 0
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_RIGHT)
        BTFSC u8Button_2,1
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    if (u8Button & BTN_UP)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC u8Button_2,3
                        ;    {
                        ;		bAz450=TRUE;
        BSF   0x2F,bAz450
                        ;    }
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("El range: 0-90 U");
        MOVLW 213
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("0-180 D (R=esc)");
        MOVLW 230
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(0);
        MOVLW 0
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_RIGHT)
        BTFSC u8Button_2,1
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    if (u8Button & BTN_UP)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC u8Button_2,3
                        ;    {
                        ;		bEl90=TRUE;
        BSF   0x2F,bEl90
                        ;    }
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("CCW stop North=U");
        MOVLW 246
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("South=D (R=esc)");
        MOVLW 7
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        MOVLW 1
        MOVWF psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(0);
        MOVLW 0
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_RIGHT)
        BTFSC u8Button_2,1
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    if (u8Button & BTN_DOWN)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSC u8Button_2,2
                        ;    {
                        ;      bSNS=TRUE;
        BSF   0x2F,bSNS
                        ;    }
                        ;
                        ;    LCDClear();
        BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;    LCDMsg("EE Write: U");
        MOVLW 23
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        MOVLW 1
        MOVWF psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;    LCDSetCursor(40);
        MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;    LCDMsg("(R=esc)");
        MOVLW 175
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        CLRF  psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;
                        ;    u8Button=ButtonGetPress(0);
        MOVLW 0
        CALL  ButtonGetPress
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8Button_2
                        ;    if (u8Button & BTN_RIGHT)
        BTFSC u8Button_2,1
                        ;    {
                        ;      return;
        RETURN
                        ;    }
                        ;    if (u8Button & BTN_UP)
        BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS u8Button_2,3
        GOTO  m210
                        ;    {
                        ;      FLAG_SNS=bSNS;
        BCF   _u16Flags,1
        BTFSC 0x2F,bSNS
        BSF   _u16Flags,1
                        ;      FLAG_AZ450=bAz450;
        BCF   _u16Flags,0
        BTFSC 0x2F,bAz450
        BSF   _u16Flags,0
                        ;      _u16AzOff=u16AzMin;
        MOVF  u16AzMin,W
        MOVWF _u16AzOff
        MOVF  u16AzMin+1,W
        MOVWF _u16AzOff+1
                        ;      _u16ElOff=u16ElMin;
        MOVF  u16ElMin,W
        MOVWF _u16ElOff
        MOVF  u16ElMin+1,W
        MOVWF _u16ElOff+1
                        ;      _fAzMul=(float)(u16AzMax-u16AzMin);
        CLRF  arg1f24+2
        MOVF  u16AzMin+1,W
        SUBWF u16AzMax+1,W
        MOVWF arg1f24+1
        MOVF  u16AzMin,W
        SUBWF u16AzMax,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval,W
        MOVWF _fAzMul
        MOVF  rval+1,W
        MOVWF _fAzMul+1
        MOVF  rval+2,W
        MOVWF _fAzMul+2
                        ;      if (bAz450)
        BTFSS 0x2F,bAz450
        GOTO  m204
                        ;      {
                        ;        _fAzMul/=450.0;
        MOVF  _fAzMul,W
        MOVWF arg1f24
        MOVF  _fAzMul+1,W
        MOVWF arg1f24+1
        MOVF  _fAzMul+2,W
        MOVWF arg1f24+2
        CLRF  arg2f24
        MOVLW 97
        MOVWF arg2f24+1
        MOVLW 135
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fAzMul
        MOVF  arg1f24+1,W
        MOVWF _fAzMul+1
        MOVF  arg1f24+2,W
        MOVWF _fAzMul+2
                        ;      }
                        ;      else
        GOTO  m205
                        ;      {
                        ;        _fAzMul/=360.0;
m204    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fAzMul,W
        MOVWF arg1f24
        MOVF  _fAzMul+1,W
        MOVWF arg1f24+1
        MOVF  _fAzMul+2,W
        MOVWF arg1f24+2
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 135
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fAzMul
        MOVF  arg1f24+1,W
        MOVWF _fAzMul+1
        MOVF  arg1f24+2,W
        MOVWF _fAzMul+2
                        ;      }
                        ;      _fElMul=(float)(u16ElMax-u16ElMin);
m205    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg1f24+2
        MOVF  u16ElMin+1,W
        SUBWF u16ElMax+1,W
        MOVWF arg1f24+1
        MOVF  u16ElMin,W
        SUBWF u16ElMax,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval,W
        MOVWF _fElMul
        MOVF  rval+1,W
        MOVWF _fElMul+1
        MOVF  rval+2,W
        MOVWF _fElMul+2
                        ;      if (bEl90)
        BTFSS 0x2F,bEl90
        GOTO  m206
                        ;      {
                        ;        _fElMul/=90;
        MOVF  _fElMul,W
        MOVWF arg1f24
        MOVF  _fElMul+1,W
        MOVWF arg1f24+1
        MOVF  _fElMul+2,W
        MOVWF arg1f24+2
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 133
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fElMul
        MOVF  arg1f24+1,W
        MOVWF _fElMul+1
        MOVF  arg1f24+2,W
        MOVWF _fElMul+2
                        ;      }
                        ;      else
        GOTO  m207
                        ;      {
                        ;        _fElMul/=180;
m206    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fElMul,W
        MOVWF arg1f24
        MOVF  _fElMul+1,W
        MOVWF arg1f24+1
        MOVF  _fElMul+2,W
        MOVWF arg1f24+2
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 134
        MOVWF arg2f24+2
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fElMul
        MOVF  arg1f24+1,W
        MOVWF _fElMul+1
        MOVF  arg1f24+2,W
        MOVWF _fElMul+2
                        ;      }
                        ;      LCDClear();
m207    BSF   0x0A,PA1
        CALL  LCDClear
        BCF   0x0A,PA1
                        ;      if (EEWriteAll())
        BCF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  EEWriteAll
        BSF   0x0A,PA0
        BCF   0x0A,PA1
        BTFSS 0x03,Carry
        GOTO  m208
                        ;      {
                        ;        LCDMsg("EEPROM write OK");
        MOVLW 35
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        MOVLW 1
        MOVWF psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;      }
                        ;      else
        GOTO  m209
                        ;      {
                        ;        LCDMsg("EEPROM failed!");
m208    MOVLW 51
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        MOVLW 1
        MOVWF psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;      }
                        ;      LCDSetCursor(40);
m209    MOVLW 40
        BSF   0x0A,PA1
        CALL  LCDSetCursor
        BCF   0x0A,PA1
                        ;      LCDMsg("Push btn to exit");
        MOVLW 66
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz_2
        MOVLW 1
        MOVWF psz_2+1
        BSF   0x0A,PA1
        CALL  LCDMsg
        BCF   0x0A,PA1
                        ;      ButtonGetPress(0);
        MOVLW 0
        GOTO  ButtonGetPress
                        ;    }
                        ;  }
                        ;}
m210    RETURN
                        ;
                        ;#ifdef __CC5X__
                        ;#pragma codepage 0
        ORG 0x0051
                        ;#endif
                        ;
                        ;// Zeroise target and stop tracking
                        ;static void TargetInit(void)
                        ;{
TargetInit
                        ;  _s16AzTarget=0;
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  _s16AzTarget
        CLRF  _s16AzTarget+1
                        ;  _s16ElTarget=0;
        CLRF  _s16ElTarget
        CLRF  _s16ElTarget+1
                        ;  _bAzTrack=FALSE;
        BCF   0x4E,_bAzTrack
                        ;  _bElTrack=FALSE;
        BCF   0x4E,_bElTrack
                        ;}
        RETURN
                        ;
                        ;static U8 EasyCommCommand(U8 *pu8Pos)
                        ;{
EasyCommCommand
                        ;  // Look for a two character Easycomm command...
                        ;  char c1='\0';
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  c1
                        ;  char c2='\0';
        CLRF  c2
                        ;  U8 u8Pos=*pu8Pos;
        BCF   0x03,IRP
        MOVF  pu8Pos_4,W
        MOVWF FSR
        MOVF  INDF,W
        MOVWF u8Pos_4
                        ;  U8 u8Command=EC_NONE;
        CLRF  u8Command
                        ;
                        ;  ParseWhite(&u8Pos); // ignore white space
        MOVLW 45
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;
                        ;  if (u8Pos<=_u8LinePos+1) // make sure there's at least two more characters
        BCF   0x03,RP0
        BCF   0x03,RP1
        INCF  _u8LinePos,W
        SUBWF u8Pos_4,W
        BTFSS 0x03,Carry
        GOTO  m211
        BTFSS 0x03,Zero_
        GOTO  m216
                        ;  {
                        ;    c1=toupper(_acLine[u8Pos++]);
m211    MOVLW 144
        BCF   0x03,RP0
        BCF   0x03,RP1
        ADDWF u8Pos_4,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        BSF   0x0A,PA1
        CALL  toupper
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c1
        INCF  u8Pos_4,1
                        ;    c2=toupper(_acLine[u8Pos++]);
        MOVLW 144
        ADDWF u8Pos_4,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        BSF   0x0A,PA1
        CALL  toupper
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c2
        INCF  u8Pos_4,1
                        ;
                        ;    if (c1=='A' && c2=='Z')
        MOVF  c1,W
        XORLW 65
        BTFSS 0x03,Zero_
        GOTO  m212
        MOVF  c2,W
        XORLW 90
        BTFSS 0x03,Zero_
        GOTO  m212
                        ;    {
                        ;      u8Command=EC_AZ; 
        MOVLW 1
        MOVWF u8Command
                        ;    }
                        ;    else if (c1=='E' && c2=='L')
        GOTO  m215
m212    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c1,W
        XORLW 69
        BTFSS 0x03,Zero_
        GOTO  m213
        MOVF  c2,W
        XORLW 76
        BTFSS 0x03,Zero_
        GOTO  m213
                        ;    {
                        ;      u8Command=EC_EL;
        MOVLW 2
        MOVWF u8Command
                        ;    }
                        ;    else if (c1=='U' && c2=='P')
        GOTO  m215
m213    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c1,W
        XORLW 85
        BTFSS 0x03,Zero_
        GOTO  m214
        MOVF  c2,W
        XORLW 80
        BTFSS 0x03,Zero_
        GOTO  m214
                        ;    {
                        ;      u8Command=EC_UP;
        MOVLW 3
        MOVWF u8Command
                        ;    }
                        ;    else if (c1=='D' && c2=='N')
        GOTO  m215
m214    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c1,W
        XORLW 68
        BTFSS 0x03,Zero_
        GOTO  m215
        MOVF  c2,W
        XORLW 78
        BTFSS 0x03,Zero_
        GOTO  m215
                        ;    {
                        ;      u8Command=EC_DN;
        MOVLW 4
        MOVWF u8Command
                        ;    }
                        ;    if (u8Command!=EC_NONE)
m215    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8Command,1
        BTFSC 0x03,Zero_
        GOTO  m216
                        ;    {
                        ;      *pu8Pos=u8Pos;
        BCF   0x03,IRP
        MOVF  pu8Pos_4,W
        MOVWF FSR
        MOVF  u8Pos_4,W
        MOVWF INDF
                        ;    }
                        ;  }
                        ;  return u8Command;
m216    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8Command,W
        RETURN
                        ;}
                        ;
                        ;static void RotatorSet(S16 s16Az,S16 s16El)
                        ;{
RotatorSet
                        ;  // Set the rotator position. 
                        ;  // If El or Az are 32767, then no attempt is made to move that axis
                        ;  if (s16Az!=32767)
        BCF   0x03,RP0
        BCF   0x03,RP1
        INCFSZ s16Az_2,W
        GOTO  m217
        MOVF  s16Az_2+1,W
        XORLW 127
        BTFSC 0x03,Zero_
        GOTO  m227
                        ;  {
                        ;    if (FLAG_SNS)
m217    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS _u16Flags,1
        GOTO  m220
                        ;    {
                        ;      if (s16Az<180)
        BTFSC s16Az_2+1,7
        GOTO  m218
        MOVF  s16Az_2+1,W
        BTFSS 0x03,Zero_
        GOTO  m219
        MOVLW 180
        SUBWF s16Az_2,W
        BTFSC 0x03,Carry
        GOTO  m219
                        ;      {
                        ;        s16Az+=180;
m218    MOVLW 180
        BCF   0x03,RP0
        BCF   0x03,RP1
        ADDWF s16Az_2,1
        BTFSC 0x03,Carry
        INCF  s16Az_2+1,1
                        ;      }
                        ;      else
        GOTO  m220
                        ;      {
                        ;        s16Az-=180;
m219    MOVLW 180
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF s16Az_2,1
        BTFSS 0x03,Carry
        DECF  s16Az_2+1,1
                        ;      }
                        ;    }
                        ;    _s16AzTarget=s16Az;
m220    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16Az_2,W
        MOVWF _s16AzTarget
        MOVF  s16Az_2+1,W
        MOVWF _s16AzTarget+1
                        ;
                        ;    s16Az=(S16)ADCGet(0);
        MOVLW 0
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF s16Az_2
        MOVF  u16Sum+1,W
        MOVWF s16Az_2+1
                        ;
                        ;    //AK
                        ;    //s16Az-=_u16AzOff;
                        ;    //s16Az/=_fAzMul;
                        ;    s16Az = (S16)DEGfromADC(s16Az);
        MOVF  s16Az_2,W
        MOVWF u16ADC
        MOVF  s16Az_2+1,W
        MOVWF u16ADC+1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  DEGfromADC
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16DEG,W
        MOVWF s16Az_2
        MOVF  u16DEG+1,W
        MOVWF s16Az_2+1
                        ;
                        ;    if (_s16AzTarget>s16Az)
        MOVF  _s16AzTarget+1,W
        SUBWF s16Az_2+1,W
        BTFSC 0x03,Zero_
        GOTO  m221
        MOVF  _s16AzTarget+1,W
        XORWF s16Az_2+1,W
        BTFSC 0x03,Carry
        XORLW 128
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m222
        GOTO  m223
m221    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _s16AzTarget,W
        SUBWF s16Az_2,W
        BTFSC 0x03,Carry
        GOTO  m223
                        ;    {
                        ;      ROT_RIGHT=1;
m222    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   PORTC,1
                        ;      ROT_LEFT=0;
        BCF   PORTC,0
                        ;      _bAzTrack=TRUE;
        BSF   0x4E,_bAzTrack
                        ;    }
                        ;    else
        GOTO  m227
                        ;    {
                        ;      if (_s16AzTarget<s16Az)
m223    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16Az_2+1,W
        SUBWF _s16AzTarget+1,W
        BTFSC 0x03,Zero_
        GOTO  m224
        MOVF  _s16AzTarget+1,W
        XORWF s16Az_2+1,W
        BTFSC 0x03,Carry
        XORLW 128
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m225
        GOTO  m226
m224    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16Az_2,W
        SUBWF _s16AzTarget,W
        BTFSC 0x03,Carry
        GOTO  m226
                        ;      {
                        ;        ROT_LEFT=1;
m225    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   PORTC,0
                        ;        ROT_RIGHT=0;
        BCF   PORTC,1
                        ;        _bAzTrack=TRUE;
        BSF   0x4E,_bAzTrack
                        ;      }
                        ;      else
        GOTO  m227
                        ;      {
                        ;        ROT_LEFT=0;
m226    BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   PORTC,0
                        ;        ROT_RIGHT=0;
        BCF   PORTC,1
                        ;        _bAzTrack=FALSE;
        BCF   0x4E,_bAzTrack
                        ;      }
                        ;    }
                        ;  }
                        ;  if (s16El!=32767)
m227    BCF   0x03,RP0
        BCF   0x03,RP1
        INCFSZ s16El_2,W
        GOTO  m228
        MOVF  s16El_2+1,W
        XORLW 127
        BTFSC 0x03,Zero_
        GOTO  m235
                        ;  {
                        ;    _bElTrack=TRUE;
m228    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x4E,_bElTrack
                        ;    _s16ElTarget=s16El;
        MOVF  s16El_2,W
        MOVWF _s16ElTarget
        MOVF  s16El_2+1,W
        MOVWF _s16ElTarget+1
                        ;
                        ;    s16El=(S16)ADCGet(1);
        MOVLW 1
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF s16El_2
        MOVF  u16Sum+1,W
        MOVWF s16El_2+1
                        ;
                        ;    s16El-=_u16ElOff;
        MOVF  _u16ElOff+1,W
        SUBWF s16El_2+1,1
        MOVF  _u16ElOff,W
        SUBWF s16El_2,1
        BTFSS 0x03,Carry
        DECF  s16El_2+1,1
                        ;    s16El/=_fElMul;
        MOVF  s16El_2,W
        MOVWF arg1f24
        MOVF  s16El_2+1,W
        MOVWF arg1f24+1
        CLRF  arg1f24+2
        BTFSC arg1f24+1,7
        DECF  arg1f24+2,1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fElMul,W
        MOVWF arg2f24
        MOVF  _fElMul+1,W
        MOVWF arg2f24+1
        MOVF  _fElMul+2,W
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _float24ToInt24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        MOVWF s16El_2
        MOVF  rval_3+1,W
        MOVWF s16El_2+1
                        ;
                        ;    if (_s16ElTarget>s16El)
        MOVF  _s16ElTarget+1,W
        SUBWF s16El_2+1,W
        BTFSC 0x03,Zero_
        GOTO  m229
        MOVF  _s16ElTarget+1,W
        XORWF s16El_2+1,W
        BTFSC 0x03,Carry
        XORLW 128
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m230
        GOTO  m231
m229    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _s16ElTarget,W
        SUBWF s16El_2,W
        BTFSC 0x03,Carry
        GOTO  m231
                        ;    {
                        ;      ROT_UP=1;
m230    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   PORTC,3
                        ;      ROT_DOWN=0;
        BCF   PORTC,2
                        ;      _bElTrack=TRUE;
        BSF   0x4E,_bElTrack
                        ;    }
                        ;    else
        GOTO  m235
                        ;    {
                        ;      if (_s16ElTarget<s16El)
m231    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16El_2+1,W
        SUBWF _s16ElTarget+1,W
        BTFSC 0x03,Zero_
        GOTO  m232
        MOVF  _s16ElTarget+1,W
        XORWF s16El_2+1,W
        BTFSC 0x03,Carry
        XORLW 128
        ANDLW 128
        BTFSC 0x03,Zero_
        GOTO  m233
        GOTO  m234
m232    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16El_2,W
        SUBWF _s16ElTarget,W
        BTFSC 0x03,Carry
        GOTO  m234
                        ;      {
                        ;        ROT_DOWN=1;
m233    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   PORTC,2
                        ;        ROT_UP=0;
        BCF   PORTC,3
                        ;        _bElTrack=TRUE;
        BSF   0x4E,_bElTrack
                        ;      }
                        ;      else
        GOTO  m235
                        ;      {
                        ;        ROT_DOWN=0;
m234    BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   PORTC,2
                        ;        ROT_UP=0;
        BCF   PORTC,3
                        ;        _bElTrack=FALSE;
        BCF   0x4E,_bElTrack
                        ;      }
                        ;    }
                        ;  }
                        ;}
m235    RETURN
                        ;
                        ;// Parse the input line if one's available
                        ;static void RS232Check(void)
                        ;{
RS232Check
                        ;  while (RS232RxCharReady())
m236    BSF   0x0A,PA1
        CALL  RS232RxCharReady
        BCF   0x0A,PA1
        BTFSS 0x03,Carry
        GOTO  m280
                        ;  {
                        ;    char c=RS232RxGetChar();
        BSF   0x0A,PA1
        CALL  RS232RxGetChar
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c_12
                        ;
                        ;////
                        ;////
                        ;////
                        ;////RS232TxPutChar(c);
                        ;	
                        ;
                        ;    if (c=='\n' || c=='\r')
        MOVF  c_12,W
        XORLW 10
        BTFSC 0x03,Zero_
        GOTO  m237
        MOVF  c_12,W
        XORLW 13
        BTFSS 0x03,Zero_
        GOTO  m279
                        ;    {
                        ;	    
                        ;////
                        ;////
                        ;////
                        ;////RS232TxPutChar('\n');
                        ;      U8 u8Pos=0;
m237    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u8Pos_5
                        ;      U8 u8EasyCommCommand=EasyCommCommand(&u8Pos);
        MOVLW 39
        MOVWF pu8Pos_4
        CALL  EasyCommCommand
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8EasyCommCommand
                        ;
                        ;      if (u8EasyCommCommand!=EC_NONE)
        MOVF  u8EasyCommCommand,1
        BTFSC 0x03,Zero_
        GOTO  m249
                        ;      {
                        ;        while (u8EasyCommCommand!=EC_NONE)
m238    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8EasyCommCommand,1
        BTFSC 0x03,Zero_
        GOTO  m278
                        ;        {
                        ;          BOOL bError=FALSE;
        BCF   0x29,bError
                        ;
                        ;          switch (u8EasyCommCommand)
        MOVF  u8EasyCommCommand,W
        XORLW 1
        BTFSC 0x03,Zero_
        GOTO  m239
        XORLW 3
        BTFSC 0x03,Zero_
        GOTO  m239
        XORLW 1
        BTFSC 0x03,Zero_
        GOTO  m242
        XORLW 7
        BTFSC 0x03,Zero_
        GOTO  m242
        GOTO  m246
                        ;          {
                        ;            case EC_AZ:
                        ;            case EC_EL:
                        ;              {
                        ;                float f;
                        ;                S16 s16;
                        ;
                        ;                ParseWhite(&u8Pos);
m239    MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;                if (!ParseFloat(&u8Pos,&f))
        MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_3
        MOVLW 42
        MOVWF pf
        BSF   0x0A,PA1
        CALL  ParseFloat
        BCF   0x0A,PA1
        BTFSC 0x03,Carry
        GOTO  m240
                        ;                {
                        ;                  bError=TRUE;
        BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x29,bError
                        ;                  break;
        GOTO  m247
                        ;                }
                        ;                s16=(S16)f;
m240    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  f_4,W
        MOVWF arg1f24
        MOVF  f_4+1,W
        MOVWF arg1f24+1
        MOVF  f_4+2,W
        MOVWF arg1f24+2
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _float24ToInt24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        MOVWF s16_5
        MOVF  rval_3+1,W
        MOVWF s16_5+1
                        ;                if (u8EasyCommCommand==EC_AZ)
        DECFSZ u8EasyCommCommand,W
        GOTO  m241
                        ;                {
                        ;                  RotatorSet(s16,32767);
        MOVF  s16_5,W
        MOVWF s16Az_2
        MOVF  s16_5+1,W
        MOVWF s16Az_2+1
        MOVLW 255
        MOVWF s16El_2
        MOVLW 127
        MOVWF s16El_2+1
        CALL  RotatorSet
                        ;                }
                        ;                else
        GOTO  m247
                        ;                {
                        ;                  RotatorSet(32767,s16);
m241    MOVLW 255
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF s16Az_2
        MOVLW 127
        MOVWF s16Az_2+1
        MOVF  s16_5,W
        MOVWF s16El_2
        MOVF  s16_5+1,W
        MOVWF s16El_2+1
        CALL  RotatorSet
                        ;                }
                        ;              }
                        ;              break;
        GOTO  m247
                        ;            case EC_UP:
                        ;            case EC_DN:
                        ;              // Parsed but ignored...
                        ;              {
                        ;                float f;
                        ;                char acMode[3];
                        ;                U8 u8=0;
m242    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u8_5
                        ;
                        ;                ParseWhite(&u8Pos);
        MOVLW 39
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;                if (!ParseFloat(&u8Pos,&f))
        MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_3
        MOVLW 42
        MOVWF pf
        BSF   0x0A,PA1
        CALL  ParseFloat
        BCF   0x0A,PA1
        BTFSC 0x03,Carry
        GOTO  m243
                        ;                {
                        ;                  bError=TRUE;
        BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x29,bError
                        ;                  break;
        GOTO  m247
                        ;                }
                        ;                ParseWhite(&u8Pos);
m243    MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;                while (u8Pos<_u8LinePos && u8<3)
m244    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u8LinePos,W
        SUBWF u8Pos_5,W
        BTFSC 0x03,Carry
        GOTO  m245
        MOVLW 3
        SUBWF u8_5,W
        BTFSC 0x03,Carry
        GOTO  m245
                        ;                {
                        ;                  char c=_acLine[u8Pos++];
        MOVLW 144
        ADDWF u8Pos_5,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        MOVWF c_13
        INCF  u8Pos_5,1
                        ;
                        ;                  acMode[u8++]=c;
        MOVLW 45
        ADDWF u8_5,W
        MOVWF FSR
        BCF   0x03,IRP
        MOVF  c_13,W
        MOVWF INDF
        INCF  u8_5,1
                        ;                }
        GOTO  m244
                        ;                while (u8<3)
m245    MOVLW 3
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF u8_5,W
        BTFSC 0x03,Carry
        GOTO  m247
                        ;                {
                        ;                  acMode[u8++]=' ';
        MOVLW 45
        ADDWF u8_5,W
        MOVWF FSR
        BCF   0x03,IRP
        MOVLW 32
        MOVWF INDF
        INCF  u8_5,1
                        ;                }
        GOTO  m245
                        ;              }
                        ;              break;
                        ;            default:
                        ;              bError=TRUE;
m246    BCF   0x03,RP0
        BCF   0x03,RP1
        BSF   0x29,bError
                        ;              break;
                        ;          }
                        ;          if (bError)
m247    BCF   0x03,RP0
        BCF   0x03,RP1
        BTFSS 0x29,bError
        GOTO  m248
                        ;          {
                        ;            RS232TxPutError();
        BSF   0x0A,PA1
        CALL  RS232TxPutError
        BCF   0x0A,PA1
                        ;            u8EasyCommCommand=EC_NONE;
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  u8EasyCommCommand
                        ;          }
                        ;          else
        GOTO  m238
                        ;          {
                        ;            u8EasyCommCommand=EasyCommCommand(&u8Pos);
m248    MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_4
        CALL  EasyCommCommand
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF u8EasyCommCommand
                        ;          }
                        ;        }
        GOTO  m238
                        ;      }
                        ;      else
                        ;      { // This is a GS232 command
                        ;        // Ignore leading spaces...
                        ;        while (u8Pos<_u8LinePos)
m249    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u8LinePos,W
        SUBWF u8Pos_5,W
        BTFSC 0x03,Carry
        GOTO  m250
                        ;        {
                        ;          c=toupper(_acLine[u8Pos]);
        MOVLW 144
        ADDWF u8Pos_5,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        BSF   0x0A,PA1
        CALL  toupper
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF c_12
                        ;          u8Pos++;
        INCF  u8Pos_5,1
                        ;          if (c!=' ' && c!='\t')
        MOVF  c_12,W
        XORLW 32
        BTFSC 0x03,Zero_
        GOTO  m249
        MOVF  c_12,W
        XORLW 9
        BTFSC 0x03,Zero_
        GOTO  m249
                        ;          {
                        ;            break;
                        ;          }
                        ;        };
                        ;        if (u8Pos<=_u8LinePos)
m250    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u8Pos_5,W
        SUBWF _u8LinePos,W
        BTFSS 0x03,Carry
        GOTO  m278
                        ;        {
                        ;          // Get command char
                        ;          switch (c)
        MOVF  c_12,W
        XORLW 70
        BTFSC 0x03,Zero_
        GOTO  m251
        XORLW 21
        BTFSC 0x03,Zero_
        GOTO  m266
        XORLW 16
        BTFSC 0x03,Zero_
        GOTO  m267
        XORLW 20
        BTFSC 0x03,Zero_
        GOTO  m274
        XORLW 26
        BTFSC 0x03,Zero_
        GOTO  m274
        GOTO  m277
                        ;          {
                        ;            case 'F':
                        ;              if (u8Pos<_u8LinePos)
m251    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u8LinePos,W
        SUBWF u8Pos_5,W
        BTFSC 0x03,Carry
        GOTO  m265
                        ;              {
                        ;                char cAzEl=toupper(_acLine[u8Pos++]);
        MOVLW 144
        ADDWF u8Pos_5,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        BSF   0x0A,PA1
        CALL  toupper
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF cAzEl
        INCF  u8Pos_5,1
                        ;  
                        ;                if (cAzEl=='W')
        MOVF  cAzEl,W
        XORLW 87
        BTFSS 0x03,Zero_
        GOTO  m253
                        ;                {
                        ;                  if (EEWriteAll())
        BSF   0x0A,PA1
        CALL  EEWriteAll
        BCF   0x0A,PA1
        BTFSS 0x03,Carry
        GOTO  m252
                        ;                  {
                        ;                    RS232TxMsg("EEPROM write OK\r\n");
        MOVLW 83
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                  }
                        ;                  else
        GOTO  m278
                        ;                  {
                        ;                    RS232TxMsg("EEPROM failed\r\n");
m252    MOVLW 101
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                  }
                        ;                  break;
        GOTO  m278
                        ;                }
                        ;  
                        ;                if (cAzEl=='S') // Set South stop rotator
m253    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 83
        BTFSS 0x03,Zero_
        GOTO  m254
                        ;                {
                        ;                  FLAG_SNS=TRUE;
        BSF   _u16Flags,1
                        ;                  RS232TxMsg("South stop\r\n");
        MOVLW 117
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                  break;
        GOTO  m278
                        ;                }
                        ;  
                        ;                if (cAzEl=='N') // Set 0 - 360/450 rotator
m254    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 78
        BTFSS 0x03,Zero_
        GOTO  m255
                        ;                {
                        ;                  FLAG_SNS=FALSE;
        BCF   _u16Flags,1
                        ;                  RS232TxMsg("North stop\r\n");
        MOVLW 130
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                  break;
        GOTO  m278
                        ;                }
                        ;  
                        ;                if (u8Pos<_u8LinePos)
m255    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u8LinePos,W
        SUBWF u8Pos_5,W
        BTFSC 0x03,Carry
        GOTO  m265
                        ;                {
                        ;                  char cEndStart=toupper(_acLine[u8Pos++]);
        MOVLW 144
        ADDWF u8Pos_5,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        BSF   0x0A,PA1
        CALL  toupper
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF cEndStart
        INCF  u8Pos_5,1
                        ;  
                        ;                  if (cAzEl=='A' && cEndStart=='O') // Set Az Offset directly
        MOVF  cAzEl,W
        XORLW 65
        BTFSS 0x03,Zero_
        GOTO  m256
        MOVF  cEndStart,W
        XORLW 79
        BTFSS 0x03,Zero_
        GOTO  m256
                        ;                  {
                        ;                    U16 u16;
                        ;  
                        ;                    ParseWhite(&u8Pos);
        MOVLW 39
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;                    if (ParseU16(&u8Pos,&u16))
        MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_2
        MOVLW 43
        MOVWF pu16
        BSF   0x0A,PA1
        CALL  ParseU16
        BCF   0x0A,PA1
        BTFSS 0x03,Carry
        GOTO  m256
                        ;                    {
                        ;                      _u16AzOff=u16;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_10,W
        MOVWF _u16AzOff
        MOVF  u16_10+1,W
        MOVWF _u16AzOff+1
                        ;            		  TargetInit();
        CALL  TargetInit
                        ;                      RS232TxMsg("Az off=");
        MOVLW 143
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                      RS232TxPutU16(_u16AzOff);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u16AzOff,W
        MOVWF u16_3
        MOVF  _u16AzOff+1,W
        MOVWF u16_3+1
        BSF   0x0A,PA1
        CALL  RS232TxPutU16
        BCF   0x0A,PA1
                        ;                      RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                      break;
        GOTO  m278
                        ;                    }
                        ;                  }
                        ;                  if (cAzEl=='A' && cEndStart=='M') // Set Az Mul directly
m256    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 65
        BTFSS 0x03,Zero_
        GOTO  m257
        MOVF  cEndStart,W
        XORLW 77
        BTFSS 0x03,Zero_
        GOTO  m257
                        ;                  {
                        ;                    float f;
                        ;  
                        ;                    ParseWhite(&u8Pos);
        MOVLW 39
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;                    if (ParseFloat(&u8Pos,&f))
        MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_3
        MOVLW 43
        MOVWF pf
        BSF   0x0A,PA1
        CALL  ParseFloat
        BCF   0x0A,PA1
        BTFSS 0x03,Carry
        GOTO  m257
                        ;                    {
                        ;                      _fAzMul=f;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  f_6,W
        MOVWF _fAzMul
        MOVF  f_6+1,W
        MOVWF _fAzMul+1
        MOVF  f_6+2,W
        MOVWF _fAzMul+2
                        ;            					TargetInit();
        CALL  TargetInit
                        ;                      RS232TxMsg("Az mul=");
        MOVLW 162
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                      RS232TxPutFloat(_fAzMul);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fAzMul,W
        MOVWF f_2
        MOVF  _fAzMul+1,W
        MOVWF f_2+1
        MOVF  _fAzMul+2,W
        MOVWF f_2+2
        BSF   0x0A,PA1
        CALL  RS232TxPutFloat
        BCF   0x0A,PA1
                        ;                      RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                      break;
        GOTO  m278
                        ;                    }
                        ;                  }
                        ;                  if (cAzEl=='A' && cEndStart=='S')
m257    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 65
        BTFSS 0x03,Zero_
        GOTO  m258
        MOVF  cEndStart,W
        XORLW 83
        BTFSS 0x03,Zero_
        GOTO  m258
                        ;                  {
                        ;                    _u16AzOff=ADCGet(0);
        MOVLW 0
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF _u16AzOff
        MOVF  u16Sum+1,W
        MOVWF _u16AzOff+1
                        ;                    TargetInit();
        CALL  TargetInit
                        ;                    RS232TxMsg("Az Offset=");
        MOVLW 151
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    RS232TxPutU16(_u16AzOff);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u16AzOff,W
        MOVWF u16_3
        MOVF  _u16AzOff+1,W
        MOVWF u16_3+1
        BSF   0x0A,PA1
        CALL  RS232TxPutU16
        BCF   0x0A,PA1
                        ;                    RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    break;
        GOTO  m278
                        ;                  }
                        ;                  if (cAzEl=='A' && cEndStart=='E')
m258    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 65
        BTFSS 0x03,Zero_
        GOTO  m259
        MOVF  cEndStart,W
        XORLW 69
        BTFSS 0x03,Zero_
        GOTO  m259
                        ;                  {
                        ;                    U16 u16=ADCGet(0);
        MOVLW 0
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF u16_11
        MOVF  u16Sum+1,W
        MOVWF u16_11+1
                        ;                    
                        ;                    _fAzMul=((float)(u16-_u16AzOff))/360.0;
        CLRF  arg1f24+2
        MOVF  _u16AzOff+1,W
        SUBWF u16_11+1,W
        MOVWF arg1f24+1
        MOVF  _u16AzOff,W
        SUBWF u16_11,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 135
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fAzMul
        MOVF  arg1f24+1,W
        MOVWF _fAzMul+1
        MOVF  arg1f24+2,W
        MOVWF _fAzMul+2
                        ;                    FLAG_AZ450=FALSE;
        BCF   _u16Flags,0
                        ;                    TargetInit();
        CALL  TargetInit
                        ;                    RS232TxMsg("Az mul=");
        MOVLW 162
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    RS232TxPutFloat(_fAzMul);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fAzMul,W
        MOVWF f_2
        MOVF  _fAzMul+1,W
        MOVWF f_2+1
        MOVF  _fAzMul+2,W
        MOVWF f_2+2
        BSF   0x0A,PA1
        CALL  RS232TxPutFloat
        BCF   0x0A,PA1
                        ;                    RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    break;
        GOTO  m278
                        ;                  }
                        ;                  if (cAzEl=='A' && cEndStart=='F')
m259    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 65
        BTFSS 0x03,Zero_
        GOTO  m260
        MOVF  cEndStart,W
        XORLW 70
        BTFSS 0x03,Zero_
        GOTO  m260
                        ;                  {
                        ;                    U16 u16=ADCGet(0);
        MOVLW 0
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF u16_12
        MOVF  u16Sum+1,W
        MOVWF u16_12+1
                        ;                    
                        ;                    _fAzMul=((float)(u16-_u16AzOff))/450.0;
        CLRF  arg1f24+2
        MOVF  _u16AzOff+1,W
        SUBWF u16_12+1,W
        MOVWF arg1f24+1
        MOVF  _u16AzOff,W
        SUBWF u16_12,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg2f24
        MOVLW 97
        MOVWF arg2f24+1
        MOVLW 135
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fAzMul
        MOVF  arg1f24+1,W
        MOVWF _fAzMul+1
        MOVF  arg1f24+2,W
        MOVWF _fAzMul+2
                        ;                    FLAG_AZ450=TRUE;
        BSF   _u16Flags,0
                        ;                    TargetInit();
        CALL  TargetInit
                        ;                    RS232TxMsg("Az mul=");
        MOVLW 162
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    RS232TxPutFloat(_fAzMul);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fAzMul,W
        MOVWF f_2
        MOVF  _fAzMul+1,W
        MOVWF f_2+1
        MOVF  _fAzMul+2,W
        MOVWF f_2+2
        BSF   0x0A,PA1
        CALL  RS232TxPutFloat
        BCF   0x0A,PA1
                        ;                    RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    break;
        GOTO  m278
                        ;                  }
                        ;                  if (cAzEl=='E' && cEndStart=='O') // Set El Offset directly
m260    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 69
        BTFSS 0x03,Zero_
        GOTO  m261
        MOVF  cEndStart,W
        XORLW 79
        BTFSS 0x03,Zero_
        GOTO  m261
                        ;                  {
                        ;                    U16 u16;
                        ;  
                        ;                    ParseWhite(&u8Pos);
        MOVLW 39
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;                    if (ParseU16(&u8Pos,&u16))
        MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_2
        MOVLW 43
        MOVWF pu16
        BSF   0x0A,PA1
        CALL  ParseU16
        BCF   0x0A,PA1
        BTFSS 0x03,Carry
        GOTO  m261
                        ;                    {
                        ;                      _u16ElOff=u16;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16_13,W
        MOVWF _u16ElOff
        MOVF  u16_13+1,W
        MOVWF _u16ElOff+1
                        ;                      TargetInit();
        CALL  TargetInit
                        ;                      RS232TxMsg("El off=");
        MOVLW 170
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                      RS232TxPutU16(_u16ElOff);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u16ElOff,W
        MOVWF u16_3
        MOVF  _u16ElOff+1,W
        MOVWF u16_3+1
        BSF   0x0A,PA1
        CALL  RS232TxPutU16
        BCF   0x0A,PA1
                        ;                      RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                      break;
        GOTO  m278
                        ;                    }
                        ;                  }
                        ;                  if (cAzEl=='E' && cEndStart=='M') // Set El Mul directly
m261    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 69
        BTFSS 0x03,Zero_
        GOTO  m262
        MOVF  cEndStart,W
        XORLW 77
        BTFSS 0x03,Zero_
        GOTO  m262
                        ;                  {
                        ;                    float f;
                        ;  
                        ;                    ParseWhite(&u8Pos);
        MOVLW 39
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;                    if (ParseFloat(&u8Pos,&f))
        MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_3
        MOVLW 43
        MOVWF pf
        BSF   0x0A,PA1
        CALL  ParseFloat
        BCF   0x0A,PA1
        BTFSS 0x03,Carry
        GOTO  m262
                        ;                    {
                        ;                      _fElMul=f;
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  f_7,W
        MOVWF _fElMul
        MOVF  f_7+1,W
        MOVWF _fElMul+1
        MOVF  f_7+2,W
        MOVWF _fElMul+2
                        ;                      TargetInit();
        CALL  TargetInit
                        ;                      RS232TxMsg("El mul=");
        MOVLW 189
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                      RS232TxPutFloat(_fElMul);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fElMul,W
        MOVWF f_2
        MOVF  _fElMul+1,W
        MOVWF f_2+1
        MOVF  _fElMul+2,W
        MOVWF f_2+2
        BSF   0x0A,PA1
        CALL  RS232TxPutFloat
        BCF   0x0A,PA1
                        ;                      RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                      break;
        GOTO  m278
                        ;                    }
                        ;                  }
                        ;                  if (cAzEl=='E' && cEndStart=='S')
m262    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 69
        BTFSS 0x03,Zero_
        GOTO  m263
        MOVF  cEndStart,W
        XORLW 83
        BTFSS 0x03,Zero_
        GOTO  m263
                        ;                  {
                        ;                    _u16ElOff=ADCGet(1);
        MOVLW 1
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF _u16ElOff
        MOVF  u16Sum+1,W
        MOVWF _u16ElOff+1
                        ;                    TargetInit();
        CALL  TargetInit
                        ;                    RS232TxMsg("El Offset=");
        MOVLW 178
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    RS232TxPutU16(_u16ElOff);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u16ElOff,W
        MOVWF u16_3
        MOVF  _u16ElOff+1,W
        MOVWF u16_3+1
        BSF   0x0A,PA1
        CALL  RS232TxPutU16
        BCF   0x0A,PA1
                        ;                    RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    break;
        GOTO  m278
                        ;                  }
                        ;                  if (cAzEl=='E' && cEndStart=='E')
m263    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 69
        BTFSS 0x03,Zero_
        GOTO  m264
        MOVF  cEndStart,W
        XORLW 69
        BTFSS 0x03,Zero_
        GOTO  m264
                        ;                  {
                        ;                    U16 u16=ADCGet(1);
        MOVLW 1
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF u16_14
        MOVF  u16Sum+1,W
        MOVWF u16_14+1
                        ;                    
                        ;                    _fElMul=((float)(u16-_u16ElOff))/180.0;
        CLRF  arg1f24+2
        MOVF  _u16ElOff+1,W
        SUBWF u16_14+1,W
        MOVWF arg1f24+1
        MOVF  _u16ElOff,W
        SUBWF u16_14,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 134
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fElMul
        MOVF  arg1f24+1,W
        MOVWF _fElMul+1
        MOVF  arg1f24+2,W
        MOVWF _fElMul+2
                        ;                    TargetInit();
        CALL  TargetInit
                        ;                    RS232TxMsg("El mul=");
        MOVLW 189
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    RS232TxPutFloat(_fElMul);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fElMul,W
        MOVWF f_2
        MOVF  _fElMul+1,W
        MOVWF f_2+1
        MOVF  _fElMul+2,W
        MOVWF f_2+2
        BSF   0x0A,PA1
        CALL  RS232TxPutFloat
        BCF   0x0A,PA1
                        ;                    RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    break;
        GOTO  m278
                        ;                  }
                        ;                  if (cAzEl=='E' && cEndStart=='N')
m264    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  cAzEl,W
        XORLW 69
        BTFSS 0x03,Zero_
        GOTO  m265
        MOVF  cEndStart,W
        XORLW 78
        BTFSS 0x03,Zero_
        GOTO  m265
                        ;                  {
                        ;                    U16 u16=ADCGet(1);
        MOVLW 1
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF u16_15
        MOVF  u16Sum+1,W
        MOVWF u16_15+1
                        ;  
                        ;                    _fElMul=((float)(u16-_u16ElOff))/90.0;
        CLRF  arg1f24+2
        MOVF  _u16ElOff+1,W
        SUBWF u16_15+1,W
        MOVWF arg1f24+1
        MOVF  _u16ElOff,W
        SUBWF u16_15,W
        MOVWF arg1f24
        BTFSS 0x03,Carry
        DECF  arg1f24+1,1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  arg2f24
        MOVLW 52
        MOVWF arg2f24+1
        MOVLW 133
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  arg1f24,W
        MOVWF _fElMul
        MOVF  arg1f24+1,W
        MOVWF _fElMul+1
        MOVF  arg1f24+2,W
        MOVWF _fElMul+2
                        ;                    TargetInit();
        CALL  TargetInit
                        ;                    RS232TxMsg("El mul=");
        MOVLW 189
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                    RS232TxPutFloat(_fElMul);
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fElMul,W
        MOVWF f_2
        MOVF  _fElMul+1,W
        MOVWF f_2+1
        MOVF  _fElMul+2,W
        MOVWF f_2+2
        BSF   0x0A,PA1
        CALL  RS232TxPutFloat
        BCF   0x0A,PA1
                        ;                    RS232TxMsg("\r\n");
        MOVLW 98
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF psz
        MOVLW 1
        MOVWF psz+1
        BSF   0x0A,PA1
        CALL  RS232TxMsg
        BCF   0x0A,PA1
                        ;                  }
                        ;                }
                        ;              }
                        ;              RS232TxPutError();
m265    BSF   0x0A,PA1
        CALL  RS232TxPutError
        BCF   0x0A,PA1
                        ;              break;
        GOTO  m278
                        ;            case 'S':
                        ;              _bAzTrack=FALSE;
m266    BCF   0x03,RP0
        BCF   0x03,RP1
        BCF   0x4E,_bAzTrack
                        ;              _bElTrack=FALSE;
        BCF   0x4E,_bElTrack
                        ;              ROT_LEFT=0;
        BCF   PORTC,0
                        ;              ROT_RIGHT=0;
        BCF   PORTC,1
                        ;              ROT_UP=0;
        BCF   PORTC,3
                        ;              ROT_DOWN=0;
        BCF   PORTC,2
                        ;              break;
        GOTO  m278
                        ;            case 'C':
                        ;              if (u8Pos<_u8LinePos)
m267    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _u8LinePos,W
        SUBWF u8Pos_5,W
        BTFSC 0x03,Carry
        GOTO  m268
                        ;              {
                        ;                c=_acLine[u8Pos];
        MOVLW 144
        ADDWF u8Pos_5,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  INDF,W
        MOVWF c_12
                        ;              }
                        ;              // Send out AZ
                        ;              {
                        ;                S16 s16=(S16)ADCGet(0);
m268    MOVLW 0
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF s16_6
        MOVF  u16Sum+1,W
        MOVWF s16_6+1
                        ;  
                        ;                //AK
                        ;                //s16-=(S16)_u16AzOff;
                        ;                //s16/=_fAzMul;
                        ;                s16 = (S16)DEGfromADC(s16);
        MOVF  s16_6,W
        MOVWF u16ADC
        MOVF  s16_6+1,W
        MOVWF u16ADC+1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  DEGfromADC
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16DEG,W
        MOVWF s16_6
        MOVF  u16DEG+1,W
        MOVWF s16_6+1
                        ;                
                        ;                if (FLAG_SNS)
        BTFSS _u16Flags,1
        GOTO  m271
                        ;                {
                        ;                  if (s16<180)
        BTFSC s16_6+1,7
        GOTO  m269
        MOVF  s16_6+1,W
        BTFSS 0x03,Zero_
        GOTO  m270
        MOVLW 180
        SUBWF s16_6,W
        BTFSC 0x03,Carry
        GOTO  m270
                        ;                  {
                        ;                    RS232TxPutS16(s16+180);
m269    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16_6+1,W
        MOVWF s16+1
        MOVLW 180
        ADDWF s16_6,W
        MOVWF s16
        BTFSC 0x03,Carry
        INCF  s16+1,1
        BSF   0x0A,PA1
        CALL  RS232TxPutS16
        BCF   0x0A,PA1
                        ;                  }
                        ;                  else
        GOTO  m272
                        ;                  {
                        ;                    RS232TxPutS16(s16-180);
m270    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16_6+1,W
        MOVWF s16+1
        MOVLW 180
        SUBWF s16_6,W
        MOVWF s16
        BTFSS 0x03,Carry
        DECF  s16+1,1
        BSF   0x0A,PA1
        CALL  RS232TxPutS16
        BCF   0x0A,PA1
                        ;                  }
                        ;                }
                        ;                else
        GOTO  m272
                        ;                {
                        ;                  RS232TxPutS16(s16);
m271    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16_6,W
        MOVWF s16
        MOVF  s16_6+1,W
        MOVWF s16+1
        BSF   0x0A,PA1
        CALL  RS232TxPutS16
        BCF   0x0A,PA1
                        ;                }
                        ;  
                        ;                if (c=='2')
m272    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c_12,W
        XORLW 50
        BTFSS 0x03,Zero_
        GOTO  m273
                        ;                {
                        ;                  //Send out EL
                        ;                  S16 s16=(S16)ADCGet(1);
        MOVLW 1
        BSF   0x0A,PA1
        CALL  ADCGet
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  u16Sum,W
        MOVWF s16_7
        MOVF  u16Sum+1,W
        MOVWF s16_7+1
                        ;  
                        ;                  s16-=(S16)_u16ElOff;
        MOVF  _u16ElOff+1,W
        SUBWF s16_7+1,1
        MOVF  _u16ElOff,W
        SUBWF s16_7,1
        BTFSS 0x03,Carry
        DECF  s16_7+1,1
                        ;                  s16/=_fElMul;
        MOVF  s16_7,W
        MOVWF arg1f24
        MOVF  s16_7+1,W
        MOVWF arg1f24+1
        CLRF  arg1f24+2
        BTFSC arg1f24+1,7
        DECF  arg1f24+2,1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _int24ToFloat24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  _fElMul,W
        MOVWF arg2f24
        MOVF  _fElMul+1,W
        MOVWF arg2f24+1
        MOVF  _fElMul+2,W
        MOVWF arg2f24+2
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _fdiv24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  _float24ToInt24
        BCF   0x0A,PA0
        BCF   0x0A,PA1
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  rval_3,W
        MOVWF s16_7
        MOVF  rval_3+1,W
        MOVWF s16_7+1
                        ;                  RS232TxPutS16(s16);
        MOVF  s16_7,W
        MOVWF s16
        MOVF  s16_7+1,W
        MOVWF s16+1
        BSF   0x0A,PA1
        CALL  RS232TxPutS16
        BCF   0x0A,PA1
                        ;                }
                        ;                RS232TxPutChar('\r');
m273    MOVLW 13
        BSF   0x0A,PA1
        CALL  RS232TxPutChar
        BCF   0x0A,PA1
                        ;                RS232TxPutChar('\n');
        MOVLW 10
        BSF   0x0A,PA1
        CALL  RS232TxPutChar
        BCF   0x0A,PA1
                        ;              }
                        ;              break;
        GOTO  m278
                        ;            case 'W':
                        ;            case 'M': // 'M' command added - supposed to set Az only
                        ;              {
                        ;                S16 s16Az=32767; // wild numbers signify ignore setting
m274    MOVLW 255
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF s16Az_3
        MOVLW 127
        MOVWF s16Az_3+1
                        ;                S16 s16El=32767;
        MOVLW 255
        MOVWF s16El_3
        MOVLW 127
        MOVWF s16El_3+1
                        ;  
                        ;                ParseWhite(&u8Pos);
        MOVLW 39
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;                if (!ParseU16(&u8Pos,(U16 *)&s16Az))
        MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_2
        MOVLW 41
        MOVWF pu16
        BSF   0x0A,PA1
        CALL  ParseU16
        BCF   0x0A,PA1
        BTFSC 0x03,Carry
        GOTO  m275
                        ;                {
                        ;                  RS232TxPutError();
        BSF   0x0A,PA1
        CALL  RS232TxPutError
        BCF   0x0A,PA1
                        ;                }
                        ;                else
        GOTO  m278
                        ;                {
                        ;           				if (c=='W') // only for W command do we need El
m275    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  c_12,W
        XORLW 87
        BTFSS 0x03,Zero_
        GOTO  m276
                        ;           				{
                        ;           					// If no El, we ignore.
                        ;   	                ParseWhite(&u8Pos);
        MOVLW 39
        MOVWF pu8Pos
        BSF   0x0A,PA1
        CALL  ParseWhite
        BCF   0x0A,PA1
                        ;       	            ParseU16(&u8Pos,(U16 *)&s16El);
        MOVLW 39
        BCF   0x03,RP0
        BCF   0x03,RP1
        MOVWF pu8Pos_2
        MOVLW 43
        MOVWF pu16
        BSF   0x0A,PA1
        CALL  ParseU16
        BCF   0x0A,PA1
                        ;           				}
                        ;                  RotatorSet(s16Az,s16El);
m276    BCF   0x03,RP0
        BCF   0x03,RP1
        MOVF  s16Az_3,W
        MOVWF s16Az_2
        MOVF  s16Az_3+1,W
        MOVWF s16Az_2+1
        MOVF  s16El_3,W
        MOVWF s16El_2
        MOVF  s16El_3+1,W
        MOVWF s16El_2+1
        CALL  RotatorSet
                        ;                }
                        ;              }
                        ;              break;
        GOTO  m278
                        ;            default:
                        ;              RS232TxPutError();
m277    BSF   0x0A,PA1
        CALL  RS232TxPutError
        BCF   0x0A,PA1
                        ;              break;
                        ;          }
                        ;        }
                        ;      }
                        ;      _u8LinePos=0;
m278    BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  _u8LinePos
                        ;    }
                        ;    else
        GOTO  m236
                        ;    {
                        ;      if (_u8LinePos<RS232_LINESIZE)
m279    MOVLW 60
        BCF   0x03,RP0
        BCF   0x03,RP1
        SUBWF _u8LinePos,W
        BTFSC 0x03,Carry
        GOTO  m236
                        ;      {
                        ;        _acLine[_u8LinePos]=c;
        MOVLW 144
        ADDWF _u8LinePos,W
        MOVWF FSR
        BSF   0x03,IRP
        MOVF  c_12,W
        MOVWF INDF
                        ;        _u8LinePos++;
        INCF  _u8LinePos,1
                        ;      }
                        ;    }
                        ;  }
        GOTO  m236
                        ;}
m280    RETURN
                        ;
                        ;
                        ;void main(void)
                        ;{
main
                        ;  //AK
                        ;//  _u16AzOffset[0] = 100;
                        ;//  _u16AzOffset[1] = 1100;
                        ;//  _u16AzOffset[2] = 1500;
                        ;//  _u16AzOffset[3] = 1800;
                        ;//  _u16AzOffset[4] = 2000;
                        ;//  _fAzMult[0] = 11.11;
                        ;//  _fAzMult[1] = 4.44;
                        ;//  _fAzMult[2] = 3.33;
                        ;//  _fAzMult[3] = 2.22;
                        ;//  _fAzMult[4] = 1.11;
                        ;//  U16 u16ADC = ADCfromDEG(46);
                        ;//  U16 u16Degrees = DEGfromADC(650);
                        ;    
                        ;  // TRISB Init to disable LCD E line (PORTB.2) and
                        ;  // enable portb.3 as output and set to 1 (disable buttons)
                        ;  // Bit 7 is CTS back to PC - needs to be set on or else TXD
                        ;  // connects thru to CTS via 4.7k resistor and makes some 
                        ;  // host progs loopy (eg SatPC32).
                        ;  TRISB=0b.0100.0000;
        MOVLW 64
        BSF   0x03,RP0
        BCF   0x03,RP1
        MOVWF TRISB
                        ;  PORTB=0b.1000.1000; // Set CTS on!
        MOVLW 136
        BCF   0x03,RP0
        MOVWF PORTB
                        ;  _u8Buttons=0;
        CLRF  _u8Buttons
                        ;
                        ;  // ADC Init
                        ;  TRISA = 0b.1111.1111;  /* 1=input */
        MOVLW 255
        BSF   0x03,RP0
        MOVWF TRISA
                        ;
                        ;#ifdef __CC5X__
                        ;  PCFG0=0; // VDD/VSS ref, zero channels Must do this or PORTA disabled for output
        BCF   0x9F,PCFG0
                        ;  PCFG1=1;
        BSF   0x9F,PCFG1
                        ;  PCFG2=1;
        BSF   0x9F,PCFG2
                        ;  PCFG3=0;
        BCF   0x9F,PCFG3
                        ;#endif
                        ;
                        ;#ifdef __CC8E__
                        ;    PCFG0=1; // VDD/VSS ref, zero channels
                        ;    PCFG1=1;
                        ;    PCFG2=1;
                        ;    PCFG3=1;
                        ;		VCFG0=0;
                        ;		VCFG1=0;
                        ;#endif
                        ;
                        ;  // TMR0 Init
                        ;  _u16Timer=0;
        BCF   0x03,RP0
        CLRF  _u16Timer
        CLRF  _u16Timer+1
                        ;  PSA=0; /* Prescaler assigned to TMR0 */
        BSF   0x03,RP0
        BCF   0x81,PSA
                        ;  PS0=1; /* 256 prescale on FOSC/4 */
        BSF   0x81,PS0
                        ;  PS1=1;
        BSF   0x81,PS1
                        ;  PS2=1; 
        BSF   0x81,PS2
                        ;  T0CS=0; /* select crystal osc (FOSC/4) */
        BCF   0x81,T0CS
                        ;  T0IE=1; /* Enable timer interrupt */
        BSF   0x0B,T0IE
                        ;
                        ;  // USART init
                        ;  RS232TxInit();
        BSF   0x0A,PA1
        CALL  RS232TxInit
        BCF   0x0A,PA1
                        ;  RS232RxInit();
        BSF   0x0A,PA1
        CALL  RS232RxInit
        BCF   0x0A,PA1
                        ;  SPBRG=RS232_BRG;
        MOVLW 25
        BSF   0x03,RP0
        BCF   0x03,RP1
        MOVWF SPBRG
                        ;  BRGH=1; /* High speed Baud rate */
        BSF   0x98,BRGH
                        ;  SYNC=0; /* Async USART mode */
        BCF   0x98,SYNC
                        ;  TX9=0; /* 8 bit */
        BCF   0x98,TX9
                        ;  RX9=0;
        BCF   0x03,RP0
        BCF   0x18,RX9
                        ;  SPEN=1; /* Enable serial port */
        BSF   0x18,SPEN
                        ;  TXEN=1; /* Enable TX */
        BSF   0x03,RP0
        BSF   0x98,TXEN
                        ;  CREN=1; /* Enable Rx */
        BCF   0x03,RP0
        BSF   0x18,CREN
                        ;  TRISC.7=1; /* RX bit input */
        BSF   0x03,RP0
        BSF   TRISC,7
                        ;  TRISC.6=0; /* TX bit output */
        BCF   TRISC,6
                        ;  TXIE=0; /* Disable Tx interrupt (for now) */
        BCF   0x8C,TXIE
                        ;  RCIE=1; /* Enable Rx interrupt */
        BSF   0x8C,RCIE
                        ;  PEIE=1; /* Peripheral enable interrupts */
        BSF   0x0B,PEIE
                        ;
                        ;  // Enable Rotator bits as output
                        ;  TRISC.0=0;
        BCF   TRISC,0
                        ;  TRISC.1=0;
        BCF   TRISC,1
                        ;  TRISC.2=0;
        BCF   TRISC,2
                        ;  TRISC.3=0;
        BCF   TRISC,3
                        ;
                        ;  // No movement required
                        ;  ROT_LEFT=0;
        BCF   0x03,RP0
        BCF   PORTC,0
                        ;  ROT_RIGHT=0;
        BCF   PORTC,1
                        ;  ROT_DOWN=0;
        BCF   PORTC,2
                        ;  ROT_UP=0;
        BCF   PORTC,3
                        ;
                        ;  // LCD Init
                        ;  LCDInit();
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  LCDInit
        BCF   0x0A,PA0
        BCF   0x0A,PA1
                        ;
                        ;  // Welcome message on LCD
                        ;  LCDWelcome();
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        CALL  LCDWelcome
        BCF   0x0A,PA0
        BCF   0x0A,PA1
                        ;
                        ;  // Initialise variables from EEPROM
                        ;  EEInit();
        BSF   0x0A,PA1
        CALL  EEInit
        BCF   0x0A,PA1
                        ;
                        ;  // 'zeroise' target Az/El and stop tracking
                        ;  TargetInit();
        CALL  TargetInit
                        ;
                        ;  // Initialise line input parser
                        ;  _u8LinePos=0;
        BCF   0x03,RP0
        BCF   0x03,RP1
        CLRF  _u8LinePos
                        ;
                        ;  //EEInit();
                        ;
                        ;  GIE=1; // Enable interrupts
        BSF   0x0B,GIE
                        ;
                        ;  // If a button's pressed during boot, then we need to go into config mode
                        ;  ButtonConfig();
        BSF   0x0A,PA0
        CALL  ButtonConfig
        BCF   0x0A,PA0
                        ;
                        ;  // Here we go...
                        ;  while (1)
                        ;  {
                        ;    ButtonCheck(0x0F); // Deal with any button presses
m281    MOVLW 15
        BSF   0x0A,PA0
        CALL  ButtonCheck
        BCF   0x0A,PA0
                        ;    RS232Check(); // Deal with any incoming RS232 stuff
        CALL  RS232Check
                        ;    LCDUpdate(); // Update the LCD display
        BSF   0x0A,PA0
        BSF   0x0A,PA1
        GOTO  LCDUpdate
                        ;    RotatorUpdate(); // Update rotator settings - stop tracking if reached destination
m282    BSF   0x0A,PA0
        CALL  RotatorUpdate
        BCF   0x0A,PA0
                        ;  }
        GOTO  m281

        END


; *** KEY INFO ***

; 0x0004 P0   77 word(s)  3 % : serverX
; 0x0051 P0    9 word(s)  0 % : TargetInit
; 0x005A P0  112 word(s)  5 % : EasyCommCommand
; 0x00CA P0  243 word(s) 11 % : RotatorSet
; 0x01BD P0 1295 word(s) 63 % : RS232Check
; 0x06CC P0   93 word(s)  4 % : main

; 0x0800 P1   17 word(s)  0 % : ButtonGetRaw
; 0x0811 P1   67 word(s)  3 % : ButtonGet
; 0x0854 P1   55 word(s)  2 % : ButtonCheck
; 0x088B P1  184 word(s)  8 % : RotatorUpdate
; 0x0943 P1   27 word(s)  1 % : ButtonGetPress
; 0x095E P1  775 word(s) 37 % : ButtonConfig

; 0x12C8 P2   10 word(s)  0 % : RS232RxCharReady
; 0x12D2 P2   23 word(s)  1 % : RS232RxGetChar
; 0x12E9 P2   34 word(s)  1 % : RS232RxPutChar
; 0x130B P2   10 word(s)  0 % : RS232TxCharReady
; 0x1315 P2   23 word(s)  1 % : RS232TxGetChar
; 0x132C P2   36 word(s)  1 % : RS232TxPutChar
; 0x1104 P2   17 word(s)  0 % : toupper
; 0x1115 P2   19 word(s)  0 % : Delay
; 0x1128 P2   10 word(s)  0 % : TimerGet
; 0x1132 P2   53 word(s)  2 % : ParseWhite
; 0x1167 P2   94 word(s)  4 % : ParseU16
; 0x11C5 P2  153 word(s)  7 % : ParseFloat
; 0x125E P2  106 word(s)  5 % : ADCGet
; 0x1350 P2  197 word(s)  9 % : RS232TxPutU16
; 0x1415 P2   24 word(s)  1 % : RS232TxPutS16
; 0x142D P2  117 word(s)  5 % : RS232TxPutFloat
; 0x14A2 P2   32 word(s)  1 % : RS232TxMsg
; 0x1000 P2  260 word(s) 12 % : _const1
; 0x14C2 P2    8 word(s)  0 % : RS232TxPutError
; 0x14CA P2    5 word(s)  0 % : RS232TxInit
; 0x14CF P2    5 word(s)  0 % : RS232RxInit
; 0x14D4 P2   12 word(s)  0 % : EEReadByte
; 0x14E0 P2   21 word(s)  1 % : EEReadWord
; 0x14F5 P2   27 word(s)  1 % : EEReadFloat
; 0x1510 P2   26 word(s)  1 % : EEWriteByte
; 0x152A P2   15 word(s)  0 % : EEWriteWord
; 0x1539 P2   31 word(s)  1 % : EEWriteFloat
; 0x1558 P2   25 word(s)  1 % : EECalcSum
; 0x1571 P2   14 word(s)  0 % : EEWriteSum
; 0x157F P2   27 word(s)  1 % : EECheck
; 0x159A P2  255 word(s) 12 % : EEInit
; 0x1699 P2  150 word(s)  7 % : EEWriteAll

; 0x1800 P3  105 word(s)  5 % : _fmul24
; 0x1869 P3  147 word(s)  7 % : _fdiv24
; 0x18FC P3  201 word(s)  9 % : _fadd24
; 0x19C5 P3    9 word(s)  0 % : _fsub24
; 0x19CE P3   80 word(s)  3 % : _int24ToFloat24
; 0x1A1E P3   99 word(s)  4 % : _float24ToInt24
; 0x1A81 P3  162 word(s)  7 % : DEGfromADC
; 0x1B23 P3   23 word(s)  1 % : LCDWriteNibble
; 0x1B3A P3   31 word(s)  1 % : LCDReadByte
; 0x1B59 P3   44 word(s)  2 % : LCDWaitReady
; 0x1B85 P3   18 word(s)  0 % : LCDWriteData
; 0x1B97 P3   17 word(s)  0 % : LCDCommand
; 0x1BA8 P3    9 word(s)  0 % : LCDSetCursor
; 0x1BB1 P3    7 word(s)  0 % : LCDClear
; 0x1BB8 P3   45 word(s)  2 % : LCDInit
; 0x1BE5 P3    5 word(s)  0 % : LCDPutCh
; 0x1BEA P3  295 word(s) 14 % : LCDPutU16
; 0x1D11 P3   26 word(s)  1 % : LCDPutS16
; 0x1D2B P3   33 word(s)  1 % : LCDMsg
; 0x1D4C P3   48 word(s)  2 % : LCDPutAz
; 0x1D7C P3   42 word(s)  2 % : LCDWelcome
; 0x1DA6 P3  162 word(s)  7 % : LCDUpdate

; RAM usage: 238 bytes (44 local), 130 bytes free
; Maximum call level: 5 (+3 for interrupt)
;  Codepage 0 has 1830 word(s) :  89 %
;  Codepage 1 has 1125 word(s) :  54 %
;  Codepage 2 has 1839 word(s) :  89 %
;  Codepage 3 has 1608 word(s) :  78 %
; Total of 6402 code words (78 %)
