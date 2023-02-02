//From PicProg.c
// Osc=XT, WDT off, PUT on, BOD off, LVP off, FPW on, DBG off, data EE prot off, CodeProt off
//au16Data[0x2007]=0x3F31;
//
//#pragma chip PIC16F876
//#pragma config WDTE = OFF
//#pragma config FOSC = XT
//#pragma config BOREN = OFF
//#pragma config PWRTE = ON
#pragma config = 0x3F31
// Todo:
// * Make work without LCD - OK 0.2
// * Test -180 - 0 - +180 rotator. OK 0.2
// * Test 0 - 450 rotator ("FAF" command) OK 0.2 but no with S-N-S rotator
// * Add Easycomm interface - fixed 0.7
// * Add more GS-232 commands - 'M###' added 0.7
// * Fix 16F876A and combined right/up button push problem. OK 0.2
// * Allow calibration from buttons - switch on with one of the front panel buttons pressed OK 0.2
// * Random Az/El target displayed on boot up OK 0.2
// * Random Az/El target displayed after calibration OK 0.3
// * Floating point OK 0.3
// * Include speed control
// * Fix overshoot
// * Fix SatPC32 problem of repeatedly rewriting - fixed 0.5
// * Bug when Elevation not moving in certain scenarios - fixed 0.6
// * Reduce LCD flicker by only updating occasionally - fixed 0.6
// * Smooth ADC readings by taking an averaged number - fixed 0.6
// * Change defualt ADC constants for my rotator ;-) - fixed 0.6
// * Add 'M' GS-232 command to set Az only - fixed 0.7
// * Multi-platform build for 16F876 and 18F2620 - 0.8
// * Predict compatibility - ignore nuls on RS232 - 0.8
// * Adjust BRG slightly to cope with strange Nova behaviour - 0.9
//
// Modifications by Alex KR1ST. March 2021
// Version 1.2
// Added calibration points for 0, 90, 180, 270, and 360 degrees to support
// rotators with a non-linear azimuth feedback voltage.


#ifdef __CC5X__
#include "int16CXX.h"
#endif

#ifdef __CC8E__

bit PCFG0 @ ADCON1.0;
bit PCFG1 @ ADCON1.1;
bit PCFG2 @ ADCON1.2;
bit PCFG3 @ ADCON1.3;

bit T0PS0 @ T0CON.0;
bit T0PS1 @ T0CON.1;
bit T0PS2 @ T0CON.2;

bit ACQT0 @ ADCON2.3;
bit ACQT1 @ ADCON2.4;
bit ACQT2 @ ADCON2.5;

#include "int18XXX.h"

#define T0IF TMR0IF
#define T0IE TMR0IE
#define TMR0 TMR0L

#define PS0 T0PS0
#define PS1 T0PS1
#define PS2 T0PS2

#endif


#define FALSE 0
#define TRUE 1

#define DELAY_CONSTANT 13  /* Software delay constant: 16 for 4.9152MHz, 13 for 4MHz */

#define LCD_RS PORTB.0
#define LCD_RW PORTB.1
#define LCD_E PORTB.2
#define LCD_TIMERWAIT 200 // Time to wait in ms to update LCD to stop flickering

#define ROT_LEFT PORTC.0
#define ROT_RIGHT PORTC.1
#define ROT_DOWN PORTC.2
#define ROT_UP PORTC.3

#define BTN_LEFT 1
#define BTN_RIGHT 2
#define BTN_DOWN 4
#define BTN_UP 8

#define FLAG_AZ450 (_u16Flags.0) // 450 degree azimuth flag
#define FLAG_SNS (_u16Flags.1) // -180 - 0 - +180 flag

#define EC_NONE 0 // EasyComm command decode numbers
#define EC_AZ 1
#define EC_EL 2
#define EC_UP 3
#define EC_DN 4

//#define RS232_BRG 26 /* Baud rate generator BR=FOSC/(16(SPBRG+1)) 9600 bps 26 for 4MHz, 31 for 4.9152MHz */
// BRG adjustment for 0.9
#define RS232_BRG 25 /* Baud rate generator BR=FOSC/(16(SPBRG+1)) 9600 bps 26 for 4MHz, 31 for 4.9152MHz */
#define RS232_RXBUFFERSIZE 60
#define RS232_TXBUFFERSIZE 20
#define RS232_LINESIZE 60

#define ADC_NUMSAMPLES 25 // number of ADC samples to take to try to smooth things

#define EE_AZMUL 0
#define EE_AZOFF 3
#define EE_ELMUL 5
#define EE_ELOFF 8
#define EE_FLAGS 10
//AK
//#define EE_MAX 11 /* Total number of EE bytes used -1 for checksum purposes */
#define EE_AZOFF_0   11
#define EE_AZOFF_90  13
#define EE_AZOFF_180 15
#define EE_AZOFF_270 17
#define EE_AZOFF_360 19
#define EE_AZMUL_0   21
#define EE_AZMUL_90  24
#define EE_AZMUL_180 27
#define EE_AZMUL_270 30
#define EE_AZMUL_360 33
#define EE_MAX       36 /* Total number of EE bytes used -1 for checksum purposes */

typedef uns32 U32;
typedef uns16 U16;
typedef int16 S16;
typedef uns8 U8;
typedef bit BOOL;

typedef union
{
  float f;
  struct
  {
    U8 u8Lo;
    U8 u8Mid;
    U8 u8Hi;
  } fs;
} FLOATUNION;

static U8 _u8Buttons;
static U8 _u8ButtonsLast;

// RS232 buffers etc
#ifdef __CC5X__
#pragma rambank 2
#endif
static char _acRS232RxData[RS232_RXBUFFERSIZE]; /* Rx Buffer */
#ifdef __CC5X__
#pragma rambank 1
#endif
static char _cRS232RxIn;
static char _cRS232RxOut;
static char _acRS232TxData[RS232_TXBUFFERSIZE]; /* Tx Buffer */
static char _cRS232TxIn;
static char _cRS232TxOut;
#ifdef __CC5X__
#pragma rambank 0
#endif

static BOOL RS232RxCharReady(void);
static char RS232RxGetChar(void);
static BOOL RS232RxPutChar(char c);
static BOOL RS232TxCharReady(void);
static char RS232TxGetChar(void);
static BOOL RS232TxPutChar(char c);

/* Line input parser */
#ifdef __CC5X__
#pragma rambank 3
#endif
static char _acLine[RS232_LINESIZE];
#ifdef __CC5X__
#pragma rambank 0
#endif
static U8 _u8LinePos;

// Are we moving? 
static BOOL _bAzTrack;
static BOOL _bElTrack;

// Target Az/El
static S16 _s16AzTarget;
static S16 _s16ElTarget;

// EEPROM stored values
static float _fAzMul;
static U16 _u16AzOff; // Raw ADC offset for zero Az
static float _fElMul;
static U16 _u16ElOff; // Raw ADC offset for zero El
static U16 _u16Flags; // Flags

//AK
static U16 _u16AzOffset[5];
static float _fAzMult[5];

// Incremented by timer 0 interrupt
static U16 _u16Timer; 

static BOOL _bLCDActive;
static U16 _u16LCDTimer; // Timer setting last time LCD was updated to stop flicker

#ifdef __CC5X__
#pragma origin 4
#endif

#ifdef __CC8E__
#pragma origin 8
#endif

interrupt serverX(void)
{
  // W and STATUS are saved by the next macro.
  // PCLATH is also saved if necessary.
  // The code produced is strongly CPU-dependent.

#ifdef __CC5X__
  int_save_registers    // W, STATUS (and PCLATH)
  char sv_FSR; sv_FSR = FSR;  // if required
#endif

#ifdef __CC8E__
	uns16 sv_FSR0=FSR0;
#endif

  // handle the timer interrupt
  if (T0IF && T0IE)
  {
    _u16Timer++;
    TMR0=250; /* Increments and overflows when rolls over past 255 */
    T0IF=0; /* Must remember to clear overflow or will continuously interrupt! */
  }

  // handle the tx serial interrupt
  if (TXIF && TXIE)
  {
    if (RS232TxCharReady())
    {
      TXREG=RS232TxGetChar();
    }
    else
    {
      TXIE=0;
    }
  }

  // handle the rx serial interrupt
  if (RCIF && RCIE)
  {
    if (OERR)
    {
      CREN=0; /* Must reset Rx logic on overrun */
      CREN=1;
    }
    else
    {
			char c=RCREG; /* New for v0.8 for Predict compatibility */

			if (c!='\0')
			{
	      RS232RxPutChar(c);
			}
    }
  }

#ifdef __CC5X__
  FSR = sv_FSR;               // if required
  int_restore_registers // W, STATUS (and PCLATH)
#endif
#ifdef __CC8E__
	FSR0=sv_FSR0;
#pragma fastMode
#endif
}


#ifdef __CC5X__
#pragma codepage 3
#endif

#include "math24f.h"

#ifdef __CC5X__
#pragma codepage 2
#endif

// Some glue functions...

//static U16 ByteToWord(U8 u8Lo,U8 u8Hi)
//{
//  U16 u16=((U16)u8Hi)<<8;
//  return u16+u8Lo;
//}

static char toupper(char c)
{
  if (c>='a' && c<='z')
  {
    c-='a'-'A';
  }
  return c;
}

static void Delay(unsigned char uc) /* software delay uc * 100us */
{
  while (uc)
  {
    char uc2;

    uc--;

    for (uc2=0;uc2<DELAY_CONSTANT;uc2++)
    {
    }
  }
}

static U16 TimerGet(void)
{
  U16 u16;

  GIE=0;
  u16=_u16Timer;
  GIE=1;

  return u16;
}

static BOOL ParseWhite(U8 *pu8Pos)
{
  // Ignore white space in _acLine. Return FALSE if no white space...
  BOOL bFinished=FALSE;
  BOOL bWhiteFound=FALSE; // at end of string
  U8 u8Pos=*pu8Pos;
  char c;

  while (!bFinished)
  {
    if (u8Pos<_u8LinePos)
    {
      c=_acLine[u8Pos];
      if (c!=' ' && c!='\t')
      {
        bFinished=TRUE;
      }
      else
      {
        bWhiteFound=TRUE;
        u8Pos++;
      }
    }
    else
    {
      bFinished=TRUE;
    }
  }
  *pu8Pos=u8Pos;
  return bWhiteFound;  
}

static BOOL ParseU16(U8 *pu8Pos,U16 *pu16)
{
  // Parse _acLine from *pu8Pos for a number. Return FALSE if error...
  BOOL bFinished=FALSE;
  BOOL bNumFound=FALSE; // at end of string
  U8 u8Pos=*pu8Pos;
  char c;
  U16 u16=0;

  while (!bFinished)
  {
    if (u8Pos<_u8LinePos)
    {
      c=_acLine[u8Pos];
      if (c<'0' || c>'9')
      {
        bFinished=TRUE;
      }
      else
      {
        bNumFound=TRUE;
        u16*=10;
        c-='0';
        u16+=c;
        u8Pos++;
      }
    }
    else
    {
      bFinished=TRUE;
    }
  }
  *pu8Pos=u8Pos;
  if (bNumFound)
  {
    *pu16=u16;
  }
  return bNumFound;  
}

static BOOL ParseFloat(U8 *pu8Pos,float *pf)
{
  U16 u16Int;
  U16 u16Frac=0;
  float f;
  U8 u8;

  if (!ParseU16(pu8Pos,&u16Int))
  {
    return FALSE;
  }
  f=u16Int;
  u8=*pu8Pos;
  if (_acLine[u8]=='.')
  {
    U8 u8Pos;

    (*pu8Pos)++;

    u8Pos=*pu8Pos; // So we know how many digits in the fractional part

    if (ParseU16(pu8Pos,&u16Frac))
    {
      float fFrac=u16Frac;

      u8Pos=*pu8Pos-u8Pos;
      while (u8Pos)
      {
        fFrac/=10;
        u8Pos--;
      }
      f+=fFrac;
    }
  }
  *pf=f;
  return TRUE;
}

/*************** ADC functions ****************/

static U16 ADCGet(U8 u8Channel)
{
  int n;
  U16 u16Sum=0;

  for (n=0;n<ADC_NUMSAMPLES;n++)
  {
    U16 u16Result=0;

    ADFM=1; // Right Justified

#ifdef __CC5X__
    PCFG0=0; // VDD/VSS ref, three channels
    PCFG1=0;
    PCFG2=1;
    PCFG3=0;
#endif

#ifdef __CC8E__
    PCFG0=1; // VDD/VSS ref, two channels
    PCFG1=0;
    PCFG2=1;
    PCFG3=1;
		VCFG0=0;
		VCFG1=0;
#endif

    CHS0=u8Channel.0;
    CHS1=u8Channel.1;
    CHS2=u8Channel.2;

#ifdef __CC8E__
	  CHS3=u8Channel.3; /********NEW********/
#endif


    ADCS0=0; // FOSC/32
    ADCS1=1;

#ifdef __CC8E__
	  ADCS2=0; /********NEW********/
#endif

    ADON=1; // switch on A/D

    Delay(1); // Wait for required acquisition time

    GO=1; // Set Go/!Done bit to start conversion

    while (GO) // Wait for Go/!Done bit to clear
    {
    }

    u16Result=((U16)(ADRESH))<<8;
    u16Result|=ADRESL;

    ADON=0; // switch off A/D

#ifdef __CC5X__
    PCFG0=0; // VDD/VSS ref, zero channels
    PCFG1=1;
    PCFG2=1;
    PCFG3=0;
#endif

#ifdef __CC8E__
    PCFG0=1; // VDD/VSS ref, zero channels
    PCFG1=1;
    PCFG2=1;
    PCFG3=1;
		VCFG0=0;
		VCFG1=0;
#endif


    u16Sum+=u16Result;  
  }
  u16Sum/=ADC_NUMSAMPLES;
  return u16Sum;
}

/********* RS232 Functions *************/

static BOOL RS232RxCharReady(void)
{
  if (_cRS232RxIn==_cRS232RxOut)
  {
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}

static char RS232RxGetChar(void)
{
  if (RS232RxCharReady())
  {
    char c=_acRS232RxData[_cRS232RxOut];
    _cRS232RxOut++;
    if (_cRS232RxOut>=RS232_RXBUFFERSIZE)
    {
      _cRS232RxOut=0;
    }
    return c;
  }
  else
  {
    return FALSE;
  }
}

static BOOL RS232RxPutChar(char c)
{
  char cNextIdx=_cRS232RxIn+1;

  if (cNextIdx>=RS232_RXBUFFERSIZE)
  {
    cNextIdx=0;
  }

  if (cNextIdx==_cRS232RxOut)
  {
    return FALSE;
  }
  _acRS232RxData[_cRS232RxIn]=c;
  _cRS232RxIn=cNextIdx;
  return TRUE;
}

static BOOL RS232TxCharReady(void)
{
  if (_cRS232TxIn==_cRS232TxOut)
  {
    return FALSE;
  }
  else
  {
    return TRUE;
  }
}

static char RS232TxGetChar(void)
{
  if (RS232TxCharReady())
  {
    char c=_acRS232TxData[_cRS232TxOut];
    _cRS232TxOut++;
    if (_cRS232TxOut>=RS232_TXBUFFERSIZE)
    {
      _cRS232TxOut=0;
    }
    return c;
  }
  else
  {
    return FALSE;
  }
}

static BOOL RS232TxPutChar(char c)
{
  char cNextIdx=_cRS232TxIn+1;

  if (cNextIdx>=RS232_TXBUFFERSIZE)
  {
    cNextIdx=0;
  }

  if (cNextIdx==_cRS232TxOut)
  {
    return FALSE;
  }
  _acRS232TxData[_cRS232TxIn]=c;
  _cRS232TxIn=cNextIdx;

  if (!TXIE)
  {
    TXIE=1;
  }
  return TRUE;
}

static void RS232TxPutU16(U16 u16)
{
  U16 u16a;
  U16 u16b;

  u16a=u16/1000;
  u16b=u16a*1000;

  RS232TxPutChar((U8)u16a+'0');

  u16-=u16b;
  u16a=u16/100;
  u16b=100*u16a;
  RS232TxPutChar((U8)u16a+'0');

  u16-=u16b;
  u16a=u16/10;
  u16b=u16a*10;
  RS232TxPutChar((U8)u16a+'0');

  u16-=u16b;
  RS232TxPutChar((U8)u16+'0');
}

static void RS232TxPutS16(S16 s16)
{
  char c='+';

  if (s16<0)
  {
    s16=-s16;
    c='-';
  }
  RS232TxPutChar(c);
  RS232TxPutU16((U16)s16);
}

static void RS232TxPutFloat(float f)
{
  float f2;

  if (f<0.0)
  {
    f=0.0;
  }

  RS232TxPutU16((U16)f);
  RS232TxPutChar('.');
  f2=(U16)f; // remove integer part
  f-=f2;
  f*=10000.0;
  RS232TxPutU16((U16)f);
}

static BOOL RS232TxMsg(const char *psz)
{
  while (*psz)
  {
    BOOL b=RS232TxPutChar(*psz++);

    if (!b)
    {
      return FALSE;
    }
  }
  return TRUE;
}

static void RS232TxPutError(void)
{
  RS232TxPutChar('?');
  RS232TxPutChar('>');
  RS232TxPutChar('\r');
  RS232TxPutChar('\n');
}

static void RS232TxInit(void)
{
  _cRS232TxIn=0;
  _cRS232TxOut=0;
}

static void RS232RxInit(void)
{
  _cRS232RxIn=0;
  _cRS232RxOut=0;
}

#pragma codepage 3
/**************** Degree from ADC conversion functions *******************/
//AK

//static U16 ADCfromDEG(U16 u16DEG)
//{
//    U8    u8Index;
//    U16   u16Offset;
//    float fMult;
//    U16   u16Mod;
//    U16   u16ADC;
//    
//    u8Index   = u16DEG / 90;
//    u16Offset = _u16AzOffset[u8Index];
//    fMult     = _fAzMult[u8Index];
//    u16Mod    = u16DEG % 90;
//    u16ADC    = u16Mod * fMult;
//    u16ADC   += u16Offset;
//    
//    return u16ADC; 
//}

static U16 DEGfromADC(U16 u16ADC)
{
    U8    u8Index;
    U16   u16Index;
    U16   u16Offset;
    float fMult;
    U16   u16DEG;
    U16   u16DEGOffset;

    if (u16ADC <= _u16AzOffset[0]) 
    {
        u16DEG = 0;
    }
    else
    {
        if (u16ADC >= _u16AzOffset[4])
        {
            u16DEG = 360;
        }
        else 
        {
            for (u8Index = 0; u8Index < 5; u8Index++) {
                u16Offset = _u16AzOffset[u8Index];
                if (u16Offset > u16ADC)
                    break;
            }
            u8Index--;
            u16Offset    = _u16AzOffset[u8Index];  
            fMult        = _fAzMult[u8Index];
            u16DEG       = u16ADC - u16Offset;
            u16DEG       = u16DEG / fMult;
            u16Index     = u8Index;
            u16DEGOffset = 90 * u16Index;
            u16DEG       = u16DEG + u16DEGOffset;
        }
    }
    return u16DEG;
}
/***************** LCD functions *****************/

static void LCDWriteNibble(unsigned char uc) /* RS must be set/reset before calling */
{
  uc<<=2; /* Align with bits 2-5 */
  LCD_RW=0;
  TRISA=0b.1100.0011; /* Set to output bits 2-5 */
  PORTA=uc;
  LCD_E=1;
  nop();
  nop();
  LCD_E=0;
  LCD_RW=1;
  TRISA=0b.1111.1111; /* Set to input bits 2-5 */  
}

static unsigned char LCDReadByte(void) /* RS must be set/reset before calling */
{
  unsigned char uc,uc2;

  LCD_RW=1;
  LCD_E=1;
  nop();
  nop();
  uc=PORTA;
  LCD_E=0;
  uc<<=2;
  uc&=0xF0;
  LCD_E=1;
  nop();
  nop();
  uc2=PORTA;
  LCD_E=0;
  uc2>>=2;
  uc2&=0x0F;
  uc|=uc2;
  return uc;
}

static BOOL LCDWaitReady(void)
{
  if (_bLCDActive)
  {
    U16 u16=TimerGet();

    while (LCDReadByte() & 0x80)
    {
      U16 u16Diff=TimerGet()-u16;

      if (u16Diff>5) // Timeout
      {
        _bLCDActive=FALSE;
        return FALSE;
      }
    }
  }
  return TRUE;
}

static void LCDWriteData(unsigned char uc)
{
  if (_bLCDActive)
  {
    LCD_RS=0;
    LCDWaitReady();
    LCD_RS=1;

    LCDWriteNibble(uc>>4);
    LCDWriteNibble(uc);
  }
}

//static unsigned char LCDReadData(void)
//{
//  if (_bLCDActive)
//  {
//    LCD_RS=0;
//    LCDWaitReady();
//    LCD_RS=1;
//    return LCDReadByte();
//  }
//  else
//  {
//    return 0;
//  }
//}

static void LCDCommand(unsigned char uc)
{
  if (_bLCDActive)
  {
    LCD_RS=0; /* Instruction mode */
    LCDWaitReady();

    LCDWriteNibble(uc>>4);
    LCDWriteNibble(uc);
  }
}

static void LCDSetCursor(unsigned char ucPos)
{
  if (_bLCDActive)
  {
    LCDCommand(0x80 | ucPos);
  }
}

static void LCDClear(void)
{
  if (_bLCDActive)
  {
    LCDCommand(0x01);
  }
}

static void LCDInit(void)
{
  _bLCDActive=TRUE; // Let's be optimistic
  _u16LCDTimer=0; // Initialize timer

  LCD_E=0;
  LCD_RS=0;
  Delay(150);
  LCDWriteNibble(3);
  Delay(50);
  LCDWriteNibble(3);
  Delay(2);
  LCDWriteNibble(3);
  Delay(2);
  LCDWriteNibble(2);
  Delay(2);
  LCDCommand(0b.0010.1000);
  LCDCommand(0b.0000.1000);
  LCDCommand(0b.0000.0001);
  LCDCommand(0b.0000.0110);
  LCDCommand(0b.0000.1111);
}

static void LCDPutCh(char c)
{
  LCDWriteData(c);
}

static void LCDPutU16(U16 u16,U8 u8NumDigs)
{
  if (_bLCDActive)
  {
    U16 u16a;
    U16 u16b;

    u16a=u16/1000;
    u16b=u16a*1000;

    if (u8NumDigs>=5)
    {
      LCDPutCh((U8)u16a+'0');
    }

    u16-=u16b;
    u16a=u16/1000;
    u16b=u16a*1000;
    if (u8NumDigs>=4)
    {
      LCDPutCh((U8)u16a+'0');
    }

    u16-=u16b;
    u16a=u16/100;
    u16b=100*u16a;
    if (u8NumDigs>=3)
    {
      LCDPutCh((U8)u16a+'0');
    }

    u16-=u16b;
    u16a=u16/10;
    u16b=u16a*10;
    if (u8NumDigs>=2)
    {
      LCDPutCh((U8)u16a+'0');
    }

    u16-=u16b;
    if (u8NumDigs>=1)
    {
      LCDPutCh((U8)u16+'0');
    }
  }
}

static void LCDPutS16(S16 s16,U8 u8NumDigs)
{
  char c='+';

  if (s16<0)
  {
    s16=-s16;
    c='-';
  }
  LCDPutCh(c);
  LCDPutU16((U16)s16,u8NumDigs);
}

// Display adhoc string message on the LCD
static void LCDMsg(const char *psz)
{
  if (_bLCDActive)
  {
    while (*psz)
    {
      LCDPutCh(*psz++);
    }
  }
}

// Display signed Azimuth
static void LCDPutAz(S16 s16)
{
  if (_bLCDActive)
  {
    if (FLAG_SNS)
    {
      if (s16<180)
      {
        LCDPutS16(s16+180,3);
      }
      else
      {
        LCDPutS16(s16-180,3);
      }
    }
    else
    {
      LCDPutS16(s16,3);
    }
  }
}

 static void LCDWelcome(void)
{
  if (_bLCDActive)
  {
    U16 u16;
    LCDClear();
    LCDMsg("LVB Tracker");
    LCDSetCursor(40);
    LCDMsg("Firmware v1.2");
    for (u16=0;u16<200;u16++) // wait 2s
    {
      Delay(100);
    }
    LCDClear();
  }
}

// Update LCD display
static void LCDUpdate(void)
{
  U16 u16Timer=TimerGet();

  u16Timer-=_u16LCDTimer;

  if (_bLCDActive && u16Timer>LCD_TIMERWAIT)
  {
    S16 s16;

    _u16LCDTimer+=u16Timer;

    LCDSetCursor(0);

    LCDMsg("Az ");
 
    //AK
    s16=(S16)ADCGet(0);
    //s16-=(S16)_u16AzOff;
    //s16/=_fAzMul;
    s16 = (S16)DEGfromADC(s16);

    LCDPutAz(s16);
    LCDPutCh(0xDF);

    LCDMsg(" (");

    LCDPutAz(_s16AzTarget);
    LCDPutCh(0xDF);
    LCDPutCh(')');

    LCDSetCursor(40);

    LCDMsg("El ");
 
    s16=(S16)ADCGet(1);
    s16-=(S16)_u16ElOff;
    s16/=_fElMul;

    LCDPutS16(s16,3);
    LCDPutCh(0xDF);

    LCDMsg(" (");

    LCDPutS16((S16)_s16ElTarget,3);
    LCDPutCh(0xDF);
    LCDPutCh(')');
  }
}

#ifdef __CC5X__
//#pragma codepage 2
#pragma codepage 2
#endif

/***************** EEPROM functions ******************/

// Read a byte of EEPROM data
static U8 EEReadByte(U8 u8Addr)
{
  EEADR=u8Addr;
  EEPGD=0; // Clear EEPGD
  EECON1.0=1; // Set control bit RD
  return EEDATA;
}

// Read a Word of EEPROM data
static U16 EEReadWord(U8 u8Addr)
{
  U16 u16;

  u16=EEReadByte(u8Addr+1);
  u16<<=8;
  u16+=EEReadByte(u8Addr);

  return u16;
}

static float EEReadFloat(U8 u8Addr)
{
  FLOATUNION fu;

  fu.fs.u8Lo=EEReadByte(u8Addr);
  fu.fs.u8Mid=EEReadByte(u8Addr+1);
  fu.fs.u8Hi=EEReadByte(u8Addr+2);

  return fu.f;
}

// Write a byte of EEPROM data
static void EEWriteByte(U8 u8Addr,U8 u8Data)
{
  EEADR=u8Addr;
  EEDATA=u8Data;
  EEPGD=0; // Clear EEPGD
  WREN=1;
  GIE=0;
  EECON2=0x55;
  EECON2=0xAA;
  WR=1;
  GIE=1;
  WREN=0;  
  while (WR)
  {
  }  
}

// Write a Word of EEPROM data
static void EEWriteWord(U8 u8Addr,U16 u16Data)
{
  EEWriteByte(u8Addr,(U8)u16Data);
  u16Data>>=8;
  EEWriteByte(u8Addr+1,u16Data);
}

// Write a Float of EEPROM data
static void EEWriteFloat(U8 u8Addr,float f)
{
  FLOATUNION *pfu=&f;
  EEWriteByte(u8Addr,pfu->fs.u8Lo);
  EEWriteByte(u8Addr+1,pfu->fs.u8Mid);
  EEWriteByte(u8Addr+2,pfu->fs.u8Hi);
}

// Calculate the EEPROM checksum
static U16 EECalcSum(void)
{
  U8 u8;
  U16 u16Sum=0;

  for (u8=0;u8<EE_MAX;u8++)
  {
    u16Sum+=(U8)(EEReadByte(u8)+u8);
  }
  return u16Sum;
}

// Write the EEPROM checksum
static void EEWriteSum(void)
{
  U16 u16=EECalcSum();

  EEWriteWord(254,u16);
}

// Check the EEPROM checksum is OK
static BOOL EECheck(void)
{
  BOOL b;

  U16 u16SumRead=EEReadWord(254);
  U16 u16SumCalc=EECalcSum();

  if (u16SumRead==u16SumCalc)
  {
    return TRUE;
  }
  return FALSE;
}

// If EEPROM values are OK, load them up, otherwise choose some default values.
// Warns on LCD if EEPROM values are wrong.
static void EEInit()
{
  if (!EECheck())
  {
    U16 u16;

    LCDClear();
    LCDMsg("Warning: bad");
    LCDSetCursor(40);
    LCDMsg("EEPROM data.");
    _fAzMul=1.8583;
    _u16AzOff=8;
    _fElMul=4.5555;
    _u16ElOff=13;
    _u16Flags=0;
    //AK
    _u16AzOffset[0] = 100;
    _u16AzOffset[1] = 1100;
    _u16AzOffset[2] = 1500;
    _u16AzOffset[3] = 1800;
    _u16AzOffset[4] = 2000;
    _fAzMult[0] = 11.11;
    _fAzMult[1] = 4.44;
    _fAzMult[2] = 3.33;
    _fAzMult[3] = 2.22;
    _fAzMult[4] = 1.11;
    
    for (u16=0;u16<200;u16++) // wait 2s
    {
      Delay(100);
    }
    LCDClear();
  }
  else
  {
    _fAzMul=EEReadFloat(EE_AZMUL);
    _u16AzOff=EEReadWord(EE_AZOFF);
    _fElMul=EEReadFloat(EE_ELMUL);    
    _u16ElOff=EEReadWord(EE_ELOFF);
    _u16Flags=EEReadWord(EE_FLAGS);
    //AK
    _u16AzOffset[0] = EEReadWord(EE_AZOFF_0);
    _u16AzOffset[1] = EEReadWord(EE_AZOFF_90);
    _u16AzOffset[2] = EEReadWord(EE_AZOFF_180);
    _u16AzOffset[3] = EEReadWord(EE_AZOFF_270);
    _u16AzOffset[4] = EEReadWord(EE_AZOFF_360);
    _fAzMult[0]     = EEReadFloat(EE_AZMUL_0);
    _fAzMult[1]     = EEReadFloat(EE_AZMUL_90);
    _fAzMult[2]     = EEReadFloat(EE_AZMUL_180);
    _fAzMult[3]     = EEReadFloat(EE_AZMUL_270);
    _fAzMult[4]     = EEReadFloat(EE_AZMUL_360);
  }
}

// Write current RAM values to EEPROM
static BOOL EEWriteAll(void)
{
  EEWriteFloat(EE_AZMUL,_fAzMul);
  EEWriteWord(EE_AZOFF,_u16AzOff);
  EEWriteFloat(EE_ELMUL,_fElMul);
  EEWriteWord(EE_ELOFF,_u16ElOff);
  EEWriteWord(EE_FLAGS,_u16Flags);
  //EEWriteSum();
  //AK
  EEWriteWord(EE_AZOFF_0,_u16AzOffset[0]);
  EEWriteWord(EE_AZOFF_90,_u16AzOffset[1]);
  EEWriteWord(EE_AZOFF_180,_u16AzOffset[2]);
  EEWriteWord(EE_AZOFF_270,_u16AzOffset[3]);
  EEWriteWord(EE_AZOFF_360,_u16AzOffset[4]);
  EEWriteFloat(EE_AZMUL_0,_fAzMult[0]);
  EEWriteFloat(EE_AZMUL_90,_fAzMult[1]);
  EEWriteFloat(EE_AZMUL_180,_fAzMult[2]);
  EEWriteFloat(EE_AZMUL_270,_fAzMult[3]);
  EEWriteFloat(EE_AZMUL_360,_fAzMult[4]);
  EEWriteSum();
        
  return EECheck();
}

#ifdef __CC5X__
//#pragma codepage 0
#pragma codepage 1
#endif


/**************** Button functions *******************/

static U8 ButtonGetRaw(void)
{
  // Not debounced.
  U8 u8;

  PORTB.3=0; // Enable buttons
  nop(); // Need this delay!!! 16F876A does not settle in time otherwise.
  u8=PORTA; // Read buttons
  PORTB.3=1; // Disable buttons
  u8>>=2;
  u8&=0x0F;
  u8^=0x0F;
  return u8;
}

static U8 ButtonGet(void)
{
  // Wait for 5ms of completely stable buttons to debounce
  U8 u8Org=ButtonGetRaw();
  U16 u16Org=TimerGet();
  U8 u8New=u8Org;
  U16 u16New=u16Org;
  U16 u16Diff;

  do
  {
 
    u8New=ButtonGetRaw();
    u16New=TimerGet();

    if (u8New!=u8Org)
    {
      u8Org=u8New;
      u16Org=u16New; // reset timer
    }
    u16Diff=u16New-u16Org;
  } while (u16Diff<5);

  return u8New;
}

// Deal with any button pushes and return immediately
// Only moves rotator according to mask
// Mask bit 0 Left
// Mask bit 1 Right
// Mask bit 2 Down
// Mask bit 3 Up
// Returns button state
static U8 ButtonCheck(U8 u8Mask)
{
  U8 u8;

  // Any change in button status?
  _u8Buttons=ButtonGet();

  u8=_u8Buttons^_u8ButtonsLast;

  if (u8!=0)
  {
    if (u8 & 0x03 & u8Mask)
    {
      ROT_LEFT=0;
      ROT_RIGHT=0;
      _bAzTrack=FALSE;
      if (_u8Buttons & BTN_LEFT)
      {
        ROT_LEFT=1;
      }
      else
      {
        if (_u8Buttons & BTN_RIGHT)
        {
          ROT_RIGHT=1;
        }
      }
    }
    if (u8 & 0x0C & u8Mask)
    {
      ROT_DOWN=0;
      ROT_UP=0;
      _bElTrack=FALSE;
      if (_u8Buttons & BTN_DOWN)
      {
        ROT_DOWN=1;
      }
      else
      {
        if (_u8Buttons & BTN_UP)
        {
          ROT_UP=1;
        }
      }
    }
    _u8ButtonsLast=_u8Buttons;
  }
  return _u8Buttons;
}

// Stop the rotator when it's reached its destination
static void RotatorUpdate(void)
{
  if (_bAzTrack)
  {
    S16 s16Az=(S16)ADCGet(0);

    //AK
    //s16Az-=_u16AzOff;
    //s16Az/=_fAzMul;
    s16Az = (S16)DEGfromADC(s16Az);
    
    if ((_s16AzTarget>=s16Az && ROT_LEFT) ||
      (_s16AzTarget<=s16Az && ROT_RIGHT))
    {
      ROT_LEFT=0;
      ROT_RIGHT=0;
      _bAzTrack=FALSE;
    }
  }

  if (_bElTrack)
  {
    S16 s16El=(S16)ADCGet(1);

    s16El-=_u16ElOff;
    s16El/=_fElMul;

    if ((_s16ElTarget>=s16El && ROT_DOWN) ||
      (_s16ElTarget<=s16El && ROT_UP))
    {
      ROT_DOWN=0;
      ROT_UP=0;
      _bElTrack=FALSE;
    }
  }
}

// Wait for buttons to be pressed not in mask. Allow Rotator to move according to those in mask.
// Return buttons pressed.
static U8 ButtonGetPress(U8 u8Mask)
{
  U8 u8Button;

  do
  {
    u8Button=ButtonCheck(u8Mask);
  } while ((u8Button & ~u8Mask)!=0); // wait for buttons to be released first

  do
  {
    u8Button=ButtonCheck(u8Mask);
  } while ((u8Button & ~u8Mask)==0);

  return u8Button;
}

static void ButtonConfig(void)
{
  if (_bLCDActive && ButtonGet())
  {
    U8 u8Button;
    BOOL bFinished;
    U16 u16AzMin;
    U16 u16AzMax;
    U16 u16ElMin;
    U16 u16ElMax;
    BOOL bAz450=FALSE;
    BOOL bEl90=FALSE;
    BOOL bSNS=FALSE;

    LCDClear();
    LCDMsg("Set Az=0 &");
    LCDSetCursor(40);
    LCDMsg("press D (U=esc)");

    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
    if (u8Button & BTN_UP)
    {
      return;
    }
    u16AzMin=ADCGet(0);
    
    //AK
    _u16AzOffset[0]=u16AzMin;

    LCDClear();
    LCDMsg("Set Az=90 &");
    LCDSetCursor(40);
    LCDMsg("press D (U=esc)");

    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
    if (u8Button & BTN_UP)
    {
      return;
    }
    _u16AzOffset[1]=ADCGet(0);
    _fAzMult[0]=(float)(_u16AzOffset[1]-_u16AzOffset[0])/90;
    
    LCDClear();
    LCDMsg("Set Az=180 &");
    LCDSetCursor(40);
    LCDMsg("press D (U=esc)");

    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
    if (u8Button & BTN_UP)
    {
      return;
    }
    _u16AzOffset[2]=ADCGet(0);
    _fAzMult[1]=(float)(_u16AzOffset[2]-_u16AzOffset[1])/90;

    LCDClear();
    LCDMsg("Set Az=270 &");
    LCDSetCursor(40);
    LCDMsg("press D (U=esc)");

    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
    if (u8Button & BTN_UP)
    {
      return;
    }
    _u16AzOffset[3]=ADCGet(0);
    _fAzMult[2]=(float)(_u16AzOffset[3]-_u16AzOffset[2])/90;
    
    LCDClear();
    LCDMsg("Set Az=360 &");
    LCDSetCursor(40);
    LCDMsg("press D (U=esc)");

    u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
    if (u8Button & BTN_UP)
    {
      return;
    }
    _u16AzOffset[4]=ADCGet(0);
    _fAzMult[3]=(float)(_u16AzOffset[4]-_u16AzOffset[3])/90;

    //AK
    //LCDClear();
    //LCDMsg("Set Az=max &");
    //LCDSetCursor(40);
    //LCDMsg("press D (U=esc)");
    //
    //u8Button=ButtonGetPress(BTN_LEFT | BTN_RIGHT);
    //if (u8Button & BTN_UP)
    //{
    //  return;
    //}
    //AK
    //u16AzMax=ADCGet(0);
    u16AzMax=_u16AzOffset[4]; //make it the same as the 360 degree value
    //AK
    //_fAzMult[4]=(float)(u16AzMax-_u16AzOffset[4])/90;
    _fAzMult[4]=(float)0;

    LCDClear();
    LCDMsg("Set El=min &");
    LCDSetCursor(40);
    LCDMsg("press L (R=esc)");

    u8Button=ButtonGetPress(BTN_UP | BTN_DOWN);
    if (u8Button & BTN_RIGHT)
    {
      return;
    }
    u16ElMin=ADCGet(1);

    LCDClear();
    LCDMsg("Set El=max &");
    LCDSetCursor(40);
    LCDMsg("press L (R=esc)");

    u8Button=ButtonGetPress(BTN_UP | BTN_DOWN);
    if (u8Button & BTN_RIGHT)
    {
      return;
    }
    u16ElMax=ADCGet(1);

    LCDClear();
    LCDMsg("Az range: 450 U");
    LCDSetCursor(40);
    LCDMsg("360 D (R=esc)");

    u8Button=ButtonGetPress(0);
    if (u8Button & BTN_RIGHT)
    {
      return;
    }
    if (u8Button & BTN_UP)
    {
		bAz450=TRUE;
    }

    LCDClear();
    LCDMsg("El range: 0-90 U");
    LCDSetCursor(40);
    LCDMsg("0-180 D (R=esc)");

    u8Button=ButtonGetPress(0);
    if (u8Button & BTN_RIGHT)
    {
      return;
    }
    if (u8Button & BTN_UP)
    {
		bEl90=TRUE;
    }

    LCDClear();
    LCDMsg("CCW stop North=U");
    LCDSetCursor(40);
    LCDMsg("South=D (R=esc)");

    u8Button=ButtonGetPress(0);
    if (u8Button & BTN_RIGHT)
    {
      return;
    }
    if (u8Button & BTN_DOWN)
    {
      bSNS=TRUE;
    }

    LCDClear();
    LCDMsg("EE Write: U");
    LCDSetCursor(40);
    LCDMsg("(R=esc)");

    u8Button=ButtonGetPress(0);
    if (u8Button & BTN_RIGHT)
    {
      return;
    }
    if (u8Button & BTN_UP)
    {
      FLAG_SNS=bSNS;
      FLAG_AZ450=bAz450;
      _u16AzOff=u16AzMin;
      _u16ElOff=u16ElMin;
      _fAzMul=(float)(u16AzMax-u16AzMin);
      if (bAz450)
      {
        _fAzMul/=450.0;
      }
      else
      {
        _fAzMul/=360.0;
      }
      _fElMul=(float)(u16ElMax-u16ElMin);
      if (bEl90)
      {
        _fElMul/=90;
      }
      else
      {
        _fElMul/=180;
      }
      LCDClear();
      if (EEWriteAll())
      {
        LCDMsg("EEPROM write OK");
      }
      else
      {
        LCDMsg("EEPROM failed!");
      }
      LCDSetCursor(40);
      LCDMsg("Push btn to exit");
      ButtonGetPress(0);
    }
  }
}

#ifdef __CC5X__
#pragma codepage 0
#endif

// Zeroise target and stop tracking
static void TargetInit(void)
{
  _s16AzTarget=0;
  _s16ElTarget=0;
  _bAzTrack=FALSE;
  _bElTrack=FALSE;
}

static U8 EasyCommCommand(U8 *pu8Pos)
{
  // Look for a two character Easycomm command...
  char c1='\0';
  char c2='\0';
  U8 u8Pos=*pu8Pos;
  U8 u8Command=EC_NONE;

  ParseWhite(&u8Pos); // ignore white space

  if (u8Pos<=_u8LinePos+1) // make sure there's at least two more characters
  {
    c1=toupper(_acLine[u8Pos++]);
    c2=toupper(_acLine[u8Pos++]);

    if (c1=='A' && c2=='Z')
    {
      u8Command=EC_AZ; 
    }
    else if (c1=='E' && c2=='L')
    {
      u8Command=EC_EL;
    }
    else if (c1=='U' && c2=='P')
    {
      u8Command=EC_UP;
    }
    else if (c1=='D' && c2=='N')
    {
      u8Command=EC_DN;
    }
    if (u8Command!=EC_NONE)
    {
      *pu8Pos=u8Pos;
    }
  }
  return u8Command;
}

static void RotatorSet(S16 s16Az,S16 s16El)
{
  // Set the rotator position. 
  // If El or Az are 32767, then no attempt is made to move that axis
  if (s16Az!=32767)
  {
    if (FLAG_SNS)
    {
      if (s16Az<180)
      {
        s16Az+=180;
      }
      else
      {
        s16Az-=180;
      }
    }
    _s16AzTarget=s16Az;

    s16Az=(S16)ADCGet(0);

    //AK
    //s16Az-=_u16AzOff;
    //s16Az/=_fAzMul;
    s16Az = (S16)DEGfromADC(s16Az);

    if (_s16AzTarget>s16Az)
    {
      ROT_RIGHT=1;
      ROT_LEFT=0;
      _bAzTrack=TRUE;
    }
    else
    {
      if (_s16AzTarget<s16Az)
      {
        ROT_LEFT=1;
        ROT_RIGHT=0;
        _bAzTrack=TRUE;
      }
      else
      {
        ROT_LEFT=0;
        ROT_RIGHT=0;
        _bAzTrack=FALSE;
      }
    }
  }
  if (s16El!=32767)
  {
    _bElTrack=TRUE;
    _s16ElTarget=s16El;

    s16El=(S16)ADCGet(1);

    s16El-=_u16ElOff;
    s16El/=_fElMul;

    if (_s16ElTarget>s16El)
    {
      ROT_UP=1;
      ROT_DOWN=0;
      _bElTrack=TRUE;
    }
    else
    {
      if (_s16ElTarget<s16El)
      {
        ROT_DOWN=1;
        ROT_UP=0;
        _bElTrack=TRUE;
      }
      else
      {
        ROT_DOWN=0;
        ROT_UP=0;
        _bElTrack=FALSE;
      }
    }
  }
}

// Parse the input line if one's available
static void RS232Check(void)
{
  while (RS232RxCharReady())
  {
    char c=RS232RxGetChar();

////
////
////
////RS232TxPutChar(c);
	

    if (c=='\n' || c=='\r')
    {
	    
////
////
////
////RS232TxPutChar('\n');
      U8 u8Pos=0;
      U8 u8EasyCommCommand=EasyCommCommand(&u8Pos);

      if (u8EasyCommCommand!=EC_NONE)
      {
        while (u8EasyCommCommand!=EC_NONE)
        {
          BOOL bError=FALSE;

          switch (u8EasyCommCommand)
          {
            case EC_AZ:
            case EC_EL:
              {
                float f;
                S16 s16;

                ParseWhite(&u8Pos);
                if (!ParseFloat(&u8Pos,&f))
                {
                  bError=TRUE;
                  break;
                }
                s16=(S16)f;
                if (u8EasyCommCommand==EC_AZ)
                {
                  RotatorSet(s16,32767);
                }
                else
                {
                  RotatorSet(32767,s16);
                }
              }
              break;
            case EC_UP:
            case EC_DN:
              // Parsed but ignored...
              {
                float f;
                char acMode[3];
                U8 u8=0;

                ParseWhite(&u8Pos);
                if (!ParseFloat(&u8Pos,&f))
                {
                  bError=TRUE;
                  break;
                }
                ParseWhite(&u8Pos);
                while (u8Pos<_u8LinePos && u8<3)
                {
                  char c=_acLine[u8Pos++];

                  acMode[u8++]=c;
                }
                while (u8<3)
                {
                  acMode[u8++]=' ';
                }
              }
              break;
            default:
              bError=TRUE;
              break;
          }
          if (bError)
          {
            RS232TxPutError();
            u8EasyCommCommand=EC_NONE;
          }
          else
          {
            u8EasyCommCommand=EasyCommCommand(&u8Pos);
          }
        }
      }
      else
      { // This is a GS232 command
        // Ignore leading spaces...
        while (u8Pos<_u8LinePos)
        {
          c=toupper(_acLine[u8Pos]);
          u8Pos++;
          if (c!=' ' && c!='\t')
          {
            break;
          }
        };
        if (u8Pos<=_u8LinePos)
        {
          // Get command char
          switch (c)
          {
            case 'F':
              if (u8Pos<_u8LinePos)
              {
                char cAzEl=toupper(_acLine[u8Pos++]);
  
                if (cAzEl=='W')
                {
                  if (EEWriteAll())
                  {
                    RS232TxMsg("EEPROM write OK\r\n");
                  }
                  else
                  {
                    RS232TxMsg("EEPROM failed\r\n");
                  }
                  break;
                }
  
                if (cAzEl=='S') // Set South stop rotator
                {
                  FLAG_SNS=TRUE;
                  RS232TxMsg("South stop\r\n");
                  break;
                }
  
                if (cAzEl=='N') // Set 0 - 360/450 rotator
                {
                  FLAG_SNS=FALSE;
                  RS232TxMsg("North stop\r\n");
                  break;
                }
  
                if (u8Pos<_u8LinePos)
                {
                  char cEndStart=toupper(_acLine[u8Pos++]);
  
                  if (cAzEl=='A' && cEndStart=='O') // Set Az Offset directly
                  {
                    U16 u16;
  
                    ParseWhite(&u8Pos);
                    if (ParseU16(&u8Pos,&u16))
                    {
                      _u16AzOff=u16;
            		  TargetInit();
                      RS232TxMsg("Az off=");
                      RS232TxPutU16(_u16AzOff);
                      RS232TxMsg("\r\n");
                      break;
                    }
                  }
                  if (cAzEl=='A' && cEndStart=='M') // Set Az Mul directly
                  {
                    float f;
  
                    ParseWhite(&u8Pos);
                    if (ParseFloat(&u8Pos,&f))
                    {
                      _fAzMul=f;
            					TargetInit();
                      RS232TxMsg("Az mul=");
                      RS232TxPutFloat(_fAzMul);
                      RS232TxMsg("\r\n");
                      break;
                    }
                  }
                  if (cAzEl=='A' && cEndStart=='S')
                  {
                    _u16AzOff=ADCGet(0);
                    TargetInit();
                    RS232TxMsg("Az Offset=");
                    RS232TxPutU16(_u16AzOff);
                    RS232TxMsg("\r\n");
                    break;
                  }
                  if (cAzEl=='A' && cEndStart=='E')
                  {
                    U16 u16=ADCGet(0);
                    
                    _fAzMul=((float)(u16-_u16AzOff))/360.0;
                    FLAG_AZ450=FALSE;
                    TargetInit();
                    RS232TxMsg("Az mul=");
                    RS232TxPutFloat(_fAzMul);
                    RS232TxMsg("\r\n");
                    break;
                  }
                  if (cAzEl=='A' && cEndStart=='F')
                  {
                    U16 u16=ADCGet(0);
                    
                    _fAzMul=((float)(u16-_u16AzOff))/450.0;
                    FLAG_AZ450=TRUE;
                    TargetInit();
                    RS232TxMsg("Az mul=");
                    RS232TxPutFloat(_fAzMul);
                    RS232TxMsg("\r\n");
                    break;
                  }
                  if (cAzEl=='E' && cEndStart=='O') // Set El Offset directly
                  {
                    U16 u16;
  
                    ParseWhite(&u8Pos);
                    if (ParseU16(&u8Pos,&u16))
                    {
                      _u16ElOff=u16;
                      TargetInit();
                      RS232TxMsg("El off=");
                      RS232TxPutU16(_u16ElOff);
                      RS232TxMsg("\r\n");
                      break;
                    }
                  }
                  if (cAzEl=='E' && cEndStart=='M') // Set El Mul directly
                  {
                    float f;
  
                    ParseWhite(&u8Pos);
                    if (ParseFloat(&u8Pos,&f))
                    {
                      _fElMul=f;
                      TargetInit();
                      RS232TxMsg("El mul=");
                      RS232TxPutFloat(_fElMul);
                      RS232TxMsg("\r\n");
                      break;
                    }
                  }
                  if (cAzEl=='E' && cEndStart=='S')
                  {
                    _u16ElOff=ADCGet(1);
                    TargetInit();
                    RS232TxMsg("El Offset=");
                    RS232TxPutU16(_u16ElOff);
                    RS232TxMsg("\r\n");
                    break;
                  }
                  if (cAzEl=='E' && cEndStart=='E')
                  {
                    U16 u16=ADCGet(1);
                    
                    _fElMul=((float)(u16-_u16ElOff))/180.0;
                    TargetInit();
                    RS232TxMsg("El mul=");
                    RS232TxPutFloat(_fElMul);
                    RS232TxMsg("\r\n");
                    break;
                  }
                  if (cAzEl=='E' && cEndStart=='N')
                  {
                    U16 u16=ADCGet(1);
  
                    _fElMul=((float)(u16-_u16ElOff))/90.0;
                    TargetInit();
                    RS232TxMsg("El mul=");
                    RS232TxPutFloat(_fElMul);
                    RS232TxMsg("\r\n");
                  }
                }
              }
              RS232TxPutError();
              break;
            case 'S':
              _bAzTrack=FALSE;
              _bElTrack=FALSE;
              ROT_LEFT=0;
              ROT_RIGHT=0;
              ROT_UP=0;
              ROT_DOWN=0;
              break;
            case 'C':
              if (u8Pos<_u8LinePos)
              {
                c=_acLine[u8Pos];
              }
              // Send out AZ
              {
                S16 s16=(S16)ADCGet(0);
  
                //AK
                //s16-=(S16)_u16AzOff;
                //s16/=_fAzMul;
                s16 = (S16)DEGfromADC(s16);
                
                if (FLAG_SNS)
                {
                  if (s16<180)
                  {
                    RS232TxPutS16(s16+180);
                  }
                  else
                  {
                    RS232TxPutS16(s16-180);
                  }
                }
                else
                {
                  RS232TxPutS16(s16);
                }
  
                if (c=='2')
                {
                  //Send out EL
                  S16 s16=(S16)ADCGet(1);
  
                  s16-=(S16)_u16ElOff;
                  s16/=_fElMul;
                  RS232TxPutS16(s16);
                }
                RS232TxPutChar('\r');
                RS232TxPutChar('\n');
              }
              break;
            case 'W':
            case 'M': // 'M' command added - supposed to set Az only
              {
                S16 s16Az=32767; // wild numbers signify ignore setting
                S16 s16El=32767;
  
                ParseWhite(&u8Pos);
                if (!ParseU16(&u8Pos,(U16 *)&s16Az))
                {
                  RS232TxPutError();
                }
                else
                {
           				if (c=='W') // only for W command do we need El
           				{
           					// If no El, we ignore.
   	                ParseWhite(&u8Pos);
       	            ParseU16(&u8Pos,(U16 *)&s16El);
           				}
                  RotatorSet(s16Az,s16El);
                }
              }
              break;
            default:
              RS232TxPutError();
              break;
          }
        }
      }
      _u8LinePos=0;
    }
    else
    {
      if (_u8LinePos<RS232_LINESIZE)
      {
        _acLine[_u8LinePos]=c;
        _u8LinePos++;
      }
    }
  }
}


void main(void)
{
  //AK
//  _u16AzOffset[0] = 100;
//  _u16AzOffset[1] = 1100;
//  _u16AzOffset[2] = 1500;
//  _u16AzOffset[3] = 1800;
//  _u16AzOffset[4] = 2000;
//  _fAzMult[0] = 11.11;
//  _fAzMult[1] = 4.44;
//  _fAzMult[2] = 3.33;
//  _fAzMult[3] = 2.22;
//  _fAzMult[4] = 1.11;
//  U16 u16ADC = ADCfromDEG(46);
//  U16 u16Degrees = DEGfromADC(650);
    
  // TRISB Init to disable LCD E line (PORTB.2) and
  // enable portb.3 as output and set to 1 (disable buttons)
  // Bit 7 is CTS back to PC - needs to be set on or else TXD
  // connects thru to CTS via 4.7k resistor and makes some 
  // host progs loopy (eg SatPC32).
  TRISB=0b.0100.0000;
  PORTB=0b.1000.1000; // Set CTS on!
  _u8Buttons=0;

  // ADC Init
  TRISA = 0b.1111.1111;  /* 1=input */

#ifdef __CC5X__
  PCFG0=0; // VDD/VSS ref, zero channels Must do this or PORTA disabled for output
  PCFG1=1;
  PCFG2=1;
  PCFG3=0;
#endif

#ifdef __CC8E__
    PCFG0=1; // VDD/VSS ref, zero channels
    PCFG1=1;
    PCFG2=1;
    PCFG3=1;
		VCFG0=0;
		VCFG1=0;
#endif

  // TMR0 Init
  _u16Timer=0;
  PSA=0; /* Prescaler assigned to TMR0 */
  PS0=1; /* 256 prescale on FOSC/4 */
  PS1=1;
  PS2=1; 
  T0CS=0; /* select crystal osc (FOSC/4) */
  T0IE=1; /* Enable timer interrupt */

  // USART init
  RS232TxInit();
  RS232RxInit();
  SPBRG=RS232_BRG;
  BRGH=1; /* High speed Baud rate */
  SYNC=0; /* Async USART mode */
  TX9=0; /* 8 bit */
  RX9=0;
  SPEN=1; /* Enable serial port */
  TXEN=1; /* Enable TX */
  CREN=1; /* Enable Rx */
  TRISC.7=1; /* RX bit input */
  TRISC.6=0; /* TX bit output */
  TXIE=0; /* Disable Tx interrupt (for now) */
  RCIE=1; /* Enable Rx interrupt */
  PEIE=1; /* Peripheral enable interrupts */

  // Enable Rotator bits as output
  TRISC.0=0;
  TRISC.1=0;
  TRISC.2=0;
  TRISC.3=0;

  // No movement required
  ROT_LEFT=0;
  ROT_RIGHT=0;
  ROT_DOWN=0;
  ROT_UP=0;

  // LCD Init
  LCDInit();

  // Welcome message on LCD
  LCDWelcome();

  // Initialise variables from EEPROM
  EEInit();

  // 'zeroise' target Az/El and stop tracking
  TargetInit();

  // Initialise line input parser
  _u8LinePos=0;

  //EEInit();

  GIE=1; // Enable interrupts

  // If a button's pressed during boot, then we need to go into config mode
  ButtonConfig();

  // Here we go...
  while (1)
  {
    ButtonCheck(0x0F); // Deal with any button presses
    RS232Check(); // Deal with any incoming RS232 stuff
    LCDUpdate(); // Update the LCD display
    RotatorUpdate(); // Update rotator settings - stop tracking if reached destination
  }
}
