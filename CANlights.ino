/* file: CANlights.ino
 *------------------------------------------------------------------------------
 *
 * CBUS module controls 10 PWM LED strings for overhead/street/building. 
*/
const char sTITLE[] { "CANlights © Dave Harris 2021 (MERG M2740)" };

const uint8_t VER_MAJOR { 0 };          /* 0-255.      CBUS version style */
const char    VER_MINOR { 'c' };        /* a-z character.                 */
const uint8_t VER_BETA  { 4 };          /* 0-255. 0 is Released, > 0 beta */

/*------------------------ © Copyright and License -----------------------------
 *  
 * This code is © Dave Harris 2021 (contact at https://github.com/daveharrisuk)
 *
 * This work is licensed under the Creative Commons 
 *   Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 * To view a copy of this license,
 *  visit http://creativecommons.org/licenses/by-nc-sa/4.0/   or 
 *  send a letter to Creative Commons, PO Box1866, Mountain View, CA 94042 USA.
 *
 *-------------------------- Program Description ------------------------------
 *
 * CBUS layout lighting control module for 10 LED strings.
 *
 * - 10x 12 Volt PWM channels driving LED strings. 
 *
 * - CBUS event used as day/night trigger.
 * - CBUS events to put channels into a test mode
 * - CBUS event used to shutdown all channels
 * 
 * - each channel has 6 configuration in CBUS NVs
 *
 * - Input (Day/Night) triggers channel modes...
 *    DAYNIGHT : Duty cycle (DC) transition on input change
 *    DAWN     : DC transitions on input going on then off
 *    DUSK     : DC transitions on input going on then off
 *    DUSKDAWN : DC transitions on input going on then off at either change
 *    NIGHT010 : During night time, LEDs transition at durations
 *    DAY010   : During day time, LEDs transition at durations
 *    ALWAYS010: tba. LEDs transition at durations, not tied to day/night input
 *    RANDOM010: tba. LEDs transition at random duration, no day/night input
 *
 * - Transition configuration...
 *    Delay seconds before transitions.
 *    Duty cycle Transition duration in seconds.
 *
 * - Duty Cycle for on and off (or Day/Night) in configuration.
 *
 * - Over current protection. Audio alarm sounder plus CBUS event.
 * - diagnostics via Arduino Serial Monitor. 
 * 
 * - LEDs
 *    Red    = Alarm (see serial monitor for detail)
 *    Yellow = FLiM mode
 *    Green  = pre FliM mode and CAN activity flash
 *    Orange = on is Night time, off is Day time
 *    Blue   = 12v indicator
 * 
 *-------------------------------- target -------------------------------------
 *  
 *  Framework : Arduino 1.8.13 (Developed on Ubuntu 20.4)
 *  PCB       : CANlights revB - has MCU board as daughter board (KiCad)
 *  MCU board : 'MEGA 2560 PRO' (Arduino Mega2560 should work, on breadboard)
 *  Processor : ATmega2560 
 *    Clock   : 16 MHz
 *    Flash   : 26.2 KB of 256 KB (10%).  (NB: 8 KB is used by bootloader)
 *    SRAM    : 2.4 KB of 8 KB used (28%).
 *    EEPROM  : 0.2 KB of 4 KB used (5%).
 *    Timer   : 6 of 6 used.  (100%).
 *    PWM chan: 10 of 16 used (100% as 2 timers[6 comparators] otherwise used)
 *    ADC chan: 2 of 16 ADC used (13%).
 *    IO pins : 30 of 70 used (43%).
 *  USB Serial: CHG340G @ 12 MHz.
 *  CAN ctrl  : MCP2515 on hardware SPI bus at 16 MHz (see PIN.h).
 *
 *------------------------------- History -------------------------------------
 *
 *  4-Jan-2021 Dave Harris (DH) Lights_Control_1 Project started
 *  9-Jan-2021 DH, v0.01  breadboard first test
 * 23-Jan-2021 DH, v0.02  add channel Modes
 *  2-Feb-2021 DH, v0.03  add TimerOne lib and 1 ms ISR
 *  4-Feb-2021 DH, v1.00  released on Lights_Control_1 PCB
 *  5-Feb-2021 DH, v1.01  correct over Amp code. Sense cap 0.1uF to 1.0uF
 * 11-Feb-2021 DH, v1.02  efficiency and tidy up changes
 * 23-Feb-2021 DH, v1.03  fix, DC wrong after over Amp condition.
 * 28-Feb-2021 DH, Lights_Control_1 frozen and code copied to CANlights.
 *          CANlights.ino  Versioning now in CBUS style.
 *  1-Mar-2021 DH, v0a beta1 add MERG CBUS library and MCP2515 to SPI bus
 * 11-Apr-2021 DH, v0a beta2 CANlights basic working (a few bugs to go)
 *  9-May-2021 DH, v0b beta3 revB PCB changes
 * 18-May-2021 DH, v0c beta4 confusion on event DayNight, renamed to NightSw 
 * 
 * 
*--------------------------- file includes ---------------------------------*/

#include <ACAN2515.h>         /* 2.0.8   MCP2515/25625 CAN ctrl             */

/* Duncan Greenwood © CBUS libraries    https://github.com/MERG-DEV         */
#include <CBUS2515.h>         /* 1.1.12  CBUS for MCP2515 CAN chip          */
#include <CBUSconfig.h>       /* 1.1.11                                     */
#include <CBUS.h>             /* 1.1.15  implements CBUS module             */
#include <cbusdefs.h>         /* 8t      in CBUS library                    */
#include <CBUSswitch.h>       /* 1.1.7                                      */
#include <CBUSLED.h>          /* 1.1.6                                      */

/* standard Arduino libraries                                               */
#include <SPI.h>              /* 1.0                                        */
#include <Streaming.h>        /* 6.0.8  Serial Monitor communications       */
#include <TimerOne.h>         /* 1.1    1 ms foreground process             */

/* The following must be in the CANlights folder...                         */
#include "CANlights.h"        /* class and function declarations            */
#include "DATADEF.h"          /* data structure definitions                 */
#include "PIN.h"              /* pin definitions ** plus PIN.cpp **         */
#include "GAMMA8.h"           /* Gamma correction table                     */
#include "DEFAULTNV.h"        /* factory reset NV array                     */


/*---------------------------- global variables -----------------------------*/


uint8_t  params[21];           /* CBUS params array                          */

volatile var_t var[QTY_CHAN];  /* channel variable array. Is modified by ISR */
                               /* A mix of NVs, derived and tracking values  */
                               
volatile      INPUT_t  input;        /* input state 0 = DAY or 1 = NIGHT     */
volatile bool inputChange { true };  /* reset this flag when actioned        */

EVAL_t evCommand { EVAL_NIGHTSW }; /* the current event command              */
uint8_t testDC   { 1 };          /* current duty cycle for test mode         */
bool    testCh   { false };      /* false= no test chan, true= testing chan  */
bool stopISR     { false };      /* flag disables 1 ms ISR process, if true  */
bool noCAN       { false };      /* false = CBUS on, true = noCAN = no CBUS  */
bool debug       { false };      /* false = quiet, true = verbose            */
bool muteAlarm   { false };      /* false = quiet, true = sounding           */


/*------------------------------- objects -----------------------------------*/


CBUS2515   CBUS;              /* CBUS object                                 */
CBUSConfig config;            /* CBUSconf object                             */
CBUSLED    CBUSgreen;         /* LED green object                            */
CBUSLED    CBUSyellow;        /* LED yellow object                           */
CBUSSwitch CBUS_PB;           /* CBUS mode push switch object                */
CBUSSwitch INPUT_SW;          /* Input switch object                         */

SerMon SerialMon;             /* Serial Monitor print object                 */
GetNV  NV;                    /* lookup NV for chan and datatype             */
Power  PWR;                   /* Power object                                */


/*---------------------------------------------------- loop() ----------------- 
 * 
 * this is the Background process.
*/

void loop() 
{
  if( noCAN == false )
  {
    CBUS.process();
  };
  
  PWR.testAmpAndVolt();

//  if( INPUT_SW.isPressed() != input )
//  {
//    input = (INPUT_t) ! input;
//    sendEvent( (ONOFF_t) input, EN_NIGHTSW );
//  }

  if( inputChange == true )  /* flag set by Event handler  */
  {
    inputChange = false;     /* clear the flag             */
    startNewPhase();
  }
  
  SerialMon.processKeyBoard();
}


/*--------------------------------------------- processChannels() ----------
 * 
 * timer1 Interrupt Service Routine.  1 ms foreground process.
*/
                                /* !!! This is ISR which runs every 1 ms !!! */
void processChannels()          /*        keep it light as possible          */
{
  PINTP_D31_HIGH;               /* is ISR run time = min 21 us to peak 87 us */

  if( stopISR == false )        /* other code sets true, if it changes DCs   */
  {                 
    bool transits { false };    /* lights LED_BUILTIN if Transits are active */

    for( uint8_t ch = 0; ch < QTY_CHAN; ch++ )      /* loop all channels     */
    {
      if( ch + 1 == evCommand )     /* TEST?, ch start at 0, cmd ch starts 1 */
      {
        analogWrite( PWMPIN[ch], testDC );    /* test DC is either 1 or 254  */
      }
      else
      {
        if( evCommand != EVAL_SHUTDOWN )  /* SHUTDOWN is done in eventHandler*/
        {
          processChan( ch, transits );
        }
      }
    }
    digitalWrite( LED_BUILTIN, transits );         /* show TRANSIT activity */
  }
  PINTP_D31_LOW;   /* time from pin high to low is ISR run time */
}


/*--------------------------------------------- processChan() ---------------
 * 
 * Process 1 channel. Called from the foreground processChannels() 
*/

void processChan( uint8_t ch, bool & transits )
{
  if( var[ch].state != STATE_STEADY )    /* is state not STEADY?        */
  {
    var[ch].msCount++;
    
    if( var[ch].state == STATE_DELAY )           /* is state Delay?     */
    {
      if( var[ch].msCount >= 1000 )              /* count delay seconds */
      {
        var[ch].msCount = 0;
        var[ch].secCount++;
        
        if( var[ch].secCount > var[ch].secDelay[var[ch].phase] )
        {
          var[ch].state = STATE_TRANSIT;         /* end state Delay     */
          var[ch].secCount = 0;
        }
      }
    }
    else /* so, state must be TRANSIT */
    {
      transits = true;                           /* drives LED_builtin  */
      
      if( var[ch].msCount > var[ch].msPerStep )
      {
        var[ch].msCount = 0;
        if( var[ch].dcCur == var[ch].dc[var[ch].phase] )
        { 
          var[ch].state = STATE_STEADY;          /* dc transit complete */
          
          switch( var[ch].mode )                 /* trigger a new phase */
          {
            case MODE_NIGHT010 :
              if( input == INPUT_NIGHT )
              {
                var[ch].phase = ! var[ch].phase;
                var[ch].state = STATE_DELAY;
              }
              break;
            case MODE_DAY010 :
              if( input == INPUT_DAY )
              {
                var[ch].phase = ! var[ch].phase;
                var[ch].state = STATE_DELAY;
              }
              break;
            case MODE_DUSK :
              if( var[ch].phase == 1 )
              {
                var[ch].phase = 0;
                var[ch].state = STATE_DELAY;                  
              }
              break;
            case MODE_DAWN :
              if( var[ch].phase == 1  )
              {
                var[ch].phase = 0;
                var[ch].state = STATE_DELAY;                  
              }
              break;
            case MODE_DUSKDAWN :
              if( var[ch].phase == 1 )
              {
                var[ch].phase = 0;
                var[ch].state = STATE_DELAY;
              }
              break;  
            case MODE_DAYNIGHT : 
              break;
          }
        }
        else  /* in Transit & current DC is GT or LT target DC */
        {
          if( var[ch].dcCur > var[ch].dc[var[ch].phase] )    /* which? */
              { var[ch].dcCur--; }                       /* GT so decrement */
          else
              { var[ch].dcCur++; }                       /* LT so increment */

          analogWrite( PWMPIN[ch], GAMMA8[var[ch].dcCur] );  /* set new dc */
        }
      }
    }
  }
}


/*-------------------------------------------- startNewPhase() ---------------
 * 
 * The day/night input has changed, so setup new phase for each channel 
*/

void startNewPhase()
{
  stopISR = true; /* all variables here are also manipulated by ISR */
  digitalWrite( PINLEDORA, input );

  for( uint8_t ch = 0; ch < QTY_CHAN; ch++ )   /* loop all channels */
  {
    var[ch].secCount = 0;          /* reset counters                         */
    var[ch].msCount = 0;
    var[ch].state = STATE_TRANSIT; /* default values, some updated by switch */
    var[ch].phase = 0;             /* this allows dc to transit back to dc0  */

    switch( var[ch].mode )
    {
      case MODE_DAYNIGHT :        /* either input edge starts MODE_DAYNIGHT   */
        var[ch].state = STATE_DELAY;
        var[ch].phase = input;    /* phase 0 = 0 (day), phase 1 = 1 (night)   */
        break;
      case MODE_DUSK :            /* day to night starts dusk                 */
        if( input == INPUT_NIGHT )
        {
          var[ch].state = STATE_DELAY;
          var[ch].phase = 1;
        }
        break;
      case MODE_DAWN :           /* night to day starts dawn                 */
        if( input == INPUT_DAY )
        {
          var[ch].state = STATE_DELAY;
          var[ch].phase = 1;
        }
        break;
      case MODE_DUSKDAWN :       /* either input edge starts DUSKDAWN         */
        var[ch].state = STATE_DELAY;
        var[ch].phase = 1;
        break;
      case MODE_NIGHT010 :       /* day to night starts NIGHT010              */
        if( input == INPUT_NIGHT )
        {
          var[ch].state = STATE_DELAY;
          var[ch].phase = 1;
        }
        break;
      case MODE_DAY010 :         /* night to day starts DAY010                */
        if( input == INPUT_DAY )
        {
          var[ch].state = STATE_DELAY;
          var[ch].phase = 1;
        }
        break;
    }
  }                /* end ch loop */
  stopISR = false;
}


/*---------------------------------------- setupChannels() -------------------
 * 
 * set channel variables from channel NVs and set the PWM
*/

void setupChannels()
{
  for( uint8_t ch = 0; ch < QTY_CHAN; ch++ )     /* loop through channels   */
  {
    var[ch].dc[0]       = NV.dc( ch, 0 );
    var[ch].dc[1]       = NV.dc( ch, 1 );
    var[ch].secDelay[0] = NV.dly( ch, 0 );
    var[ch].secDelay[1] = NV.dly( ch, 1 );
    var[ch].secTrans    = NV.tran( ch );
    var[ch].mode        = (MODE_t) NV.mode( ch );
    if( var[ch].mode >= QTY_MODE )
    {
      Serial << "!bad mode " << var[ch].mode << " Reset ch" << ch+1 << endl;
      var[ch].mode = MODE_DAYNIGHT;
    }
                     /* calculate ms per step rate of change value */
    if( var[ch].secTrans == 0 )   /* zero seconds transit is special          */
    { 
      var[ch].msPerStep = 1;      /* dc transits rapidly to target duty cycle */
    }
    else                          /* non-zero transit secs, calc ms per step  */
    {
      uint16_t diff = abs( var[ch].dc[0] - var[ch].dc[1] );

      if( diff < 4 )              /* diff under 4, makes msPerStep >65k error */
      {                           /* Its a pointless difference, so ...       */
        var[ch].msPerStep = 9;    /* Set to arbitary rate and dc levels.      */
        var[ch].dc[0] = 90;
        var[ch].dc[1] = 170;
        Serial << endl << "!DC diff < 4 Reset ch" << ch +1 << endl;
      }
      else /* calculate number of milliseconds per step */
      {
        var[ch].msPerStep = ( var[ch].secTrans * 1000UL ) / diff;
      }
    }
    var[ch].dcCur = MAXDC / 2;        /* */
    var[ch].state = STATE_STEADY;     /* */
    var[ch].phase = 0;                /* */

    analogWrite( PWMPIN[ch], var[ch].dcCur );
  }
  startNewPhase();
}


/*--------------------------------------------------- setup() -----------------
 *
 * runs at power on and reset
*/

void setup() 
{
  setupPins();
  noCAN = ! digitalRead( PINCAN );

  Serial.begin( 115200 );
  SerialMon.about( '_' );
  
  PWR.alarm( 250 );                             /* test sounder for 0.25 s    */

  if( ( config.readEEPROM( 0 ) > 2 ) ||         /* invalid SLiM/FLiM          */
      ( config.readEEPROM( 1 ) > 127 ) )        /* invalid CANID              */
  {    /* EEPROM map 0=SLiM/FLiM, 1=CANID, 2/3=NNhi&lo, 5=reset, 6-9=reserved */
    Serial << "!Invalid EEPROM contents." << endl;
  }

  setupCBUS();  
  
  setupChannels();
  
  SerialMon.variables();
  
  Timer1.initialize( 1000 );                    /* T1 runs at 1000 us (1 ms)  */
  Timer1.attachInterrupt( processChannels );    /* T1 ISR is Foreground task  */

  Serial << " debug=" << debug << " mute=" << muteAlarm;
  PWR.isOverAmp();
  PWR.printAmps();
}


/*-------------------------------------------- setupCBUS() -----------------
 * 
 *  called by setup()
*/

void setupCBUS()
{
  config.setEEPROMtype( EEPROM_INTERNAL );
  config.EE_NVS_START = 10;                /*  as good a place as anywhere   */
  config.EE_NUM_NVS = QTY_NV;              /*     setDefaultNVs uses this    */
  config.EE_EVENTS_START = ( config.EE_NVS_START + QTY_NV );
  config.EE_MAX_EVENTS = QTY_EVENT;
  config.EE_NUM_EVS = 1;
  config.EE_BYTES_PER_EVENT = ( config.EE_NUM_EVS + 4 );
  config.begin();
 
  params[0] = 20;                     /* Number of parameters               */
  params[1] = MANU_MERG;              /* Manufacturer ID                    */
  params[2] = VER_MINOR;              /* minor version                      */
  params[3] = CBUSMODULEID;           /* module id, 99 = undefined          */
  params[4] = config.EE_MAX_EVENTS;   /* number of events                   */
  params[5] = config.EE_NUM_EVS;      /* number event variables per event   */
  params[6] = config.EE_NUM_NVS;      /* number of Node Variables           */
  params[7] = VER_MAJOR;              /* major version                      */
  params[8] = ( PF_FLiM | PF_COMBI ); /* Node Flags                         */
  params[9] = 0x32;                   /* processor ID                       */
  params[10] = PB_CAN;                /* interface protocol                 */
  params[11] = 0x00;                  /* download load address              */
  params[12] = 0x00;                  /*      -"-                           */
  params[13] = 0x00;                  /*      -"-                           */
  params[14] = 0x00;                  /*      -"-                           */
  params[15] = '2';                   /* Processor code                     */
  params[16] = '5';                   /*      -"-                           */
  params[17] = '6';                   /*      -"-                           */
  params[18] = '0';                   /*      -"-                           */
  params[19] = CPUM_ATMEL;            /* Manufacturer code                  */
  params[20] = VER_BETA;              /* 0 is released, >0 is beta version  */
    
  CBUS.setName( sCBUSNAME );          /* assign module name                 */
  CBUS.setParams( params );           /* assign params to CBUS              */
  
  CBUSgreen.setPin( PINLEDGRN );
  CBUSyellow.setPin( PINLEDYEL );
  CBUS_PB.setPin( PINCBUS, LOW );     /* initialise CBUS mode switch        */
  CBUS_PB.run();
  
  if( CBUS_PB.isPressed() && ! config.FLiM ) 
  {
    Serial << "! switch was pressed at startup in SLiM mode" << endl;
    config.resetModule( CBUSgreen, CBUSyellow, CBUS_PB );
  }

  CBUS.setEventHandler( eventHandler );   /* call on every learned event      */
  CBUS.setFrameHandler( frameHandler );   /* call on every CAN frame.         */

  CBUS.setLEDs( CBUSgreen, CBUSyellow );  /* green=SLiM, yellow=FLiM          */
  CBUS.setSwitch( CBUS_PB );
  CBUS.indicateMode( config.FLiM );       /* set CBUS LEDs for current mode   */
  
  CBUS.setNumBuffers( 4 );                /* more buffers, more memory        */
  
  CBUS.setOscFreq( 16 * 1e6f );           /* SPI frequency MHz                */
  CBUS.setPins( PINSPI_SS, PINSPI_INT );  /* SPI pins, outside of H/W SPI     */

  if( noCAN == true )
  {
    Serial << "NoCAN link is in." << endl;
  }
  else
  {
    while( CBUS.begin() == false )
    {
      Serial << " ErrFlg 0x" << _HEX( CBUS.canp->errorFlagRegister() ) << endl;
      PWR.alarm( 250 );
    }
    SerialMon.cbusState();
    SerialMon.storedEvents();
    sendEvent( ONOFF_ON, EN_POWERON );
  }
}


/*------------------------------------------ Power::alarm() -----------------
 * 
 * Sound Audio Warning Device piezo buzzer for n milli seconds.
 * Timers are all used up, but this can be done OK in a loop.
*/

void Power::alarm( uint16_t duration_ms )
{
  digitalWrite( PINLEDRED, HIGH );
  SerialMon.processKeyBoard();
  uint16_t loopCount = duration_ms * 4;
     
  for( uint16_t k = 0; k < loopCount; k++ )
  {
    if( muteAlarm == false )
    {                                   /* 4 kHz, cycle high or low, 125 us  */
      digitalWrite( PINAWDSIG, k & 1 ); /* LSB of k, even-odd-even, 0-1-0... */
      delayMicroseconds( 121 );         /* 121 allows for overheads in loop  */
    }
  }
  digitalWrite( PINLEDRED, LOW );
}


/*---------------------------------------- Power::isUnderVolt() ----------
 * 
 * return true if 12 V line failed or PolyFuse tripped.
*/

bool Power::isUnderVolt()
{
  return ( analogRead( PINBLUE ) < 1022 ); 
}


/*----------------------------------------- Power::isOverAmp() --------------
 * 
 * Read Volts on sense resistor to measure Amps, return true if over Amp
*/

bool Power::isOverAmp()
{
  amps = analogRead( PINSENSE );
  return ( amps > MAXAMPADCREAD );
}


/*-------------------------------- Power::printAmps() ----------------------
 * 
 * prints, approx, measured mA to Serial
 * displays as multiples of 22 mA   ## anything below that shows as zero ##
*/

void Power::printAmps()
{
  Serial << " Amp=" << ( amps * AMPCALIBRATE ) << "mA" << endl;
}


/*-------------------------------------- Power::testAmpAndVolt() -------------
 * 
 * If under Volt (poly fuse?) or over current then alarm & reduce duty cycles.
*/

void Power::testAmpAndVolt()
{  
  if( isOverAmp() == true || isUnderVolt() == true )
  {
    stopISR = true;                 /* stop ISR as this code overides DC  */
    digitalWrite( PINLEDRED, HIGH );
    sendEvent( ONOFF_ON, EN_ALARM );
    
    turnOffPWMs();
    Serial << "! OverAmp/UnderVolt";
    printAmps();
    do
    {
      alarm( 250 ); /* sound AWD for n milli secs */
    }
    while( isOverAmp() == true || isUnderVolt() == true );
    
    restorePWMs();
    stopISR = false;
    sendEvent( ONOFF_OFF, EN_ALARM );
  }
}


/*----------------------------- turnOffPWMs() --------------------------
 * 
 * set all PWM channel duty cycles to zero value
*/

void turnOffPWMs()
{
  Serial << "! PWMs off" << endl;
  for( uint8_t ch = 0; ch < QTY_CHAN; ch++ )
  {
    analogWrite( PWMPIN[ch], 0 );
  }
}


/*------------------------------ restorePWMs() -------------------------
 * 
 * restore all PWM to running duty cycles
*/

void restorePWMs()
{
  Serial << ". PWMs restored" << endl;
  for( uint8_t ch = 0; ch < QTY_CHAN; ch++ )
  {
    analogWrite( PWMPIN[ch], GAMMA8[var[ch].dcCur] );
  }
}


/*-------------------------------- GetNV::tran() ---------------------------
 *
 * return transition NV for chan     NV.tran( 5 ) is chan 5 transition
*/

uint8_t GetNV::tran( uint8_t chan )
{
  return config.readNV( NVmap[NV_TRAN][0] + chan );
}


/*-------------------------------- GetNV::dly() ----------------------------
 * 
 * return delay NV for chan & index     NV.dly( 8, 1 ) is chan 8 Dly1
*/

uint8_t GetNV::dly( uint8_t chan, bool indx )
{
  return config.readNV( NVmap[NV_DLY][indx] + chan );
}


/*------------------------------------ GetNV::dc() ------------------------
 *
 * return duty cycle NV for chan & index      NV.dc( 9, 1 ) is chan 9 DC1
*/

uint8_t GetNV::dc( uint8_t chan, bool indx )
{
  return config.readNV( NVmap[NV_DC][indx] + chan );
}


/*---------------------------------- GetNV::mode() --------------------------
 *
 * return mode NV for chan                 NV.mode( 0 ) is chan 0 Mode
*/

uint8_t GetNV::mode( uint8_t chan )
{
  return config.readNV( NVmap[NV_MODE][0] + chan );
}


/*---------------------------------------- eventHandler() -------------------
 * 
 * event process function called from CBUS library when learned event is rx
*/

void eventHandler( uint8_t idx, CANFrame *msg ) 
{
  PINTP_D30_HIGH;
  
  EVAL_t  evVal = (EVAL_t) config.getEventEVval( idx, 1 );
  uint8_t opc = msg->data[0];
  bool evOn = ( opc == OPC_ASON || opc == OPC_ACON );
  
  Serial << endl << "> Ev" << idx << (evOn ? " xON":" xOF") <<" eVal="<< evVal;
  switch( evVal )
  {
    case EVAL_NIGHTSW:
      Serial << ' ' << sEVAL[EVAL_NIGHTSW];
      if( input != (INPUT_t) evOn )
      {
        input = (INPUT_t) evOn;
        inputChange = true;             /* flag to cause change phases */
        Serial << sINPUT[evOn];
      }
      else Serial << "ignore";
      break;
    case EVAL_SHUTDOWN:
      Serial << " Shutdown";
      if( evCommand == EVAL_SHUTDOWN && evOn == false )
      {
         evCommand = EVAL_NIGHTSW; /* revert to normal operations  */
         restorePWMs();
      }
      else
      {
        if( evCommand != EVAL_SHUTDOWN && evOn == true )
        {
          evCommand = EVAL_SHUTDOWN; /* turn on shutdown */
          turnOffPWMs();
        }
      }
      break;
    case EVAL_TESTEND:
      Serial << ' ' << sEVAL[EVAL_TESTEND];
      if( testCh == true )
      {
        Serial << sEVAL[evCommand];
        analogWrite( PWMPIN[(evCommand -1)], var[(evCommand -1)].dcCur );
        evCommand = EVAL_NIGHTSW;
        testCh = false;
      }
      else Serial << "ignore";
      break; 
    default:
      if( evVal > EVAL_TESTCH10 )
      {
        Serial << " unknow!";
      }
      else /* therefore its a TestChX eval */
      {
        if( evCommand != evVal && testCh == true )   
        {
          Serial << " end" << sEVAL[evCommand];
          analogWrite( PWMPIN[(evCommand -1)], var[(evCommand -1)].dcCur );          
        }
        testCh = true;
        testDC = ( evOn ? (MAXDC - 1) : (MINDC + 1) ); 
        evCommand = evVal;
        Serial << ' ' << sEVAL[evVal] << " dc=" << testDC;
      }
  }
  Serial << endl;
  PINTP_D30_LOW;
}


/*--------------------------------- frameHandler() ----------------------------
 * 
 * CBUS frame processing called from the CBUS library for every CAN frame rx
*/

void frameHandler( CANFrame *msg ) 
{
  PINTP_D30_HIGH;
  if( debug == true )     /* debug is toggled by serial monitor command */
  {
    Serial << "> rxFrame id" << (msg->id & 0x7f) << " len" << msg->len;
    for( uint8_t i = 0; i < msg->len; i++ ) 
    {
      Serial << ' ' << _HEX( msg->data[i] );
    }
    Serial << endl;
  }
  PINTP_D30_LOW;
}


/*--------------------------------------- sendEvent() -------------------
 * 
 * Send CBUS message ACON / ACOF and EN
*/

void sendEvent( ONOFF_t onOff, uint16_t en )
{
  if( config.FLiM == true && noCAN == false )
  {
    CANFrame msg;   /* create and initialise a message object */
    msg.len = 5;
    msg.data[0] = ( onOff == true ? OPC_ACON : OPC_ACOF );
    msg.data[1] = highByte( config.nodeNum );
    msg.data[2] = lowByte( config.nodeNum );
    msg.data[3] = highByte( en );
    msg.data[4] = lowByte( en );
    
    if( ! CBUS.sendMessage( &msg ) )
    {
      Serial << endl << "! Error 0x" << _HEX( CBUS.canp->errorFlagRegister() );
    }
    Serial << "< tx ACO" << ( onOff == true ? "N n" : "F n" ) << config.nodeNum 
      << " e" << en << ( ( en >= QTY_EN ) ? "=?" : sEN[en] ) << endl << endl;
  }
}


/*-------------------------------- SerMon::about() -----------------------------
 *
 * print title, code version and serial monitor menu
*/

void SerMon::about(char boot) 
{
  Serial << endl << endl << boot << sTITLE
  << " v" << VER_MAJOR << VER_MINOR << " β" << VER_BETA << endl
  << " menu: c=bus, e=ev, v=var, m=mem, A=Amp, T=tx, /=debug, *=mute, DN=DefNv"
  << endl;
}


/*----------------------------------- SerMon::cbusState() ---------------------
 *
 * print module CBUS state - NOCAN or SLiM/FLiM + CANID & NN
*/

void SerMon::cbusState()
{
  if( noCAN == true )
  {
    Serial << " NoCAN" << endl;
  }
  else
  {
    Serial << ( config.FLiM == true ? " FLiM" : " SLiM" )
     << " CANID=" << config.CANID << " NN=" << config.nodeNum << endl << ' ';
    CBUS.printStatus();
  }
}


/*----------------------------- SerMon::variables() ---------------------------
 *
 * print Variables
*/

void SerMon::variables()
{
  const uint8_t BSIZE { 75 };
  char buf[BSIZE];
  const char fmt[]
    { " %2d  %3ds %5dms %3ds %3ds  %3d  %3d  %s   %d  %s  %3d %3ds" };
    
  Serial << " Vars. DayNight=" << sINPUT[input] << " evCmd=" <<
     sEVAL[evCommand] << endl <<
     " ch Trans perStep Dly0 Dly1  DC0  DC1  mode   phase state dcCur count";
  for( uint8_t ch = 0; ch < QTY_CHAN; ch++ )
  {
    snprintf( buf, BSIZE, fmt,
      ch +1, var[ch].secTrans, var[ch].msPerStep, var[ch].secDelay[0],
      var[ch].secDelay[1], var[ch].dc[0], var[ch].dc[1], sMODE[var[ch].mode],
      var[ch].phase, sSTATE[var[ch].state], var[ch].dcCur, var[ch].secCount );
    Serial << endl << buf;
  }
  Serial << endl;
}


/*----------------------------- setDefaultNVs() --------------------------
*
* set Node Variables to default test config.
*/

void setDefaultNVs()
{
  for( uint8_t nv = 0; nv < QTY_NV; nv++ )
  {
    config.writeNV( nv + 1, DEFAULTNV[nv] );
  }
  setupChannels();
  Serial << " NVs Reset. In FCU, do Node>ReadNVs and save." << endl;
}


/*--------------------------- SerMon::storedEvents() --------------------------
 *
 * print Stored Events
*/

void SerMon::storedEvents()
{
  Serial << endl << " Stored Events" << endl;
  uint8_t count { 0 };
  
  for( uint8_t j = 0; j < config.EE_MAX_EVENTS; j++ ) /* for each storedEvent */
  {
    if( config.getEvTableEntry( j ) != 0 )            /* is ev entry valid?   */
    {
      count++;
      uint8_t v[config.EE_BYTES_PER_EVENT];

      for( uint8_t e = 0; e < ( config.EE_NUM_EVS + 4 ); e++ ) /* each event  */
      {
        v[e] = config.readEEPROM(
              config.EE_EVENTS_START + ( j * config.EE_BYTES_PER_EVENT ) + e );
      };
      uint8_t eval = config.getEventEVval( j, 1 );

      Serial << ' ' << j << "  n" << ( v[0] * 256 ) + v[1]
        << " e" << ( v[2] * 256 ) + v[3] << " \teVal=" << eval << ' '
        << ( ( eval >= QTY_EVAL ) ? "!unknown" : sEVAL[eval] ) << endl;   
    }
  }
  Serial << "   " << count << '/' << config.EE_MAX_EVENTS << endl;
}


/*---------------------------- SerMon::processKeyBoard() ------------------------
 *
 * command interpreter for Serial Monitor input
*/

void SerMon::processKeyBoard() 
{
  if( Serial.available() ) 
  {
    switch( Serial.read() )   
    {
      case 'D':                     /* Default Node variables. R plus N */
        while( ! Serial.available() ){};
        if( Serial.read() == 'N' )
        { setDefaultNVs(); }
        break;
      case 'A':           /* print estimate LED Amps                    */
        PWR.printAmps();
        break;
      case 'c':           /* CBUS status                                */
        SerialMon.cbusState();
        break;
      case 'e':           /* stored learned Event data table            */
        SerialMon.storedEvents();
        break;
      case 'v':           /* Variables table                            */
        SerialMon.variables();
        break;
      case 'y':           /* reset CAN bus & CBUS processing            */     
        CBUS.reset();
        break;
      case 'm':
        Serial << " freeSRAM=" << config.freeSRAM() << endl;
        break;
      case 't':           /* send test message ACOF                     */
        sendEvent( ONOFF_OFF, EN_TESTMSG );
        break;
      case 'T':           /* send test message ACON                     */
        sendEvent( ONOFF_ON, EN_TESTMSG );
        break;
      case '/':           /* debug toggle                               */
        debug = ! debug;
        Serial << " debug=" << debug << endl;
        break;
      case '*':
        muteAlarm = ! muteAlarm;
        Serial << " mute=" << muteAlarm << endl;
        break;
      case '?':           /* about                                     */
        SerialMon.about();
      default:            /* all unhandled input chars, do nothing     */  
        break;
    }
  }
}

/*----------------- CANlights_1.ino  EoF --------------------------------------
*/
