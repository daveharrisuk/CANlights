/* file: CANlights.ino
 *------------------------------------------------------------------------------
 *
 * CANlights CBUS module. 10x LED string controller for overhead/street/building 
*/

const char  sMODTITLE[] { "CANlights © Dave Harris (MERG M2740) 2021" };
const char sCBUSTITLE[] { "CBUS library © Duncan Greenwood (MERG M5767) 2019" };
const byte VERSIONMAJOR { 0 };            /* CBUS module versioning style   */
const char VERSIONMINOR { 'a' };
const byte VERSIONBETA  { 2 };            /* 0 is Released, > 0 is beta     */
const byte MODULE_ID    { 99 };
unsigned char sCBUSNAME[7] { 'L', 'I', 'G', 'H', 'T', 'S', ' ' };

const float SPI_MHz { 16.0 };             /* frequency of CAN SPI bus clock */

/*------------------------ © Copyright and License -----------------------------
 *  
 * This code is © Dave Harris 2021 (contact at https://github.com/daveharrisuk)
 *  and uses, un-modified, the 'CBUS Library' repositories, which are all
 *  © Duncan Greenwood 2017 (contact at https://github.com/obdevel).
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
 * - Duty Cycle for on/off or Day/Night in configuration.
 *
 * - Over current protection. Audio alarm sounder.
 * - diagnostics via Arduino Serial Monitor. 
 * - CBUS test mode for channels
 * 
 * - CBUS event used as day/night trigger.
 * 
 * - configuration is in 60 CBUS NVs
 *
 *-------------------------------- target -------------------------------------
 *  
 *  Framework : Arduino 1.8.13 (on Ubuntu 20)
 *  PCB       : CANlights revB - has MCU board as daughter board
 *  MCU board : 'MEGA 2560 PRO' (Arduino Mega2560 should work, on breadboard)
 *  Processor : ATmega2560 
 *    Clock   : 16 MHz
 *    Flash   : 25.7 KB of 256 KB (note 8 KB is reserved for bootloader)
 *    SRAM    : 2.4 KB of 8 KB used.
 *    EEPROM  : 0.1 KB of 4 KB used.
 *    Timer   : 6 of 6 used.  (100%)
 *    PWM chan: 10 of 16 used (100% as 2 timers[6 comparator] otherwise used)
 *    ADC chan: 2 of 16 ADC used.
 *    IO pins : 29 of 70 used.
 *  USB Serial: CHG340G @ 12 MHz.
 *  CAN ctrl  : MCP2515 on hardware SPI bus (see const SPI_MHz).
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
 *  1-Mar-2021 DH, v0a beta1 add CBUS library and MCP2515 to SPI bus
 * 11-Apr-2020 DH, v0a beta2 CANlights basic working (a few bugs to go)
 *
 *
 *-------------------------- file includes ----------------------------------
*/

#include <cbusdefs.h>     /* 8r       https://github.com/MERG-DEV    */
#include <CBUS2515.h>     /* 1.1.11   https://github.com/MERG-DEV    */
#include <CBUSconfig.h>   /* 1.1.9    https://github.com/MERG-DEV    */

#include "CANlights.h"    /* class and function declarations         */
#include "DATADEF.h"      /* data structure definitions              */
#include "PIN.h"          /* module pin definitions                  */
#include "GAMMA8.h"       /* Gamma correction table                  */
#include "DEFAULTNV.h"    /* factory reset NV array                  */

#include <ACAN2515.h>     /* 2.0.6  MCP2515/25625 CAN controller     */
#include <Streaming.h>    /* 6.0.8  Serial                           */
#include <TimerOne.h>     /* 1.1      1 ms foreground process        */

#include <CBUSswitch.h>   /* 1.1.7    https://github.com/MERG-DEV    */
#include <CBUSLED.h>      /* 1.1.6    https://github.com/MERG-DEV    */


/*---------------------------- global variables ------------------------------*/


unsigned char params[21];     /* CBUS params                                  */

volatile var_t var[CHANQTY];  /* channel variable array. Is modified by ISR.  */
                              /* A mix of NVs, derived and tracking values    */

volatile INPUT_t  input;               /* input state 0 DAY or 1 NIGHT        */

volatile bool inputChanged { true };   /* reset this flag when actioned       */

EVAL_t evCommand { EVAL_TESTEND };

byte testDC { 1 };

bool debug { false };         /* false = quiet, true = verbose                */
bool muteAlarm { false };     /* false = quiet, true = sounding               */

bool stopISR { false };       /* flag to disable 1 ms ISR process, if true    */



/*------------------------------- objects ----------------------------------- */


CBUS2515 CBUS;                /* CBUS object                                  */
  
CBUSConfig config;            /* CBUSconf object                              */

CBUSLED CBUSgreen;            /* LED green object                             */
  
CBUSLED CBUSyellow;           /* LED yellow object                            */
  
CBUSSwitch CBUSmode_PB;       /* CBUS mode push switch object                 */

SerMon SerialMon;             /* Serial Monitor print object                  */

GetNV NV;                     /* lookup NV for chan and datatype              */

Power PWR;                    /* Power object                                 */



/*---------------------------------------------------- loop() ----------------- 
 * 
 * Background process.
*/

void loop() 
{
  CBUS.process();

  PWR.test();

  if( inputChanged == true )  /* flag set by Event handler  */
  {
    inputChanged = false;     /* clear the flag             */
    
    startNewPhase();
  }
  
  SerialMon.processinput();
}



/*--------------------------------------------- processChan( ch ) ------
 * 
 * called from 1 ms foreground process. Cycle though the channels.
*/

void processChan( byte ch, bool & transits )
{
  if( var[ch].state != STATE_STEADY )   /* is state not STEADY?        */
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
        else          /* in Transit & current DC is GT or LT target DC */
        {
          if( var[ch].dcCur > var[ch].dc[var[ch].phase] )    /* which? */
          {
            var[ch].dcCur--;                        /* GT so decrement */
          }
          else
          {
            var[ch].dcCur++;                        /* LT so increment */
          }
          analogWrite( PWMPIN[ch], GAMMA8[var[ch].dcCur] );  /* new dc */
        }
      }
    }
  }
}



/*--------------------------------------------- processChannels() ----------
 * 
 * timer1 Interrupt Service Routine.  1 ms foreground process.
*/
                                /* !!! This is ISR which runs every 1 ms !!! */
void processChannels()          /*        keep it light as possible          */
{                     
  PINTP_D31_HIGH;               /* is ISR run time = min 21 us to peak 87 us */

  bool transits { false };      /* lights LED_BUILTIN if Transits are active */

  if( stopISR == false )        /* other code that changes DCs, sets it true */
  {                 
    for( byte ch = 0; ch < CHANQTY; ch++ )          /* loop all channels     */
    {
      if( ch + 1 == evCommand )     /* TEST?  ch start at 0, cmd ch starts 1 */
      {
        analogWrite( PWMPIN[ch], testDC );    /* test DC is either 1 or 254  */
      }
      else
      if( evCommand != EVAL_SHUTDOWN ) /* SHUTDOWN is handled in eventHandler*/
      {
        processChan( ch, transits );
      }
    }
    digitalWrite( LED_BUILTIN, transits );         /* show TRANSIT activity */
  }
  PINTP_D31_LOW;   /* time from pin high to low is ISR run time */
}



/*-------------------------------------------- startNewPhase() ---------------
 * 
 * The day/night input has changed, so setup new phase for each channel 
*/

void startNewPhase()
{
  stopISR = true; /* all variables here are also manipulated by ISR */
  
  for( byte ch = 0; ch < CHANQTY; ch++ ) /* loop all channels */
  {
    var[ch].secCount = 0;
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
  }                              /* end ch loop */
  stopISR = false;
}



/*---------------------------------------- setupChannels() -------------------
 * 
 * preset channel variables from channel NVs and set the PWM
 * called from setup(), before timerOne is started
*/

void setupChannels()
{
  for( byte ch = 0; ch < CHANQTY; ch++ )     /* loop through channels   */
  {
    var[ch].dc[0]       = NV.dc( ch, 0 );
    var[ch].dc[1]       = NV.dc( ch, 1 );
    var[ch].secDelay[0] = NV.dly( ch, 0 );
    var[ch].secDelay[1] = NV.dly( ch, 1 );
    var[ch].secTrans    = NV.tran( ch );
    var[ch].mode        = (MODE_t) NV.mode( ch );
    if( var[ch].mode >= MODEQTY )
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
        var[ch].dc[0] = 100;
        var[ch].dc[1] = 150;
        
        Serial << endl << "!DC diff < 4 Reset ch" << ch +1 << endl;
      }
      else /* calculate */
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
  setPins();

  Serial.begin( 115200 );
  SerialMon.about();
  
  config.EE_NVS_START = 10;
  config.EE_NUM_NVS = NVQTY;
  config.EE_EVENTS_START = 75;
  config.EE_MAX_EVENTS = EVENTSQTY;
  config.EE_NUM_EVS = 1;
  config.EE_BYTES_PER_EVENT = ( config.EE_NUM_EVS + 4 );
  
  config.setEEPROMtype( EEPROM_INTERNAL );
  
  config.begin();

  params[0] = 20;                     /* Number of parameters                */
  params[1] = MANU_MERG;              /* Manufacturer ID                     */
  params[2] = VERSIONMINOR;           /* minor version                       */
  params[3] = MODULE_ID;              /* module id, 99 = undefined           */
  params[4] = config.EE_MAX_EVENTS;   /* number of events                    */
  params[5] = config.EE_NUM_EVS;      /* number event variables per event    */
  params[6] = config.EE_NUM_NVS;      /* number of Node Variables            */
  params[7] = VERSIONMAJOR;           /* major version                       */
  params[8] = 0x07;                   /* Node Flags  PF_COMBI & PF_FLiM      */
  params[9] = 0x32;                   /* processor ID                        */
  params[10] = PB_CAN;                /* interface protocol - CAN / Ethernet */
  params[11] = 0x00;                  /* download load address               */
  params[12] = 0x00;                  /*      -"-                            */
  params[13] = 0x00;                  /*      -"-                            */
  params[14] = 0x00;                  /*      -"-                            */
  params[15] = '2';                   /* Processor code                      */
  params[16] = '5';                   /*      -"-                            */
  params[17] = '6';                   /*      -"-                            */
  params[18] = '0';                   /*      -"-                            */
  params[19] = CPUM_ATMEL;            /* Manufacturer code                   */
  params[20] = VERSIONBETA;           /* 0 is released, >0 is beta version   */

  Serial << SPI_MHz << " MHz SPI.  Param[8]flags=" << params[8] << endl;
    
  CBUS.setParams( params );           /* assign params to CBUS               */
  CBUS.setName( sCBUSNAME );          /* assign module name to CBUS          */
  
  CBUSgreen.setPin( PINLEDGRN );
  CBUSyellow.setPin( PINLEDYEL );

  CBUSmode_PB.setPin( PINSW0, LOW );          /* initialise CBUS mode switch */

  // module reset - if switch is depressed at startup and module is in SLiM mode
  
  CBUSmode_PB.run();

  if( CBUSmode_PB.isPressed() && ! config.FLiM ) 
  {
    Serial << " switch was pressed at startup in SLiM mode" << endl;
    
    config.resetModule( CBUSgreen, CBUSyellow, CBUSmode_PB );
  }

  CBUS.setEventHandler( eventHandler );       /* call on every learned event  */

  CBUS.setFrameHandler( frameHandler );       /* call on every CAN frame.     */

  CBUS.setLEDs( CBUSgreen, CBUSyellow );
  
  CBUS.setSwitch( CBUSmode_PB );

  CBUS.indicateMode( config.FLiM );         /* set CBUS LEDs for current mode */
  
  CBUS.setNumBuffers( 4 );           /* more buffers, more memory, fewer less */
  
  CBUS.setOscFreq( SPI_MHz * 1000000 );         /* SPI clock frequency        */
  CBUS.setPins( PINSPI_SS, PINSPI_INT );        /* SPI pins, bar defaults     */
  
  while( CBUS.begin() == false )
  {
    digitalWrite( PINLEDRED, HIGH );
    PWR.alarm( 250 );
  }
  digitalWrite( PINLEDRED, LOW );
  
  setupChannels();

  analogReference( INTERNAL1V1 );               /* MEGA unique, 1.075 mV/bit  */
  
  Timer1.initialize( 1000 );                    /* ISR to run @ 1000 us, 1 ms */
  Timer1.attachInterrupt( processChannels );    /* Foreground process ISR     */

  SerialMon.cbusState();
  SerialMon.freeSRAM();
  SerialMon.variables();
  SerialMon.storedEvents();
    
  sendCBUSmessage( 1, EN_POWER );
}



/*--------------------------------------- Power::checkSerialSend() -----------
 * 
 * check and action serial input, spacebar toggles mute
*/

void Power::checkSerialSend()
{
  if( Serial.available() > 0  &&  Serial.read() == ' ' )
  {
      muteAlarm = ! muteAlarm;
  }  
}



/*------------------------------------------ Power::alarm() -----------------
 * 
 * Sound Audio Warning Device for n milli seconds.
 * Timers are all used up, but this can be done OK on a loop.
 * AWD piezo buzzer is 4 kHz, is cycle time 250 us (125 us high and low).
*/

void Power::alarm( uint16_t duration_ms )
{
  checkSerialSend();
  
  uint16_t loopCount = duration_ms * 4;
  
  if( muteAlarm == false )
  {
    for( uint16_t k = 0; k < loopCount; k++ )
    {
      digitalWrite( PINAWDSIG, k & 1 );  /* LSB of k, even-odd-even, 0-1-0... */
      delayMicroseconds( 121 );          /* 121 allows for overheads in loop  */
    }
  }
  else delay( duration_ms );
}



/*---------------------------------------- Power::isUnderVolt() ----------
 * 
 * If blue LED is not lit, the 12 V line failed or PolyFuse tripped.
 * 12 V feeds onto Blue LED, Vf ~ 2.8 V. ADC ref is 1.1 V.
*/

bool Power::isUnderVolt()
{
  return ( analogRead( PINBLUE ) < 1022 ); 
}



/*----------------------------------------- Power::isOverAmp() --------------
 * 
 * Read Volts on sense resistor to measure Amps.
 * return true if over Amp and false if Amps OK
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
  Serial << ' ' << ( amps * AMPCALIBRATE ) << "mA" << endl;
}


/*--------------------------------------------- Power::test() --------------
 * 
 * If under Volt (poly fuse?) or over current then alarm & reduce duty cycles.
*/

void Power::test()
{  
  if( isOverAmp() == true || isUnderVolt() == true )
  {
    stopISR = true;                 /* stop ISR as this code overides DC  */
    digitalWrite( PINLEDRED, HIGH );
    sendCBUSmessage( 1, EN_ALARM );
    
    turnOffPWMs();      

    Serial << "! OverAmp or UnderVolt";
    printAmps();
    do
    {
      alarm( 250 ); /* sound AWD for n milli secs */
    }
    while( isOverAmp() == true || isUnderVolt() == true );
    
    restorePWMs();
    
    sendCBUSmessage( 0, EN_ALARM );
    digitalWrite( PINLEDRED, LOW );
    stopISR = false;
  }
}



/*----------------------------- turnOffPWMs() --------------------------
 * 
 * set all channel duty cycles to zero value
*/

void turnOffPWMs()
{
  Serial << "! all PWM off" << endl;
  for( byte ch = 0; ch < CHANQTY; ch++ )
  {
    analogWrite( PWMPIN[ch], 0 );
  }
}



/*------------------------------ restorePWMs() -------------------------
 * 
 * restore all running duty cycles
*/

void restorePWMs()
{
  Serial << ". all PWM restored" << endl;
  for( byte ch = 0; ch < CHANQTY; ch++ )
  {
    analogWrite( PWMPIN[ch], GAMMA8[var[ch].dcCur] );
  }
}



/*-------------------------------- GetNV::tran() ---------------------------
 *
 * return transition NV for chan     NV.tran( 5 ) is chan 5 transition
*/

byte GetNV::tran( byte chan )
{
  return config.readNV( NVmap[NV_TRAN][0] + chan );
}



/*-------------------------------- GetNV::dly() ----------------------------
 * 
 * return delay NV for chan & index     NV.dly( 8, 1 ) is chan 8 Dly1
*/

byte GetNV::dly( byte chan, bool indx )
{
  return config.readNV( NVmap[NV_DLY][indx] + chan );
}



/*------------------------------------ GetNV::dc() ------------------------
 *
 * return duty cycle NV for chan & index      NV.dc( 9, 1 ) is chan 9 DC1
*/

byte GetNV::dc( byte chan, bool indx )
{
  return config.readNV( NVmap[NV_DC][indx] + chan );
}



/*---------------------------------- GetNV::mode() --------------------------
 *
 * return mode NV for chan                 NV.mode( 0 ) is chan 0 Mode
*/

byte GetNV::mode( byte chan )
{
  return config.readNV( NVmap[NV_MODE][0] + chan );
}



/*---------------------------------------- eventHandler() -------------------
 * 
 * event process function called from CBUS library when learned event is rx
*/

void eventHandler( byte index, CANFrame *msg ) 
{
  EVAL_t  evVal = (EVAL_t) config.getEventEVval( index, 1 );
  
  byte opc = msg->data[0];
  bool evOn = ( opc == OPC_ASON || opc == OPC_ACON );
  
  Serial << "> rx Event opc=0x" << _HEX(opc) << " on/off=" << evOn 
                  << " evVal=" << evVal << ' ';
  switch( evVal )
  {
    case EVAL_DAYNIGHT:
      Serial << sINPUT[evOn];
      if( input != (INPUT_t) evOn )
      {
        input = (INPUT_t) evOn;
        inputChanged = true;        /* flag for loop() to change phases */
      }
      else Serial << " ignored";
      break;
      
    case EVAL_SHUTDOWN:
      Serial << "Shutdown";
      if( evCommand == EVAL_SHUTDOWN )
      {
        Serial << " ignored";
      }
      else
      {
        if( evOn == true )
        {
          evCommand = EVAL_SHUTDOWN;
          turnOffPWMs();
        }
        else
        {
            evCommand = EVAL_DAYNIGHT;
            restorePWMs();
        }
      }
      break;
      
    case EVAL_TESTEND:
      if( evCommand >= EVAL_TESTCH1 && evCommand <= EVAL_TESTCH10 )
      {
        Serial << "TestEnd ch" << evCommand;
        analogWrite( PWMPIN[(evCommand -1)], var[(evCommand -1)].dcCur );
        evCommand = EVAL_DAYNIGHT;
      }
      else Serial << "TestEnd ignored";
      break;
      
    default:
      if( evVal <= EVAL_TESTCH10 ) /* then it is TestCh1 to TestCh10 */
      {
        if( evCommand >= EVAL_TESTCH1 && evCommand <= EVAL_TESTCH10 )
        {
          Serial << " (testend" << evCommand << ')';
          analogWrite( PWMPIN[(evCommand -1)], var[(evCommand -1)].dcCur );          
        }
        testDC = ( evOn ? 254 : 1 ); 
        evCommand = evVal;
        Serial << sEVAL[evVal] << " dc=" << testDC;        
      }
      else Serial << "! unknow evVal";
  }
  Serial << endl;
}



/*--------------------------------- frameHandler() ----------------------------
 * 
 * CBUS frame processing called from the CBUS library for every CAN frame rx
*/

void frameHandler( CANFrame *msg ) 
{
  if( debug == true )     /* set/cleared by Serial monitor command */
  {
    Serial << "> rx frame id" << (msg->id & 0x7f) << " len" << msg->len;
    for( byte i = 0; i < msg->len; i++ ) 
    {
      Serial << ' ' << _HEX( msg->data[i] );
    }
    Serial << endl;
  }
}



/*------------------------------------- sendCBUSmessage() -------------------
 * 
 * Send CBUS message ACON / ACOF and EN
*/

void sendCBUSmessage( bool onOff, uint16_t en )
{
  if( config.FLiM == true )
  {
    CANFrame msg;   /* create and initialise a message object */
    msg.len = 5;
    msg.data[0] = ( onOff ? OPC_ACON : OPC_ACOF );
    msg.data[1] = highByte( config.nodeNum );
    msg.data[2] = lowByte( config.nodeNum );
    msg.data[3] = highByte( en );
    msg.data[4] = lowByte( en );
    
    if( ! CBUS.sendMessage( &msg ) )
    {
      Serial << endl << "!Failed";
    }
    Serial << "< tx AC" << (onOff?"ON":"OF") << " n" << config.nodeNum << " e"
      << en << ( ( en >= SIZE_EN ) ? "!unknown" : sEN[en] ) << endl << endl;
  }
}



/*-------------------------------- SerMon::about() -----------------------------
 *
 * print titles and code version
*/

void SerMon::about() 
{
  Serial << endl<< endl << sMODTITLE << "  v" << VERSIONMAJOR << VERSIONMINOR
  << ( VERSIONBETA ? " beta ":" " ) << VERSIONBETA << endl << sCBUSTITLE << endl
  << " menu c=bus, e=ev, v=var, m=mem, @=Amp, t=tx, /=debug, *=mute, RN=RstNv"
  << endl << endl;
}



/*----------------------------------- SerMon::state() ---------------------------
 *
 * print module CBUS state - SLiM/FLiM, CANID & NN
*/

void SerMon::cbusState()
{
  Serial << endl << " CBUS " << ( config.FLiM ? "FLiM":"SLiM" )
  << " CANID=" << config.CANID << " NN=" << config.nodeNum << endl << ' ';
  CBUS.printStatus();
}



/*----------------------------- SerMon::variables() -----------------------------
 *
 * print Variables
*/

void SerMon::variables()
{
  const byte BUFSIZE { 72 };
  char buf[BUFSIZE];

  char fmt[BUFSIZE]
    = " %2d  %3ds %5dms  %3ds  %3ds  %3d  %3d  %s  %d   %s  %3d  %3ds";

  Serial << endl << " Vars.  Input=" << sINPUT[input] << " evCmd=" <<
   sEVAL[evCommand] << endl <<
  " ch Trans perStep  Dly0  Dly1  DC0  DC1  mode    phase state dcCur count"
   << endl;
  for( byte ch = 0; ch < CHANQTY; ch++ )
  {
    snprintf( buf, BUFSIZE, fmt,
      ch +1, var[ch].secTrans, var[ch].msPerStep, var[ch].secDelay[0],
      var[ch].secDelay[1], var[ch].dc[0], var[ch].dc[1], sMODE[var[ch].mode],
      var[ch].phase, sSTATE[var[ch].state], var[ch].dcCur, var[ch].secCount
      );
    Serial << buf << endl;
  }
  Serial << endl;
}



/*----------------------------- resetNodeVariables() --------------------------
*
* reset Node Variables to factory test config.
*/

void resetNodeVariables()
{
  for( byte nv = 0; nv < NVQTY; nv++ )
  {
    config.writeNV( nv + 1, DEFAULTNV[nv] );
  }
  setupChannels();
  
  Serial << " Reset NVs. Now, in the FCU, do Node>ReadNVs and save." << endl;
}



/*--------------------------- SerMon::storedEvents() --------------------------
 *
 * print Stored Events
*/

void SerMon::storedEvents()
{
  Serial << " Stored Events" << endl;
  byte count { 0 };
  
  for( byte j = 0; j < config.EE_MAX_EVENTS; j++ )   /* for each stored event */
  {
    if( config.getEvTableEntry( j ) != 0 )           /* is ev entry valid?    */
    {
      count++;
      byte v[config.EE_BYTES_PER_EVENT];

      for( byte e = 0; e < ( config.EE_NUM_EVS + 4 ); e++ ) /* for each event*/
      {
        v[e] = config.readEEPROM(
              config.EE_EVENTS_START + ( j * config.EE_BYTES_PER_EVENT ) + e );
      };
      byte eval = config.getEventEVval( j, 1 );

      Serial << ' ' << j << " n" << ( v[0] * 256 ) + v[1]
        << " e" << ( v[2] * 256 ) + v[3] << " eVal=" << eval << ' '
        << ( ( eval >= SIZE_EVAL ) ? "unknown" : sEVAL[eval] ) << endl;   
    }
  }
  Serial << "   " << count << '/' << config.EE_MAX_EVENTS << endl << endl;
}



/*--------------------------------- SerMon::freeSRAM() ---------------------------
 *
 * print free SRAM
*/

void SerMon::freeSRAM()
{
  Serial << endl << " freeSRAM " << config.freeSRAM() << endl;
}



/*---------------------------- SerMon::processinput() ------------------------
 *
 * command interpreter for Serial Monitor input
*/

void SerMon::processinput() 
{
  if( Serial.available() ) 
  {
    switch( Serial.read() )   
    {
      case 'R':                 /* Reset Node variables. R followed by N */
        while( ! Serial.available() ){};
        if( Serial.read() == 'N' )
        {
          resetNodeVariables();
        }
        break;
      case '@':           /* estimate LED Amps                          */
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
      case 'm':           /* free SRAM memory                           */
        SerialMon.freeSRAM();
        break;
      case 't':           /* send test message ACOF                     */
        sendCBUSmessage( 0, EN_TESTMSG );
        break;
      case 'T':           /* send test message ACON                     */
        sendCBUSmessage( 1, EN_TESTMSG );
        break;
      case '/':           /* debug toggle                               */
        debug = ! debug;
        Serial << " debug" << debug << endl;
        break;
      case '*':
        muteAlarm = ! muteAlarm;
        Serial << " muteAlarm" << muteAlarm << endl;
        break;
      case '?':           /* about                                      */
        SerialMon.about();
        break; 
      default:            /* all unhandled input chars.. Do nothing     */  
        break;
    }
  }
}



/*------------------------------ setPins() ------------------------------------
 * setup() calls this
*/

void setPins()
{
  pinMode( PINLEDRED, OUTPUT );       /* Red LED: over Amp/under Volt alarm   */
  pinMode( PINLEDYEL, OUTPUT );       /* Yellow LED:  TBA                     */
  pinMode( PINLEDGRN, OUTPUT );       /* Green LED: input state LED           */
  pinMode( LED_BUILTIN, OUTPUT );     /* Red LED on MEGA, DC Transit active   */
  
  digitalWrite( PINLEDRED, 1 );       /* all LED on for 0.25 Sec self test    */
  digitalWrite( PINLEDYEL, 1); 
  digitalWrite( PINLEDGRN, 1 );
  digitalWrite( LED_BUILTIN, 1 );

  pinMode( PINSW0, INPUT_PULLUP );    /* Tactile push switch                  */
  
  pinMode( PINBLUE,   INPUT );        /* ADC in: Vf sensed on Power blue LED  */
  pinMode( PINSENSE,  INPUT );        /* ADC in: Amp sense resistor           */

  pinMode( PINENCPHA, INPUT );        /* Rotary encoder phase A               */
  pinMode( PINENCPHB, INPUT );        /* Rotary encoder phase B               */
  pinMode( PINENCSW,  INPUT );        /* Rotary encoder switch via            */

  pinMode( PINAWDSIG, OUTPUT );       /* AudioWarningDevice: Piezo buzzer     */
  PWR.alarm( 250 );                   /* test sounder for 0.25 s              */
  
  pinMode( PINTP_D30, OUTPUT );       /*Test Point: Scope measure TBA         */
  PINTP_D30_HIGH;                     /* macro fast pin method. See PIN.h     */
  PINTP_D30_LOW;
  
  pinMode( PINTP_D31, OUTPUT );       /*Test Point: measure ISR duration      */
  PINTP_D31_HIGH;
  PINTP_D31_LOW;                      /* macro fast pin method. See PIN.h     */

  for( byte ch = 0; ch < CHANQTY; ch++ ) /* loop all PWM channels             */
  {
    pinMode( PWMPIN[ch], OUTPUT );
  }

  digitalWrite( PINLEDRED, 0 );       /* all LEDs off, after 0.25s (bar blue) */
  digitalWrite( PINLEDYEL, 0 ); 
  digitalWrite( PINLEDGRN, 0 );
  digitalWrite( LED_BUILTIN, 0 );
}


/*----------------- CANlights_1.ino  EoF -----------------------------------
*/
