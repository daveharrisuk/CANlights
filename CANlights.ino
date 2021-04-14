/* file: CANlights.ino
* ------------------------------------------------------------------------------
*
* CANlights CBUS module - Controller for overhead/street/building LED strings. 
*/

const char  sMODTITLE[] {"CANlights © Dave Harris (MERG M2740) 2021"};

const char sCBUSTITLE[] {"CBUS library © Duncan Greenwood (MERG M5767) 2019"};

const byte VERSIONMAJOR { 0 };          /* CBUS module versioning style */

const char VERSIONMINOR { 'a' };

const byte  VERSIONBETA { 2 };          /* 0 is Released, > 0 is beta   */

const byte    MODULE_ID { 99 };

unsigned char sCBUSNAME[7] { 'L', 'I', 'G', 'H', 'T', 'S', ' ' };


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
 *  visit http://creativecommons.org/licenses/by-nc-sa/4.0/ or send
 *  a letter to Creative Commons, PO Box1866, Mountain View, CA 94042 USA.
*/


/*-------------------------- Description --------------------------------------
 *
 * CBUS layout lighting control module for 10 LED strings.
 *
 * - 10x 12 Volt PWM channels driving LED strings. 
 *
 * - Input (Day/Night) triggers channel modes...
 *    MODE_DAYNIGHT : Duty cycle (DC) transition on input change
 *    DAWN     : DC transitions on input changing to day
 *    DUSK     : DC transitions on input changing to night
 *    DUSKDAWN : DC transitions on input change
 *    NIGHT010 : During night time, LEDs transition at durations
 *    DAY010   : During day time, LEDs transition at durations
 *    tba ALWAYS010: LEDs transition at durations, not tied to day/night
 *
 * - Transition configuration...
 *    Delay seconds before transitions.
 *    Duty cycle Transition duration in seconds.
 *
 * - Duty Cycle for on/off / Day and Night in configuration.
 *
 * - Over current protection. Audio alarm sounder.
 * - diagnostics via Arduino Serial Monitor. 
 * - CBUS test mode for channels
 * 
 * - CBUS event used as day/night trigger.
 * 
 * - configuration is in CBUS NVs
*/


/*--------------------------- target ------------------------------------------
 *  
 *  Framework : Arduino 1.8.13 (on Ubuntu 20)  
 *  TargetPCB : CANlights (see KiCad files) has MCU board as a daughter board.
 *  MCU board : 'MEGA 2560 PRO' (Arduino Mega should work, on breadboard)
 *  Processor : ATmega2560 
 *    Clock   : 16 MHz
 *    Flash   : 25.4 kB of 256 KB (but 8 KB is bootloader)
 *    SRAM    : 2.2 kB of 8 kB used.
 *    EEPROM  : 0.1 kB of 4 kB used.
 *    Timer   : 6 of 6 used.   * 100% *
 *    PWM chan: 10 of 16 used. * 100% as 6 chans unusable *
 *    ADC chan: 2 of 16 ADC used.
 *    IO pins : 29 of 70 used
 *  USB Serial: CHG340G @ 12 MHz
 *  CAN contrl: MCP2515 SPI bus @ 8MHz.
*/


/*---------------------------- History ----------------------------------------
 *
 *  4-Jan-2021 Dave Harris (DH) Lights_Control_1 Project started
 *  9-Jan-2021 DH, v0.01  breadboard first test
 * 23-Jan-2021 DH, v0.02  add channel Modes
 *  2-Feb-2021 DH, v0.03  add TimerOne lib and 1 ms ISR
 *  4-Feb-2021 DH, v1.00  released on Lights_Control_1 PCB
 *  5-Feb-2021 DH, v1.01  correct over Amp code. Sense RC 0.1uF to 1.0uF
 * 11-Feb-2021 DH, v1.02  efficiency and tidy up changes
 * 23-Feb-2021 DH, v1.03  Bug fix, DC wrong after over Amp condition
 * 28-Feb-2021 DH, Lights_Control_1 frozen and code copied to CANlights.
 *          CANlights.ino  Versioning now in CBUS style.
 *  1-Mar-2021 DH, v0a b1 add CBUS library and MCP2515 to SPI bus
 * 11-Apr-2020 DH, v0a b2 CANlights basic working
 *
*/ 


/*-------------------------- file includes ----------------------------------*/

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



/*---------------------------- global variables -----------------------------
 *
*/


unsigned char params[21];     /* CBUS params                                 */



volatile var_t var[CHANQTY];  /* channel variable array. Is modified by ISR.  */
                              /* A mix of NVs, derived and tracking values    */

volatile INPUT_t  input;               /* input state 0 DAY or 1 NIGHT                 */

volatile bool inputChanged = true;     /* reset when actioned                          */
  

byte testChan = 11;

byte testDC = 1;


bool debug = false;           /* false = quiet mode, true = verbose           */

bool stopISR = false;         /* flag to disable 1 ms ISR process, if true    */



/*------------------------------- objects ------------------------------------
 *
*/


CBUS2515 CBUS;                /* CBUS object                                  */
  
CBUSConfig config;            /* CBUSconf object                              */

CBUSLED CBUSgreen;            /* LED green object                             */
  
CBUSLED CBUSyellow;           /* LED yellow object                            */
  
CBUSSwitch CBUSmode_PB;       /* CBUS mode push switch object                 */

SerMon SerialMon;             /* Serial Monitor print object                  */

GetNV NV;                     /* lookup NV for chan and datatype              */

Power PWR;                    /* Power object                                 */



void chanState()
{
  Serial << ' ' << sINPUT[input] << endl;
  for( byte ch = 0; ch < CHANQTY; ch++ )
  {
    Serial << " ch" << ch +1 
          << ' ' << sSTATE[var[ch].state]
          << ' ' << sPHASE[var[ch].phase]
          << " dc=" << var[ch].dcCur
          << ' ' << sMODE[var[ch].mode]
          << endl;
  }
}

/*---------------------------------- printChanConfig() ------------------------
 * 
 * print one channel config to Serial
*/

void printChanConfig( byte ch )
{
  static const byte BUFSIZE { 75 };
  
  static char buf[BUFSIZE];
  
  static const char sFMT[]
  {"ch%02u Tran=%03us Dly0=%03us Dly1=%03us dc0=%03u dc1=%03u step=%05ums %s"};
  
  snprintf( buf, BUFSIZE, sFMT
     , ( ch + 1 )                       /* chan number          */
     , var[ch].secTrans                 /* Transit seconds      */
     , var[ch].secDelay[0]              /* Delay 0 secs         */
     , var[ch].secDelay[1]              /* Delay 1 secs         */
     , var[ch].dc[0]                    /* duty cycle DC0       */
     , var[ch].dc[1]                    /* duty cycle DC1       */
     , var[ch].msPerStep                /* derived config value */
     , sMODE[var[ch].mode]              /* string, mode of chan */
    );
  Serial << buf << endl;
}




/*------------------------------------- loop() -------------------------------- 
 * 
 * Background process.
*/

void loop() 
{
  CBUS.process();

  PWR.test();

  if( inputChanged == true )  /* set by Event handler */
  {
    inputChanged = false;     /* clear the flag       */
    
    startNewPhase();
  }
  
  SerialMon.processinput();
}



/*-------------------------- processChannels() -----------------------------
 * 
 * timer1 Interrupt Service Routine.  1 ms foreground process
 * 
 * Cycle though the channels and process them
*/

void processChannels()         /* !!! This is ISR which runs every 1 ms !!! */
{                              /*        keep it light as possible          */
  
  SetPINTP_D31;                /* Scope TP on pin D31 ~= ISR run time       */
                               /* on ATmega2560 = min 21 us to peak 87 us   */

  bool transits = false;       /* lights LED_builtin if Transits are active */


  if( stopISR == false )                  /* allow this to change DC values */
  {                 
    for( uint8_t ch = 0; ch < CHANQTY; ch++ )      /* loop all channels     */
    {
      if( var[ch].state != STEADY )                /* is state not STEADY?  */
      {
        var[ch].msCount++;
        
        if( var[ch].state == DELAY )                 /* is state DELAY?     */
        {
          if( var[ch].msCount > 1000 )               /* count delay seconds */
          {
            var[ch].msCount = 0;
            var[ch].secCount++;
            
            if( var[ch].secCount > var[ch].secDelay[var[ch].phase] )
            {
              var[ch].state = TRANSIT;               /* end state DELAY     */
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
              var[ch].state = STEADY;                /* dc transit complete */
              
              switch( var[ch].mode )                 /* trigger a new phase */
              {
                case MODE_NIGHT010 :
                  if( input == INPUT_NIGHT )
                  {
                    var[ch].phase = ! var[ch].phase;
                    var[ch].state = DELAY;
                  }
                  break;
  
                case MODE_DAY010 :
                  if( input == INPUT_DAY )
                  {
                    var[ch].phase = ! var[ch].phase;
                    var[ch].state = DELAY;
                  }
                  break;
  
                case MODE_DUSK :
                  if( var[ch].phase == 1 )
                  {
                    var[ch].phase = 0;
                    var[ch].state = DELAY;                  
                  }
                  break;
                  
                case MODE_DAWN :
                  if( var[ch].phase == 1  )
                  {
                    var[ch].phase = 0;
                    var[ch].state = DELAY;                  
                  }
                  break;
                  
                case MODE_DUSKDAWN :
                  if( var[ch].phase == 1 )
                  {
                    var[ch].phase = 0;
                    var[ch].state = DELAY;
                  }
                  break;
                  
                case MODE_DAYNIGHT : 
                  break;
              }
            }
            else            /* in Transit & current DC is GT or LT target DC */
            {
              if( var[ch].dcCur > var[ch].dc[var[ch].phase] )   /* which? */
              {
                var[ch].dcCur--;                          /* GT so decrement */
              }
              else
              {
                var[ch].dcCur++;                          /* LT so increment */
              }
              analogWrite( PWMPIN[ch], GAMMA8[var[ch].dcCur] );    /* new dc */
            }
          }
        }
      }  /* end if state not STEADY   */
    }  /* end for ch loop             */
  
    digitalWrite( LED_BUILTIN, transits );         /* show TRANSIT activity */
    
  } /* end not stopISR   */
  
  ClrPINTP_D31;   /* time from pin high to low is ISR run time */
}



/*------------------------------ startNewPhase() -----------------------------
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

    var[ch].state = TRANSIT;      /* default values, some updated by switch   */
    var[ch].phase = 0;            /* this allows dc to transit back to dc0    */

    switch( var[ch].mode )
    {
      case MODE_DAYNIGHT :        /* either input edge starts MODE_DAYNIGHT   */
        var[ch].state = DELAY;
        var[ch].phase = input;    /* phase 0 = 0 (day), phase 1 = 1 (night)   */
        break;
        
      case MODE_DUSK :            /* day to night starts dusk                 */
        if( input == INPUT_NIGHT )
        {
          var[ch].state = DELAY;
          var[ch].phase = 1;
        }
        break;
        
      case MODE_DAWN :           /* night to day starts dawn                 */
        if( input == INPUT_DAY )
        {
          var[ch].state = DELAY;
          var[ch].phase = 1;
        }
        break;
        
      case MODE_DUSKDAWN :       /* either input edge starts DUSKDAWN         */
        var[ch].state = DELAY;
        var[ch].phase = 1;
        break;
        
      case MODE_NIGHT010 :       /* day to night starts NIGHT010              */
        if( input == INPUT_NIGHT )
        {
          var[ch].state = DELAY;
          var[ch].phase = 1;
        }
        break;
        
      case MODE_DAY010 :         /* night to day starts DAY010                */
        if( input == INPUT_DAY )
        {
          var[ch].state = DELAY;
          var[ch].phase = 1;
        }
        break;
    }
  }                              /* end ch loop */
  stopISR = false;
}



/*---------------------- setupChannels() -------------------------------------
 * 
 * preset channel variables from channel NVs and set the PWM
 *  
 * called from setup(), before timerOne is started
*/

void setupChannels()
{
  for( byte ch = 0; ch < CHANQTY; ch++ )     /* loop through channels   */
  {
    /* fetch NVs */
    
    var[ch].dc[0]       = NV.dc( ch, 0 );
    var[ch].dc[1]       = NV.dc( ch, 1 );
    var[ch].secDelay[0] = NV.dly( ch, 0 );
    var[ch].secDelay[1] = NV.dly( ch, 1 );
    var[ch].secTrans    = NV.tran( ch );
    var[ch].mode        = (MODE_t) NV.mode( ch );
    if( var[ch].mode >= MODEQTY )
    {
      Serial << F("!bad mode ") << var[ch].mode << " Reset ch" << ch+1 << endl;
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
        
        Serial << endl << F("!DC diff < 4 Reset ch") << ch +1 << endl;
      }
      else /* calculate */
      {
        var[ch].msPerStep = ( var[ch].secTrans * 1000UL ) / diff;
      }
    }
    
    var[ch].dcCur = MAXDC / 2;  /* */
    var[ch].state = STEADY;     /* */
    var[ch].phase = 0;          /* */

    analogWrite( PWMPIN[ch], var[ch].dcCur );
    
    printChanConfig( ch );
  } 
  //delay( 500 );
  
  startNewPhase();
}



/*------------------------------------- setup() -------------------------------
 *
 * runs at power on and reset
*/

void setup() 
{
  setPins();

  Serial.begin( 115200 );

  SerialMon.about();

  /* set CBUSconf layout parameters */
  
  config.EE_NVS_START = 10;
  config.EE_NUM_NVS = NVQTY;
  config.EE_EVENTS_START = 75;
  config.EE_MAX_EVENTS = EVENTSQTY;
  config.EE_NUM_EVS = 1;
  config.EE_BYTES_PER_EVENT = ( config.EE_NUM_EVS + 4 );


  /* initialise and load CBUSconfuration */
  
  config.setEEPROMtype( EEPROM_INTERNAL );
  
  config.begin();                     /* start CBUS config                   */

  params[0] = 20;                     /* Number of parameters                */
  params[1] = MANU_MERG;              /* Manufacturer ID                     */
  params[2] = VERSIONMINOR;           /* minor version                       */
  params[3] = MODULE_ID;              /* module id, 99 = undefined           */
  params[4] = config.EE_MAX_EVENTS;   /* number of events                    */
  params[5] = config.EE_NUM_EVS;      /* number event variables per event    */
  params[6] = config.EE_NUM_NVS;      /* number of Node Variables            */
  params[7] = VERSIONMAJOR;           /* major version                       */
  params[8] = 7;                      /* Node Flags  PF_COMBI & PF_FLiM      */
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

  Serial << F(" param 8(flags)=") << params[8] << endl;
  
  CBUS.setParams( params );           /* assign params to CBUS               */
  CBUS.setName( sCBUSNAME );          /* assign module name to CBUS          */

  /* initialise CBUS LEDs  */
  
  CBUSgreen.setPin( PINLEDGRN );
  CBUSyellow.setPin( PINLEDYEL );


  CBUSmode_PB.setPin( PINSW0, LOW );          /* initialise CBUS mode switch */

  // module reset - if switch is depressed at startup and module is in SLiM mode
  
  CBUSmode_PB.run();

  if( CBUSmode_PB.isPressed() && ! config.FLiM ) 
  {
    Serial << F(" switch was pressed at startup in SLiM mode") << endl;
    
    config.resetModule( CBUSgreen, CBUSyellow, CBUSmode_PB );
  }


  /* register CBUS event handlers */

  CBUS.setEventHandler( eventHandler );       /* call on every learned event  */

  CBUS.setFrameHandler( frameHandler );       /* call on every CAN frame.     */


  /* set LEDs and switch */

  CBUS.setLEDs( CBUSgreen, CBUSyellow );
  
  CBUS.setSwitch( CBUSmode_PB );

  CBUS.indicateMode( config.FLiM );         /* set CBUS LEDs for current mode */


  /* CBUSconfure and start CAN bus and CBUS message processing  */
  
  CBUS.setNumBuffers( 4 );           /* more buffers, more memory, fewer less */
  
  CBUS.setOscFreq( SPI_FREQ );                  /* SPI clock frequency        */
  CBUS.setPins( PINSPI_SS, PINSPI_INT );        /* SPI pins, bar defaults     */
  
  CBUS.begin();


  setupChannels();

  analogReference( INTERNAL1V1 );   /* most sensitive on MEGA, 1.075 mV/unit */
  
  Timer1.initialize( 1000 );                   /* ISR to run @ 1000 us, 1 ms */
  Timer1.attachInterrupt( processChannels );   /* Foreground process         */


  SerialMon.state();
  SerialMon.freeSRAM();
  SerialMon.variables();
  SerialMon.storedEvents();
  SerialMon.menu();
  
  Serial << " input=" << sINPUT[input] << endl;
  
  sendMsg( 1, EN_POWER );
}



/*-------------------- Power::checkSerialSend() ---------------------------
 * 
 * check and action serial input, spacebar toggles mute
*/

void Power::checkSerialSend()
{
  if( Serial.available() > 0 ) 
  {
    if( Serial.read() == ' ' )  /* spacebar toggle mute */
    {
      muteAlarm = ! muteAlarm;
    }
  }  
}



/*-------------------------- Power::alarm() ---------------------------------
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
  else
    delay( duration_ms );
}



/*-------------------------- Power::isUnderVolt() ----------------------
 * 
 * If blue LED is not lit, the 12 V line failed or PolyFuse tripped
 * 12 V feeds onto Blue LED, Vf ~ 2.8 V. 
 * ADC ref is 1.1 V. If blue < 1.09 V (1022) then underVolt condition.
 * 
 * return true if under Volt and false if 12 V is OK
*/
bool Power::isUnderVolt()
{
  return ( analogRead( PINBLUE ) < 1022 ); 
}



/*------------------------------ Power::isOverAmp() -------------------------
 * 
 * read Volts on sense resistor to measure Amps.
 * 
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
  Serial << " " << ( amps * AMPCALIBRATE ) << "mA" << endl;
}


/*----------------------------- Power::test() ------------------------------
 * 
 * If over current then alarm and reduce duty cycles.
 * If under Volt (poly fuse?) then just alarm.
*/

void Power::test()
{
  byte shiftR = 1;               /* shiftR  1 is divide by 2           */

  while( isOverAmp() == true )      /* are we over Amps?                  */
  {
    stopISR = true;                 /* stop ISR as this code overides DC  */

    sendMsg( 1, EN_OVERAMP );
      
    digitalWrite( PINLEDRED, HIGH );
    Serial << F("! OverAmp");
    printAmps();

    if( shiftR < 8 )
    {
      overrideDC( shiftR++ );       /* cut DC by half     */          
    }
    alarm( 250 );                   /* constant beep      */
  }

  if( shiftR > 1 )
  {
    Serial << F(". DC recover") << endl;
    restoreDC();
    sendMsg( 0, EN_OVERAMP );
  }
  
  stopISR = false;  /* restart ISR processing LED channels */

  while( isUnderVolt() == true )
  {
    sendMsg( 1, EN_UNDERVOLT );
    
    digitalWrite( PINLEDRED, HIGH );
    Serial << F("! UnderVolt") << endl;

    while( isUnderVolt() == true )
    {
      alarm( 250 );  /*  beep pause beep pause beep...  */
      delay( 250 );
    }
    sendMsg( 0, EN_UNDERVOLT );
  }
  
  muteAlarm = false;
  
  digitalWrite( PINLEDRED, LOW );
}



/*----------------------------- Power::overrideDC() --------------------------
 * 
 * In case of fault, set channel duty cycle to lower value
 * 
*/

void Power::overrideDC( byte shiftR )
{
  for( byte ch = 0; ch < CHANQTY; ch++ )
  {
    byte newDC = var[ch].dcCur >> shiftR;     /* divide DC down       */
    
    analogWrite( PWMPIN[ch], GAMMA8[newDC] );
    
    Serial << " ch" << ch << '=' << newDC;
  }
  Serial << endl;
}



/*------------------------------- Power::restoreDC() -------------------------
 * 
 * restore running dc
 * 
*/

void Power::restoreDC()
{
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
  return config.readNV( NVmap[TRAN][0] + chan );
}



/*-------------------------------- GetNV::dly() ----------------------------
 * 
 * return delay NV for chan & index     NV.dly( 0, 0 ) is chan 0 Dly0
 *                                      NV.dly( 8, 1 ) is chan 8 Dly1
*/

byte GetNV::dly( byte chan, bool indx )
{
  return config.readNV( NVmap[DLY][indx] + chan );
}



/*------------------------- GetNV::dc() -----------------------------------
 *
 * return duty cycle NV for chan & index   NV.dc( 0, 0 ) is chan 0 DC0 
 *                                         NV.dc( 9, 1 ) is chan 9 DC1
*/
byte GetNV::dc( byte chan, bool indx )
{
  return config.readNV( NVmap[DC][indx] + chan );
}



/*------------------------- GetNV::mode() -----------------------------------
 *
 * return mode NV for chan                 NV.mode( 0 ) is chan 0 Mode
*/

byte GetNV::mode( byte chan )
{
  return config.readNV( NVmap[MODE][0] + chan );
}



/*------------------------------ eventHandler() -------------------------------
 * 
 * event processing function called from CBUS library when a learned event 
 * is received.
*/

void eventHandler( byte index, CANFrame *msg ) 
{
  byte evVal = config.getEventEVval( index, 1 );
  
  byte opc = msg->data[0];
  
  bool onOff = ( opc == OPC_ASON || opc == OPC_ACON );

  Serial << "< rx Event opc0x" << _HEX(opc) << " on/off=" << onOff 
         << " evVal=" << evVal << ' ';
      
  if( evVal >= SIZE_EVAL )
  {
    Serial << "!evVal unknown";
  }
  else
  {
    if( evVal == EVAL_MODE_DAYNIGHT )
    {
      Serial << sINPUT[onOff];
      inputChanged = ( input != (INPUT_t) onOff );
      if( input != (INPUT_t) onOff )
      {
        input = (INPUT_t) onOff;
        inputChanged = true;        /* trigger for loop() to change phase */
        Serial << " set";
      }
      else
      {
        Serial << " unchanged";
      }
    }
    else /* all the evVal of TESTCH? and TESTOFF */
    { 
      testDC = ( onOff ? 1 : 254 ); 
      testChan = evVal;
      Serial << sEVAL[evVal] ;
    }
  }
  Serial << endl << endl;
}



/*--------------------------------- frameHandler() ----------------------------
 * 
 * user-defined CBUS frame processing function
 * called from the CBUS library for every CAN frame received
*/

void frameHandler( CANFrame *msg ) 
{
  if( debug == true )
  {
    Serial << "< rx frame id" << (msg->id & 0x7f) << " len" << msg->len;
    for( byte i = 0; i < msg->len; i++ ) 
    {
      Serial << " 0x" << _HEX( msg->data[i] );
    }
    Serial << endl;
  }
}



/*---------------------------- sendMsg() -------------------------------------
 * 
 * Send CBUS message ACON / ACOF and EN
*/

void sendMsg( bool onOff, uint16_t en )
{
  CANFrame msg;   /* create and initialise a message object */
  
  msg.len = 5;    /* size of the data bytes  */
  
  msg.data[0] = ( onOff ? OPC_ACON : OPC_ACOF );

  uint16_t nn = config.nodeNum;
  
  msg.data[1] = highByte( nn );
  msg.data[2] = lowByte( nn );

  msg.data[3] = highByte( en );
  msg.data[4] = lowByte( en );
  
  if( ! CBUS.sendMessage( &msg ) )
  {
    Serial << endl << F("! failed");
  }
  Serial << "> tx " << (onOff ? "ACON" : "ACOF") << " n" << nn << " e" << en;

  if( en >= SIZE_EN )
  {
    Serial << F("!unknown") << endl; 
  }
  else
  {
    Serial << ' ' << sEN[en] << endl;
  }
}



/*-------------------------------- SerMon::about() -----------------------------
 *
 * print titles and code version
*/

void SerMon::about() 
{
  Serial << endl << endl << sMODTITLE << "  v" << VERSIONMAJOR << VERSIONMINOR
    << ( VERSIONBETA ? " beta " : " " ) << VERSIONBETA
    << endl << sCBUSTITLE << endl << endl;
}



/*----------------------------------- SerMon::state() ---------------------------
 *
 * print module state - SLiM/FLiM, CANID & NN
*/

void SerMon::state()
{
  Serial << endl << F(" CBUS ") << ( config.FLiM ? "FLiM" : "SLiM" )
     << F(" CANID=") << config.CANID << F(" NN=") << config.nodeNum << endl;
}



/*----------------------------- SerMon::variables() -----------------------------
 *
 * print Variables
*/

void SerMon::variables()
{
  static const byte BUFSIZE { 11 };
  
  static char buf[BUFSIZE];
  
  byte nv = 1;

  Serial << endl << F(" NVs   ch  1        2        3        4        5      ")
                 << F("  6        7        8        9       10") << endl;

  for( byte r = 0; r < 6; r++)
  {
    Serial << sNVGROUP[r];
      
    for( byte ch = 0; ch < CHANQTY; ch++ )
    {
      byte v = config.readNV( nv++ );
      
      if( r == 5 )
      {
        if( v >= MODEQTY )
        {
          Serial << F(" !unknown mode");
        }
        else 
        {
          Serial << sMODE[v]; 
        }
      }
      else /* r = 0 to 4 */
      {
        snprintf( buf, BUFSIZE, "   %03d   ", v );
        Serial << buf;
      }
    }
    Serial << endl;
  }
}



/*----------------------------- resetNodeVariables() --------------------------
*
* reset Node Variables to factory test config.
*
*/

void resetNodeVariables()
{
  Serial << "reset factory test config";
  
  for( byte nv = 0; nv < NVQTY; nv++ )
  {
    config.writeNV( nv + 1, DEFAULTNV[nv] );
  }
  setupChannels();
  
  Serial << F(" Now, in the FCU, do Node>ReadNVs and save.") << endl;
}



/*--------------------------- SerMon::storedEvents() --------------------------
 *
 * print Stored Events
*/

void SerMon::storedEvents()
{
  Serial << endl <<" Stored Events"<< endl;

  byte count = 0;
  
  for( byte j = 0; j < config.EE_MAX_EVENTS; j++ )  /* for each stored event */
  {
    if( config.getEvTableEntry( j ) != 0 ) 
    {
      count++;

      byte v[5];

      for( byte e = 0; e < ( config.EE_NUM_EVS + 4 ); e++ ) /* for each event*/
      {
        v[e] = config.readEEPROM(
              config.EE_EVENTS_START + ( j * config.EE_BYTES_PER_EVENT ) + e );
      };
      
      Serial << " ev" << j
          << " nn=" << ( v[0] * 256 ) + v[1]
          << " en=" << ( v[2] * 256 ) + v[3]
          << " eVal=" << config.getEventEVval( j, 1 ) << endl;
    }
  }
  Serial << ' ' << count << '/' << config.EE_MAX_EVENTS << endl;
}



/*--------------------------------- SerMon::freeSRAM() ---------------------------
 *
 * print free SRAM
*/

void SerMon::freeSRAM()
{
  Serial << endl << F(" freeSRAM=") << config.freeSRAM() << endl;
}



/*------------------------------ SerMon::menu() ---------------------------
 *
 * print menu for serial commands
*/

void SerMon::menu()
{
  Serial << endl <<
  F(" menu: c=bus, e=ev, v=var, m=mem, *=Amp, t=tx, +-=debug, RN=RstNv, ?=about")
   << endl << endl;
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
      case '*':
        PWR.printAmps();
        break;
        
      case 'R':           // Reset Node variables. R followed by N
        while( ! Serial.available() ){};
        if( Serial.read() == 'N' )
        {
          resetNodeVariables();
        }
        break;

      case 'c':           // CBUS node status
        SerialMon.state();
        break;

      case 'e':           // stored learned Event data table
        SerialMon.storedEvents();
        break;

      case 'v':           // Variables table
        SerialMon.variables();
        break;
 
      case 'y':           // reset CAN bus & CBUS processing      
        CBUS.reset();
        break;

      case 'm':           // free SRAM memory      
        SerialMon.freeSRAM();
        break;

      case 's':
        chanState();
        break;

      case 't':           // send test message ACOF
        sendMsg( 0, EN_TESTMSG );
        break;
      case 'T':           // send test message ACON
        sendMsg( 1, EN_TESTMSG );
        break;
        
      case '-':           // debug off
        debug = false;
        break;
      case '+':           // debug on = verbose Serial output
        debug = true;
        break;
        
      case '?':           // about
        SerialMon.about();
        break;
        
      default:            // unknown command
        break;
    }
  }
}



/*------------------------------ setPins() ------------------------------------
 * 
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
  SetPINTP_D30;                       /* macro fast pin method. See PIN.h     */
  ClrPINTP_D30;
  
  pinMode( PINTP_D31, OUTPUT );       /*Test Point: measure ISR duration      */
  SetPINTP_D31;
  ClrPINTP_D31;                       /* macro fast pin method. See PIN.h     */

  for( byte c = 0; c < CHANQTY; c++ ) /* loop all PWM channels                */
  {
    pinMode( PWMPIN[c], OUTPUT );
  }

  digitalWrite( PINLEDRED, 0 );       /* all LEDs off, after 0.25s (bar blue) */
  digitalWrite( PINLEDYEL, 0 ); 
  digitalWrite( PINLEDGRN, 0 );
  digitalWrite( LED_BUILTIN, 0 );
}



/*
 ------------------ CANlights_1.ino  EoF ------------------------------------
*/
