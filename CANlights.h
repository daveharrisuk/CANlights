/*file: CANlights.h         This is include file for CANlights.ino sketch
 *----------------------------------------------------------------------------
 * 
 *  CANlights - control module for 10 channels
 * 
 * Author: Dave Harris. Andover, UK. © Dave Harris 2021
 *
 * This work is licensed under the Creative Commons
 *    Attribution-NonCommercial-ShareAlike 4.0 International License.
 * To view a copy of this license, 
 *  visit http://creativecommons.org/licenses/by-nc-sa/4.0/ or  
 *  send letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 * 
*/
#ifndef CANLIGHTS_H__  /* include guard */
#define CANLIGHTS_H__

#include "CANlights.cpp"


unsigned char sCBUSNAME[8] { "LIGHTS " };   /* 7 chars, trailing space pad */

const uint8_t CBUSMODULEID { 99 };
  

extern const uint8_t QTY_CHAN;  /* qty of PWM channels for LED chans.   PIN.h */

const uint8_t QTY_NV { 60 };    /* qty of Node Variables                      */
  
const uint8_t QTY_EVENT { 13 }; /* total events required                      */


const uint8_t MINDC { 0 };      /* Duty Cycle ranges 0                        */
const uint8_t MAXDC { 255 };    /*     to 255                                 */



enum EN_t : uint8_t    /* Event Numbers(EN) for ACON/ACOF message sending */
{
  EN_NIGHTSW = 0,             /* (extInput) On event= Night, Off event= Day  */
  EN_POWERON = 1,             /* On event = Power On, Off event = na         */
  EN_ALARM   = 2,             /* On event = Alarm on, Off event = No Alarm   */
  EN_TESTMSG = 3              /* On event NA, Off event NA                   */
};
const uint8_t QTY_EN { 4 };


enum MODE_t : uint8_t    /* channel mode codes. For global var[] */
{
  MODE_DAYNIGHT = 0,
  MODE_DUSK     = 1,
  MODE_DAWN     = 2,
  MODE_DUSKDAWN = 3,
  MODE_NIGHT010 = 4,
  MODE_DAY010   = 5
};
const uint8_t QTY_MODE { 6 };


enum STATE_t : uint8_t  /* channel state codes. For var[] */
{
  STATE_STEADY,
  STATE_TRANSIT,
  STATE_DELAY 
};
const uint8_t OTY_STATE { 3 };


enum ONOFF_t : bool   /* CBUS on or off codes */
{
  ONOFF_OFF = 0,
  ONOFF_ON  = 1
};
const uint8_t OTY_ONOFF{ 2 };


enum INPUT_t : bool   /* module input codes. for input global var */
{
  INPUT_DAY   = 0,
  INPUT_NIGHT = 1
};
const uint8_t QTY_INPUT { 2 };


enum EVAL_t : uint8_t  /* EV values on stored events */
{
  EVAL_NIGHTSW  = 0,     /* On event = NIGHT, Off event = DAY.                */
  EVAL_TESTCH1  = 1,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH2  = 2,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH3  = 3,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH4  = 4,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH5  = 5,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH6  = 6,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH7  = 7,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH8  = 8,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH9  = 9,     /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH10 = 10,    /* On event DC=254, Off event DC=1                   */  
  EVAL_TESTEND  = 11,    /* On event Test End, Off event NA                   */
  EVAL_SHUTDOWN = 12     /* On = All chans DC=0, Off event = normal operation */
};
const uint8_t QTY_EVAL { 13 };


class Lights
{
  public:
  
};

/*----------------------------- class Power ------------------------------------
 *
 *  processing over Amp or under Volt conditions
*/

class Power
{
  public:
 
    void alarm(uint16_t duration_ms); /* Sound AWD for n milli second.        */
    bool isUnderVolt();               /* 12 V PolyFuse tripped                */
    bool isOverAmp();                 /* read Volts on sense resistor as Amps */
    void printAmps();                 /* print, approx, measured mA to Serial */
    void testAmpAndVolt();            /* if fault alarm & reduce duty cycles  */

  private :
  
    uint16_t amps;                  /* latest Amp reading                     */

    /*  Amps calibrate
     * Rsense = 0R050,   2.0 A = 0.100 V on PINSENSE, into ADC
     * ADC range 0-1023, Vref = 1.10 V.  ADC reads 0.001074 V per unit. 
     * Max reading is 0.10 / 0.001074 = 93
    */
    static const uint16_t MAXAMPADCREAD { 93 };  /* 2.0 A  ADC threshold      */
    static const uint16_t AMPCALIBRATE { 22 };   /* multiplier for true Amps  */
  
}; /* end of class Power */



/*-------------------------------- class GetNV ---------------------------------
 *
 *  return a NV address for a channel and param type
*/

class GetNV
{
  public :
   
    uint8_t tran( uint8_t );  	    /* return transition NV for chan      */
    uint8_t dly( uint8_t, bool );   /* return delay NV for chan & index   */
    uint8_t dc( uint8_t, bool );    /* return DutyCycle NV for chan/index */
    uint8_t mode( uint8_t );        /* return mode NV for chan            */
  
    enum NV_t : uint8_t { NV_TRAN = 0, NV_DLY, NV_DC, NV_MODE, NV_MAPSIZE };

  private :

    static constexpr uint8_t NVmap[NV_MAPSIZE][2] 
    {
      { 1,  1 },    /* Transition Secs ... for phase 0 or 1 are the same  */
      { 11, 21 },   /* Delay Seconds for phase 0 and 1                    */
      { 31, 41 },   /* DutyCylce for phase 0 and 1                        */
      { 51, 51 }    /* Modes ... for phase 0 or 1 are the same            */
    };

}; /* end of class GetNV */



/*-------------------------------- class SerMon ------------------------------
 *
 *  print various data to Serial
*/

class SerMon
{
  public :
  
    void about(char boot = ' ');
    void cbusState();
    void variables();
    void storedEvents();
    void processKeyBoard();
    
}; /* end of class SerMon */



/*-----------------------------function declararioins -------------------------
*/

void loop();                          /* bacground process                    */
void processChannels();               /* foreground process 1ms timer1        */
void processChan(uint8_t);            /* process single chan, from foreground */

void startNewPhase();                 /* called from loop or setUpChannels    */

void eventHandler( uint8_t, CANFrame* ); /* function registerd to CBUS library*/
void frameHandler( CANFrame* );       /* function registerd to CBUS library   */ 

void sendEvent( ONOFF_t, uint16_t ); /* format and send CBUS message    */

void setDefaultNVs();                 /* preset NVs. Called by SerMon command */

void turnOffPWMs();                   /* set all PWM to duty cycle zero       */
void restorePWMs();                   /* set all PWM to duty cycle to old val */

void setup();                         /* called on power on or reset          */
void setupCBUS();                     /* called by setup()                    */
void setupChannels();                 /* called by setup()                    */
extern void setupPins();              /* called by setup()  in PIN.cpp        */



#endif /* CANLIGHTS_H__ 
 --------------------------------------- EoF ------------------------------
*/
