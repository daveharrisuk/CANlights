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

//#include "CANlights.cpp"


unsigned char sCBUSNAME[8] { "LIGHTS " };   /* 7 chars, trailing space pad */

const uint8_t CBUSMODULEID { 99 };
  

extern const uint8_t QTY_CHAN;  /* qty of PWM channels for LED chans. * PIN.h */

const uint8_t QTY_NV { 60 };    /* qty of Node Variables                      */
  
const uint8_t QTY_EVENT { 13 }; /* total events required                      */


enum DC_t : uint8_t
{
  DC_MIN = 0,               /* Duty Cycle ranges  0                       */
  DC_MID = 127,             /*                    to                      */
  DC_MAX = 255              /*                    255                     */
};


enum EN_t : uint8_t   /* Event Numbers(EN) for ACON/ACOF message sending  */
{
  EN_NIGHTSW = 0,           /* (extInput) On event= Night, Off event= Day */
  EN_POWERON = 1,           /* On event = Power On, Off event = na        */
  EN_ALARM   = 2,           /* On event = Alarm on, Off event = No Alarm  */
  EN_TESTMSG = 3            /* On event NA, Off event NA                  */
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


enum NIGHTSW_t : bool   /* module input codes. for nightSw global var */
{
  NIGHTSW_DAY   = 0,
  NIGHTSW_NIGHT = 1
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


enum PWM_t : uint8_t 
{
  PWM_OFF = 0,
  PWM_RESTORE = 1
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

  private:
  
    uint16_t amps;                  /* latest Amp reading                     */

    /* Rsense = 0R050,   2.0 A = 0.100 V on PINSENSE, into ADC
     * ADC range 0-1023, Vref = 1.10 V.  ADC reads 0.001074 V per unit. 
     * Max reading is 0.10 / 0.001074 = 93
    */
    static const uint16_t MAXAMPADCREAD { 93 };  /* 2.0 A  ADC threshold      */
    static const uint16_t AMPCALIBRATE { 22 };   /* multiplier for true Amps  */
  
}; /* end of class Power */



/*-------------------------------- class ADRNV ---------------------------------
 *
 *  return address of NV  for a channel and param type
*/

class ADRNV
{
  public:
  
    ADRNV() { };
    ~ADRNV() { };

    uint8_t tran( uint8_t chan )           /* return transition NV for chan */
    {
      return ( NVmap[NV_TRAN][0] + chan );
    }
    
    uint8_t dly( uint8_t chan, bool indx ) /* return delay NV for chan & idx */
    {
      return ( NVmap[NV_DLY][indx] + chan ); 
    }

    uint8_t dc( uint8_t chan, bool indx )   /* return DC NV for chan/index  */
    {
      return ( NVmap[NV_DC][indx] + chan ); 
    }
    
    uint8_t mode( uint8_t chan)             /* return mode NV addr for chan */
    {
      return ( NVmap[NV_MODE][0] + chan );
    }

  private:
    enum NV_t : uint8_t 
    {
      NV_TRAN = 0, NV_DLY, NV_DC, NV_MODE, NV_MAPSIZE
    };
    static constexpr uint8_t NVmap[NV_MAPSIZE][2] 
    {
      { 1,  1 },    /* Transition Secs are same for phase 0 or 1          */
      { 11, 21 },   /* Delay Seconds for phase 0 and 1                    */
      { 31, 41 },   /* DutyCylce for phase 0 and 1                        */
      { 51, 51 }    /* Modes ... for phase 0 or 1 are the same            */
    };
}; /* end of class ADRNV */



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
void processChan( uint8_t );          /* process single chan, from foreground */

void startNewPhase();                 /* called from loop or setUpChannels    */

void eventHandler( uint8_t, CANFrame*); /* function registerd to CBUS library */
void frameHandler( CANFrame* );       /* function registerd to CBUS library   */ 

void sendEvent( ONOFF_t, uint16_t );  /* format/send ASON/ASOF event with EN  */

void setDefaultNVs();                 /* preset NVs. Called by SerMon command */

void setAllPWM( PWM_t );              /* set all PWM DC to 0 or reset         */

void setup();                         /* called on power on or reset          */
void setupCBUS();                     /* called by setup()                    */
void setupChannels();                 /* called by setup()                    */

extern void setupPins();              /* called by setup()  in PIN.cpp        */


/*------------------------- data definitions --------------------------------*/


const char sEVAL[QTY_EVAL][9] /* fixed width strings */
{
  "NightSw ",             /* switch day or night          */
  "TestCh1 ",             /* Chan 1 DC = 1 or 254         */
  "TestCh2 ",             /* Chan 2 DC = 1 or 254         */
  "TestCh3 ",             /* Chan 3 DC = 1 or 254         */
  "TestCh4 ",             /* Chan 4 DC = 1 or 254         */
  "TestCh5 ",             /* Chan 5 DC = 1 or 254         */
  "TestCh6 ",             /* Chan 6 DC = 1 or 254         */
  "TestCh7 ",             /* Chan 7 DC = 1 or 254         */
  "TestCh8 ",             /* Chan 8 DC = 1 or 254         */
  "TestCh9 ",             /* Chan 9 DC = 1 or 254         */
  "TestCh10",             /* Chan 10 DC = 1 or 254        */
  "TestEnd ",             /* Test mode end                */
  "ShutDown"              /* all channels duty cycle = 0  */
};

const char sEN[QTY_EN][9]       /* fixed width strings  */
{
  " NightSw",
  " PowerOn",
  " Alarm  ",
  " TestMsg"
};

const char sINPUT[QTY_INPUT][6]  /* fixed width strings */
{
  "Day  ",
  "Night" 
};

const char sSTATE[OTY_STATE][5]   /* fixed width strings */
{
  "Stdy",
  "Tran",
  "Dly " 
};

const char sMODE[QTY_MODE][9]  /* fixed width strings */
{
  "DayNight",
  "Dusk    ",
  "Dawn    ",
  "DuskDawn",
  "Night010",
  "Day010  "
};



struct var_t   /* channel data structure. Populated at boot time from NVs    */
{
  uint8_t   secTrans;     /* Transition seconds                      0 - 255 */
  
  uint8_t   secDelay[2];  /* Delay seconds                           0 - 255 */

  MODE_t    mode;         /* chan mode                                 0 - 5 */  

  uint8_t   dc[2];        /* DC0 & DC1 phase 0 & phase 1 targets     0 - 255 */
  
  uint8_t   dcCur;        /* current DC value                        0 - 255 */
  
                          /* Derived from NV Transition seconds              */
  uint16_t  msPerStep;    /* ms between duty cycle inc/dec           0 - 65k */

     /* following are status/trackers */
                          
  uint32_t  msCount;      /* millis() counter for step or delay   0 - 255000 */
                      
  uint8_t   secCount;     /* seconds counter                         0 - 255 */
  
  STATE_t   state;        /* Stdy, Tran, Dlay trackers                 0 - 2 */
  
  bool      phase;        /* phase 0 or 1                              0 - 1 */                        
};


/* Channel Modes...
 *  
 *  MODE_DAYNIGHT 
 *  --------
 *  Input change to NIGHT,
 *   after secDelay[1], dc transitions from current dc to dc[1].
 *   Stay steady until,
 *  Input change to DAY,
 *   after secDelay[0], dc transitions from current dc to dc[0].
 *  
 *  DUSKDAWN
 *  -----------
 *  Input change to NIGHT,
 *   after secDelay[1], dc transitions from current dc to dc[1],
 *   after secDelay[0], dc transitions from current dc to dc[0].
 *  Stay steady until,
 *  Input change to DAY... as per nightSw change to 1.
 *   
 *  NIGHT010
 *  ---------
 *   Input change to NIGHT,
 *    after secDelay[1], transitions from current dc to dc[1],
 *    after secDelay[0], transitions from dc[1] to dc[0],
 *    after secDelay[1], transitions from dc[0] to dc[1],
 *    ... repeats until nightSw change to 0.
 *   Input change to DAY,
 *    after secDelay[0], transitions from current dc to dc[0].  
 *    
 *  DAY010
 *  -------
 *   Input change to DAY,
 *    after secDelay[1], transitions from current dc to dc[1],
 *    after secDelay[0], transitions from dc[1] to dc[0],
 *    after secDelay[1], transitions from dc[0] to dc[1],
 *    ... repeats until nightSw change to 0.
 *   Input change to NIGHT,
 *    after secDelay[0], transitions from current dc to dc[0].  
 *    
 *  [0] is day, [1] is night
 *  dc[0] and dc[1] can be configured as any value you want.
*/


#endif /* CANLIGHTS_H__ 
 --------------------------------------- EoF ------------------------------
*/
