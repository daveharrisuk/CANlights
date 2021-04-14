/*file: DATADEF.h       This is include file for CANlights.ino sketch
 *----------------------------------------------------------------------------
 * 
 * data structure definitions
 *
 * Layout lighting control module for 10 channels
 * Author: Dave Harris. Andover, UK. © Dave Harris 2021
 *
 * This work is licensed under the Creative Commons
 *    Attribution-NonCommercial-ShareAlike 4.0 International License.
 * To view a copy of this license, 
 *  visit http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a  
 *  letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
 *
*/
#ifndef DATADEF_H__  /* include guard */
#define DATADEF_H__

/*------------------------- data definitions -------------------------------*/



enum EVAL_t : byte  /* EV values on stored events */
{
  EVAL_MODE_DAYNIGHT = 0,     /* On event NIGHT, Off event DAY.  Set input global */
  EVAL_TESTCH1 = 1,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH2 = 2,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH3 = 3,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH4 = 4,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH5 = 5,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH6 = 6,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH7 = 7,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH8 = 8,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH9 = 9,      /* On event DC=254, Off event DC=1                  */
  EVAL_TESTCH10 = 10,    /* On event DC=254, Off event DC=1                  */  
  EVAL_TESTOFF = 11      /* On event all UnTest, Off event NA                */
};

const byte SIZE_EVAL { 12 };

const char sEVAL[SIZE_EVAL][9]
{
  "DayNight",               /* fixed width strings */
  "TestCh1 ",
  "TestCh2 ",
  "TestCh3 ",
  "TestCh4 ",
  "TestCh5 ",
  "TestCh6 ",
  "TestCh7 ",
  "TestCh8 ",
  "TestCh9 ",
  "TestCh10",
  "TestOff "
};



/* Event Numbers (EN) for OPC ACON and ACOF message sending */

enum CBUS_EN_t : byte 
{
  EN_TESTMSG   = 0,      /* On event NA, Off event NA                     */
  EN_OVERAMP   = 1,      /* On event = OverAmps, Off event = No Alarm     */
  EN_UNDERVOLT = 2,      /* On event = UnderVolt, Off event = No Alarm    */
  EN_POWER     = 3,      /* On event = Power On, Off event = Power Off    */
  EN_PBPRESS   = 4       /* On event = Pressed, Off event = released      */
};

const byte SIZE_EN { 5 };

const char sEN[SIZE_EN][10] 
{
  " TestMsg ",               /* fixed width strings */
  " OverAmp ",
  "UnderVolt",
  " Power   ",
  " PBpress "
};


/* table sizes */

const byte CHANQTY { 10 };   /* qty of PWM channels used for light strings */

const byte MAXDC { 255 };    /* Duty Cycle ranges 0 to 255                 */

const byte NVQTY { 60 };     /* qty of Node Variables                      */
  
const byte EVENTSQTY { 12 }; /* total events required                      */



/* channel phase codes.  for input global var */

enum PHASE_t : byte { PHASE0 = 0, PHASE1 };
    
const char sPHASE[2][4] { "ph0", "ph1" };           /* fixed width strings */



/* module input codes. for input global var */
  
enum INPUT_t : byte { INPUT_DAY = 0, INPUT_NIGHT };

const char sINPUT[2][6] { "Day  ", "Night" };        /* fixed width strings */



/* channel state codes. For var[] */

enum STATE_t : byte { STEADY = 0, TRANSIT, DELAY };

const char sSTATE[3][5] { "Stdy", "Tran", "Dly " };  /* fixed width strings */



/* channel mode codes. For global var[] */

const byte MODEQTY { 6 };

enum MODE_t : byte
{
  MODE_DAYNIGHT = 0,
  MODE_DUSK     = 1,
  MODE_DAWN     = 2,
  MODE_DUSKDAWN = 3,
  MODE_NIGHT010 = 4,
  MODE_DAY010   = 5
};

const char sMODE[MODEQTY][10]    /* Mode code string    */
{
  "DayNight ",                   /* fixed width strings */
  "  Dusk   ",
  "  Dawn   ",
  "DuskDawn ",
  "Night010 ",
  " Day010  "
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
 *  Input change to DAY... as per input change to 1.
 *   
 *  NIGHT010
 *  ---------
 *   Input change to NIGHT,
 *    after secDelay[1], transitions from current dc to dc[1],
 *    after secDelay[0], transitions from dc[1] to dc[0],
 *    after secDelay[1], transitions from dc[0] to dc[1],
 *    ... repeats until input change to 0.
 *   Input change to DAY,
 *    after secDelay[0], transitions from current dc to dc[0].  
 *    
 *  DAY010
 *  -------
 *   Input change to DAY,
 *    after secDelay[1], transitions from current dc to dc[1],
 *    after secDelay[0], transitions from dc[1] to dc[0],
 *    after secDelay[1], transitions from dc[0] to dc[1],
 *    ... repeats until input change to 0.
 *   Input change to NIGHT,
 *    after secDelay[0], transitions from current dc to dc[0].  
 *    
 *  [0] is day, [1] is night
 *  dc[0] and dc[1] can be configured as any value you want.
*/



/* module NV array groups */

enum NVGROUP_t : byte { NVTRAN = 0, NVDLY0, NVDLY1, NVDC0, NVDC1, NVMODE };

const char sNVGROUP[6][8] 
{
  " Tran s",                /* fixed width strings */
  " Dly0 s",
  " Dly1 s",
  "  DC0  ",
  "  DC1  ",
  " Mode  "
};



struct var_t   /* channel data structure */
{
  byte      dc[2];        /* DC0 and DC1  Phase 0 & phase 1 target   0 - 255 */
  
  byte      dcCur;        /* current DC value                        0 - 255 */
  
  uint32_t  msCount;      /* millis() count for step or delay     0 - 255000 */
                      
  byte      secCount;     /* working reg for seconds count           0 - 255 */
  
  STATE_t   state;        /* "Stdy", "Tran", "Dlay"                    0 - 2 */
  
  bool      phase;        /* phase 0 or 1                              0 - 1 */
  
  uint16_t  msPerStep;    /* ms between duty cycle inc/dec           0 - 65k */
                          /* Derived from NV Transit seconds during setup()  */
                         
  byte      secTrans;     /* Transition secs copied from NV at boot  0 - 255 */
  
  byte      secDelay[2];  /* Delay secs copied from NV at boot time  0 - 255 */

  MODE_t    mode;         /* chan mode copied from NV at boot time     0 - 5 */  
};



const uint32_t MAXmsPERSTEP { 65534 };  /* fix overflow if abs(DC0-DC1) < 4  */



#endif /* DATADEF_H__
 --------------------------------------- EoF ------------------------------
*/
