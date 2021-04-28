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
  EVAL_DAYNIGHT = 0,     /* On event NIGHT, Off event DAY.  Set input global  */
  EVAL_TESTCH1 = 1,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH2 = 2,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH3 = 3,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH4 = 4,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH5 = 5,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH6 = 6,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH7 = 7,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH8 = 8,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH9 = 9,      /* On event DC=254, Off event DC=1                   */
  EVAL_TESTCH10 = 10,    /* On event DC=254, Off event DC=1                   */  
  EVAL_TESTEND  = 11,    /* On event Test End, Off event NA                   */
  EVAL_SHUTDOWN = 12     /* On = All chans DC=0, Off event = normal operation */
};

const byte SIZE_EVAL { 13 };

const char sEVAL[SIZE_EVAL][9] /* fixed width strings */
{
  "DayNight",             /* switch day or night          */
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




enum CBUS_EN_t : byte  /* Event Numbers(EN) for ACON/ACOF message sending */
{
  EN_TESTMSG   = 0,    /* On event NA, Off event NA                         */
  EN_ALARM     = 1,    /* On event = Alarm, Off event = No Alarm            */
  EN_POWER     = 2,    /* On event = Power On, Off event = Power Off        */
  EN_PBPRESS   = 3     /* On event = Pressed, Off event = released          */
};

const byte SIZE_EN { 5 };

const char sEN[SIZE_EN][10]   /* fixed width strings  */
{                          
  " TestMsg ",                /* test message         */
  " Alarm   ",                /* Alarm                */
  " Power   ",                /* CPU running          */
  " PBpress "                 /* push button pressed  */
};



/* table sizes */

const byte MAXDC { 255 };    /* Duty Cycle ranges 0 to 255                 */


const byte NVQTY { 60 };     /* qty of Node Variables                      */
  
const byte EVENTSQTY { 64 }; /* total events required                      */



enum INPUT_t : bool   /* module input codes. for input global var */
{
  INPUT_DAY = 0, INPUT_NIGHT = 1 
};
const char sINPUT[2][6] { "Day  ", "Night" };        /* fixed width strings */



enum STATE_t : byte  /* channel state codes. For var[] */
{
  STATE_STEADY = 0, STATE_TRANSIT, STATE_DELAY 
};
const char sSTATE[3][5] { "Stdy", "Tran", "Dly " };  /* fixed width strings */



enum MODE_t : byte    /* channel mode codes. For global var[] */
{
  MODE_DAYNIGHT = 0,
  MODE_DUSK,
  MODE_DAWN,
  MODE_DUSKDAWN,
  MODE_NIGHT010,
  MODE_DAY010
};
const byte MODEQTY { 6 };

const char sMODE[MODEQTY][10]    /* Mode code string    */
{
  "DayNight ",                   /* fixed width strings */
  "Dusk     ",
  "Dawn     ",
  "DuskDawn ",
  "Night010 ",
  "Day010   "
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



struct var_t   /* channel data structure. Populated at boot time from NVs    */
{
  byte      dc[2];        /* DC0 & DC1 phase 0 & phase 1 targets     0 - 255 */
  
  byte      dcCur;        /* current DC value                        0 - 255 */
  
  uint32_t  msCount;      /* millis() counter for step or delay   0 - 255000 */
                      
  byte      secCount;     /* working reg for seconds count           0 - 255 */
  
  STATE_t   state;        /* "Stdy", "Tran", "Dlay" live trackers      0 - 2 */
  
  bool      phase;        /* phase 0 or 1                              0 - 1 */
  
  uint16_t  msPerStep;    /* ms between duty cycle inc/dec           0 - 65k */
                          /* Derived from NV Transit seconds                 */
                         
  byte      secTrans;     /* Transition seconds                      0 - 255 */
  
  byte      secDelay[2];  /* Delay seconds                           0 - 255 */

  MODE_t    mode;         /* chan mode                                 0 - 5 */  
};



#endif /* DATADEF_H__
 --------------------------------------- EoF ------------------------------
*/
