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



#endif /* DATADEF_H__
 --------------------------------------- EoF ------------------------------
*/
