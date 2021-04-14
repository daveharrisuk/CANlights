/*file: DEFAULTNV.h       This is include file for CANlights.ino sketch
 *----------------------------------------------------------------------------
 * 
 * default NV array for factory reset of module NVs
 *
 *  CANlights control module for 10 channels
 * 
 * Author: Dave Harris. Andover, UK. © Dave Harris 2021
 * 
*/
#ifndef DEFAULTNV_H__  /* include guard */
#define DEFAULTNV_H__

  
  const byte DEFAULTNV[NVQTY]
  {
  /*   ch1/2 = DayNight   Night blue
   *   ch3 = Night010     night blue
   *   ch4 = Dusk         red
   *   ch5 = DuskDawn     red
   *   ch6 = Dawn         red
   *   ch7 = Night010     white
   *   ch8 = Day010       day white
   *   ch9/10 = DayNight  Day white

                 ch1  ch2  ch3  ch4  ch5  ch6  ch7  ch8  ch9  ch10  */
  /* Transit */    6,   6,   1,   3,   3,   3,   0,   1,   6,   6, 
  /*  Delay0 */    0,   0,   4,   0,   0,   0,   2,   3,   0,   0, 
  /*  Delay1 */    0,   0,   4,   0,   0,   0,   1,   4,   0,   0, 
  /*     DC0 */  250, 251,  32,  33,  34,  35,  36,  37,  38,  39, 
  /*     DC1 */   30,  31, 252, 253, 224, 225, 236, 237, 208, 209, 
  /*    Mode */    0,   0,   4,   1,   3,   2,   4,   5,   0,   0  
  };
  

#endif /* DEFAULTNV_H__ 
 --------------------------------------- EoF ------------------------------
 */
 
