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


/*----------------------------- class Power ------------------------------------
 *
 *  processing over Amp or under Volt conditions
*/

class Power
{
  public:
 
    void alarm( uint16_t );         /* Sound AWD for milli second.            */
    bool isUnderVolt();             /* 12 V line failed / PolyFuse tripped    */
    bool isOverAmp();               /* read Volts on sense resistor as Amps   */
    void printAmps();               /* print, approx, measured mA to Serial   */
    void test();                    /* if fault alarm & reduce duty cycles    */

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
   
    byte tran( byte );  	      /* return transition NV for chan      */
    byte dly( byte, bool );     /* return delay NV for chan & index   */
    byte dc( byte, bool );      /* return DutyCycle NV for chan/index */
    byte mode( byte );          /* return mode NV for chan            */
  
    enum NV_t : byte { NV_TRAN = 0, NV_DLY, NV_DC, NV_MODE, NV_MAPSIZE };

  private :

    static constexpr byte NVmap[NV_MAPSIZE][2] 
    {
      { 1,  1 },  /* Transition Seconds for phase 0 or 1 are the same */
      { 11, 21 }, /* Delay Seconds for phase 0 and 1                  */
      { 31, 41 }, /* DutyCylce for phase 0 and 1                      */
      { 51, 51 }  /* Modes for phase 0 or 1 are the same              */
    };

}; /* end of class GetNV */



/*-------------------------------- class SerMon ------------------------------
 *
 *  print various data to Serial
*/

class SerMon
{
  public :
  
    void about();
    void cbusState();
    void variables();
    void storedEvents();
    void processKeyBoard();
    
}; /* end of class SerMon */



/*-----------------------------function declararioins -------------------------
*/

void setup();

void loop();

void processChannels();     /* foreground 1ms timer1 process     */

void startNewPhase();

void eventHandler( byte, CANFrame * );

void frameHandler( CANFrame * ); 

void setupChannels();

void sendCBUSmessage( bool, uint16_t );

void resetNodeVariables();

extern void setPins();




#endif /* CANLIGHTS_H__ 
 --------------------------------------- EoF ------------------------------
*/
