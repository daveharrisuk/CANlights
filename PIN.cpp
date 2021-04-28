/*file: PIN.cpp
 *----------------------------------------------------------------------------
 * 
 * Pin setup 
 *
 *  CANlights control module for 10 channels
 * 
 * Author: Dave Harris. Andover, UK. © Dave Harris 2021
*/
#include "PIN.h"


/*------------------------------ setPins() ------------------------------------
 * setup() calls this
*/

void setPins()
{
  pinMode( PINLEDRED, OUTPUT );       /* Red LED: Error/Alarm                 */
  pinMode( PINLEDYEL, OUTPUT );       /* Yellow LED: FLiM state LED           */
  pinMode( PINLEDGRN, OUTPUT );       /* Green LED:  SLiM state LED           */
  pinMode( PINLEDORA, OUTPUT );       /* Orange LED: Night state LED          */
  pinMode( LED_BUILTIN, OUTPUT );     /* Red LED on MEGA, DC Transit active   */
  
  digitalWrite( PINLEDRED, 1 );       /* all LED on for 0.25 Sec self test    */
  digitalWrite( PINLEDYEL, 1 ); 
  digitalWrite( PINLEDGRN, 1 );
  digitalWrite( PINLEDORA, 1 );
  digitalWrite( LED_BUILTIN, 1 );

  pinMode( PINBLUE,   INPUT );        /* ADC in: Vf sensed on Power blue LED  */
  pinMode( PINSENSE,  INPUT );        /* ADC in: Amp sense resistor           */

  pinMode( PINSW0,    INPUT_PULLUP ); /* Tactile push switch                  */
  pinMode( PINENCPHA, INPUT_PULLUP ); /* Rotary encoder phase A               */
  pinMode( PINENCPHB, INPUT_PULLUP ); /* Rotary encoder phase B               */
  pinMode( PINENCSW,  INPUT_PULLUP ); /* Rotary encoder switch via            */

  pinMode( PINAWDSIG, OUTPUT );       /* AudioWarningDevice: Piezo buzzer     */
  
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
  digitalWrite( PINLEDORA, 0 );
  digitalWrite( LED_BUILTIN, 0 );
}



/*-------------------------------PIN.cpp EoF ------------------------------*/
