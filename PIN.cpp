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


/*------------------------------ setupPins() ----------------------------------
 * setup() calls this
*/

void setupPins()
{
  pinMode( PINLEDRED, OUTPUT );        /* Red LED: Error/Alarm                */
  pinMode( PINLEDYEL, OUTPUT );        /* Yellow LED: FLiM state LED          */
  pinMode( PINLEDGRN, OUTPUT );        /* Green LED:  Activity/SLiM state LED */
  pinMode( PINLEDORA, OUTPUT );        /* Orange LED: Night state LED         */
  pinMode( LED_BUILTIN, OUTPUT );      /* Red LED on MEGA, DC Transit active  */
  
  digitalWrite( PINLEDRED, 0 );        /* Alarm LED                           */
  digitalWrite( PINLEDYEL, 0 );        /* FLiM LED                            */
  digitalWrite( PINLEDGRN, 1 );        /* No FLiM and CBUS activity           */
  digitalWrite( PINLEDORA, 0 );        /* Night indicator                     */
  digitalWrite( LED_BUILTIN, 0 );      /* LED change active                   */

  pinMode( PINBLUE,   INPUT );         /* ADC in: Vf sensed on Power blue LED */
  pinMode( PINSENSE,  INPUT );         /* ADC in: Amp sense resistor          */
  
  analogReference( INTERNAL1V1 );      /* MEGA unique, 1.075 mV/bit           */

  pinMode( PINNIGHTSW, INPUT_PULLUP);  /* external nightSw switch. Day is low */
  pinMode( PINCAN, INPUT_PULLUP );     /* No CAN link. Active low             */
  pinMode( PINCBUS, INPUT_PULLUP );    /* CBUS mode sw. Active low            */
  
  pinMode( PINENCPHA, INPUT_PULLUP );  /* Rotary encoder phase A              */
  pinMode( PINENCPHB, INPUT_PULLUP );  /* Rotary encoder phase B              */
  pinMode( PINENCSW,  INPUT_PULLUP );  /* Rotary encoder switch via           */

  pinMode( PINAWDSIG, OUTPUT );        /* AudioWarningDevice: Piezo buzzer    */
  
  pinMode( PINTP_D30, OUTPUT );        /*Test Point: Scope measure TBA        */
  PINTP_D30_HIGH;                      /* macro fast pin method. See PIN.h    */
  PINTP_D30_LOW;
  
  pinMode( PINTP_D31, OUTPUT );        /*Test Point: measure ISR duration     */
  PINTP_D31_HIGH;
  PINTP_D31_LOW;                       /* macro fast pin method. See PIN.h    */

  for( uint8_t ch = 0; ch < QTY_CHAN; ch++ )
  {                                    /*   loop all PWM channels             */
    pinMode( PWMPIN[ch], OUTPUT );
    analogWrite( PWMPIN[ch], 0 );
  }
}



/*-------------------------------PIN.cpp EoF ------------------------------*/
