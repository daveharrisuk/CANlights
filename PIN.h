/*file: PIN.h     This is include file for CANlights.ino sketch
 *------------------------------------------------------------------------------
 *
 * Lighting Control module for 10 channels
 * 
 * Author: Dave Harris. Andover, UK.    © Dave Harris 2021
 * 
 * IO pin definitions
 * 
 * Target MCU is Arduino compatable 'MEGA 2560 PRO (EMBED)' or Arduino 'MEGA'
 * 
 * Target PCB is CANlights   Rev A
 * 
*/
#ifndef PIN_H__  /* include guard */
#define PIN_H__


/* Define the pins used
 * ---------------------
 *  
 *  
 * MEGA ATmega2560 has...
 *  External interrupt INT pins are 2, 3, 18, 19, 20 and 21. 
 *   2/3 used by timer comparators 3B and 3C. 2 assigned to SPI_INT.
 *   18/19 free, assigned to Encoder. 
 *   20/21 reserved for I2C. 
 *
 *  The choice of pins for PWM is governed by the usage of timers(0-5) and
 *   timer comparators (A, B & C) and their pins.
 * 
 *  Following PWM pins and timer usage...
 *  Pin Timer Usage by fuction/library
 *  --- ----  -------------------------
 *   2  3B    Free, but is an INT pin. Assigned to SPI_INT signal.
 *   3  3C    Free.  Assigned to PWM0.
 *   4  0B    millis, used in this code.
 *   5  3A    Free.  Assigned to PWM1.
 *   6  4A    Free.  Assigned to PWM2.
 *   7  4B    Free.  Assigned to PWM3.
 *   8  4C    Free.  Assigned to PWM4.
 *   9  2B    tone, lib not used.  Assigned to PWM5.
 *  10  2A    tone, lib not used.  Assigned to PWM6.
 *  11  1A    timer1/servo. TimerOne lib used in this code.
 *  12  1B    timer1/servo. TimerOne lib used in this code.
 *  13  0A    millis, used in this code.
 *  44  5C    Free.  Assigned to PWM7.
 *  45  5B    Free.  Assigned to PWM8.
 *  46  5A    Free.  Assigned to PWM9.
 *  
*  
 * Array of PWM pins
*/
const byte PWMPIN[CHANQTY]  { 3, 5, 6, 7, 8, 9, 10, 44, 45, 46 };



/* Alarms and status       */

const byte PINAWDSIG = 37; /* AudioWarningDevice signal          */

const byte PINLEDRED = 36; /* Red LED.    Error                  */

const byte PINLEDGRN = 35; /* Green LED.  CBUS SLiM              */

const byte PINLEDYEL = 34; /* Yellow LED. CBUS FLiM              */

const byte PINLEDORA = 33; /* Orange LED. Night mode             */



/* inputs        */

const byte PINSENSE = A0;   /* All MOSFET via 0R05 for 2.6A max  */

const byte PININPUT = A6;   /* module input via OptoIsolator     */

const byte PINBLUE = A14;   /* PolyFuse sense, Volts on blue LED */

const byte PINSW0 = 32;              // push button switch pin

//const byte PINTACTSW = A12; /* Tactile push button switch        */



/* SPI bus      */

const byte PINSPIMISO = 50;  /* Master In, Slave Out  (HW SPI 50) */
const byte PINSPIMOSI = 51;  /* Master Out, Slave In  (HW SPI 51) */
const byte PINSPISCK  = 52;  /* clock                 (HW SPI 52) */
const byte PINSPI_SS  = 53;  /* Slave Select                      */
const byte PINSPI_INT = 2;   /* Interrupt (must be an INT pin)    */

const unsigned long SPI_FREQ = 8000000UL; /* frequency of SPI clock */



/* Rotary Encoder     */

const byte PINENCPHA = 18;   /* encoder phase A (must be INT pin) */
const byte PINENCPHB = 17;   /* encoder phase B                   */
const byte PINENCSW  = 19;   /* encoder SW (must be INT pin)      */



/* I2C bus            */

const byte PINI2CSCL = 20;   /* I2C clock     (HW I2C pin is 20 ) */
const byte PINI2CSDA = 21;   /* I2C data      (HW I2C pin is 21 ) */



/*  Test Points      */

const byte PINTP_D30 = 30;     /* MEGA MCU pin D30 maps CPU pin PC7 */
                               /* macros to fast digitalWrite       */
                               
#define SetPINTP_D30   PORTC = PORTC | B10000000
#define ClrPINTP_D30   PORTC = PORTC & B01111111


const byte PINTP_D31 = 31;     /* MEGA MCU pin D31 maps CPU pin PC6 */
                               /* macros to fast digitalWrite       */
                               
#define SetPINTP_D31   PORTC = PORTC | B01000000
#define ClrPINTP_D31   PORTC = PORTC & B10111111


#endif /* PIN_H__ 
 ------------------------ PIN.h EoF---------------------------------
*/
