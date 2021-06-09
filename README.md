# CANlights

CBUS layout lighting control module for 10 LED strings.

- 10x 12 Volt PWM channels driving LED strings. (PCB gives total 2.6A) 

- CBUS event is day/night trigger (or local nightSw).     evValue 1 = 0
- CBUS events to put channels into a test mode.           evValue 1 = 1-10
- CBUS event ends channel test mode.                      evValue 1 = 11
- CBUS event used to shutdown all channels.               evValue 1 = 12
- FCU / CBUS library deals with all event teaching. This code expects it.
- Each channel has the following 6 variables (from CBUS NVs)...
   - Transition time. Range 0 (<0.25s), 1 second to 255 seconds (4:15)
   - Delay0 and Delay1 time.  Range 0 seconds to 255 seconds.
   - DC0 and DC1 duty cycles. Range 0 (fully off) to 255 (fully on).
   - Mode = DAYNIGHT, DAWN, DUSK, DUSKDAWN, NIGHT010, DAY010, ALWAY010
     (byte enum 0 to 6).
- A trigger starts a 'Phase', which changes a channels DC (PWM Duty Cycle).
  -  The trigger varies on a channels Mode.
  -  'Phase0' = after 'Delay0' secs, the DC transitions to DC0 & steady after.
  -  'Phase1' = after 'Delay1' secs, the DC transitions to DC1 & steady after.
  -  The primary trigger is the NightSw input (CBUS event or local switch).
  -  A secondary trigger can be a Phase completing.
  -  DAYNIGHT : NightSw On, starts Phase1 and NightSw off, triggers a Phase0.
  -  DAWN     : NightSw Off starts Phase1 and when done, triggers a Phase0.
  -  DUSK     : NightSw On starts Phase1 followed by Phase0.
  -  DUSKDAWN : NightSw change starts Phase1 followed by Phase0
  -  NIGHT010 : While NightSw on, Phase1 triggers Phase0 which starts Phase1...
  -  DAY010   : While NightSw off, Phase1 triggers Phase0 which starts Phase1..
  -  ALWAYS010: From power on, Phase1 triggers Phase0 which starts Phase1...
 no external trigger used.

- Over current protection. Gives red LED, Audio Warning Device & CBUS event.
- Diagnostic messages via Arduino Serial Monitor. 
- LED indicators...
   - Red    = Alarm (see Arduino Serial Monitor for detail).
   - Yellow = FLiM mode.
   - Green  = pre FLiM mode (aka SLiM) or CAN activity indicator in FLiM mode.
   - Orange = on is Night time, off is Day time.
   - Blue   = 12v indicator.
   - MCU green = MCU 5v on.
   - MCU red   = DC transitions are active.

