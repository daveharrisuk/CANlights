# CANlights

CBUS layout lighting control module for 10 LED strings.

 - 10x 12 Volt PWM channels driving LED strings. 

 - CBUS event used as day/night trigger (NightSw) or local nightSw.
 - CBUS events to put channels into a test mode (TestCh)
 - CBUS event used to shutdown all channels (ShutDown)
 
 - Each channel has the following 6 variables (from CBUS NVs)...
    - Transintion seconds. Range 0-255.
    - Delay0 and Delay1 seconds. For Phase0 or Phase1. Range 0-255.
    - DC0 and DC1 duty cycles. For Phase0 or Phase1. Range 0 (off) through to 255 (on).
    - Mode = DAYNIGHT DAWN DUSK DUSKDAWN NIGHT010 DAY010 ALWAY010.
 - A trigger causes a channels DC (PWM duty cycle) to change.
    - (A trigger depends on the channels Mode)
    - Phase 0 = after Delay0 secs, the DC transitions to DC0 & steady after.
    - Phase 1 = after Delay1 secs, the DC transitions to DC1 & steady after.
    - The primary triger is the NightSw input (CBUS event or local switch)
    - A secondary trigger can be a Phase completing.
    - DAYNIGHT : NightSw On starts Phase1 and NightSw off starts Phase0
    - DAWN     : NightSw Off starts Phase1 followed by Phase0
    - DUSK     : NightSw On starts Phase1 followed by Phase0
    - DUSKDAWN : NightSw change starts Phase1 followed by Phase0
    - NIGHT010 : While NightSw on, Phase1 triggers Phase0 which starts Phase1..
    - DAY010   : While NightSw off, Phase1 triggers Phase0 which starts Phase1..
    - ALWAYS010: from power on, Phase1 triggers Phase0 which starts Phase1...

 - Over current protection. Audio alarm sounder plus CBUS event.
 - diagnostics via Arduino Serial Monitor. 
 - LEDs
    - Red    = Alarm (see SerialMonitor for detail)
    - Yellow = FLiM mode
    - Green  = pre FliM mode (aka SLiM) or CAN activity indicator in FLiM.
    - Orange = on is Night time, off is Day time
    - Blue   = 12v indicator
 
