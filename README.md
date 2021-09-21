# LIS2DW12
Arduino sketches demonstrating operation of ST's LIS2DW12 low-power accelerometer

**The DataReady sketch** shows how to configure the accelerometer power modes, full scale range, ODR and bandwidth. The sketch perfroms the self test, calculates the offset biases, and initiates the sensor for data measurement. The sketch sets up the data ready interrupt and reads te data when the interrupt is triggered. Finally, the sketch outputs the data to the serial monitor when the RTC alarm is triggered (every second). The main loop is entirely interrupt based such that when no interrupt is being handled the STM32L42 MCU waits in a low-power (STOP) state. Power measurements TBD.

**The SleepWake sketch** configures the two interrupts for wake and sleep. Int1 is chosen to be the wake interrupt and is configured to trigger when a motion threshod on any axis exceeds a user programmable threshold (62 mg in the sketch) for a user programmable time (0.16s in the sketch). When the wake interrupt is triggered, the sketch checks the status register to determine what kind of wake event has occurred. The sketch uses the 32-sample FIFO to capture the acceleration data prior to and during the wake event for further diagnosis. If the accelerometer motion falls below the wake threshold for a user-programmable time (20 s in the sketch), the sensor enters sleep mode.  In sleep mode, the sensor runs at a fixed 12.5 Hz in the low-power state selected by the user. When entering sleep mode, an interrupt is triggered on INT2 and the FIFO is restarted. 

Using this Sleep/Wake toggling the application can run at a high rate when in motion and automatically return to a low-power sleep state when the motion has stopped. Similarly, the application can sit in a low-power (as low as 1 uA) state while waiting for motion and then analyze the waking motion captured by the FIFO.
