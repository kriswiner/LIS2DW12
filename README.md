# LIS2DW12
Arduino sketches demonstrating operation of ST's LIS2DW12 low-power accelerometer

![image](https://user-images.githubusercontent.com/6698410/134245151-12ffb94b-2f41-4cb0-af91-9ddacd7b5a6e.jpg)

**The DataReady sketch** shows how to configure the accelerometer power modes, full scale range, ODR and bandwidth. The sketch performs the self test, calculates the offset biases, and initializes the sensor for data measurement. The sketch sets up the data ready interrupt and reads the accelerometer data when the interrupt is triggered. Finally, the sketch outputs the data to the serial monitor when the RTC alarm is triggered (every second). The main loop is entirely interrupt based such that when no interrupt is being handled the STM32L42 MCU waits in a low-power (SLEEP or STOP) state. 

**The SleepWake sketch** configures the two interrupts for wake and sleep detection. INT1 is chosen to be the wake interrupt and is configured to trigger when motion on one or any axis exceeds a user-programmable threshold (62 mg in the sketch) for a user programmable time (0.16s in the sketch). When the wake interrupt is triggered, the sketch checks the status register to determine what kind of wake event has occurred. The sketch uses the 32-sample FIFO to capture the acceleration data prior to and during the wake event for further diagnosis. If the accelerometer motion falls below the wake threshold for a user-programmable time (20s in the sketch), the sensor enters sleep mode.  In sleep mode, the sensor runs at a fixed 12.5 Hz in the low-power state selected by the user. When entering sleep mode, an interrupt is triggered on INT2 and the FIFO is restarted.

Using this Sleep/Wake toggling the application can run at a high rate when in motion and automatically return to a low-power sleep state when the motion has stopped. Similarly, the application can sit in a low-power (as low as 1 uA) state while waiting for motion and then analyze the waking motion captured by the FIFO.

**In LPMode2** (14-bit data) I measured ~6.3 uA when the LIS2DW12 is awake at 25 Hz, ~4.8 uA when it is asleep at 12.5 Hz and the Ladybug uses ~2.2 uA in STOP mode. So the LIS2DW12 is using ~4.1 uA (@3V3) when awake at 25 Hz compared to the 3 uA (@1V8) expected from Table 10 of the data sheet and ~2.6 uA (@3V3) when asleep at 12.5 Hz instead of the 1.6 uA (@1V8) from the data sheet. So there is a ~1 uA greater current usage than expected.

**In LPMode1** (12-bit data) I measured 4.8 uA with FIFO active and 4.5 uA without (so there is a cost to use the FIFO) in wake mode at 25 Hz and 3.8 uA in sleep mode (3.7 uA w/o FIFO). So in the most likely (LPMode1) mode for wake-on-motion/sleep-on-no-motion functionality, the LIS2DW12 uses ~1.5 uA (@3V3) in sleep mode. Compare with the 850 nA (@1V8) used in sleep mode used by the [BMA400](https://wiki.pine64.org/images/c/cc/Bst-bma400-ds000.pdf) and the 0.3 uA (@3V3) I measured for the [MC3635](https://github.com/kriswiner/MC3635) in "sniff" mode. The sizes of all three sensors are the same, their cost is not. BMA400 (if you can find it, now out of production) is ~$3, [LIS2DW12](https://www.lcsc.com/product-detail/Motion-Sensors-Accelerometers_STMicroelectronics-LIS2DW12TR_C189624.html) is ~$1, and the MC3635 is ~$1.50 each for quantity 100.


**The Tap_OrientationDetect** sketch configures the sensor for embedded single- and double-tap detection and for 6D orientation change detection functions. The data ready interrupt is routed to INT2 so that the accelerometer data can be output every second on RTC alarm just like in the DataReady sketch. INT1 has routed to it the single- and double- tap and 6D-orientation-change interrupts. Upon triggering of INT1 the type of interrupt, *i.e.,* orientation change, single or doubletap, *etc* is printed to the serial monitor as well as the source of the interrupt (single tap on positive y-axis, orientation change to XH orientation, *etc.* The latency between taps, the acceleration threshold and timing of the tap "shock" and quiet time after the tap "shock" are all configurable allowing the user to tailor the interaction to the specific application.

LIS2DW12 [data sheet](https://www.st.com/resource/en/datasheet/lis2dw12.pdf).

LIS2DW12 [application note](https://www.st.com/resource/en/application_note/dm00401877-lis2dw12-alwayson-3d-accelerometer-stmicroelectronics.pdf).

Sketches were developed using an STM32L432 Ladybug development board available on [Tindie](https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/).

LIS2DW12 breakout board available in the OSH Park shared space [here](https://oshpark.com/shared_projects/GVUiTiyH).
