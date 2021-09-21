/* 9/18/21 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer   
 *  
 *  The LIS2DW12 is an inexpensive (~$1), three-axis, medium-resolution (12- or 14-bit), ultra-low power 
 *  (<1 uA low power mode) accelerometer in a tiny 2 mm x 2 mm LGA12 package with a 192-byte FIFO, 
 *  two multifunction interrupts and widely configurable sample rate (1.6 - 1600 Hz), full range (2 - 16 g), 
 *  low power modes, and interrupt detection behaviors. This accelerometer is nice choice for motion-based 
 *  wake/sleep, tap detection, step counting, and simple orientation estimation.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include <RTC.h>
#include "LIS2DW12.h"

// Ladybug pin assignments
#define myLed       13 // red

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   //  8 characters HH:MM:SS

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

bool SerialDebug = true;

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
 
volatile bool alarmFlag = false;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, Temperature;

//LIS2DW12 definitions
#define LIS2DW12_intPin1   8    // interrupt1 pin definitions, wake-up from STANDBY pin
#define LIS2DW12_intPin2   9    // interrupt2 pin definitions, data ready or sleep interrupt

// Specify sensor parameters //
LPMODE   lpMode = LIS2DW12_LP_MODE_2;      // choices are low power modes 1, 2, 3, or 4
MODE     mode   = LIS2DW12_MODE_HIGH_PERF; // choices are low power, high performance, and one shot modes
//ODR    odr    = LIS2DW12_ODR_12_5_1_6HZ; //  1.6 Hz in lpMode, max is 200 Hz in LpMode
ODR      odr    = LIS2DW12_ODR_400_200Hz;  // choices are 12.5, 25, 50, 100, 200, 400, 800, 1600 Hz, 1.6 Hz in lpMode, and POWERDOWN
FS       fs     = LIS2DW12_FS_2G;          // choices are 2, 4, 8, or 16 g
BW_FILT  bw     = LIS2DW12_BW_FILT_ODR10;  // choices are ODR divided by 2, 4, 10, or 20
bool lowNoise = true;                     // low noise or lowest power

float aRes = 0;         // Sensor data scale in mg/LSB
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3];        // holds accel bias offsets
float stress[3];        // holds results of the self test
uint8_t status = 0, source = 0, wakeSource = 0, FIFOstatus = 0, numFIFOSamples = 0, sixDSource = 0, tapSource = 0;
volatile bool tapSign = false;

// Logic flags to keep track of device states
volatile bool LIS2DW12_dataReady_flag = false;
volatile bool LIS2DW12_tap_orient_flag = false;

LIS2DW12 LIS2DW12(&i2c_0); // instantiate LIS2DW12 class


void setup()
{
  /* Enable USB UART */
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial enabled!");

  // Test the rgb led, active LOW
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);   // toggle red led on
  delay(1000);                 // wait 1 second


  pinMode(LIS2DW12_intPin1, INPUT);  // define LIS2DW12 wake and sleep interrupt pins as L082 inputs
  pinMode(LIS2DW12_intPin2, INPUT);

  /* initialize two wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(1000);

  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect LIS2DW12 at 0x14 and BME280 at 0x77
  delay(1000);
  
  /* Check internal STML082 and battery power configuration */
  VDDA = STM32.getVREF();
  Temperature = STM32.getTemperature();
  
  // Internal STM32L4 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
  Serial.println(" "); 

  // Read the LIS2DW12 Chip ID register, this is a good test of communication
  Serial.println("LIS2DW12 accelerometer...");
  byte LIS2DW12_ID = LIS2DW12.getChipID();  // Read CHIP_ID register for LIS2DW12
  Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
  Serial.println(" ");
  delay(1000); 

  if(LIS2DW12_ID == 0x44) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("LIS2DW12 is online..."); Serial.println(" ");
   
   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);      

   LIS2DW12.selfTest(stress);                                       // perform sensor self test
   Serial.print("x-axis self test = "); Serial.print(stress[0], 1); Serial.println("mg, should be between 70 and 1500 mg");
   Serial.print("y-axis self test = "); Serial.print(stress[1], 1); Serial.println("mg, should be between 70 and 1500 mg");
   Serial.print("z-axis self test = "); Serial.print(stress[2], 1); Serial.println("mg, should be between 70 and 1500 mg");
   delay(1000);                                                     // give some time to read the screen

   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);                                                     

   aRes = 0.000244f * (1 << fs);                                    // scale resolutions per LSB for the sensor at 14-bit data 
   if(lpMode == 0) {aRes = 0.000976f * (1 << fs);}                  // special case of 12-bit data in low power mode 1

   Serial.println("hold flat and motionless for bias calibration");
   delay(5000);
   LIS2DW12.Compensation(fs, odr, mode, lpMode, bw, lowNoise, offset); // quickly estimate offset bias in normal mode
   Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
   Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
   Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg");

   LIS2DW12.init(fs, odr, mode, lpMode, bw, lowNoise);               // Initialize sensor in desired mode for application                     
   delay(1000); // let sensor settle
   }
  else 
  {
   if(LIS2DW12_ID != 0x44) Serial.println(" LIS2DW12 not functioning!");
  }
  
  /* Set the RTC time */
  SetDefaultRTC();
  
  // set alarm to update the RTC periodically
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  RTC.attachInterrupt(alarmMatch);

  attachInterrupt(LIS2DW12_intPin1, myinthandler1, RISING);  // attach 6D orientation detect interrupt for INT1 pin output of LIS2DW12
  attachInterrupt(LIS2DW12_intPin2, myinthandler2, RISING);  // attach data ready            interrupt for INT2 pin output of LIS2DW12 

  LIS2DW12.getStatus(); // read status of interrupts to clear
    
}/* end of setup */

void loop()
{
  /* LIS2DW12 tap/orientation detect*/
  if(LIS2DW12_tap_orient_flag)
  {
     LIS2DW12_tap_orient_flag = false;  // clear the tap-orient flag if tap/orient event
   
   status = LIS2DW12.getAllIntSource(); // clears the embedded function interrupt flags   
   if(status & 0x10) Serial.println("6D orientation change detected!");
   if(status & 0x08) Serial.println("Double Tap Detected!");
   if(status & 0x04) Serial.println("Single Tap Detected!");

   // Get detailed info on sources
   source = LIS2DW12.get6DSource();        // read source of 6D orientation change
      if(source & 0x40) {                  // change in position detected
      if(source & 0x20) Serial.println("ZH over threshold!");
      if(source & 0x10) Serial.println("ZL over threshold!");
      if(source & 0x08) Serial.println("YH over threshold!");
      if(source & 0x04) Serial.println("YL over threshold!");
      if(source & 0x02) Serial.println("XH over threshold!");
      if(source & 0x01) Serial.println("XL over threshold!");
      } 

   source = LIS2DW12.getTapSource();       // read source of tap detection
     if(source & 0x08) {
        if(source & 0x04) Serial.println("Tap detected on positive X-Axis!");
        if(source & 0x02) Serial.println("Tap detected on positive Y-Axis!");
        if(source & 0x01) Serial.println("Tap detected on positive Z-Axis!");
     }
     else {
        if(source & 0x04) Serial.println("Tap detected on negative X-Axis!");
        if(source & 0x02) Serial.println("Tap detected on negative Y-Axis!");
        if(source & 0x01) Serial.println("Tap detected on negative Z-Axis!");
     }

  } /* end of LIS2DW12 tap/orient detect */
  
  
  /* LIS2DW12 data ready detect*/
  if(LIS2DW12_dataReady_flag)
  {
   LIS2DW12_dataReady_flag = false;    // clear the wake flag if wake event
   status = LIS2DW12.getStatus(); // read status of interrupts to clear

   if (status & 0x01) { // if this is a data ready interrupt, read accel data
     LIS2DW12.readAccelData(accelCount); // get 12-bit signed accel data

     // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes - offset[1];   
     az = (float)accelCount[2]*aRes - offset[2]; 
   }
  } /* end of LIS2DW12 data ready interrupt handling*/


 
  /*RTC*/
  if (alarmFlag) { // update RTC output at the alarm
      alarmFlag = false;
    
    if(SerialDebug) {     
    Serial.println(" ");
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.println(" ");
    }

  VDDA = STM32.getVREF();
  Temperature = STM32.getTemperature();
    if(SerialDebug) {
      Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
      Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
      Serial.println(" ");
    }

    tempCount = LIS2DW12.readTempData();  // Read the accel chip temperature adc values
    temperature =  ((float) tempCount) + 25.0f; // 8-bit accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        

  Serial.println("RTC:");
  Day = RTC.getDay();
  Month = RTC.getMonth();
  Year = RTC.getYear();
  Seconds = RTC.getSeconds();
  Minutes = RTC.getMinutes();
  Hours   = RTC.getHours();     
  if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
  Serial.print(":"); 
  if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
  Serial.print(":"); 
  if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

  Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
  Serial.println(" ");
  
    digitalWrite(myLed, HIGH); delay(1);  digitalWrite(myLed, LOW); // toggle red led on
 } // end of RTC alarm section
    
//    STM32.stop();        // Enter STOP mode and wait for an interrupt
    STM32.sleep();        // Enter SLEEP mode and wait for an interrupt
   
}  /* end of loop*/


/* Useful functions */
void myinthandler2()
{
  LIS2DW12_dataReady_flag = true; 
}


void myinthandler1()
{
  LIS2DW12_tap_orient_flag = true;
}


void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()                                                                                 // Function sets the RTC to the FW build date-time...
{
  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];                                                                       // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for(uint8_t i=0; i<3; i++)
  {
    build_mo += Build_mo[i];
  }
  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if(build_date[4] != 32)                                                                            // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48)*10 + build_date[5]  - 48;                                           // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1]  - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4]  - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
