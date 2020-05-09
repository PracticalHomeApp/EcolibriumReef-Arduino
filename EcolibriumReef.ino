//*** EcolibriumReef ***//
//*** Application - Aquarium Control.
//***   Tested on Arduino Nano compatible.
//***   Processor: ATmega328P (Old Bootloader)


//*** Rev 1 ***//
//***   ( TESTED 2020-05-08 )
//*** Initial release


//*** Power Supply ***//
//*** Specifications:
//***   5VDC 2.1A
//*** Connections
//***   Power Supply - Arduino
//***   +5VDC        - VIN
//***   -5VDC        - GND


//*** 4-Channel Relay (control light & fans) ***//
//*** Relay specifications:
//***   SRD-05VDC-SL-C 
//***     SRD: Model
//***     05VDC: Nominal coil voltage or relay activation voltage
//***     S: Sealed type structure.
//***     L: Coil sensitivity 0.36W
//***     C: Contact rating (10A, 250VAC)
//***   These relay board input controls are ACTIVE LOW, which means set them LOW turn them ON.
//***   Each relay draws 72mA (0.36W / 5V = 0.072A).
//***   For complete opto-isolation, remove JD-VCC jumper, connect JD to external 5V & GND to external GND, connect VCC to Arduino +5V
//*** Light specifications:
//***   Aquamana blue & full spectrum led 120VAC, 165W (55x3W) dimmable.
//*** Fan specifications:
//***   Noctua fans: 
//***     NF-A14 FLX
//***     140x140x25mm, 3-pin connector, 12VDC, 0.08A, 1200 RPM
//***     Air flow: 115.5 cu.m/h (67.98 cfm); 101.9 cu.m/h with LNA (59.98 cfm), 88.7 cu.m/h with ULNA (52.20 cfm)
//***     Noise: 19.2 dBA; 16.4 dB with LNA; 13.8 dB with ULNA
//***   JMC/DATECH (Dell) fan:
//***     9232-12HBTL-2
//***     4x4x1.8, 3-pin connector 12VDC, 0.85A, 1100-4000 RPM
//***     Air flow: 82.0 cfm
//***     Noise: 45.0 dBA
//*** Functions:
//***   Control 2 lights & 2 fans.
//*** Connections:
//***   Relays 1 & 2 are connected to 120VAC power source.
//***     Control Aquamana light.
//***   Relays 3 & 4 are connected to 12VDC power source.
//***     3 controls two Noctua fans.
//***     4 controls JMC/DATECH fan.
//***   Relay - Arduino
//***   VCC   - VIN  (from 2.1A power supply)
//***   GND   - GND 
//***   IN1   - pin 12  full spectrum light
//***   IN2   - pin 11  blue light
//***   IN3   - pin 7   Noctua fans (2)
//***   IN4   - pin 6   JMC/DATECH fan 


//*** 4-Channel Relay (control solenoid valves) ***//
//*** Relay specifications:
//***   SRD-05VDC-SL-C 
//***     SRD: Model
//***     05VDC: Nominal coil voltage or relay activation voltage
//***     S: Sealed type structure.
//***     L: Coil sensitivity 0.36W
//***     C: Contact rating (10A, 250VAC)
//***   These relay board input controls are ACTIVE LOW, which means set them LOW turns them ON.
//***   Each relay draws 72mA (0.36W / 5V = 0.072A).
//***   For complete opto-isolation, remove JD-VCC jumper, connect JD to external 5V & GND to external GND, connect VCC to Arduino +5V
//*** Solenoid specifications:
//***   Autotopoff.com
//***   Stainless steel normally closed solenoid valve
//***   120VAC, 115psi, 1/4" NPT port with 1/4" quick-connect fitting, 0.125 orifice, 0.4Cv value.
//*** Functions:
//***   Control 4 solenoid valves.
//*** Connections:
//***   Relay - Arduino
//***   VCC   - VIN (from 2.1A power supply)
//***   GND   - GND 
//***   IN1   - pin 2   Solenoid 1 - open/close
//***   IN2   - pin 3   Solenoid 2 - open/close
//***   IN3   - pin 4   Solenoid 3 - direct flow to ATO
//***   IN4   - pin 5   Solenoid 4 - direct flow drinking bottle


//*** Bluetooth Module ***//
//*** Specifications ***//
//***   HC-05
//***   This module can be a master or a slave
//***   Device MAC address: 20:16:06:15:28:98
//***   HC-05 must be paired with Android device first (passcode: 1234)
//*** Functions:
//***   Communicate with Android device using AltSoftSerial.
//***   AltSoftSerial can simultaneously transmit & receive.
//***     Slower baud rates are recommended (9600 is used).
//***     AltSoftSerial uses pin 8 (TX), pin 9 (RX) & pin 10 (State)
//***     AltSoftSerial uses a 16 bit timer, so pin 10 PWM is disabled, however digitalRead() or digitalWrite() will work normally
//***   Below is the list of byte commands from Android
//***     127 COMMAND_SYNCH
//***     126 COMMAND_ATO_ENABLE
//***     125 COMMAND_ATO_DISABLE
//***     124 COMMAND_ATO_BYPASS_ENABLE
//***     123 COMMAND_ATO_BYPASS_DISABLE
//***     122 COMMAND_SET_ATO_MAX_DURATION
//***     121 COMMAND_SET_ATO_BYPASS_DURATION
//***     120 COMMAND_TEST_MODE_ENABLE
//***     119 COMMAND_TEST_MODE_DISABLE
//***     118 COMMAND_TEST
//***   Each command is followed by 9 bytes: atoMaxDuration, atoBypassDuration, relayStates, year, month, date, hour, minute, second
//***     Relay states is a binary coded byte
//***       Light White On  0000 0001
//***       Light Blue On   0000 0010
//***       Fan 1 On        0000 0100
//***       Fan 2 On        0000 1000
//***       Valve 1 On      0001 0000
//***       Valve 2 On      0010 0000
//***       Valve 3 On      0100 0000
//***       Valve 4 On      1000 0000
//***   Arduino sends 28 bytes to Android every second:
//***     COMMAND_SYNCH (1), ato state (1), atoBypass (1), atoMaxDuration (1), atoBypassDuration (1), ph (3), water temperature (7), air temperature (7), humidity (5), COMMAND_SYNCH (1)
//*** Connections:
//***   HC-05 - Arduino
//***   State - pin 10
//***   RX    - pin 8
//***   TX    - pin 9
//***   GND   - GND
//***   VCC   - 5V (range 3.6V to 6V)
//***   EN    - not used


//*** Temperature Sensor ***//
//*** Specifications:
//***   DS18B20
//***   Sensor address: 0x28, 0xAD, 0xBF, 0x5F, 0x07, 0x00, 0x00, 0x10 (Sparkfun black)
//***   Notes: 
//***     Multiple sensors can be connected in parallel to the same pin.
//***     Each sensor has its own unique address.
//*** Functions:
//***   Detect water temperature & turn cooling fans on/off via 4 channel relay.
//*** Connections:
//***   Look from flat side with leads pointing down
//***   From left to right
//***   DS18B20                       - Arduino
//***   Pin 1 (black)                 - GND
//***   Pin 2 (white data)            - Pin A1
//***   Pin 2 (white data) - 4.7k Ohm - 5v
//***   Pin 3 (red)                   - 5V


//*** Float Switch ***//
//*** Specifications:
//***   Stainless steel
//*** Functions:
//***   Detect water level & trigger auto top-off solenoids on/off.
//*** Connections:
//***   Float Switch - Arduino
//***   Lead1        - Pin A2
//***   Lead2        - GND (use internal pull-up resistor)


//*** Air Temperature & Humidity Sensor ***//
//*** Specifications:
//***   DHT22
//*** Functions:
//***   Detect air temperature & humidity.
//*** Connections (from left to right)
//***   DHT22                       - Arduino
//***   Pin 1 (+)                   - 5V
//***   Pin 2 (data)                - Pin A3
//***   Pin 2 (data) - 10k resistor - 5V
//***   Pin 3                       - not used
//***   Pin 4 (-)                   - GND


//*** Real time clock (RTC) ***//
//*** Specifications:
//***   DS3231RTC 
//***   Uses rechargeable backup battery CR2032.
//*** Functions:
//***   RTC & Arduino clocks are set to Pacific Standard time (PST).
//***   Clock is synchronized everytime Android sends COMMAND_SYNCH.
//***   Light timers are set to Pago Pago time which is 3 hours behind PST.
//*** Connections:
//***   RTC - Arduino
//***   GND - GND
//***   VCC - 5V
//***   SDA - Pin A4
//***   SCL - Pin A5


//*** Liquid Crystal Display (LCD) ***//
//*** Specifications:
//***   LCD20x4 I2C (20 columns x 4 rows)
//***   Module address 0x3F
//*** Functions:
//***   Display
//*** Connections:
//***   LCD - Arduino
//***   GND - GND
//***   VCC - 5V
//***   SDA - Pin A4
//***   SCL - Pin A5


//*** Momentary button ***//
//*** Functions:
//***   Reset
//*** Connections:
//***   Button - Arduino
//***   Lead 1 - Reset
//***   Lead 2 - GND


//*** Header Files ***//
#include <AltSoftSerial.h>
#include <DHT.h>
#include <DS3232RTC.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <TimeLib.h>


//*** Global variables ***//
#define DISABLED  0                          // ato states
#define ENABLED   1
#define OFF       0                          // ato bypass states
#define ON        1


//*** Arduino Pins Configuration ***//
const byte hardwareRx        = 0;                 //    (not used)
const byte hardwareTx        = 1;                 //    (not used)
const byte relayPin[8] = {12,11,7,6,5,4,3,2};     // full spectrum (white) lights, blue lights, fan1, fan2, valve 1, valve 2, valve 3, valve 4 
const byte bluetoothTxPin    = 8;                 // AltSoftSerial
const byte bluetoothRxPin    = 9;                 // AltSoftSerial
const byte bluetoothStatePin = 10;                // AltSoftSerial (read HIGH if Bluetooth devices are connected)
const byte blinkerPin        = 13;                //    (not used)
const byte A0Pin             = A0;                //    (not used)
const byte dsPin             = A1;                // DS18B20 sensor
const byte floatSwitchPin    = A2;
const byte dhtPin            = A3;                // DHT22 sensor
const byte SDAPin            = A4;                // LCD & RTC
const byte SCLPin            = A5;                // LCD & RTC
const byte A6Pin             = A6;                //    (not used)
const byte A7Pin             = A7;                //    (not used)


//*** AltSoftSerial ***//
AltSoftSerial bluetoothSerial;
const byte COMMAND_SYNCH = 127;                   // default byte value: -128 to 127
const byte COMMAND_ATO_ENABLE = 126;
const byte COMMAND_ATO_DISABLE = 125;
const byte COMMAND_ATO_BYPASS_ON = 124;
const byte COMMAND_ATO_BYPASS_OFF = 123;
const byte COMMAND_SET_ATO_MAX_DURATION = 122;
const byte COMMAND_SET_ATO_BYPASS_DURATION = 121;
const byte COMMAND_TEST_MODE_ENABLE = 120;
const byte COMMAND_TEST_MODE_DISABLE = 119;
const byte COMMAND_TEST = 118;


//*** DS3231RTC (Real time clock) ***// 
const time_t timerSetPdt        = 7200;           // at  2:00:00 PST, calculate daylight saving start/end and set Daylight variable
const time_t timerLcdOn         = 25200;          // at  7:00:00 PST, turn on LCD display  
const time_t timerBlueLightOn   = 39600;          // at 11:00:00 PST ( 8:00:00 Pago time), turn on Blue light
const time_t timerWhiteLightOn  = 43200;          // at 12:00:00 PST ( 9:00:00 Pago time), turn on White light
const time_t timerWhiteLightOff = 64800;          // at 18:00:00 PST (15:00:00 Pago time), turn off White light
const time_t timerBlueLightOff  = 68400;          // at 19:00:00 PST (16:00:00 Pago time), turn off BLue light
const time_t timerLcdOff        = 82800;          // at 23:00:00 PST, turn off LCD display
tmElements_t tm;                                  // time elements for temporary calculations
time_t temp_t;                                    // time values for temporary calculations
time_t t;                                         // time in seconds since midnight 1/1/1970
time_t s;                                         // time of day in seconds
time_t previous_s = 0;
time_t daylightSavingBegin, daylightSavingEnd;
int daylightSavingHourOffset = 0;
float rtcFahrenheit = 0;


//*** LCD display ***//
LiquidCrystal_I2C lcd(0x3F, 20, 4);               // LCD address 0x3F, 20 chars, 4 lines
boolean clockBlinkState = false;
boolean atoDisableBlinkState = false;
boolean atoBypassBlinkState = false;


//*** DHT22 Humidity & Temperature Sensor ***//
DHT dht(dhtPin, DHT22);
float dhtFahrenheit = 0;
float dhtHumidity = 0;
float pH = 8.0;


//*** Solenoid Control ***//
byte atoState = ENABLED;
byte savedAtoState = DISABLED;
byte atoMaxDuration = 0;
unsigned long atoTimerStart = 0;
unsigned long atoElapsedMillis = 0;
unsigned long atoMaxDurationMillis = 0;

byte atoBypassState = OFF;
byte atoBypassDuration = 0;
unsigned long atoBypassTimerStart = 0;
unsigned long atoBypassElapsedMillis = 0;
unsigned long atoBypassDurationMillis = 0;


//*** DS18B20 Temperature Sensor ***//
OneWire ds(dsPin);
const byte ds_addr[8] = {0x28,0xAD,0xBF,0x5F,0x07,0x00,0x00,0x10};
byte ds_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
byte ds_data_requested = false;
float dsFahrenheit = 0;


//*** Relays ***//
byte relayStates = 0;                             // current relays states
byte relayStatesChanged = 0;
byte savedRelayStates = 0;
int fanState = 0;                                 // 0 = off; 1 = 1 fan on; 2 = 2 fans on 
byte testMode = DISABLED;


//*** Misc Index ***//
byte i;


//*****************************************************************************************//
//***************************************** Setup *****************************************//
//*****************************************************************************************//

void setup()
{
// Initialize serial
  Serial.begin(9600);

// Initialize AltSoftSerial for HC-05
  bluetoothSerial.begin(9600);
  pinMode(bluetoothStatePin, INPUT);

// Setup to synchronize DS3231 RTC with Arduino internal clock  
  setSyncProvider(RTC.get);                       // clocks synchronize every five minutes by default.

// Setup LCD
  lcd.begin();
  display_static_screen_info();

// Initialize DHT
  dht.begin();

// Setup float switch pin
  pinMode(floatSwitchPin, INPUT_PULLUP);

// Setup relay pins and make sure relays are inactive at reset or power on.
  for (i = 0; i < 8; i++)
  {
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i], HIGH);              // relays are active LOW, HIGH will turn relay off
  }

  t = now();                                      // function now() returns seconds since midnight 1/1/1970.                       
  s = t % 86400;                                  // now() mod [number of seconds per day 86400] = seconds since midnight today.

  set_daylight_saving();
  
  s >= timerLcdOn && s < timerLcdOff ? lcd.backlight() : lcd.noBacklight();
  s >= timerWhiteLightOn && s < timerWhiteLightOff ? relayStates |= B00000001 : relayStates &= B11111110;
  s >= timerBlueLightOn  && s < timerBlueLightOff  ? relayStates |= B00000010 : relayStates &= B11111101;
  relayStatesChanged = B00000011;
  
// read atoMaxDuration & atoBypassDuration stored in EEPROM
  atoMaxDuration    = EEPROM.read(0);
  atoBypassDuration = EEPROM.read(1);
  atoMaxDurationMillis    = atoMaxDuration    * 60000L;
  atoBypassDurationMillis = atoBypassDuration * 60000L;
}


//*****************************************************************************************//
//***************************************** Loop ******************************************//
//*****************************************************************************************//

void loop()
{
  t = now();                                      // function now() returns seconds since midnight 1/1/1970.                       
  s = t % 86400;                                  // now() mod [number of seconds per day 86400] = seconds since midnight today.

  if (s != previous_s)                            // only run one time every second
  {
    switch (s)
    {
      case timerSetPdt:
        set_daylight_saving();
        break;
      case timerLcdOn:
        lcd.backlight();
        break;  
      case timerBlueLightOn:
        relayStates |= B00000010;
        relayStatesChanged = B00000010;
        break;
      case timerWhiteLightOn:
        relayStates |= B00000001;
        relayStatesChanged = B00000001;
        break;
      case timerWhiteLightOff:
        relayStates &= B11111110;
        relayStatesChanged = B00000001;
        break;
      case timerBlueLightOff:
        relayStates &= B11111101;
        relayStatesChanged = B00000010;
        break;
      case timerLcdOff:
        lcd.noBacklight();
        break;  
      default:;
    }
    
    read_bluetooth();                             // 1ms
    read_air_temperature();                       // 3ms
    read_water_temperature();                     // 6ms
    read_float_switch();                          // < 1ms
    bypass_ato();                                 // < 1ms
    relays_control();                             // < 1ms
    write_bluetooth();                            // 1ms

    update_calendar_display();
    update_clock_display();
    update_temperatures_display();
    update_ato_bypass_display();
    update_ato_display();
    update_relay_states_display();

    previous_s = s;
  }
}


//*****************************************************************************************//
//*********************************** Working Functions ***********************************//
//*****************************************************************************************//

void read_air_temperature()
{
// DHT22 temperature & relative humidity

  dhtFahrenheit = dht.readTemperature(true);
  dhtHumidity = dht.readHumidity();

// DS3231RTC temperature

  rtcFahrenheit = (RTC.temperature() / 4) * 9 / 5 + 32;
}  


void read_water_temperature()
{
// DS18B20 temperature 
// Set fan states

  if (!ds_data_requested)                         // only send request if command has not been sent
  {  
    if (OneWire::crc8(ds_addr, 7) != ds_addr[7])  // cyclic redundancy check make sure there is no error (line noise)
    {                                             // CRC is not valid
      return;
    }
    ds.reset();                                   // reset is needed before communicating with a device
    ds.select(ds_addr);                           // chose device by selecting address
    ds.write(0x44, 1);                            // issue command 0x44 to DS18B20 to start an temperature conversion operation
    ds_data_requested = true;                     // request command already sent, turn ds_requested_data flag on  
  }
  else
  {
    if (ds.read())                                // check if data is available
    {
      ds_data_requested = false;                  // already got data, reset ds_data_requested flag 
    
      ds.reset();
      ds.select(ds_addr);                         // when conversion is done, data is copied into Scratchpad registers
      ds.write(0xBE);                             // send command to read "scratchpad"
  
      for(i = 0; i < 9; i++) 
      {                                           // read 9 bytes
        ds_data[i] = ds.read();
      }  

      OneWire::crc8(ds_data, 8);                  // Compute a CRC check on an array of data
  
      int16_t raw = (ds_data[1] << 8) | ds_data[0];   // Convert the data to actual temperature, default is 12 bit
      dsFahrenheit = (float)raw / 16.0 * 1.8 + 32.0;  

      if (dsFahrenheit > 81.5)                    // temperature reaches above 81.5°, turn fan 2 on
      {
        if (fanState < 2)
        {
          relayStates |= B00001000;
          relayStatesChanged = B00001000;
          fanState = 2;
        }
      }
      else if (dsFahrenheit > 81.0)               // temperature reaches above 81.0°, turn fan 1 on only
      {
        if (fanState < 1)
        {
          relayStates |= B00000100;
          relayStatesChanged = B00000100;
          fanState = 1;
        }
      }
      else if (dsFahrenheit < 80.5)               // temperature drops below 80.5°, turn both fans off
      {
        if (fanState > 0)
        {
          relayStates &= B11110011;
          relayStatesChanged = B00001100;
          fanState = 0;
        }
      }
    }
  }
}


void read_float_switch()
{
//*** When float switch reads LOW, open solenoid valves, start valve open timer (debounce not necessary).
//*** Solenoid valves are set to stay open for at least half maximum time set at an estimate flow rate 3 gal per hour.
//*** After minimum time reached, if float switch reads HIGH, reset valve open timer, close solenoid valves.

//*** SAFETY SWITCH.
//*** Maximum valve-open time can be changed by Android app. 
//*** If valves have been open for more than maximum time set but float switch has not kicked off, float switch might be faulty.
//*** In which case, close solenoid valves, reset atoTimerStart, disable auto top-off.
//*** A mannual reset is required using Android app, momentary button or reset button on Arduino.

  if (atoState == ENABLED)
  {
    if (digitalRead(floatSwitchPin) == LOW)        // when water level is low, float switch reads LOW using internal pullup resistor
    {
      if (atoTimerStart == 0)                     // if solenoids have not been open (solenoid timer has not started)
      {
        atoTimerStart = millis();                 // start valve open timer
        relayStates |= B01110000;                 // turn valves on: solenoids 1, 2, 3 (bits 4th, 5th, 6th)
        relayStatesChanged = B01110000;
      }
      else                                        // solenoids are already open (solenoid timer already started)
      {
        atoElapsedMillis = millis() - atoTimerStart;
        if (atoElapsedMillis >= atoMaxDurationMillis)
                                                  //*** SAFETY SWITCH ***//
                                                  //   if solenoid valves have been open longer than maximum set time
                                                  //   close solenoid valves, reset valve open timer and disable auto top-off
        {                                        
          reset_ato();
          atoState = DISABLED;
        }
      }
    }
    else                                          // float switch reads HIGH
    {
      if (atoTimerStart > 0)                      // if solenoid valves are open, make sure they run for at least half the maximum time before shutting them off
      {
        if (millis() - atoTimerStart >= atoMaxDuration * 30000L)
        {                                         // minTime = 1/2 atoMaxDuration = atoMaxDuration * 60 seconds * 1000 milliseconds / 2
          reset_ato();
        }
      }
    }
  }    
}


void reset_ato()
{
  relayStates &= B10001111;                       // turn valves off: solenoids 1, 2, 3 (bits 4th, 5th, 6th)
  relayStatesChanged = B01110000;
  atoTimerStart = 0;                              // reset timers to zero
  atoElapsedMillis = 0;
}


void bypass_ato()
{
//*** Auto top-off state is saved and disabled when ato bypass is running
//*** Auto top-off state is restored when bypass is done
  
  if (atoBypassState == ON)                       // ato bypass is turned on manually
  {
    if (atoBypassTimerStart == 0)                 // atoBypassTimerStart has not started
    {
      atoBypassTimerStart = millis();             // start valve open timer
      relayStates |= B10110000;                   // turn valves on: solenoids 1, 2, 4 (bits 4th, 5th, 7th)
      relayStatesChanged = B10110000;
    }
    else                                          // atoBypassTimerStart has started
    {                                             // calculate elapsed time in minutes
      atoBypassElapsedMillis = millis() - atoBypassTimerStart;
      if (atoBypassElapsedMillis >= atoBypassDurationMillis)  // atoBypassDuration in minutes * 60 seconds * 1000 milliseconds
      {                                           // atoBypass time is up
        reset_ato_bypass();
        atoState = savedAtoState;
        atoBypassState = OFF;
      }
    }  
  }
}


void reset_ato_bypass()
{
  relayStates &= B01001111;                       // turn valves off: solenoids 1, 2, 4 (bits 4th, 5th, 7th)
  relayStatesChanged = B10110000;
  atoBypassTimerStart = 0;                        // reset atoBypassTimerStart
  atoBypassElapsedMillis = 0;                     // reset atoBypassElapsedMillis
}


void relays_control()
{
  for (i = 0; i < 8; i++)
  {
    if (relayStatesChanged & (B00000001 << i))
    {
      digitalWrite(relayPin[i], !(relayStates & (B00000001 << i)));
    }
  }
  relayStatesChanged = B00000000;
}


void read_bluetooth()
{
//*** Check for command coming from Android
//***   Expect 10 bytes in the following order
//***     0th byte - COMMAND
//***     1st byte - atoMaxDuration
//***     2nd byte - atoBypassDuration
//***     3rd byte - switches (8-bit binary coded)
//***        bit 0 - white light on/off
//***        bit 1 - blue light on/off
//***        bit 2 - fan 1 on/off
//***        bit 3 - fan 2 on/off
//***        bit 4 - valve 1 on/off
//***        bit 5 - valve 2 on/off
//***        bit 6 - valve 3 on/off
//***        bit 7 - valve 4 on/off
//***     4th to 9th byte - year, month, date, hour, minute, second 

  if (digitalRead(bluetoothStatePin) == HIGH)
  {
    if (bluetoothSerial.available())
    {
      byte receiveBuffer[16];
      i = 0;
      while (bluetoothSerial.available())
      {
        if (i < 10)                               // read 10 bytes and discard the rest
        {
          receiveBuffer[i++] = (byte) bluetoothSerial.read();   // force cast byte type in case data > 127
        }
        else
        {
          bluetoothSerial.read();
        }
      }

      switch (receiveBuffer[0])
      {
        case COMMAND_SYNCH:
          tm.Year = receiveBuffer[4];             // tmYear = current year - 1970 
          tm.Month = receiveBuffer[5];            // month
          tm.Day = receiveBuffer[6];              // date
          tm.Hour = receiveBuffer[7];             // hour
          tm.Minute = receiveBuffer[8];           // minute
          tm.Second = receiveBuffer[9];           // second
          temp_t = makeTime(tm);
                                                  // always keep Pacific Standard Time (PST) subtract 3600 seconds from Pacific Daylight Time (PDT)
          if (temp_t >= daylightSavingBegin && temp_t < daylightSavingEnd) temp_t -= 3600;
          RTC.set(temp_t);                        // Use the time_t value to ensure correct weekday is set
          setTime(temp_t);
          break;
        case COMMAND_ATO_ENABLE:
          if (atoBypassState == OFF) atoState = ENABLED;    // atoState can only be set if ato bypass is not running
          break;
        case COMMAND_ATO_DISABLE:
          if (atoBypassState == OFF) atoState = DISABLED;   // atoState can only be set if ato bypass is not running
          break;
        case COMMAND_ATO_BYPASS_ON:
          savedAtoState = atoState;
          atoState = DISABLED;
          atoBypassState = ON;
          break;
        case COMMAND_ATO_BYPASS_OFF:
          reset_ato_bypass();
          atoBypassState = OFF;
          atoState = savedAtoState;
          break;
        case COMMAND_SET_ATO_MAX_DURATION:        // duration can only be set if ato and ato bypass are not running
          if (atoTimerStart == 0 && atoBypassState == OFF)
          {
            atoMaxDuration = receiveBuffer[1];
            atoMaxDurationMillis = atoMaxDuration * 60000L;
            EEPROM.write(0, atoMaxDuration);
          }
          break;
        case COMMAND_SET_ATO_BYPASS_DURATION:     // duration an only be set if ato bypass is not running
          if (atoBypassState == OFF)
          {
            atoBypassDuration = receiveBuffer[2];
            atoBypassDurationMillis = atoBypassDuration * 60000L;
            EEPROM.write(1, atoBypassDuration);
          }
          break;
        case COMMAND_TEST_MODE_ENABLE:
          savedAtoState = atoState;
          atoState = DISABLED;
          savedRelayStates = relayStates;
          relayStates = 0;
          relayStatesChanged = savedRelayStates;
          testMode = ENABLED;
          break;
        case COMMAND_TEST_MODE_DISABLE:
          atoState = savedAtoState;
          relayStates = savedRelayStates;
          relayStatesChanged = relayStates;
          testMode = DISABLED;
          break;
        case COMMAND_TEST:
          relayStates = receiveBuffer[3];
          relayStatesChanged = B11111111;
          break;
        default:;
      }
    }
  }
}


void write_bluetooth()
{
  if (digitalRead(bluetoothStatePin) == HIGH)
  {
    // Assemble byte array 28 elements
    byte sendBuffer[32];
    char dtostrfBuffer[8];                        // used by dtostrf which needs an extra character

    sendBuffer[0] = COMMAND_SYNCH;                // signal start of data
      
    sendBuffer[1] = atoState;                     // ato state ENABLED/DISABLED
    sendBuffer[2] = atoBypassState;               // atoBypass state ON/OFF/DONE
    sendBuffer[3] = atoMaxDuration;
    sendBuffer[4] = atoBypassDuration;

    dtostrf(pH, 3, 1, dtostrfBuffer);             // convert pH into char array (3 bytes, 1 decimal place)
    sendBuffer[5] = dtostrfBuffer[0];
    sendBuffer[6] = dtostrfBuffer[1];
    sendBuffer[7] = dtostrfBuffer[2];

    dtostrf(dsFahrenheit, 5, 2, dtostrfBuffer);   // convert water temperatures into char array (5 bytes, 2 decimal places)
    sendBuffer[8] = dtostrfBuffer[0];
    sendBuffer[9] = dtostrfBuffer[1];
    sendBuffer[10] = dtostrfBuffer[2];
    sendBuffer[11] = dtostrfBuffer[3];
    sendBuffer[12] = dtostrfBuffer[4];
    sendBuffer[13] = 176;                         // character '°'
    sendBuffer[14] = 70;                          // character 'F'

    dtostrf(dhtFahrenheit, 5, 2, dtostrfBuffer);  // convert air temperatures into char array (5 bytes, 2 decimal places)
    sendBuffer[15] = dtostrfBuffer[0];
    sendBuffer[16] = dtostrfBuffer[1];
    sendBuffer[17] = dtostrfBuffer[2];
    sendBuffer[18] = dtostrfBuffer[3];
    sendBuffer[19] = dtostrfBuffer[4];
    sendBuffer[20] = 176;                         // character '°'
    sendBuffer[21] = 70;                          // character 'F'

    dtostrf(dhtHumidity, 4, 1, dtostrfBuffer);    // convert humidity into char array (4 bytes, 1 decimal place)
    sendBuffer[22] = dtostrfBuffer[0];
    sendBuffer[23] = dtostrfBuffer[1];
    sendBuffer[24] = dtostrfBuffer[2];
    sendBuffer[25] = dtostrfBuffer[3];
    sendBuffer[26] = 37;                          // character '%'

    sendBuffer[27] = COMMAND_SYNCH;               // signal end of data
    
    for (i = 0; i < 28; i++) bluetoothSerial.write(sendBuffer[i]);            
  }
}


//*****************************************************************************************//
//*********************************** Display Functions ***********************************//
//*****************************************************************************************//

void display_static_screen_info()
{
  lcd_print(6, 0, "-");                           // line 1: date string hyphens
  lcd_print(9, 0, "-");
  lcd_print(12, 1, 'M');                          // line 2
  lcd_print(0, 2, "W:");                          // line 3
  lcd_print(6, 2, "\xDF"" RO:  '");
  lcd_print(0, 3, "A:");                          // line 4
  lcd_print(6, 3, "\xDF""ATO:");
}


void update_calendar_display()
{
  lcd_print(0, 0, dayShortStr(weekday()));
  lcd_print(4, 0, month());
  lcd_print(7, 0, day());
  lcd_print(10, 0, year());
}


void update_clock_display()                       // clock 12-hour format, ajusted for PDT
{
  int displayedHour = hour() + daylightSavingHourOffset; 
  if(displayedHour > 24) displayedHour = 1;       // take care of daylight saving time
  if(displayedHour > 12) displayedHour -= 12;     // take care of 12-hour format
  lcd_print(2, 1, displayedHour);
  lcd_print(5, 1, minute());
  lcd_print(8, 1, second());
  lcd_print(11, 1, (displayedHour >= 12 && displayedHour < 24) ? 'P' : 'A');

  char clockBlinkChar = clockBlinkState ? ':' : ' ';
  lcd_print(4, 1, clockBlinkChar);
  lcd_print(7, 1, clockBlinkChar);
  clockBlinkState = !clockBlinkState;
}


void update_temperatures_display()                // temperatures ad humidity
{
  lcd_print(2, 2, dsFahrenheit, 1);               // water temperature from DS18B20 sensor, 1 decimal places
  lcd_print(2, 3, dhtFahrenheit, 1);              // air temperature from DHT22 sensor, 1 decimal places
}


void update_ato_bypass_display()
{

  if (atoBypassState == OFF)
  {
    lcd_print(11, 2, (int)atoBypassDuration);
  }
  else // (atoBypassState == ON)
  {
    int minutesRemaining = (int)((atoBypassDurationMillis - atoBypassElapsedMillis) / 60000L);
    if (atoBypassBlinkState) lcd_print(11, 2, minutesRemaining + 1);
    else lcd_print(11, 2, "  ");
    atoBypassBlinkState = !atoBypassBlinkState;
  }
}


void update_ato_display()
{
  if(atoState == ENABLED)
  {
    if (atoTimerStart == 0)                       // ato is not running
    {
      lcd_print(11, 3, (int)atoMaxDuration);      //    display max duration
      lcd_print(13, 3, '\'');                     //    apostrophe
    }
    else                                          // ato is running
    {                                         
      if (atoDisableBlinkState)                   //    blink max duration
      {
        lcd_print(11, 3, (int)atoMaxDuration);
        lcd_print(13, 3, '\'');
      }
      else
      {
        lcd_print(11, 3, "   ");
      }
      atoDisableBlinkState = !atoDisableBlinkState;
    }
  }
  else // (atoState == DISABLE)
  {    
    if (atoBypassState || testMode)               // ato is disabled because ato bypass is running or system is in test mode
    {                                             //    display "DIS"
      lcd_print(11, 3, "DIS");
    }
    else // (not atobypassState)                  // ato is disabled because something is wrong with auto top-off mechanism, either ato time out or faulty float switch
    {                                             //    blink "DIS"
      lcd_print(11, 3, atoDisableBlinkState ? "DIS" : "   ");
      atoDisableBlinkState = !atoDisableBlinkState;
    }
  }
}


void update_relay_states_display()
{
  const char* relayStr[] = {"LW","LB","F1","F2","V1","V2","V3","V4"};
  const byte relayCol[] = {15,18,15,18,15,18,15,18};
  const byte relayRow[] = {0,0,1,1,2,2,3,3};
  
  for (i = 0; i < 8; i++)
  {
    if (relayStates & (B00000001 << i))
    {
      lcd_print(relayCol[i], relayRow[i], relayStr[i]);
    }
    else
    {
      lcd_print(relayCol[i], relayRow[i], "**");
    }
  }
}


//*****************************************************************************************//
//********************************** Utilities Functions ************************************//
//*****************************************************************************************//

void lcd_print(byte c, byte r, char ch)           // print a character at column and row
{                                                 //    ch = 65 or 0x41 (letter A)
  lcd.setCursor(c, r);                            //    ch = 'A'
  lcd.write(ch);                                  //    ch = '\'' (escape character begin with a backslash)
}


void lcd_print(byte c, byte r, int num)           // print an integer at column and row
{                                                 //    a leading zero is added if num < 10 
  lcd.setCursor(c, r);
  if (num < 10) lcd.write('0');
  lcd.print(num);
}


void lcd_print(byte c, byte r, float num, int dec)// print a float at column and row
{                                                 //    dec is the number of decimal place(s)
  lcd.setCursor(c, r);
  lcd.print(num, dec);
}


void lcd_print(byte c, byte r, const char* s)     // print a string at column and row
{
  lcd.setCursor(c, r);
  lcd.print(s);
}


void set_daylight_saving()
{
// Calculate daylight saving start/end and set offset variables

  int secondSundayOfMarch[7] = {8, 14, 13, 12, 11, 10, 9};
                                                  // if 1st of March is Sunday, second Sunday is the 8th 
                                                  // if 1st of March is Monday, second Sunday is the 14th
                                                  // if 1st of March is Tuesday, second Sunday is the 13th
                                                  // if 1st of March is Wednesday, second Sunday is the 12th
                                                  // if 1st of March is Thursday, second Sunday is the 11th
                                                  // if 1st of March is Friday, second Sunday is the 10th
                                                  // if 1st of March is Saturday, second Sunday is the 9th
  int firstSundayOfNovember[7] = {1, 7, 6, 5, 4, 3, 2};
                                                  // if 1st of November is Sunday, first Sunday is the 1th
                                                  // if 1st of November is Monday, first Sunday is the 7th
                                                  // if 1st of November is Tuesday, first Sunday is the 6th
                                                  // if 1st of November is Wednesday, first Sunday is the 5th
                                                  // if 1st of November is Thursday, first Sunday is the 4th
                                                  // if 1st of November is Friday, first Sunday is the 3rd
                                                  // if 1st of November is Saturday, first Sunday is the 2nd
  
// Set common parameters
  tm.Year = CalendarYrToTm(year());               // get curent year 
  tm.Hour = 2;
  tm.Minute = 0;
  tm.Second = 0;
  
// Daylight savings starts second Sunday in March
  tm.Month = 3;
  tm.Day = 1;                                     // start searching from the first of the March
  temp_t = makeTime(tm);
  tm.Day = secondSundayOfMarch[weekday(temp_t) - 1];
  daylightSavingBegin = makeTime(tm);
  
// Daylight savings ends first Sunday in November
  tm.Month = 11;
  tm.Day = 1;                                     // start searching from the first of the November
  temp_t = makeTime(tm);
  tm.Day = firstSundayOfNovember[weekday(temp_t) - 1];
  daylightSavingEnd = makeTime(tm);

  if(t >= daylightSavingBegin && t < daylightSavingEnd) daylightSavingHourOffset = 1;
  else daylightSavingHourOffset = 0;
}
