//-----------------------------------------------------------------------------
// LCDnode Home Energy/Environmental Monitoring Node Firmware
// Rev2
// May 1, 2015 <tgmaxx@gmail.com>
// http://opensource.org/licenses/mit-license.php
//
// LCDnode was designed to work with Emoncms  http://openEnergyMonitor.org
// but can be used as a stand-alone environmental monitor.
//
// HW is based on Arduino Pro Mini 3.3V at 8Mhz, with RFM69W, ILI9341-based
// TFT LCD, DHT Temp/Humidity Sensor, BMP180 Pressure Sensor, DS3231
// Real-Time Clock and Rotary Encoder.
//
// Local environmental data, consisting of temperature, humidity and pressure
// is sampled every 15 seconds and displayed on the LCD. The data is sent to 
// a RF2Pi Gateway every 3 minutes. A Node-Red flow on a Raspberry Pi receives
// the data and forwards it to Emoncms for display and archiving.
//
// A Node-Red flow on a Raspberry Pi retrieves power, energy, outside
// temperature and outside humidity every 10 seconds from Emoncms feeds. The
// data is sent via a RF2Pi Gateway to the LCDnode for display on the LCD.
//
// Cost Today is calculated based on Energy Used times typical cost per kWh
// for summer or winter. The calculated cost, displayed on the LCD, is only
// a rough estimate at best.
//
// The data on the LCD display is updated as follows:
//   - Day/date is updated once a day
//   - Time is updated each minute
//   - Local environmental data is updated every 15 seconds
//   - Remote data (outdoor temp & humidity/power/energy) is updated as
//     received (every 10 seconds)
//
// Node Number is 20. Network Group is 210. RF Frequency Band is 433Mhz
//
// Libraries required:
//   https://github.com/adafruit/Adafruit-BMP085-Library
//   https://github.com/adafruit/Adafruit-GFX-Library
//   https://github.com/adafruit/Adafruit_ILI9341
//   https://github.com/adafruit/DHT-sensor-library
//   http://hacks.ayars.org/2011/04/ds3231-real-time-clock.html
//   https://github.com/jcw/jeelib
//   and others as noted (SPI, Wire, crc16)
//
// Thanks to Lady Ada and http://www.adafruit.com/ for libraries
//
// Thanks to Jean-Claude Wippler for Jeelib: http://JeeLabs.org
// Portions of code from Jeelabs.org RF12demo.12
//
// Thanks to Dr. Ayars for DS3231 RTC library and Rotary Encoder code
// http://hacks.ayars.org/
//
//-----------------------------------------------------------------------------
#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <DHT.h>
#include <DS3231.h>
#define RF69_COMPAT 1
#include <JeeLib.h>
#include <SPI.h>
#include <Wire.h>
#include <util/crc16.h>
#include <avr/eeprom.h>

#define SERIAL_BAUD 9600
#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 2                   // bump on other non-trivial changes
#define VERSION "[LCDnode R2]"

// RFM_IRQ is defined in RF12.cpp as pin 2, for RFM69W
#define Rotary_A 3    // Rotary encoder pin A
#define Rotary_B 4    // Rotary encoder pin B
#define LCD_RST 5     // LCD reset pin
#define Rotary_PB 6   // Rotary encoder pushbutton pin
#define DHTPIN 7      // DHT data pin 
#define LCD_CS 8      // LCD Chip Select pin
#define LCD_DC 9      // LCD Data/Command pin
// SPI_SS is defined in RF12.cpp as pin 10, for RFM69W
// SPI_MOSI is defined in RF12.cpp as pin 11, for RFM69W and LCD
// SPI_MISO is defined in RF12.cpp as pin 12, for RFM69W and LCD
// SPI_SCK is defined in RF12.cpp as pin 13, for RFM69W and LCD
#define LCD_BL 14     // LCD Back Light pin

// over-ride the following color definitions in Adafruit_ILI9341.h
#define ILI9341_BLACK   0x000A  // customized color for LCDnode
#define ILI9341_BLUE    0x001A  // customized color for LCDnode
#define ILI9341_RED     0xE000  // customized color for LCDnode
#define ILI9341_GREEN   0x0300  // customized color for LCDnode

Adafruit_ILI9341 lcd = Adafruit_ILI9341(LCD_CS, LCD_DC, LCD_RST);

Adafruit_BMP085 bmp;  // BMP180 Barometric Pressure Sensor

DS3231 Clock;         // DS3231 Real Time Clock
boolean h12 = false;     // 24-hour clock mode
boolean setQues = false; // Set date, time or node question
byte dowP, monthP, dayP, yearP, hourP, minuteP, secondP; // for previous values
static byte year, month, day, dow, hour, minute, second;
static byte tomonth, today, tomin, tosec;  // holds current month, day, minute, second
static int tosend;               // holds minutes counter for sending data to RF2Pi  
byte node, nodeP;                // for setting node ID from rotary encoder

char *months[] = {"Jan","Feb","Mar","Apr","May","Jun",
		  "Jul","Aug","Sep","Oct","Nov","Dec"};

char *days[] = {"Monday","Tuesday","Wednesday","Thursday",
		"Friday","Saturday","Sunday"};

byte offset;          // x offset, for location of day/date line

// #define DHTTYPE DHT11
#define DHTTYPE DHT22
// DHT dht(DHTPIN, DHTTYPE);   // for 16Mhz Pro Mini
DHT dht(DHTPIN, DHTTYPE, 4);   // for 8Mhz Pro Mini

volatile int Rotor = 0;
int rotorP;
boolean OldRotary_PBState = false;
boolean NewRotary_PBState = false;

float temp, humidity;       // Temp and humidity from DHT
float pressure;             // Pressure from BMP180
float energyf;              // kWh from RF2Pi
float costpkwh;             // Cost per kWh
float wcpkwh = 0.120;       // Winter cost per kWh
float scpkwh = 0.146;       // Summer cost per kWh
float costf;                // Calculated energy cost
static float ostempf;       // Outside temperature
static float oshumif;       // Outside humidity

byte hdgx = 12;             // Location of heading, x coordinate
byte hdgy = 84;             // Location of heading, y coordinate
byte hdgdy = 25;            // Heading line-to-line spacing
byte hdgo = 143;            // Heading x offset for location of data
boolean lcdState = true;    // the state of the backlight pin

boolean debug=0;            // Set to 1 for wireless config

typedef struct {            // Data in from RF2Pi
  int power;                // Power Now
  int energy;               // Energy Used Today
  int ostemp;               // Outside Temperature
  int oshumi;               // Outside Humidity
} Payload;
Payload rf2pi;

typedef struct {            // Data out to RF2Pi
  int temp;                 // LCDnode Temperature
  int humidity;             // LCDnode Humidity
  int pressure;             // LCDnode Pressure
  int filler;               // spare
} PayloadOut;
PayloadOut LCDnode;

typedef struct {
  byte nodeId;              // used by rf12_config, offset 0
  byte group;               // used by rf12_config, offset 1
  byte format;              // used by rf12_config, offset 2
  byte hex_output   :2;     // 0 = dec, 1 = hex, 2 = hex+ascii
  byte collect_mode :1;     // 0 = ack, 1 = don't send acks
  byte quiet_mode   :1;     // 0 = show all, 1 = show only valid packets
  byte spare_flags  :4;
  word frequency_offset;    // used by rf12_config, offset 4
  byte pad[RF12_EEPROM_SIZE-8];
  word crc;
} RF12Config;

const char INVALID1[] PROGMEM = "\rInvalid\n";
const char INITFAIL[] PROGMEM = "config save failed\n";

static RF12Config config;
static char cmd;
static word value;
static byte stack[RF12_MAXDATA+4], top, sendLen, dest;
static byte testCounter;

static void printOneChar (char c) {
  Serial.print(c);
}
static void showByte (byte value) {
  Serial.print((word) value);
}

static word calcCrc (const void* ptr, byte len) {
  word crc = ~0;
  for (byte i = 0; i < len; ++i)
      crc = _crc16_update(crc, ((const byte*) ptr)[i]);
  return crc;
}

static void loadConfig () {
  for (byte i = 0; i < sizeof config; ++ i)
      ((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + i);
}

static void saveConfig () {
  config.format = MAJOR_VERSION;
  config.crc = calcCrc(&config, sizeof config - 2);
  eeprom_write_byte(RF12_EEPROM_ADDR, ((byte*) &config)[0]);
  for (byte i = 0; i < sizeof config; ++ i)
      eeprom_write_byte(RF12_EEPROM_ADDR + i, ((byte*) &config)[i]);
  if (rf12_configSilent())
      rf12_configDump();
  else
      showString(INITFAIL);
}

static byte bandToFreq (byte band) {
   return band == 4 ? RF12_433MHZ : band == 8 ? RF12_868MHZ : band == 9 ? RF12_915MHZ : 0;
}

const char helpText1[] PROGMEM =
  "\n"
  "Available commands:\n"
  "  <nn> i     - set node ID (standard node ids are 1..30)\n"
  "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
  "  <nnnn> o   - change frequency offset within the band (default 1600)\n"
  "               96..3903 is the range supported by the RFM12B\n"
  "  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)\n"
  "  t          - broadcast max-size test packet, request ack\n"
  "  ...,<nn> a - send data packet to node <nn>, request ack\n"
  "  ...,<nn> s - send data packet to node <nn>, no ack\n"
  "  <n> q      - set quiet mode (1 = don't report bad packets)\n"
;

static void showString (PGM_P s) {
  for (;;) {
      char c = pgm_read_byte(s++);
      if (c == 0)
          break;
      if (c == '\n')
          printOneChar('\r');
      printOneChar(c);
  }
}

static void showHelp () {
  showString(helpText1);
  showString(PSTR("Current configuration:\n"));
  rf12_configDump();
}

static void handleInput (char c) {
  if ('0' <= c && c <= '9') {
      value = 10 * value + c - '0';
      return;
  }
  if (c == ',') {
      if (top < sizeof stack)
          stack[top++] = value; // truncated to 8 bits
      value = 0;
      return;
  }
  if ('a' <= c && c <= 'z') {
      showString(PSTR("> "));
      for (byte i = 0; i < top; ++i) {
          Serial.print((word) stack[i]);
          printOneChar(',');
      }
      Serial.print(value);
      Serial.println(c);
  }
  if (c == '>') {
      stack[top++] = value;
      rf12_initialize(stack[2], bandToFreq(stack[0]), stack[1],
                      config.frequency_offset);
      rf12_sendNow(stack[3], stack + 4, top - 4);
      rf12_sendWait(2);
      rf12_configSilent();
  } else if (c > ' ') {
      switch (c) {
      case 'i': // set node id
          config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
          saveConfig();
          break;
      case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
          value = bandToFreq(value);
          if (value) {
              config.nodeId = (value << 6) + (config.nodeId & 0x3F);
              config.frequency_offset = 1600;
              saveConfig();
          }
          break;
      case 'o': { // Increment frequency within band
          if ((value > 95) && (value < 3904)) { // supported by RFM12B
              config.frequency_offset = value;
              saveConfig();
          }
          byte freq = 0, band = config.nodeId >> 6;
          switch (band) {
              case RF12_433MHZ: freq = 43; break;
              case RF12_868MHZ: freq = 86; break;
              case RF12_915MHZ: freq = 90; break;
          }
          uint32_t f1 = freq * 100000L + band * 25L * config.frequency_offset;
          Serial.print((word) (f1 / 10000));
          printOneChar('.');
          word f2 = f1 % 10000;
          printOneChar('0' + f2 / 1000);
          printOneChar('0' + (f2 / 100) % 10);
          printOneChar('0' + (f2 / 10) % 10);
          printOneChar('0' + f2 % 10);
          Serial.println(" MHz");
          break;
      }
      case 'g': // set network group
          config.group = value;
          saveConfig();
          break;
      case 't': // broadcast a maximum size test packet, request an ack
          cmd = 'a';
          sendLen = RF12_MAXDATA;
          dest = 0;
          for (byte i = 0; i < RF12_MAXDATA; ++i)
              stack[i] = i + testCounter;
          showString(PSTR("test "));
          showByte(testCounter); // first byte in test buffer
          ++testCounter;
          break;
      case 'a': // send packet to node ID N, request an ack
      case 's': // send packet to node ID N, no ack
          cmd = c;
          sendLen = top;
          dest = value;
          break;
      case 'q': // turn quiet mode on or off (don't report bad packets)
          config.quiet_mode = value;
          saveConfig();
          break;
      default:
          showHelp();
      }
  }
  value = top = 0;
}

void ReadDHT(void) {
  humidity = dht.readHumidity();
  if (humidity <0.0 || humidity >100.0)
      humidity = 0.0;
  LCDnode.humidity = humidity*10;
  temp = dht.readTemperature();
  if ((temp<80.0) && (temp>-40.0)) {  // check if valid reading
    LCDnode.temp = temp*10;           // save for RF transmission
    temp = 1.8*temp + 32;             // convert to Fahreheit for local display
  }
}

void ReadBMP180(void) {
  pressure=bmp.readPressure();
  pressure=pressure/3386.;     // convert from Pascals to Inches Mercury
  LCDnode.pressure = pressure*10;
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.print("0");
  }
  lcd.print(number);
}

void printdate(void) {
  cli();
  lcd.setCursor(4,12);
  lcd.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);
  lcd.setTextSize(2);
  lcd.print("                          "); // clear the line
  if (day<10)
    offset = 6; 
  else
    offset = 0;
  lcd.setCursor(79-(strlen(days[dow-1])*12)/2 + offset, 12);
  lcd.print(days[dow-1]);
  lcd.print(", ");
  lcd.print(months[month-1]);
  lcd.print(" ");
  lcd.print(day);
  lcd.print(", 20");
  print2digits(year);
  sei();
}

void printtime(void) {
  cli();
  lcd.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  lcd.setTextSize(3);
  lcd.setCursor(93, 43);  
  if (hour >0 && hour <10) {
    lcd.print(" ");lcd.print(hour);lcd.print(":");print2digits(minute);lcd.setTextSize(2);lcd.print(" AM");
  }
  else if (hour>9 && hour <12) {
    print2digits(hour);lcd.print(":");print2digits(minute);lcd.setTextSize(2);lcd.print(" AM");
  }
  else if (hour >12 && hour <22) {
    lcd.print(" ");lcd.print(hour-12);lcd.print(":");print2digits(minute);lcd.setTextSize(2);lcd.print(" PM");
  }
  else if (hour >=22 && hour <=24) {
    print2digits(hour-12);lcd.print(":");print2digits(minute);lcd.setTextSize(2);lcd.print(" PM");
  }
  else if (hour==12) {
    print2digits(hour);lcd.print(":");print2digits(minute);lcd.setTextSize(2);lcd.print(" PM");
  }
  else if (hour==0) {
    print2digits(hour+12);lcd.print(":");print2digits(minute);lcd.setTextSize(2);lcd.print(" AM");
  }
  sei();
}
  
void printdata(void) { 
  cli();
  lcd.setTextColor(ILI9341_BLUE,ILI9341_CYAN);
  lcd.setTextSize(2);
  lcd.setCursor(hdgx+hdgo, hdgy);print_float(temp,1);
  lcd.setTextColor(ILI9341_BLACK,ILI9341_CYAN);lcd.print(" / ");
  lcd.setTextColor(ILI9341_RED,ILI9341_CYAN);print_float(ostempf,1);
  lcd.setTextColor(ILI9341_BLUE,ILI9341_CYAN);
  lcd.setCursor(hdgx+hdgo, hdgy+1*hdgdy);print_float(humidity,1);
  lcd.setTextColor(ILI9341_BLACK,ILI9341_CYAN);lcd.print(" / ");
  lcd.setTextColor(ILI9341_RED,ILI9341_CYAN);print_float(oshumif,1);
  lcd.setTextColor(ILI9341_BLUE,ILI9341_CYAN);
  lcd.setCursor(hdgx+hdgo, hdgy+2*hdgdy);print_float(pressure,2);
  lcd.print(" in Hg");
  sei();
}

// Rotary encoder Interrupt Service Routine
// Runs when pin 3 (INT1) changes from high to low
void UpdateRotation() {    
  if (digitalRead(Rotary_B)) {
    Rotor++;
  } else {
    Rotor--;
  }
}

boolean getYorN(int lcdx, int lcdy) {
  setQues = false;
  lcd.setCursor(lcdx,lcdy);
  lcd.print('N');
  rotorP = Rotor;
  while (!Rotary_PBpressed()) {
    if (Rotor != rotorP) {
      rotorP = Rotor;
      setQues = !setQues;
      lcd.setCursor(lcdx,lcdy);
      if (setQues)
        lcd.print('Y');
      else
        lcd.print('N');
    }
  }
  return(setQues);
}

void set_date (void) {
  Clock.getTime(year, month, day, dow, hour, minute, second);
  lcd.setCursor(4,12);
  lcd.print("Date: ");
  lcd.print(days[dow-1]); lcd.print(", ");
  lcd.print(month);
  lcd.print("/");
  lcd.print(day);
  lcd.print("/");
  lcd.print("20");
  lcd.print(year);
  lcd.setCursor(4,34);
  lcd.print("Set the date? (Y/N):");
  if (!getYorN(249,34)) {
    return;
  }
  // Get the day of the week
  lcd.setCursor(4,56);
  lcd.print("Enter the DoW:");
  lcd.setCursor(174,56);
  lcd.print(days[dow-1]);
  dowP = dow; // Save the initial DoW
  Rotor = dow + 16; // The extra value on rotor allows easier turn‐back.
  while (!Rotary_PBpressed()) {
    dow = constrain((Rotor % 8), 1, 7);
    if (dow != dowP) {
      lcd.setCursor(174,56);
      lcd.print("         ");
      lcd.setCursor(174,56);
      lcd.print(days[dow-1]);
      dowP = dow;
    }
  }
  // Get the Month
  lcd.setCursor(4,78);
  lcd.print("Enter the month:");
  lcd.setCursor(202,78);
  lcd.print(month);
  monthP = month; // Save the initial month
  Rotor = month + 26; // The extra value on rotor allows easier turn‐back.
  while (!Rotary_PBpressed()) {
    month = constrain((Rotor % 13), 1, 12);
    if (month != monthP) {
      lcd.setCursor(202,78);
      lcd.print("  ");
      lcd.setCursor(202,78);
      lcd.print(month);
      monthP = month;
    }
  }
  // Get the Day
  lcd.setCursor(4,100);
  lcd.print("Enter the day:");
  lcd.setCursor(174,100);
  lcd.print(day);
  dayP = day; // Save the initial day
  Rotor = day + 64;
  while (!Rotary_PBpressed()) {
    day = constrain((Rotor % 32), 1, 31);
    if (day != dayP) {
      lcd.setCursor(174,100);
      lcd.print("  ");
      lcd.setCursor(174,100);
      lcd.print(day);
      dayP = day;
    }
  }
  // Get the Year
  lcd.setCursor(4,122);
  lcd.print("Enter the year:");
  lcd.setCursor(185,122);
  lcd.print(year);
  yearP = year; // Save the initial year
  Rotor = year + 52;
  while (!Rotary_PBpressed()) {
    year = constrain((Rotor % 26), 15, 25);
    if (year != yearP) {
      lcd.setCursor(185,122);
      lcd.print("  ");
      lcd.setCursor(185,122);
      lcd.print(year);
      yearP = year;
    }
  }
  Clock.setDate(day);
  Clock.setMonth(month);
  Clock.setYear(year);   
  Clock.setDoW(dow); 
  lcd.setCursor(4,166);
  lcd.print("Date has been updated");
  delay(2000);
}

void set_time (void) {
  lcd.fillScreen(ILI9341_BLACK);
  Clock.getTime(year, month, day, dow, hour, minute, second);
  lcd.setCursor(4,12);
  lcd.print("Time: ");
  print2digits(hour);
  lcd.print(":");
  print2digits(minute);
  lcd.print(":");
  print2digits(second);
  lcd.setCursor(4,34);
  lcd.print("Set the clock? (Y/N):");
  if (!getYorN(260,34)) {
    return;
  }
  // Get the Hour
  lcd.setCursor(4,56);
  lcd.print("Enter the hour:");
  lcd.setCursor(185,56);
  lcd.print(hour);
  hourP = hour; // Save the initial hour
  Rotor = hour + 48;
  while (!Rotary_PBpressed()) {
    hour = constrain((Rotor % 24), 0, 23);
    if (hour != hourP) {
      lcd.setCursor(185,56);
      lcd.print("  ");
      lcd.setCursor(185,56);
      lcd.print(hour);
      hourP = hour;
    }
  }
  // Get the Minutes
  lcd.setCursor(4,78);
  lcd.print("Enter the minutes:");
  lcd.setCursor(222,78);
  lcd.print(minute);
  minuteP = minute; // Save the initial minutes
  Rotor = minute + 120;
  while (!Rotary_PBpressed()) {
    minute = constrain((Rotor % 60), 0, 59);
    if (minute != minuteP) {
      lcd.setCursor(222,78);
      lcd.print("  ");
      lcd.setCursor(222,78);
      lcd.print(minute);
      minuteP = minute;
    }
  }
  // Get the Seconds
  lcd.setCursor(4,100);
  lcd.print("Enter the seconds:");
  lcd.setCursor(222,100);
  lcd.print(minute);
  secondP = second; // Save the initial minutes
  Rotor = second + 60;
  while (!Rotary_PBpressed()) {
    second = constrain((Rotor % 60), 0, 59);
    if (second != secondP) {
      lcd.setCursor(222,100);
      lcd.print("  ");
      lcd.setCursor(222,100);
      lcd.print(second);
      secondP = second;
    }
  }
  // set time
  Clock.setSecond(second);
  Clock.setMinute(minute); 
  Clock.setHour(hour); 
  lcd.setCursor(4,144);
  lcd.print("Time has been updated");
  delay(2000);
}

void set_node (void) {
  lcd.fillScreen(ILI9341_BLACK);
  lcd.setCursor(4,12);
  node = config.nodeId & 0x1F;
  lcd.print("The node ID is: ");
  lcd.print(node);
  lcd.setCursor(4,34);
  lcd.print("Set the node ID? (Y/N):");
  if (!getYorN(282,34)) {
    return;
  }
  lcd.setCursor(4,56);
  lcd.print("Enter the node number:");
  lcd.setCursor(268,56);
  lcd.print(node);
  nodeP = node;
  Rotor = node + 62;
  while (!Rotary_PBpressed()) {
    node = constrain((Rotor % 31), 0, 30);
    if (node != nodeP) {
      lcd.setCursor(268,56);
      lcd.print("  ");
      lcd.setCursor(268,56);
      lcd.print(node);
      nodeP = node;
    }
  }
  config.nodeId = (config.nodeId & 0xE0) + (node & 0x1F);
  saveConfig();
  lcd.setCursor(4,100);
  lcd.print("Node ID has been updated");
  delay(2000);
}

void print_float(float f, int num_digits) {
  int f_int;
  int pows_of_ten[4] = {1, 10, 100, 1000};
  int multiplier, whole, fract, d, n;
  multiplier = pows_of_ten[num_digits];
  if (f < 0.0)
  {
      f = -f;
      lcd.print("-");
  }
  whole = (int) f;
  fract = (int) (multiplier * (f - (float)whole));
  lcd.print(whole);
  lcd.print(".");
  for (n=num_digits-1; n>=0; n--) { // print each digit with no leading zero suppression
      d = fract / pows_of_ten[n];
      lcd.print(d);
      fract = fract % pows_of_ten[n];
  }
}

// Rotary encoder PB
// Returns true if button has been pressed
boolean Rotary_PBpressed() {
boolean WasIt; // was it pressed?
NewRotary_PBState = !digitalRead(Rotary_PB);
if ((!OldRotary_PBState) && (NewRotary_PBState)) {
  WasIt = true;
} else {
  WasIt = false;
}
OldRotary_PBState = NewRotary_PBState;
return(WasIt);
}

//****************************************************
// setup
//****************************************************
void setup () {

  Serial.begin(SERIAL_BAUD); // set serial port to 9600, 8N1

  if (rf12_configSilent()) {
     loadConfig();
  } else {
     memset(&config, 0, sizeof config);
     config.nodeId = 0x54;       // 433 MHz, node 20
     config.group = 0xD2;        // default group 210
     config.frequency_offset = 1603;
     config.quiet_mode = true;   // Default flags, quiet on
     saveConfig();
     rf12_configSilent();
  }
    rf12_configDump(); //send a description of the EEPROM settings to the serial port
    if (debug)
        showHelp();

  lcd.begin();   // ILI9341 LCD
  Wire.begin();  // I2C interface
  bmp.begin();   // BMP180 Pressure Sensor

  lcd.setRotation(3);
  lcd.fillScreen(ILI9341_BLACK);
  
  // Configure the switch, button and LCD backlight
  pinMode(Rotary_PB, INPUT);
  pinMode(Rotary_A, INPUT);
  pinMode(Rotary_B, INPUT);
  pinMode(LCD_BL, OUTPUT);
  
  digitalWrite(LCD_BL, lcdState);  // turn on the LCD backlight

  // Attach interrupt 1 (pin 3) to rotary encoder pin A
  attachInterrupt(1, UpdateRotation, FALLING);

  // Go to setup routines if rotary encoder push-button is pressed
  if (Rotary_PBpressed()) {
    lcd.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);
    lcd.setTextSize(2);
    lcd.setCursor(4,12);
    lcd.print("     Wireless LCDnode");
    lcd.setCursor(4,40);
    lcd.print("    Rev 2, May 1, 2015");
    lcd.setCursor(4,68);
    lcd.print("        by tgmaxx");
    delay(5000);   
    lcd.fillScreen(ILI9341_BLACK);  
    set_date();  
    set_time();
    set_node();
  }
 
  lcd.fillScreen(ILI9341_BLACK);  
  lcd.fillRect(8, 79, 303, 75, ILI9341_CYAN);
  lcd.fillRect(8, 157, 303, 75, ILI9341_CYAN);
  lcd.drawRect(0, 0, 319, 239, ILI9341_WHITE); 
  lcd.drawRect(1, 1, 317, 237, ILI9341_WHITE); 
  lcd.drawRect(2, 2, 315, 235, ILI9341_WHITE); 
  Clock.getTime(year, month, day, dow, hour, minute, second);
  tomonth = month;
  today = day; 
  tomin = minute;
  tosec = second;
  if ((tosec + 15) >60) {
    tosec = (tosec + 15) - 60;
  }
  else
    tosec = tosec + 15;
  tosend = 0;
  if (month>5 && month<10)
    costpkwh = scpkwh;
  else
    costpkwh = wcpkwh;
  printdate();
  printtime();
  lcd.setTextColor(ILI9341_BLACK,ILI9341_CYAN);
  lcd.setTextSize(2);
  lcd.setCursor(hdgx, hdgy);
  lcd.print("In/");
  lcd.setTextColor(ILI9341_RED,ILI9341_CYAN);
  lcd.print("Out");
  lcd.setTextColor(ILI9341_BLACK,ILI9341_CYAN);
  lcd.print(" Temp");
  lcd.setCursor(hdgx, hdgy+1*hdgdy);
  lcd.print("In/");
  lcd.setTextColor(ILI9341_RED,ILI9341_CYAN);
  lcd.print("Out");
  lcd.setTextColor(ILI9341_BLACK,ILI9341_CYAN);
  lcd.print(" %RH");
  lcd.setCursor(hdgx, hdgy+2*hdgdy);
  lcd.print("Pressure");
  lcd.setCursor(hdgx, 3+hdgy+3*hdgdy);
  lcd.print("Power Now");
  lcd.setCursor(hdgx, 3+hdgy+4*hdgdy);
  lcd.print("Energy Used");
  lcd.setCursor(hdgx, 3+hdgy+5*hdgdy);
  lcd.print("Cost Today");
  delay(2000);        // DHT warm up
  dht.begin();
  ReadDHT();
  ReadBMP180();
  ostempf = 0.0;
  oshumif = 0.0;
  printdata();
}
//*******************************************************************************
// End setup
//*******************************************************************************

//****************************************************
// loop
//****************************************************
void loop () {
  if (debug)
      handleInput(Serial.read());
  if (rf12_recvDone()) {
      byte n = rf12_len;
      if (rf12_crc == 0) {
          showString(PSTR("OK"));
          if ((rf12_hdr & 0x1F) == 15) {
              cli();
              rf2pi = *(Payload*) rf12_data;
              lcd.setTextColor(ILI9341_BLUE,ILI9341_CYAN);
              lcd.setCursor(hdgx+hdgo, 3+hdgy+3*hdgdy);
              lcd.print(rf2pi.power);lcd.print(" Watts  ");
              lcd.setCursor(hdgx+hdgo, 3+hdgy+4*hdgdy);
              energyf = rf2pi.energy/100.0;
              print_float(energyf,2);lcd.print(" kWh  ");
              lcd.setCursor(hdgx+hdgo, 3+hdgy+5*hdgdy);
              costf = energyf * costpkwh; 
              lcd.setTextColor(ILI9341_GREEN,ILI9341_CYAN);
              print_float(costf,2);lcd.print(" USD  ");
              ostempf = (float)rf2pi.ostemp/10.0;
              oshumif = (float)rf2pi.oshumi/10.0;
              sei();
          }
      }
      else {
         if (config.quiet_mode)
             return;
         showString(PSTR(" ?"));
         if (n > 20) // print at most 20 bytes if crc is wrong
             n = 20;
         }
         if (config.group == 0) {
             showString(PSTR(" G"));
             showByte(rf12_grp);
         }
         printOneChar(' ');
         showByte(rf12_hdr);
         for (byte i = 0; i < n; ++i) {
              printOneChar(' ');
              showByte(rf12_data[i]);
         }
         // display RSSI value after packet data
         showString(PSTR(" ("));
         Serial.print(-(RF69::rssi>>1));
         showString(PSTR(") "));
         Serial.println();
    }

  Clock.getTime(year, month, day, dow, hour, minute, second); 

  if (month != tomonth) {
    tomonth = month;
    if (month>5 && month<10)
      costpkwh = scpkwh;
    else
      costpkwh = wcpkwh;
  }
  if (day != today) {
    today = day;
    printdate();
  }
  if (tomin != minute) {
    tomin = minute;
    printtime();
    tosend++;
    if (tosend >2) {  // send data to emoncms
      tosend = 0;
      for (byte tries = 0; tries < 3; tries++) {
        if (rf12_canSend()) {
          rf12_sendStart(0, &LCDnode, sizeof LCDnode);
          break;
        }
        delay(125);
      }
    }
  }
  if (tosec == second) {
      tosec = second;
      if ((tosec + 15) >60) {
           tosec = (tosec + 15) - 60;
      }
      else
           tosec = tosec + 15;
      ReadDHT();
      ReadBMP180();
      printdata();
  }
  if (Rotary_PBpressed()) {
        lcdState = !lcdState;
        digitalWrite(LCD_BL, lcdState);
  }
}

