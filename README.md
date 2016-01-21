# LCDnode
Arduino Wireless Environmental/Energy ILI9341 LCD Node for Emoncms

LCDnode was designed to work with Emoncms http://openEnergyMonitor.org but can be used as a stand-alone environmental monitor.

HW is based on Arduino Pro Mini 3.3V at 8Mhz, with RFM69W, ILI9341-based TFT LCD, DHT Temp/Humidity Sensor, BMP180 Pressure Sensor, DS3231 Real-Time Clock and Rotary Encoder.

Local environmental data, consisting of temperature, humidity and pressure is sampled every 15 seconds and displayed on the LCD. The data is sent to a RF2Pi Gateway every 3 minutes. A Node-Red flow on a Raspberry Pi receives the data and forwards it to Emoncms for display and archiving.

A Node-Red flow on a Raspberry Pi retrieves power, energy, outside temperature and outside humidity every 10 seconds from Emoncms feeds. The data is sent via a RF2Pi Gateway to the LCDnode for display on the LCD.

Cost Today is calculated based on Energy Used times typical cost per kWh for summer or winter. The calculated cost, displayed on the LCD, is only a rough estimate at best.

A rotary encoder with push-button is provided. The encoder's button can be pushed and held during power-up or reset, to set the RTC date/time and Node ID. During normal operation, pressing the encoder's push-button toggles the LCD backlight On/Off.

The data on the LCD display is updated as follows:

   - Day/date is updated once a day
   - Time is updated each minute
   - Local environmental data is updated every 15 seconds
   - Remote data (outdoor temp & humidity/power/energy) is 	updated as received (every 10 seconds)

Default Node Number is 20. Network Group is 210. RF Frequency Band is 433Mhz

Libraries required:

  https://github.com/adafruit/Adafruit-BMP085-Library
  https://github.com/adafruit/Adafruit-GFX-Library
  https://github.com/adafruit/Adafruit_ILI9341
  https://github.com/adafruit/DHT-sensor-library
  http://hacks.ayars.org/2011/04/ds3231-real-time-clock.html
  https://github.com/jcw/jeelib

  and others as noted (SPI, Wire)
