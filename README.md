# Channel-15
ECE 362 Final Project

To Do List:
1. An ability to use I2C to gather temperature, humidity, and pressure readings from a BME280. 

2. An ability to use SPI to display current weather readings, past trends (graph over time), and future predictions (graph over time) on a TFT LCD. 

3. An ability to use SPI + SD to log weather readings and pre-load/read weather predictions to/from a SD card, which can then be read on a computer or radio broadcasted to a base station. 

4. An ability to use PWM to play audio signals on an 8 Ohm speaker for certain weather events. 

 

Description 

A two-piece weather station, comprised of a base station and satellite measurement station, that will track temperature, humidity, and pressure. One Proton board will power the base and one Proton board will power the satellite station. The base will have a TFT LCD,  8 Ohm speaker, keypad, and SD card. It will communicate with the satellite station via a Adafruit RFM69HCW Transceiver Radio Breakout. The data collected on the SD card can be graphed to view trends over time and compared against past data. The SD card will also hold future measurement predictions preloaded from the National Weather Service, which can then be cross referenced with what the station is reading or has read on the LCD. The keypad will be used to interact with the LCD, such as to graph the temperature trends across different time periods. It can also be used to setup alerts/audible notifications that will play on the 8 Ohm speaker for different weather events, such as when reaching a certain temperature, humidity, or pressure. The satellite measurement station will be composed of a BME280 (used to gather temperature, humidity, and pressure) along with a Adafruit RFM69HCW Transceiver Radio Breakout to communicate with the base station. If time allows, the project can be extended even further by expanding the types of sensors on the satellite station. This might include, but is not limited to, LDR(s) (to track daylight), air quality sensor(s) (to track eCO2 and TVOC), and UV sensor(s) (to track ultra-violet light). 