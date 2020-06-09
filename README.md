# nano-33-sense-serial-example
An example program for the Arduino Nano 33 BLE Sense that outputs CSV data for all sensors through UART.

## Features
- Simple example for outputting raw sensor data from the Arduno Nano 33 BLE Sense through Serial.
- Ability to plot data using Serial Plotter.
- Simple macro based configuration for which sensors output data.
- Easy way to check if sensor functionality

## Usage
To use this example the Arduino nRF528x Mbed OS board package must be installed. It also requires the following libraries to be installed:
- [https://github.com/arduino-libraries/Arduino_APDS9960](Arduino_APDS9960) 
- [https://github.com/arduino-libraries/Arduino_HTS221](Arduino_HTS221)
- [https://github.com/arduino-libraries/Arduino_LPS22HB](Arduino_LPS22HB)
- [https://github.com/arduino-libraries/Arduino_LSM9DS1](Arduino_LSM9DS1)

These can all be installed using by using Library Manager and Board Manager. 
This is an example of a serial plotter plot with all sensor data enabled.

![Serial Plotter Example Output](https://dalegi.com/wp-content/uploads/sites/6/2020/06/serialPlotExampleNano33BLESense.png)

## Configuration

A series of macros exist to enable the easy enabling and disabling of sensor data. Setting them to true enables the data output, and setting them to false disables the output. Having them all true creates a pretty meaningless graph, as the scaling will be way off for each sensor, and there will be too much data to view.

```c++
#define SERIAL_PLOT_MP34DT05    (true)
#define SERIAL_PLOT_LSM9DS1     (true)
#define SERIAL_PLOT_APDS9960    (true)
#define SERIAL_PLOT_LPS22HB     (true)
#define SERIAL_PLOT_HTS221      (true)
```
