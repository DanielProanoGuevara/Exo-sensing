# Exo-sensing
Firmware development for exo-sensing project
Data processing for exo-sensing project

## ExoSensing - Exoskeleton Comfort Evaluation Through Smart Sensing

*Dissertation project*

- _Author:_ Daniel Proaño G.
- _Supervisors:_ Hugo Silva, Marcelino Santos

## Dev:

- 5 FSR sensors 
- - Acquisition under interruption
- - Oversampling at 200 sps
- - IIR filter
- - Decimation factor 5
- - Moving average, minimum, and maximum. Window size = 20
- - Cubic regression model **needs to be adjusted with new board**

- 1 NTC sensor
- - Acquisition under interruption
- - Oversampling at 200 sps
- - Decimation factor 5
- - Moving average, minimum, and maximum. Window size = 20
- - Model adjusted conversion **needs calibration and comparison with a standard instrument**

- UART Communication
- - Triggered by RTC every second
- - Sends the most relevant data

- Debug
- - GPIO 2 and 3 used to measure processing times
- - DAC availability to test the filters and analog acquisition. Uses SPI

Development on Arduino IDE 2.0 Arduino NANO 33 Sense Rev 2 development board Based on nRF52840
