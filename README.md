# Exo-sensing
Firmware development for exo-sensing project
Data processing for exo-sensing project

## ExoSensing - Exoskeleton Comfort Evaluation Through Smart Sensing

*Dissertation project*

- _Author:_ Daniel Proaño G.
- _Supervisors:_ Hugo Silva, Marcelino Santos

## Dev:

- 5 FSR sensors 
    - Acquisition under interruption
    - Oversampling at 200 sps
    - IIR filter
    - Decimation factor 5
    - Moving average, minimum, and maximum. Window size = 40
    - Cubic regression model **needs to be adjusted with new board**

- 1 NTC sensor
    - Acquisition under interruption
    - Oversampling at 200 sps
    - Decimation factor 5
    - Moving average, minimum, and maximum. Window size = 40
    - Model adjusted conversion **needs calibration and comparison with a standard instrument**
        - Equation-based model working good enough

- Debug
    - GPIO 2 and 3 used to measure processing times
    - DAC availability to test the filters and analog acquisition. Uses SPI

- Humidity and Temperature Sensor (Integrated) HS3003
    - Reading sensor and printing on serial
    - Polling data on BLE

- Pressure Sensor (Integrated) LPS22HB
    - Reading sensor and printing on serial
    -  Polling data on BLE

- BLE Peripheral
    - Triggered by RTC every second
    - Created services for force and body temperature sensors, comfort, and environmental data
    - Working under Poll mode due to facility in library usage

- UART Communication
    - Triggered by RTC every second
    - Sends the most relevant data
    - Can be disabled from sys.h

- Fuzzy Inference
    - Based on eFLL library
    - Correlate physiological data with pshychological information for prediction
    - Expanded simulation available in processing


Development on Arduino IDE 2.0
Arduino NANO 33 Sense Rev 2 development board
Based on nRF52840