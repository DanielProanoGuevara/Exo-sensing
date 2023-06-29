#include <Arduino.h>

#include <nrfx.h>
#include <nrf.h>
#include <nrf_timer.h>
#include <nrf_rtc.h>




#include "dsp.h"
#include "sys.h"
#include "setup_peripheral.h"




// Variables
uint8_t decimationCounter = 1;
float32_t FSR1_filt = 0.0f;
uint16_t DAC_o = 0;

// Interrupt flags
volatile bool timerFlag = false;
volatile bool rtcFlag = false;

// Auxiliar variable
uint32_t idx = 0;


// Filter variables
// *** IIR FSR coefficients
float32_t B_FSR[7] = { 0.00034054f, 0.00204323f, 0.00510806f, 0.00681075f, 0.00510806f, 0.00204323f, 0.00034054f };
float32_t A_FSR[7] = { 1.0f, -3.5794348f, 5.65866717f, -4.96541523f, 2.52949491f, -0.70527411f, 0.08375648f };

// *** IIR NTC coefficients
float32_t B_NTC[7] = { 1.41440730E-11f, 8.48644379E-11f, 2.12161095E-10f, 2.82881460E-10f, 2.12161095E-10f, 8.48644379E-11f, 1.41440730E-11f };
float32_t A_NTC[7] = { 1.0f, -5.87861916f, 14.40044053f, -18.81528973f, 13.82942474f, -5.42164649f, 0.88569011f };

float32_t FSR1_x[7] = { 0.0f };  // Input memory buffer
float32_t FSR1_y[7] = { 0.0f };  // Output memory buffer
float32_t FSR2_x[7] = { 0.0f };
float32_t FSR2_y[7] = { 0.0f };
float32_t FSR3_x[7] = { 0.0f };
float32_t FSR3_y[7] = { 0.0f };
float32_t FSR4_x[7] = { 0.0f };
float32_t FSR4_y[7] = { 0.0f };
float32_t FSR5_x[7] = { 0.0f };
float32_t FSR5_y[7] = { 0.0f };
float32_t NTC1_x[7] = { 0.0f };
float32_t NTC1_y[7] = { 0.0f };


// *** MA ring buffer and variables
float32_t average_FSR[5] = { 0.0f };
float32_t min_FSR[5] = { 0.0f };
float32_t max_FSR[5] = { 0.0f };


float32_t MA_ring_buffer_FSR1[M] = { 0.0f };  // FSR1 moving average ring buffer
uint8_t RB_i_FSR1 = 0;                        // FSR1 Ring buffer iterator

float32_t MA_ring_buffer_FSR2[M] = { 0.0f };
uint8_t RB_i_FSR2 = 0;

float32_t MA_ring_buffer_FSR3[M] = { 0.0f };
uint8_t RB_i_FSR3 = 0;

float32_t MA_ring_buffer_FSR4[M] = { 0.0f };
uint8_t RB_i_FSR4 = 0;

float32_t MA_ring_buffer_FSR5[M] = { 0.0f };
uint8_t RB_i_FSR5 = 0;

float32_t MA_ring_buffer_NTC1[M] = { 0.0f };
float32_t average_NTC1 = 0.0f;
float32_t min_NTC1 = 0.0f;
float32_t max_NTC1 = 0.0f;
uint8_t RB_i_NTC1 = 0;


// Raw analog readings~
uint16_t FSR1_raw = 0;
uint16_t FSR2_raw = 0;
uint16_t FSR3_raw = 0;
uint16_t FSR4_raw = 0;
uint16_t FSR5_raw = 0;
uint16_t NTC1_raw = 0;



/******** Auxiliary *********/
void readADC() {
  FSR1_raw = analogRead(A0);
  FSR2_raw = analogRead(A1);
  FSR3_raw = analogRead(A2);
  FSR4_raw = analogRead(A3);
  FSR5_raw = analogRead(A6);
  NTC1_raw = analogRead(A7);
}



void filterIIRAll() {
  IIR(FSR1_raw,
      (float32_t *)&FSR1_x,
      (float32_t *)&FSR1_y,
      A_FSR,
      B_FSR);

  IIR(FSR2_raw, (float32_t *)&FSR2_x, (float32_t *)&FSR2_y, A_FSR, B_FSR);
  IIR(FSR3_raw, (float32_t *)&FSR3_x, (float32_t *)&FSR3_y, A_FSR, B_FSR);
  IIR(FSR4_raw, (float32_t *)&FSR4_x, (float32_t *)&FSR4_y, A_FSR, B_FSR);
  IIR(FSR5_raw, (float32_t *)&FSR5_x, (float32_t *)&FSR5_y, A_FSR, B_FSR);
  //IIR((3.3f * NTC1_raw)/(4096.0f), (float32_t *)&NTC1_x, (float32_t *)&NTC1_y, A_NTC, B_NTC);
}

void filterMAAll() {
  movingAverage((float32_t *)&MA_ring_buffer_FSR1,
                M,
                RB_i_FSR1,
                FSR1_y[0],
                &average_FSR[0],
                &min_FSR[0],
                &max_FSR[0]);

  movingAverage((float32_t *)&MA_ring_buffer_FSR2, M, RB_i_FSR2, FSR2_y[0], &average_FSR[1], &min_FSR[1], &max_FSR[1]);
  movingAverage((float32_t *)&MA_ring_buffer_FSR3, M, RB_i_FSR3, FSR3_y[0], &average_FSR[2], &min_FSR[2], &max_FSR[2]);
  movingAverage((float32_t *)&MA_ring_buffer_FSR4, M, RB_i_FSR4, FSR4_y[0], &average_FSR[3], &min_FSR[3], &max_FSR[3]);
  movingAverage((float32_t *)&MA_ring_buffer_FSR5, M, RB_i_FSR5, FSR5_y[0], &average_FSR[4], &min_FSR[4], &max_FSR[4]);
  movingAverage((float32_t *)&MA_ring_buffer_NTC1, M, RB_i_NTC1, (3.3f * NTC1_raw)/(4096.0f), &average_NTC1, &min_NTC1, &max_NTC1);
}




// Regression constants for the FSR in the form y = ax^3 + bx^2 + cx + d
float32_t cubic_params_FSR[4] = { 4.6029917878732345f, -14.025875655473554f, 26.84085391232573f, -6.390769522330579 };

void updateVars(){
  /********** Local Variables **********/
  float32_t FSR_mean;
  float32_t FSR_min;
  float32_t FSR_max;

  //FSR
  float32_t interfaceingForceMean = 0.0f;
  float32_t interfaceingForceMin = 0.0f;
  float32_t interfaceingForceMax = 0.0f;
  //NTC
  float32_t skinTemperatureMean = 0.0f;
  float32_t skinTemperatureMin = 0.0f;
  float32_t skinTemperatureMax = 0.0f;

  //Environmental values
  float32_t amb_baro_pressure = 0.0f;
  float32_t amb_temperature = 0.0f;
  float32_t amb_humidity = 0.0f;

  /********** Read values **********/
  // FSR
  arm_mean_f32(average_FSR, 5, &FSR_mean);
  arm_min_f32(min_FSR, 5, &FSR_min, &idx);
  arm_max_no_idx_f32(max_FSR, 5, &FSR_max);

  // Voltage to force conversion
  interfaceingForceMean = cubicFit(FSR_mean, cubic_params_FSR);
  interfaceingForceMin = cubicFit(FSR_min, cubic_params_FSR);
  interfaceingForceMax = cubicFit(FSR_max, cubic_params_FSR);


  // NTC
  skinTemperatureMean = vToTemp(average_NTC1);
  skinTemperatureMin = vToTemp(min_NTC1);
  skinTemperatureMax = vToTemp(max_NTC1);

    /***** IMPLEMENT FUZZY PREDICTOR FOR COMFORT *****/

  // Environmental
  amb_temperature = HS300x.readTemperature();
  amb_humidity = HS300x.readHumidity();
  amb_baro_pressure = BARO.readPressure();

  
  // Communication Protocols
#if UART_EN
  Serial.print("Mean_force:");
  Serial.print(interfaceingForceMean);
  Serial.print(",");
  Serial.print("Mean_temperature:");
  Serial.print(skinTemperatureMean);
  Serial.print(",");
  Serial.print("Amb_temp:");
  Serial.print(amb_temperature);
  Serial.print(",");
  Serial.print("Amb_humidity:");
  Serial.print(amb_humidity);
  Serial.print("Amb_baro_press:");
  Serial.println(amb_baro_pressure);
#endif
#if BLE_EN

#endif

}





void setup() {
  // analog configuration
  analogReadResolution(12);
  #if DEBUG_PIN_EN
    // Initializes the debug pins
    pinMode(DEBUG, OUTPUT);
    pinMode(DEBUG2, OUTPUT);
    digitalWrite(DEBUG, HIGH);
    digitalWrite(DEBUG2, HIGH);
  #endif
  // Initializes the timer
  setupTimer();
  // Initializes the rtc
  setupRTC();
  // Initialize integrated sensors
  setupIntegrated();
  // Initialize serial
  Serial.begin(115200);
  #if DEBUG_DAC
    // Initializes SPI for the DAC
    setupDAC();
  #endif

}

void loop() {
  // Run on every ADC acquisition
  if (timerFlag) {
    timerFlag = false;

    // Read all the ports
    readADC();

    //IIR_filter
    filterIIRAll();

    //Decimation
    if (decimationCounter < 5) {
      decimationCounter++;
    } else {
      // Restart decimator counter
      decimationCounter = 1;


      //Moving_average // Compensates the operations
      filterMAAll();

      /***** DEBUG PLAY *****/
      #if DEBUG_DAC
      FSR1_filt = cubicFit(average_FSR[0], cubic_params_FSR);

      DAC_o = FSR1_filt * (4095.0f / 80.0f);
      //DAC_o = FSR1_y[0] * (4095.0f / 3.3f);

      writeDAC(DAC_o);
      #endif
    }
    #if DEBUG_PIN_EN
    digitalWrite(DEBUG, LOW);
    #endif
  }

  // Run every RTC interruption
  if(rtcFlag){
    rtcFlag = false;

    //publishValues();
    /***********************Send values over Serial Plotter**********************/
    updateVars();

    /****************************************************************************/
    #if DEBUG_PIN_EN
    digitalWrite(DEBUG2, LOW);
    #endif
  }

  __WFE();  // Wait for event instruction (sleeps waiting for event)
}



// Timer interrupt service routine
extern "C" void TIMER4_IRQHandler_v(void) {
  // Check if the interrupt was triggered by CC[0]
  if (NRF_TIMER4->EVENTS_COMPARE[0]) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;  // Clear the event

  #if DEBUG_PIN_EN
    digitalWrite(DEBUG, HIGH);
  #endif
    timerFlag = true;
  }
}

// RTC interrupt service routine
extern "C" void RTC2_IRQHandler_v(void){
  // Check if the interrupt was triggered by a tick event
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0; // Clear event 
    NRF_RTC2->TASKS_CLEAR = 1; // Clear register value

  #if DEBUG_PIN_EN
    digitalWrite(DEBUG2, HIGH);
  #endif
    rtcFlag = true;
    // Use time control below this
  }
}



