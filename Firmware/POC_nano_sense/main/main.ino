//Include libraries

#include "dsp.h"
#include "sys.h"
#include "bt_low_energy.h"
#include "setup_peripheral.h"
#include "low_power.h"


#include <Fuzzy.h>
/**************** Fuzzy Variables *************************/
// Fuzzy
Fuzzy *fuzzy = new Fuzzy();

// FuzzyInput Ambient Temperature
FuzzySet *very_cold = new FuzzySet(0, 0, 10, 16);
FuzzySet *cold = new FuzzySet(12, 15, 19, 22);
FuzzySet *neutral = new FuzzySet(20, 22, 26, 27);
FuzzySet *hot = new FuzzySet(25, 28, 32, 33);
FuzzySet *very_hot = new FuzzySet(30, 35, 50, 50);

// FuzzyInput Skin Temperature
FuzzySet *cold_nociceptor = new FuzzySet(0, 0, 10, 14);
FuzzySet *vasoconstriction = new FuzzySet(10, 14, 30, 33);
FuzzySet *homeostasis = new FuzzySet(31, 33, 36, 37);
FuzzySet *vasodilation = new FuzzySet(35, 38, 40, 45);
FuzzySet *heat_nociceptor = new FuzzySet(39, 44, 50, 50);

// FuzzyInput Force
FuzzySet *low = new FuzzySet(0, 0, 5, 15);
FuzzySet *moderate = new FuzzySet(5, 15, 30, 40);
FuzzySet *high = new FuzzySet(30, 50, 100, 100);

// FuzzyOutput Comfort
FuzzySet *very_uncomfortable = new FuzzySet(0, 0, 2, 5);
FuzzySet *uncomfortable = new FuzzySet(2, 4, 7, 9);
FuzzySet *comfortable = new FuzzySet(7, 9, 10, 10);


/*********************************************************/

// Variables
uint8_t decimationCounter = 1;
float32_t FSR1_filt = 0.0f;
uint16_t DAC_o = 0;

// Interrupt flags
volatile bool timerFlag = false;
volatile bool rtcFlag = false;
volatile uint32_t seconds = 0;

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
  movingAverage((float32_t *)&MA_ring_buffer_NTC1, M, RB_i_NTC1, NTC1_raw * (3.3f / 4096.0f) , &average_NTC1  , &min_NTC1  , &max_NTC1);
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

  //Comfort value
  float comfort_level = 0.0f;

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


  // Environmental
  amb_temperature = HS300x.readTemperature();
  amb_humidity = HS300x.readHumidity();
  amb_baro_pressure = BARO.readPressure();


  /***** IMPLEMENT FUZZY PREDICTOR FOR COMFORT *****/
  fuzzy->setInput(1, amb_temperature);
  fuzzy->setInput(2, skinTemperatureMean);
  fuzzy->setInput(3, interfaceingForceMean);

  fuzzy->fuzzify();

  comfort_level = fuzzy->defuzzify(1);


  // Check if the low power library is included
  #ifdef LOWPOWER_INCLUDED
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH); //PIN_ENABLE_SENSORS_3V3 set to HIGH:
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH); // PIN_ENABLE_I2C_PULLUP set to HIGH:  
  #endif
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
  Serial.print(",");
  Serial.print("Comfort_level:");
  Serial.println(comfort_level);
  #endif
  /*********************** Send BLE data **********************/

  updateAmbTemperature(amb_temperature);
  updateAmbPressure(amb_baro_pressure);
  updateAmbHumidity(amb_humidity);

  updateMinBodyTemp(skinTemperatureMin);
  updateAvgBodyTemp(skinTemperatureMean);
  updateMaxBodyTemp(skinTemperatureMax);

  updateMinForce(interfaceingForceMin);
  updateAvgForce(interfaceingForceMean);
  updateMaxForce(interfaceingForceMax);
  
  /**************************UPDATE ON TIME **********************/
  updateONTime(seconds);

  /**************************UPDATE COMFORT**********************/
  updateComfort(comfort_level);
}

void setupFuzzy(){
  // FuzzyInput
  FuzzyInput *t_amb = new FuzzyInput(1);

  t_amb->addFuzzySet(very_cold);
  t_amb->addFuzzySet(cold);
  t_amb->addFuzzySet(neutral);
  t_amb->addFuzzySet(hot);
  t_amb->addFuzzySet(very_hot); 
  fuzzy->addFuzzyInput(t_amb);

  // FuzzyInput
  FuzzyInput *t_skin = new FuzzyInput(2);

  t_skin->addFuzzySet(cold_nociceptor);
  t_skin->addFuzzySet(vasoconstriction);
  t_skin->addFuzzySet(homeostasis);
  t_skin->addFuzzySet(vasodilation);
  t_skin->addFuzzySet(heat_nociceptor);
  fuzzy->addFuzzyInput(t_skin);

   // FuzzyInput
  FuzzyInput *force = new FuzzyInput(3);

  force->addFuzzySet(low);
  force->addFuzzySet(moderate);
  force->addFuzzySet(high);
  fuzzy->addFuzzyInput(force);

  // FuzzyOutput
  FuzzyOutput *comfort = new FuzzyOutput(1);

  comfort->addFuzzySet(very_uncomfortable);
  comfort->addFuzzySet(uncomfortable);
  comfort->addFuzzySet(comfortable);
  fuzzy->addFuzzyOutput(comfort);

  /*************** Build Fuzzy Rule 1 (Very Uncomfortable) **********************/
  //-
  FuzzyRuleAntecedent *forceHigh = new FuzzyRuleAntecedent();
  forceHigh->joinSingle(high);
  //-
  FuzzyRuleAntecedent *tskinColdNociceptor = new FuzzyRuleAntecedent();
  tskinColdNociceptor->joinSingle(cold_nociceptor);
  //-
  FuzzyRuleAntecedent *tskinHeatNociceptor = new FuzzyRuleAntecedent();
  tskinHeatNociceptor->joinSingle(heat_nociceptor);
  //-
  FuzzyRuleAntecedent *veryColdAndVasoconstriction = new FuzzyRuleAntecedent();
  veryColdAndVasoconstriction->joinWithAND(very_cold, vasoconstriction);
  //-
  FuzzyRuleAntecedent *veryHotAndVasoconstriction = new FuzzyRuleAntecedent();
  veryHotAndVasoconstriction->joinWithAND(very_hot, vasoconstriction);
  //-
  FuzzyRuleAntecedent *veryColdAndVasodilation = new FuzzyRuleAntecedent();
  veryColdAndVasodilation->joinWithAND(very_cold, vasodilation);
  //-
  FuzzyRuleAntecedent *veryHotAndVasodilation = new FuzzyRuleAntecedent();
  veryHotAndVasodilation->joinWithAND(very_hot, vasodilation);
  //-
  FuzzyRuleAntecedent *coldAvasoconstriction = new FuzzyRuleAntecedent();
  coldAvasoconstriction->joinWithAND(cold, vasoconstriction);
  FuzzyRuleAntecedent *coldAvasconstrictionAmoderate = new FuzzyRuleAntecedent();
  coldAvasconstrictionAmoderate->joinWithAND(coldAvasoconstriction, moderate);
  //-
  FuzzyRuleAntecedent *hotAvasoconstriction = new FuzzyRuleAntecedent();
  hotAvasoconstriction->joinWithAND(hot, vasoconstriction);
  FuzzyRuleAntecedent *hotAvasconstrictionAmoderate = new FuzzyRuleAntecedent();
  hotAvasconstrictionAmoderate->joinWithAND(hotAvasoconstriction, moderate);
  //-
  FuzzyRuleAntecedent *coldAvasodilation = new FuzzyRuleAntecedent();
  coldAvasodilation->joinWithAND(cold, vasodilation);
  FuzzyRuleAntecedent *coldAvasdilationAmoderate = new FuzzyRuleAntecedent();
  coldAvasdilationAmoderate->joinWithAND(coldAvasodilation, moderate);
  //-
  FuzzyRuleAntecedent *hotAvasodilation = new FuzzyRuleAntecedent();
  hotAvasodilation->joinWithAND(hot, vasodilation);
  FuzzyRuleAntecedent *hotAvasdilationAmoderate = new FuzzyRuleAntecedent();
  hotAvasdilationAmoderate->joinWithAND(hotAvasodilation, moderate);
  //-
  FuzzyRuleAntecedent *A_1 = new FuzzyRuleAntecedent();
  A_1->joinWithOR(forceHigh,tskinColdNociceptor);
  //-
  FuzzyRuleAntecedent *B_1 = new FuzzyRuleAntecedent();
  B_1->joinWithOR(A_1, tskinHeatNociceptor);
  //-
  FuzzyRuleAntecedent *C_1 = new FuzzyRuleAntecedent();
  C_1->joinWithOR(B_1, veryColdAndVasoconstriction);
  //-
  FuzzyRuleAntecedent *D_1 = new FuzzyRuleAntecedent();
  D_1->joinWithOR(C_1, veryHotAndVasoconstriction);
  //-
  FuzzyRuleAntecedent *E_1 = new FuzzyRuleAntecedent();
  E_1->joinWithOR(D_1, veryColdAndVasodilation);
  //-
  FuzzyRuleAntecedent *F_1 = new FuzzyRuleAntecedent();
  F_1->joinWithOR(E_1, veryHotAndVasodilation);
  //-
  FuzzyRuleAntecedent *G_1 = new FuzzyRuleAntecedent();
  G_1->joinWithOR(F_1, coldAvasconstrictionAmoderate);
  //-
  FuzzyRuleAntecedent *H_1 = new FuzzyRuleAntecedent();
  H_1->joinWithOR(G_1, hotAvasconstrictionAmoderate);
  //-
  FuzzyRuleAntecedent *I_1 = new FuzzyRuleAntecedent();
  I_1->joinWithOR(H_1, coldAvasdilationAmoderate);
  //-
  FuzzyRuleAntecedent *ifJ_1 = new FuzzyRuleAntecedent();
  ifJ_1->joinWithOR(I_1, hotAvasdilationAmoderate);

  //- Consequence
  FuzzyRuleConsequent *thenVeryUncomfortable = new FuzzyRuleConsequent();
  thenVeryUncomfortable->addOutput(very_uncomfortable);

  // Rule construction
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifJ_1, thenVeryUncomfortable);
  fuzzy->addFuzzyRule(fuzzyRule1);
  /******************************************************************************/


  /*************** Build Fuzzy Rule 2 (Uncomfortable) ***************************/
  FuzzyRuleAntecedent *veryColdAndHomeostasis = new FuzzyRuleAntecedent();
  veryColdAndHomeostasis->joinWithAND(very_cold, homeostasis);
    //-
    FuzzyRuleAntecedent *veryColdAHomestasisALow = new FuzzyRuleAntecedent();
    veryColdAHomestasisALow->joinWithAND(veryColdAndHomeostasis, low); 
    //-
    FuzzyRuleAntecedent *veryColdAHomestasisAModerate = new FuzzyRuleAntecedent();
    veryColdAHomestasisAModerate->joinWithAND(veryColdAndHomeostasis, moderate);
  FuzzyRuleAntecedent *veryHotAndHomeostasis = new FuzzyRuleAntecedent();
  veryHotAndHomeostasis->joinWithAND(very_hot, homeostasis);
    //-
    FuzzyRuleAntecedent *veryHotAHomestasisALow = new FuzzyRuleAntecedent();
    veryHotAHomestasisALow->joinWithAND(veryHotAndHomeostasis, low); 
    //-
    FuzzyRuleAntecedent *veryHotAHomestasisAModerate = new FuzzyRuleAntecedent();
    veryHotAHomestasisAModerate->joinWithAND(veryHotAndHomeostasis, moderate);
  FuzzyRuleAntecedent *neutralAndVasoconstriction = new FuzzyRuleAntecedent();
  neutralAndVasoconstriction->joinWithAND(neutral, vasoconstriction);
    //-
    FuzzyRuleAntecedent *neutralAVasoconstrictionALow = new FuzzyRuleAntecedent();
    neutralAVasoconstrictionALow->joinWithAND(neutralAndVasoconstriction, low); 
    //-
    FuzzyRuleAntecedent *neutralAVasoconstrictionAModerate = new FuzzyRuleAntecedent();
    neutralAVasoconstrictionAModerate->joinWithAND(neutralAndVasoconstriction, moderate);
  FuzzyRuleAntecedent *neutralAndVasodilation = new FuzzyRuleAntecedent();
  neutralAndVasodilation->joinWithAND(neutral, vasodilation);
    //-
    FuzzyRuleAntecedent *neutralAVasodilationALow = new FuzzyRuleAntecedent();
    neutralAVasodilationALow->joinWithAND(neutralAndVasodilation, low); 
    //-
    FuzzyRuleAntecedent *neutralAVasodilationAModerate = new FuzzyRuleAntecedent();
    neutralAVasodilationAModerate->joinWithAND(neutralAndVasodilation, moderate);
  FuzzyRuleAntecedent *coldAndLow = new FuzzyRuleAntecedent();
  coldAndLow->joinWithAND(cold, low);
    //-
    FuzzyRuleAntecedent *coldALowAVasoconstriction = new FuzzyRuleAntecedent();
    coldALowAVasoconstriction->joinWithAND(coldAndLow, vasoconstriction); 
    //-
    FuzzyRuleAntecedent *coldALowAVasodilation = new FuzzyRuleAntecedent();
    coldALowAVasodilation->joinWithAND(coldAndLow, vasodilation);
  FuzzyRuleAntecedent *hotAndLow = new FuzzyRuleAntecedent();
  hotAndLow->joinWithAND(hot, low);
    //-
    FuzzyRuleAntecedent *hotALowAVasoconstriction = new FuzzyRuleAntecedent();
    hotALowAVasoconstriction->joinWithAND(hotAndLow, vasoconstriction); 
    //-
    FuzzyRuleAntecedent *hotALowAVasodilation = new FuzzyRuleAntecedent();
    hotALowAVasodilation->joinWithAND(hotAndLow, vasodilation);
  //-
  FuzzyRuleAntecedent *A_2 = new FuzzyRuleAntecedent();
  A_2->joinWithOR(veryColdAHomestasisALow,veryColdAHomestasisAModerate);
  //-
  FuzzyRuleAntecedent *B_2 = new FuzzyRuleAntecedent();
  B_2->joinWithOR(A_2,veryHotAHomestasisALow);
  //-
  FuzzyRuleAntecedent *C_2 = new FuzzyRuleAntecedent();
  C_2->joinWithOR(B_2,veryHotAHomestasisAModerate);
  //-
  FuzzyRuleAntecedent *D_2 = new FuzzyRuleAntecedent();
  D_2->joinWithOR(C_2,neutralAVasoconstrictionALow);
  //-
  FuzzyRuleAntecedent *E_2 = new FuzzyRuleAntecedent();
  E_2->joinWithOR(D_2,neutralAVasoconstrictionAModerate);
  //-
  FuzzyRuleAntecedent *F_2 = new FuzzyRuleAntecedent();
  F_2->joinWithOR(E_2,neutralAVasodilationALow);
  //-
  FuzzyRuleAntecedent *G_2 = new FuzzyRuleAntecedent();
  G_2->joinWithOR(F_2,neutralAVasodilationAModerate);
  //-
  FuzzyRuleAntecedent *H_2 = new FuzzyRuleAntecedent();
  H_2->joinWithOR(G_2,coldALowAVasoconstriction);
  //-
  FuzzyRuleAntecedent *I_2 = new FuzzyRuleAntecedent();
  I_2->joinWithOR(H_2,coldALowAVasodilation);
  //-
  FuzzyRuleAntecedent *J_2 = new FuzzyRuleAntecedent();
  J_2->joinWithOR(I_2,hotALowAVasoconstriction);
  //-
  FuzzyRuleAntecedent *ifK_2 = new FuzzyRuleAntecedent();
  ifK_2->joinWithOR(J_2,hotALowAVasodilation);

  //- Consequence
  FuzzyRuleConsequent *thenUncomfortable = new FuzzyRuleConsequent();
  thenUncomfortable->addOutput(uncomfortable);

  // Rule construction
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifK_2, thenUncomfortable);
  fuzzy->addFuzzyRule(fuzzyRule2);
  /******************************************************************************/


  /*************** Build Fuzzy Rule 3 (Comfortable) ***************************/
  FuzzyRuleAntecedent *coldAndHomeostasis = new FuzzyRuleAntecedent();
  coldAndHomeostasis->joinWithAND(cold, homeostasis);
    //-
    FuzzyRuleAntecedent *coldAHomeostasisALow = new FuzzyRuleAntecedent();
    coldAHomeostasisALow->joinWithAND(coldAndHomeostasis, low);
    //-
    FuzzyRuleAntecedent *coldAHomeostasisAModerate = new FuzzyRuleAntecedent();
    coldAHomeostasisAModerate->joinWithAND(coldAndHomeostasis, moderate);
    //-
    FuzzyRuleAntecedent *A_3  = new FuzzyRuleAntecedent();
    A_3->joinWithOR(coldAHomeostasisALow, coldAHomeostasisAModerate);
  
  FuzzyRuleAntecedent *hotAndHomeostasis = new FuzzyRuleAntecedent();
  hotAndHomeostasis->joinWithAND(hot, homeostasis);
    //-
    FuzzyRuleAntecedent *hotAHomeostasisALow = new FuzzyRuleAntecedent();
    hotAHomeostasisALow->joinWithAND(hotAndHomeostasis, low);
    //-
    FuzzyRuleAntecedent *hotAHomeostasisAModerate = new FuzzyRuleAntecedent();
    hotAHomeostasisAModerate->joinWithAND(hotAndHomeostasis, moderate);
    //-
    FuzzyRuleAntecedent *B_3  = new FuzzyRuleAntecedent();
    B_3->joinWithOR(hotAHomeostasisALow, hotAHomeostasisAModerate);
  
  FuzzyRuleAntecedent *neutralAndHomeostasis = new FuzzyRuleAntecedent();
  neutralAndHomeostasis->joinWithAND(neutral, homeostasis);
    //-
    FuzzyRuleAntecedent *neutralAHomeostasisALow = new FuzzyRuleAntecedent();
    neutralAHomeostasisALow->joinWithAND(neutralAndHomeostasis, low);
    //-
    FuzzyRuleAntecedent *neutralAHomeostasisAModerate = new FuzzyRuleAntecedent();
    neutralAHomeostasisAModerate->joinWithAND(neutralAndHomeostasis, moderate);
    //-
    FuzzyRuleAntecedent *C_3  = new FuzzyRuleAntecedent();
    C_3->joinWithOR(neutralAHomeostasisALow, neutralAHomeostasisAModerate);
  //-
  FuzzyRuleAntecedent *D_3 = new FuzzyRuleAntecedent();
  D_3->joinWithOR(A_3, B_3);
  //
  FuzzyRuleAntecedent *ifE_3 = new FuzzyRuleAntecedent();
  ifE_3->joinWithOR(C_3, D_3);
  //- Consequence
  FuzzyRuleConsequent *thenComfortable = new FuzzyRuleConsequent();
  thenComfortable->addOutput(comfortable);

  // Rule construction
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifE_3, thenComfortable);
  fuzzy->addFuzzyRule(fuzzyRule3);

  /******************************************************************************/
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
  
  #if UART_EN
    // Initialize serial
    Serial.begin(115200);
    while(!Serial);
  #endif
  #if DEBUG_DAC
    // Initializes SPI for the DAC
    setupDAC();
  #endif

  setupBluetoothLE();
  setupFuzzy();
  setupLowPower();
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

    /*********************** wake up sensors **********************/
    #ifdef LOW_POWER_H
    digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH); //PIN_ENABLE_SENSORS_3V3 set to HIGH:
    digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH); // PIN_ENABLE_I2C_PULLUP set to HIGH:
    #endif 
    /*********************** Send values **********************/
    updateVars();

    /****************************************************************************/
    #if DEBUG_PIN_EN
    digitalWrite(DEBUG2, LOW);
    #endif
  } 

  __SEV();
  __WFE();
  __WFE();  
  poll_BLE();
}



// Timer interrupt service routine
extern "C" void TIMER4_IRQHandler_v(void) {
  // Check if the interrupt was triggered by CC[0]
  if (NRF_TIMER4->EVENTS_COMPARE[0]) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;  // Clear the event
    NRF_TIMER4->TASKS_CLEAR = 1; // Clear register value
    __SEV(); // to avoid race condition

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
    __SEV(); // to avoid race condition

    #if DEBUG_PIN_EN
      digitalWrite(DEBUG2, HIGH);
    #endif

    rtcFlag = true;
    // Use time control below this
    seconds++;
  }
}