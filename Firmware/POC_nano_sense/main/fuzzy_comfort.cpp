#include "fuzzy_comfort.h"

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

float get_comfort(float t_amb, float t_skin, float force){
  fuzzy->setInput(1, t_amb);
  fuzzy->setInput(2, t_skin);
  fuzzy->setInput(3, force);

  fuzzy->fuzzify();

  return fuzzy->defuzzify(1);
}

