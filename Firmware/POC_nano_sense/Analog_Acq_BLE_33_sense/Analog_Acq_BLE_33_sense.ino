#include <nrfx.h>
#include <nrf.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"


// NRF_GPIO_PIN_MAP(port,pin) creates something that NRF can understand
#define BUTTON NRF_GPIO_PIN_MAP(1,12)
#define LED_IN NRF_GPIO_PIN_MAP(0,13)
//P0.13 -- BuiltInLED -- GPIO1 -- Pin1
//P1.12 -- D3 -- GPIO21 -- 21








void gpio_init(){
  NRF_GPIO->PIN_CNF[LED_IN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)|
                              (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)|
                              (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)|
                              (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)|
                              (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}


void setup() {
  
  gpio_init();
  NRF_GPIO->OUTSET = (1UL << LED_IN);

}

void loop() {
  NRF_GPIO->OUTSET = (1UL << LED_IN);
  nrf_delay_ms(500);
  NRF_GPIO->OUTCLR = (1UL << LED_IN);
  nrf_delay_ms(500);
}
