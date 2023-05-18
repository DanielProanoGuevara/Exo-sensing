#include "nrf_delay.h"
#include "nrf_gpio.h"

// NRF_GPIO_PIN_MAP(port,pin) creates something that NRF can understand
#define BUTTON NRF_GPIO_PIN_MAP(1,12)
#define LED_IN NRF_GPIO_PIN_MAP(0,13)
//P0.13 -- BuiltInLED -- GPIO1 -- Pin1
//P1.12 -- D3 -- GPIO21 -- 21


void setup() {
  nrf_gpio_cfg_output(LED_IN);
  // PIN, PULL_CONFIG (NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_NOPULL)
  nrf_gpio_cfg_input(BUTTON, NRF_GPIO_PIN_PULLUP);

  nrf_gpio_pin_clear(LED_IN);

}

void loop() {
  if (nrf_gpio_pin_read(BUTTON) == 0) {
    nrf_gpio_pin_set(LED_IN);
    while(nrf_gpio_pin_read(BUTTON) == 0); // Stay in this loop until the button is released
    nrf_gpio_pin_clear(LED_IN);
  } 
}
