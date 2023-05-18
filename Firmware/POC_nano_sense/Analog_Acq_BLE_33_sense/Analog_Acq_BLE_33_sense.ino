#include "nrf_delay.h"
#include "nrf_gpio.h"

#define LED NRF_GPIO_PIN_MAP(1,12)
#define LED_IN NRF_GPIO_PIN_MAP(0,13)
//P0.13 -- BuiltInLED -- GPIO1 -- Pin1
//P1.12 -- D3 -- GPIO21 -- 21


void setup() {
  // Configure builtin LED as output (P0.13 - P1)
  nrf_gpio_cfg_output(LED);
  nrf_gpio_cfg_output(LED_IN);

}

void loop() {
  nrf_gpio_pin_set(LED); // Drives PIN high
  //digitalWrite(LED, HIGH);
  nrf_delay_ms(1000); // Inbuilt delay function
  nrf_gpio_pin_clear(LED); // Drives PIN low
  //digitalWrite(LED, LOW);
  nrf_delay_ms(1000);

  nrf_gpio_pin_toggle(LED_IN);
}
