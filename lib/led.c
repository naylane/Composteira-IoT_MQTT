#include "led.h"

void init_led(uint pino) {
    gpio_init(pino);
    gpio_set_dir(pino, GPIO_OUT);
    gpio_put(pino, 0);
}