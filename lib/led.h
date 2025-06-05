#ifndef LED_H
#define LED_H

#include "hardware/gpio.h"
#include "hardware/pwm.h"

// Configura os pinos GPIO para acionamento dos LEDs da BitDogLab
void init_led(uint pino);

#endif