#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <hardware/adc.h>
#include "pico/stdlib.h"

#define JOYSTICK_X 27
#define JOYSTICK_Y 26
#define JOYSTICK_BTN 22

#define X_CENTER_ADC 2142
#define Y_CENTER_ADC 1920
#define XY_MIN_ADC 11
#define XY_MAX_ADC 4080

extern uint32_t last_time_joy_btn;    // Tempo da última interrupção do botão

void joystick_init();
void joystick_read_x(uint16_t* joy_x_adc);
void joystick_read_y(uint16_t* joy_y_adc);

#endif