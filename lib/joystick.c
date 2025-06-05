#include "joystick.h"

uint32_t last_time_joy_btn = 0;

/**
 * @brief Inicializa o joystick.
 * Configura os pinos ADC para leitura dos eixos X e Y, e o botão do joystick.
 */
void joystick_init() {
    adc_init();
    adc_gpio_init(JOYSTICK_X);
    adc_gpio_init(JOYSTICK_Y);
    
    gpio_init(JOYSTICK_BTN);
    gpio_set_dir(JOYSTICK_BTN, GPIO_IN);
    gpio_pull_up(JOYSTICK_BTN);
}

void joystick_read_x(uint16_t* joy_x_adc) {
    adc_select_input(0);
    *joy_x_adc = adc_read();
}

void joystick_read_y(uint16_t* joy_y_adc) {
    adc_select_input(1);
    *joy_y_adc = adc_read();
}