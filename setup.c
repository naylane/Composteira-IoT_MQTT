#include "setup.h"

PIO pio = pio0;
uint sm = 0;

ssd1306_t ssd;
bool cor = true;

int temperatura = 39;
int umidade = 49;
int oxigenio = 20;