#ifndef SETUP_H
#define SETUP_H

#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico
#include "pico/bootrom.h"

#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "hardware/irq.h"           // Biblioteca de hardware de interrupções
#include "hardware/adc.h"           // Biblioteca de hardware para conversão ADC
#include "hardware/pio.h"

#include "lwip/apps/mqtt.h"         // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"    // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"               // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"         // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#include "lib/ssd1306.h"
#include "lib/ws2812.h"

// Definição dos pinos dos LEDs
#define LED_PIN CYW43_WL_GPIO_LED_PIN   // GPIO do CI CYW43
#define LED_RED_PIN 13                  // Pino do LED vermelho (GPIO 13)
#define LED_BLUE_PIN 12                 // Pino do LED azul (GPIO 12)
#define LED_GREEN_PIN 11                // Pino do LED verde (GPIO 11)

#define BUTTON_B 6                  // Pino do botão B (GPIO 6)
#define DEBOUNCE_TIME 200000        // Tempo para debounce em ms
static uint32_t last_time_B = 0;    // Tempo da última interrupção do botão B

#define WS2812_PIN 7
extern PIO pio;
extern uint sm;

#define MAX_TEMP 60 // Limite máximo de temperatura
#define MIN_TEMP 40 // Limite mínimo de temperatura
#define MAX_UMID 70 // Limite máximo de umidade
#define MIN_UMID 50 // Limite mínimo de umidade
#define LIM_OXIG 15 // Limite de oxigênio

extern int temperatura;
extern int umidade;
extern int oxigenio;

#endif