 /* Composteira IoT
 *
 * Autor: Naylane Ribeiro
 * Data - 04/06/2025
 * Descrição: Se conecta a um broker MQTT, publica a temperatura do microcontrolador e controla o LED da placa Pico.
 * 
 * Código adaptado de: [link do professor Ricardo Prates]
 */

#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico

#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "hardware/irq.h"           // Biblioteca de hardware de interrupções
#include "hardware/adc.h"           // Biblioteca de hardware para conversão ADC

#include "lwip/apps/mqtt.h"         // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"    // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"               // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"         // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#define WIFI_SSID "Familia-2.4G"                  // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "31261112"      // Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "192.168.25.4"                // Substitua pelo endereço do host - broket MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "naylane"     // Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "123"     // Substitua pelo Password da host MQTT - credencial de acesso - caso exista

// Definição da escala de temperatura
#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C' // 'C' para Celsius e 'F' para Fahrenheit
#endif

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

// Dados do cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

// Temporização da coleta de temperatura - how often to measure our temperature
#define TEMP_WORKER_TIME_S 10

// Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

#include "pico/bootrom.h"
#define BUTTON_A 5
#define BUTTON_B 6                  // Botão B da placa Pico - GPIO 6
#define BUTTON_JOY 22
#define DEBOUNCE_TIME 300000        // Tempo para debounce em ms
static uint32_t last_time_A = 0;    // Tempo da última interrupção do botão A
static uint32_t last_time_B = 0;    // Tempo da última interrupção do botão B
static uint32_t last_time_joy = 0;  // Tempo da última interrupção do botão do Joystick

//------------------------------------------------------------------------------------------------
// Prototipos de funções
//------------------------------------------------------------------------------------------------
void gpio_irq_handler(uint gpio, uint32_t events);
static float read_onboard_temperature(const char unit);
static void pub_request_cb(__unused void *arg, err_t err);
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);
static void publish_temperature(MQTT_CLIENT_DATA_T *state);
static void sub_request_cb(void *arg, err_t err);
static void unsub_request_cb(void *arg, err_t err);
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
static void start_client(MQTT_CLIENT_DATA_T *state);
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);
//------------------------------------------------------------------------------------------------


/**
 * @brief Função principal do programa.
 * Inicializa as bibliotecas, configura o cliente MQTT e inicia a conexão com o broker.
 */
int main(void) {
    // Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();
    INFO_printf("mqtt client starting\n");

    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    //gpio_set_irq_enabled(BUTTON_B, GPIO_IRQ_EDGE_FALL, true);

    // Inicializa o conversor ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    // Inicializa a arquitetura do cyw43
    if (cyw43_arch_init()) {
        panic("Failed to inizialize CYW43");
    }

    // Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // This confirms the indentity of the server and the client
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
            client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");

    //Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        panic("dns request failed");
    }

    // Loop condicionado a conexão mqtt
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}


// Função de tratamento de interrupção dos botões
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    if (gpio == BUTTON_B) {
        if (current_time - last_time_B > DEBOUNCE_TIME) {
            reset_usb_boot(0, 0);
            last_time_B = current_time;
            return;
        }
    }
    else if (gpio == BUTTON_A) {
        if (current_time - last_time_A > DEBOUNCE_TIME) {
            // ...
            return;
        }
    }
}


// Leitura de temperatura do microcotrolador
/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
static float read_onboard_temperature(const char unit) {

    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C' || unit != 'F') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}


/**
 * @brief Callback para a requisição de publicação (publish).
 * @param arg Ponteiro para os dados do cliente MQTT.
 * @param err Código de erro da requisição.
 */
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}


/**
 * @brief Gera o tópico completo para o cliente MQTT.
 * @param state Ponteiro para os dados do cliente MQTT.
 * @param name Nome do tópico base.
 * @return Retorna o tópico completo, incluindo o ID do cliente se MQTT_UNIQUE_TOPIC estiver definido.
 */
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}


/**
 * @brief Controla o LED da placa Pico.
 * @param state Ponteiro para os dados do cliente MQTT.
 * @param on Indica se o LED deve ser ligado (true) ou desligado (false).
 * @note Esta função publica o estado do LED no tópico "/led/state" e também controla o LED físico na placa.
 */
static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}


/**
 * @brief Publica a temperatura do microcontrolador no tópico "/temperature".
 * @param state Ponteiro para os dados do cliente MQTT.
 * @note Esta função lê a temperatura do microcontrolador e publica o valor no tópico "/temperature" se houver alteração desde a última publicação.
 */
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
    if (temperature != old_temperature) {
        old_temperature = temperature;
        // Publish temperature on /temperature topic
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        INFO_printf("Publishing %s to %s\n", temp_str, temperature_key);
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}


/**
 * @brief Callback para a requisição de inscrição (subscribe).
 * @param arg Ponteiro para os dados do cliente MQTT.
 * @param err Código de erro da requisição.
 */
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}


/**
 * @brief Callback para quando a requisição de unsubscribe é concluída.
 * @param arg Ponteiro para os dados do cliente MQTT.
 * @param err Código de erro da requisição.
 */
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}


/**
 * @brief Inscreve ou cancela a inscrição nos tópicos.
 * @param state Ponteiro para os dados do cliente MQTT.
 * @param sub Indica se deve inscrever (true) ou cancelar a inscrição (false).
 */
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}


/** @brief Recebe os dados de entrada e processa o tópico. 
 *  É chamada quando uma mensagem chega em um tópico inscrito. 
*/
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/led") == 0)
    {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "/print") == 0) {
        INFO_printf("%.*s\n", len, data);
    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true; // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // unsubscribe
    }
}


/**
 * @brief Callback para quando uma mensagem é publicada em um tópico inscrito.
 * @param arg Ponteiro para os dados do cliente MQTT.
 * @param topic Tópico em que a mensagem foi publicada.
 * @param tot_len Comprimento total da mensagem publicada.
 */
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}


/**
 * @brief Função que publica a temperatura do microcontrolador a cada tempo que foi defindo em TEMP_WORKER_TIME_S.
 * @param context Contexto assíncrono que contém informações sobre o ambiente de execução.
 * @param worker Ponteiro para o worker assíncrono que está sendo executado.
 * @note A temperatura é lida em graus Celsius por padrão, mas pode ser configurada para Fahrenheit alterando a definição de TEMPERATURE_UNITS.
 */
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    publish_temperature(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}


/**
 * @brief Callback para a conexão MQTT.
 * @param client Ponteiro para o cliente MQTT.
 * @param arg Argumento passado para o callback, geralmente os dados do cliente MQTT.
 * @param status Status da conexão MQTT.
 */
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publish temperature every 10 sec if it's changed
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            panic("Failed to connect to mqtt server");
        }
    }
    else {
        panic("Unexpected status");
    }
}


/**
 * @brief Inicia o cliente MQTT e conecta ao broker.
 * @param state Ponteiro para os dados do cliente MQTT.
 * @note Esta função cria uma nova instância do cliente MQTT, configura as informações de conexão e inicia a conexão com o broker MQTT.
 * Ela também define o callback para conexões e mensagens recebidas, além de configurar o TLS se estiver habilitado.
 */
static void start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}


/**
 * @brief Callback para o resultado da resolução de DNS.
 * @param hostname Nome do host que foi resolvido.
 * @param ipaddr Endereço IP resolvido.
 * @param arg Argumento passado para o callback, geralmente os dados do cliente MQTT.
 * @note Esta função é chamada quando a resolução de DNS é concluída. Se o endereço IP for válido, inicia o cliente MQTT.
 */
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}