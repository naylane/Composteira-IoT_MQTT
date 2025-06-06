 /* Composteira IoT
 *
 * Autor: Naylane Ribeiro
 * Data - 04/06/2025
 * Descrição: Se conecta a um broker MQTT, publica a temperatura do microcontrolador e controla o LED da placa Pico.
 * 
 * Código adaptado de: [link do professor Ricardo Prates]
 */

#include "setup.h"                     // Inclui o arquivo de configuração do projeto

#define WIFI_SSID "SEU_SSID"           // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "SUA_SENHA"      // Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "ENDERECO_MQTT"    // Substitua pelo endereço do host - broket MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "USERNAME"       // Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "PASSWORD"       // Substitua pelo Password da host MQTT - credencial de acesso - caso exista

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
#define TEMP_WORKER_TIME_S 5

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

volatile float temperatura = 39;
volatile float umidade = 49;
volatile int oxigenio = 20;
uint16_t x_pos;
uint16_t y_pos;

//------------------------------------------------------------------------------------------------
// Prototipos de funções
//------------------------------------------------------------------------------------------------
int clamp(int val, int min_val, int max_val);
int map_value_clamped(int val, int in_min, int in_max, int out_min, int out_max);
void le_valores();
void gpio_irq_handler(uint gpio, uint32_t events);
static float read_onboard_temperature(const char unit);
static void pub_request_cb(__unused void *arg, err_t err);
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);
static void control_buzzer(MQTT_CLIENT_DATA_T *state, bool on);
static void publish_temperature(MQTT_CLIENT_DATA_T *state);
static void publish_humidity(MQTT_CLIENT_DATA_T *state);
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
    
    joystick_init();                    // Inicializa o joystick
    buzzer_setup_pwm(BUZZER_PIN, 4000); // Configura o buzzer para PWM

    // Configura os pinos GPIO para acionamento dos LEDs da BitDogLab
    init_led(LED_RED_PIN);
    init_led(LED_BLUE_PIN);
    init_led(LED_GREEN_PIN);

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

        le_valores();

        if ((temperatura > MAX_TEMP) || (umidade > MAX_UMID) || (oxigenio < LIM_OXIG)) {
            gpio_put(LED_RED_PIN, 1);
            gpio_put(LED_GREEN_PIN, 0);
            gpio_put(LED_BLUE_PIN, 0);
            buzzer_play(BUZZER_PIN, 2, 500, 1000);
        }
        else if ((MIN_TEMP < temperatura && temperatura < MAX_TEMP) && (MIN_UMID < umidade && umidade < MAX_UMID) && (oxigenio > LIM_OXIG)) {
            gpio_put(LED_RED_PIN, 0);
            gpio_put(LED_GREEN_PIN, 1);
            gpio_put(LED_BLUE_PIN, 0);
        }
        else {
            gpio_put(LED_RED_PIN, 0);
            gpio_put(LED_GREEN_PIN, 0);
            gpio_put(LED_BLUE_PIN, 1);
        }
        sleep_ms(1000);
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}


int clamp(int val, int min_val, int max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

int map_value_clamped(int val, int in_min, int in_max, int out_min, int out_max) {
    val = clamp(val, in_min, in_max);
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void le_valores() {
    joystick_read_x(&x_pos); // Temperatura
    joystick_read_y(&y_pos); // Umidade

    temperatura = map_value_clamped(x_pos, XY_MIN_ADC, XY_MAX_ADC, 20, 70);  // 20 °C – 70 °C
    umidade = map_value_clamped(y_pos, XY_MIN_ADC, XY_MAX_ADC, 90, 30);      // 90% – 30%
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
 * @brief Controla o buzzer da placa Pico.
 * @param state Ponteiro para os dados do cliente MQTT.
 * @param on Indica se o buzzer deve ser ligado (true) ou desligado (false).
 * @note Esta função publica o estado do buzzer no tópico "/buzzer/state" e também controla o buzzer físico na placa.
 */
static void control_buzzer(MQTT_CLIENT_DATA_T *state, bool on) {
    const char* message = on ? "On" : "Off";
    if (on) {
        INFO_printf("LED ON\n");
        buzzer_play(BUZZER_PIN, 1, 700, 1000);
    } else {
        buzzer_play(BUZZER_PIN, 2, 800, 10);
    }
    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/buzzer/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}


/**
 * @brief Publica a temperatura do microcontrolador no tópico "/temperatura".
 * @param state Ponteiro para os dados do cliente MQTT.
 * @note Esta função lê a temperatura do microcontrolador e publica o valor no tópico "/temperatura" se houver alteração desde a última publicação.
 */
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    const char *temperature_key = full_topic(state, "/temperatura");

    char temp_str[16];
    snprintf(temp_str, sizeof(temp_str), "%.2f", temperatura);
    INFO_printf("Publicando %s em %s\n", temp_str, temperature_key);
    mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}


/**
 * @brief Publica a umidade no tópico "/umidade".
 * @param state Ponteiro para os dados do cliente MQTT.
 */
static void publish_humidity(MQTT_CLIENT_DATA_T *state) {
    const char *humidity_key = full_topic(state, "/umidade");

    char hum_str[16];
    snprintf(hum_str, sizeof(hum_str), "%.2f", umidade);
    INFO_printf("Publicando %s em %s\n", hum_str, humidity_key);
    mqtt_publish(state->mqtt_client_inst, humidity_key, hum_str, strlen(hum_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
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
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/buzzer"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
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

    if (strcmp(basic_topic, "/buzzer") == 0)
    {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_buzzer(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_buzzer(state, false);
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
    publish_humidity(state);
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