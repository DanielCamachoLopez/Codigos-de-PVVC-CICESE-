/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h" 
#include "esp_timer.h"

#include "driver/i2c.h" // Contiene todas las funciones y estructuras necesarias para usar I2C
// El BMP180 se comunica con el ESP32 mediante I2C 

#include "bmp180.h" // Contiene las funciones y estructuras necesarias para hablar específicamente con el sensor BMP180, usando I²C como medio


#define I2C_MASTER_SCL_IO 22     // Pin SCL (Serial Clock Line) del BMP180
#define I2C_MASTER_SDA_IO 21     // Pin SDA (Serial Data Line) del BMP180

// El ESP32 tiene dos controladores I2C internos
#define I2C_MASTER_NUM I2C_NUM_0 // Aquí indica al código que usaremos el controlador 0

// El ESP32 enviará los pulsos de reloj (SCL) a 100 000 veces por segundo para comunicarse con el BMP180
#define I2C_MASTER_FREQ_HZ 100000 // Es el modo standar o la más común de usar

#define TAG "EXAMPLE"
#define CID_ESP 0x02E5


// #define POT_PIN ADC1_CHANNEL_6  // GPIO34, canal del potenciómetro

// static int lectura_pot; // Guarda la lectura cruda del ADC (valor entre 0 y 4095)
// static uint8_t valor_escalado; // Guarda el valor convertido a porcentaje (0–100%)

// Es un tipo de datos estructurado definido en la librería bmp180.h
static bmp180_dev_t bmp180_dev;  // Contendrá toda la información necesaria para controlar el sensor


// Inicialización del bus I2C, para que el ESP32 pueda comunicarse con el BMP180
static void i2c_master_init(void) { 
    i2c_config_t conf = { // Estructura que almacemará todos los parámetros de configuración del bus I2C
        .mode = I2C_MODE_MASTER, // Configura el ESP32 como maestro I2C, capaz de iniciar la comunicación con el sensor
        .sda_io_num = I2C_MASTER_SDA_IO, // Pin usado para SDA
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Activa la resistencia pull-up interna para la línea SDA, necesaria en I2C
        .scl_io_num = I2C_MASTER_SCL_IO, // Pin usado para SCL
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Activa la resistencia pull-up interna para la línea SCL, necesaria en I2C
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Velocidad del bus I2C en Hz (estandar)
    };
    i2c_param_config(I2C_MASTER_NUM, &conf); // Aplica la configuración al controlador I2C seleccionado
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0); // Instala el driver de I2C en el ESP32 con la configuración dada
    ESP_LOGI(TAG, "I2C inicializado correctamente"); // Muestra que toda la inicialización fue exitosa
}

// Inicializa el sensor BMP180
static void bmp180_init_sensor(void) {
    ESP_LOGI(TAG, "Inicializando sensor BMP180..."); // Indica que comenzará el proceso de inicialización del BMP180

    // Un descriptor es una estructura de datos que contiene toda la información necesaria para comunicarse con un dispositivo
    // En esta línea, preparamos el descriptor del sensor BMP180, necesario para la comunicación I2C
    esp_err_t ret = bmp180_init_desc(&bmp180_dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    if (ret != ESP_OK) { // Verificar si hubo error al crear el descriptor
        ESP_LOGE(TAG, "Error inicializando descriptor BMP180: %s", esp_err_to_name(ret)); // Si hubo error, lo indica
        return;
    }

    // Se inicializa el sensor BMP180 en sí, leyendo parámetros internos y dejándolo listo para medir temperatura y presión
    ret = bmp180_init(&bmp180_dev); // Guarda el resultado en la variable ret para verificar si hubo error
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando BMP180: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Sensor BMP180 inicializado correctamente"); // Si no hubo problemas, indica que todo fue exitoso
}

extern struct _led_state led_state[3];

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

// Se prepara el convertidor analógico digital
/*static void configuracion_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12); // Configura el rango de resolución a 12 bits (0–4095) del potenciómetro
    adc1_config_channel_atten(POT_PIN, ADC_ATTEN_DB_11); // Aplica una atenuación de 11 dB (para leer hasta 3.6 V)
    ESP_LOGI(TAG, "ADC configurado (GPIO34)"); // Imprime que la configuración fue exitosa
}*/

// Modelos
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_1, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_1 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_2, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server_2 = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};

// Modelo Level
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub, 2 + 5, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
};

// Define qué modelos BLE Mesh tiene tu nodo, es decir, qué tipo de mensajes puede enviar, recibir o responder
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server), // Es obligatorio. Permite que el provisionador configure el nodo.
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0), // Sirve para encender o apagar algo (Valores de 1 y 0)
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub, &level_server), // Maneja valores continuos (por ejemplo, posición de potenciómetro, brillo, velocidad, etc.)
};

// MODELOS ADICIONALES (OnOff servers extras para encender o apagar otros LEDs)
static esp_ble_mesh_model_t extend_model_0[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_1, &onoff_server_1),
};

static esp_ble_mesh_model_t extend_model_1[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_2, &onoff_server_2),
};



//////////////////////////////////////////////////////////////////////////////////////////
// Función de configuración de publicación
// Se crea un servidor Level, que maneja mensajes relacionados con “niveles”, valores que suben o bajan gradualmente (continuos)
static void configurar_publicacion_level(void) {
    esp_ble_mesh_model_t *level_model = &root_models[2]; // Apunta al tercer modelo, que es el Level Server
    
    // Configurar publicación básica
    level_model->pub->publish_addr = 0x0001; // Dirección a la que va a publicar 
    level_model->pub->app_idx = 0x0000; // Usa la primera AppKey para cifrar mensajes
    level_model->pub->cred = 0; // No usa friendship credentials. Comunicación normal
    level_model->pub->ttl = 5; // El mensaje puede pasar por hasta 5 nodos
    
    ESP_LOGI(TAG, "Publicación Level configurada en grupo 0xC000"); // Indica que la configuración fue exitosa
}

// Función que lee el potenciómetro, convierte el valor y lo publica por BLE Mesh
/*static void enviar_valor_potenciometro(void) {
    // Verificar si el nodo está provisionado, si no, no envía nada
    if (!esp_ble_mesh_node_is_provisioned()) {
        ESP_LOGW(TAG, "Nodo no provisionado aún...");
        return;
    }

    // Leer ADC
    lectura_pot = adc1_get_raw(POT_PIN); // Lee el valor crudo del potenciómetro (0-4095)
    valor_escalado = (lectura_pot * 100) / 4095; // Escala la lectura a porcentaje (0-100%)

    ESP_LOGI(TAG, "Potenciómetro: %d -> %d%%", lectura_pot, valor_escalado); // Imprime el valor crudo y el porcentaje

    // Convierte ese porcentaje a level (0–1000), que usa el modelo BLE Mesh
    int16_t nivel_pot = (int16_t)(valor_escalado * 10);

    // Actualizar estado local del servidor guardandolo en la variable level_server.state.level
    level_server.state.level = nivel_pot;

    // Sirve para que el modelo Level Server envíe un mensaje de “publicación” a la dirección que se configuró previamente
    esp_err_t err = esp_ble_mesh_model_publish(level_server.model, // Es un puntero al modelo BLE Mesh que enviará el mensaje (al que se configuró antes)
                                              ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, // Qué tipo de mensaje enviar: "Level Status" (estado del nivel)
                                              sizeof(level_server.state.level), // Cuántos bytes se van a enviar
                                              (uint8_t*)&level_server.state.level, // Dato real que se envía: el valor del potenciómetro convertido a nivel
                                              ROLE_NODE); // Quién lo envía: este nodo normal

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error publicando: %s", esp_err_to_name(err)); // Mensaje de error si no se pudo enviar
    } else {
        ESP_LOGI(TAG, "Valor %d%% enviado como Level %d", valor_escalado, nivel_pot); // Mensaje de éxito
    }
} */

// Lee al BMP180 y envía los datos por BLE Mesh
static void enviar_valor_bmp180(void) {
    if (!esp_ble_mesh_node_is_provisioned()) { // Verifica si el nodo está provisionado
        ESP_LOGW(TAG, "Nodo no provisionado aún..."); // Si no está provisionado avisa y no envía nada
        return;
    }

    float temperature = 0; // Variable para guardar la temperatura leída
    uint32_t pressure = 0; // Variable para guardar la presión leída

    // Llama a bmp180_measure para leer temperatura y presión del sensor BMP180. Se guarda en la variable ret
    esp_err_t ret = bmp180_measure(&bmp180_dev, &temperature, &pressure, BMP180_MODE_STANDARD);
    // &bmp180_dev = descriptor del sensor (ya inicializado)
    // &temperature, &pressure = direcciones de memoria donde la función guardará los resultados
    // BMP180_MODE_STANDARD = modo de medición estándar
    
    if (ret != ESP_OK) { // Se verifica si hubo error al leer el sensor
        ESP_LOGE(TAG, "Error leyendo BMP180: %s", esp_err_to_name(ret)); // Si hubo error, lo indica
        return;
    }

    // Convierte la presión de Pa a hPa dividiendo entre 100
    int16_t presion_hpa = (int16_t)(pressure / 100);

    // Muestra ambos valores en el terminal del nodo provisionado
    ESP_LOGI(TAG, "--- DATOS BMP180 (LOCAL) ---");
    ESP_LOGI(TAG, "Temperatura: %.1f°C", temperature);
    ESP_LOGI(TAG, "Presión: %lu Pa (%.1f hPa)", pressure, pressure / 100.0);
    ESP_LOGI(TAG, "Enviando al provisionador: %d hPa", presion_hpa);
    ESP_LOGI(TAG, "------------------------------------");

    // Asigna el valor de presión al estado del servidor BLE Mesh (level_server)
    level_server.state.level = presion_hpa;

    // Llama a esp_ble_mesh_model_publish para enviar el valor de presión a la red BLE Mesh
    esp_err_t err = esp_ble_mesh_model_publish(level_server.model, // Modelo BLE Mesh que publica el dato
                                              ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, // Operación BLE Mesh que indica que se envía un nivel
                                              sizeof(level_server.state.level), // Tamaño del dato que se envía
                                              (uint8_t*)&level_server.state.level, // Dato real que se envía. Dirección de memoria del valor de presión
                                              ROLE_NODE); // Indica que este dispositivo actúa como nodo de la red

    if (err != ESP_OK) { // Verifica si la publicación fue exitosa
        ESP_LOGE(TAG, "Error publicando presión: %s", esp_err_to_name(err)); // Si hubo error, lo indica
    } else {
        ESP_LOGI(TAG, " Presión enviada correctamente"); // Si no hubo error, indica que el dato se envió exitosamente
    }
}

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_0, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, extend_model_1, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
    .output_size = 0,
    .output_actions = 0,
};


static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "Provisionamiento completado!");
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    board_led_operation(LED_G, LED_OFF);
    
    // Configurar publicación después del provisionamiento
    vTaskDelay(pdMS_TO_TICKS(2000));
    configurar_publicacion_level();
    
    ESP_LOGI(TAG, "Listo para enviar datos del potenciómetro!");
}

static void example_change_led_state(esp_ble_mesh_model_t *model,
                                     esp_ble_mesh_msg_ctx_t *ctx, uint8_t onoff)
{
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    struct _led_state *led = NULL;
    uint8_t i;

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
        for (i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
                led = &led_state[i];
                board_led_operation(led->pin, onoff);
                led->current = onoff;
            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            led = &led_state[model->element->element_addr - primary_addr];
            board_led_operation(led->pin, onoff);
            led->current = onoff;
        }
    } else if (ctx->recv_dst == 0xFFFF) {
        led = &led_state[model->element->element_addr - primary_addr];
        board_led_operation(led->pin, onoff);
        led->current = onoff;
    }
}

static void example_handle_gen_onoff_msg(esp_ble_mesh_model_t *model,
                                         esp_ble_mesh_msg_ctx_t *ctx,
                                         esp_ble_mesh_server_recv_gen_onoff_set_t *set)
{
    esp_ble_mesh_gen_onoff_srv_t *srv = (esp_ble_mesh_gen_onoff_srv_t *)model->user_data;

    switch (ctx->recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
        esp_ble_mesh_server_model_send_msg(model, ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        break;

    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
        if (set->op_en == false) {
            srv->state.onoff = set->onoff;
        } else {
            srv->state.onoff = set->onoff;
        }
        if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
            esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS, sizeof(srv->state.onoff), &srv->state.onoff);
        }
        esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            sizeof(srv->state.onoff), &srv->state.onoff, ROLE_NODE);
        example_change_led_state(model, ctx, srv->state.onoff);
        break;
    default:
        break;
    }
}

// Manejar mensajes Level
static void example_handle_gen_level_msg(esp_ble_mesh_model_t *model,
                                        esp_ble_mesh_msg_ctx_t *ctx,
                                        esp_ble_mesh_server_recv_gen_level_set_t *set)
{
    esp_ble_mesh_gen_level_srv_t *srv = (esp_ble_mesh_gen_level_srv_t *)model->user_data;

    switch (ctx->recv_op) {
    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET:
        esp_ble_mesh_server_model_send_msg(model, ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(srv->state.level), (uint8_t*)&srv->state.level);
        break;

    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET:
    case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK:
        if (set->op_en == false) {
            srv->state.level = set->level;
        } else {
            srv->state.level = set->level;
        }
        if (ctx->recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET) {
            esp_ble_mesh_server_model_send_msg(model, ctx,
                ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS, sizeof(srv->state.level), (uint8_t*)&srv->state.level);
        }
        esp_ble_mesh_model_publish(model, ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_STATUS,
            sizeof(srv->state.level), (uint8_t*)&srv->state.level, ROLE_NODE);
        
        ESP_LOGI(TAG, "Nivel actualizado a: %d", srv->state.level);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    esp_ble_mesh_gen_onoff_srv_t *srv_onoff;
    esp_ble_mesh_gen_level_srv_t *srv_level;

    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04" PRIx32 ", src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x", param->value.state_change.onoff_set.onoff);
            example_change_led_state(param->model, &param->ctx, param->value.state_change.onoff_set.onoff);
        } else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
                   param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK) {
            ESP_LOGI(TAG, "level %d", param->value.state_change.level_set.level);
        }
        break;

    case ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_GET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
            srv_onoff = (esp_ble_mesh_gen_onoff_srv_t *)param->model->user_data;
            ESP_LOGI(TAG, "onoff 0x%02x", srv_onoff->state.onoff);
            example_handle_gen_onoff_msg(param->model, &param->ctx, NULL);
        } else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_GET) {
            srv_level = (esp_ble_mesh_gen_level_srv_t *)param->model->user_data;
            ESP_LOGI(TAG, "level %d", srv_level->state.level);
            example_handle_gen_level_msg(param->model, &param->ctx, NULL);
        }
        break;

    case ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT");
        if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET ||
            param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
            ESP_LOGI(TAG, "onoff 0x%02x, tid 0x%02x", param->value.set.onoff.onoff, param->value.set.onoff.tid);
            example_handle_gen_onoff_msg(param->model, &param->ctx, &param->value.set.onoff);
        } else if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET ||
                   param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK) {
            ESP_LOGI(TAG, "level %d", param->value.set.level.level);
            example_handle_gen_level_msg(param->model, &param->ctx, &param->value.set.level);
        }
        break;

    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;
    
    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");
    return err;
}

void app_main(void)
{
    esp_err_t err; // Variable que almacenará los códigos de error

    ESP_LOGI(TAG, "Initializing..."); // La inicialización del main ha comenzado

    board_init(); // Inicializa el hardware de la placa: pines GPIO, LEDs, botones, etc. 

    // Inicializa la memoria flash no volátil, que se usa para almacenar datos persistentes
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) { // Si la memoria está llena la resetea y vuelve a inicializar
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err); // Detiene el programa si hubo un error

    // Inicializa el Bluetooth del ESP32 (BLE)
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err); // Si hubo error, lo indica
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid); // Genera o obtiene el UUID del dispositivo para BLE Mesh, el cual identifica de forma única el nodo en la red mesh

    // Inicializa la pila de Bluetooth Mesh
    // Esto configura los elementos, modelos y callbacks para que tu nodo pueda enviar y recibir mensajes BLE Mesh
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err); // Si hubo error, lo indica
    }

    // configuracion_adc(); // Llama a la función que configura el ADC para leer el potenciómetro

    i2c_master_init(); // Inicializa el bus I2C para comunicarse con el BMP180
    bmp180_init_sensor(); // Inicializa el sensor BMP180
    vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos antes de continuar

    ESP_LOGI(TAG, "Iniciando envío de lecturas cada 2 segundos..."); // Indica que comenzará a enviar datos del potenciómetro

    while (1) {
        // enviar_valor_potenciometro(); // Llama a la función que lee el potenciómetro, escala el valor y lo publica por BLE Mesh
        enviar_valor_bmp180();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Espera 2 segundos antes de la siguiente lectura
    }
}
