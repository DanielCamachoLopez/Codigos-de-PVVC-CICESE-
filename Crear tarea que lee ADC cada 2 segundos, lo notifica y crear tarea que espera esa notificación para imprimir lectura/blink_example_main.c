// 4. Notificaciones entre tareas - Uso de xTaskNotifyGive y ulTaskNotifyTake.
// Realizar con código con las funciones:
// Tarea 1 (ADC): Lee el ADC cada segundo y envia una notificación cada que lee el ADC. (Usar Timer)
// Tarea 2 (Procesador): Espera la notificación e imprime el valor leído del ADC.


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define POT_PIN      ADC1_CHANNEL_6   // Pin del potenciómetro - GPIO34
#define LED_PIN      4                // Pin del LED
#define LEDC_CHANNEL LEDC_CHANNEL_0   // Selección del canal interno para señal PWM (0-7 espacios disponibles) 
#define LEDC_TIMER   LEDC_TIMER_0     // Selecciona el timer interno que usará la señal PWM (0-3 espacios disponibles). Configuración. 
#define LEDC_MODE    LEDC_LOW_SPEED_MODE // Selecciona el modo de velocidad (Alta o baja) 
#define LEDC_RES     LEDC_TIMER_8_BIT // Define los niveles de brillo del LED (1-20 bits). En este caso se usaron 8 bits (0-255)
#define TAG          "NOTIFICACION" // Etiqueta para mensajes en consola

TaskHandle_t controlador_tarea = NULL;

// Configuración ADC1
static void configuracion_adc() {
    // Función para resolución del ADC, dentro recibe ese dato.
    adc1_config_width(ADC_WIDTH_BIT_12); // Define la precisión del potenciómetro (4096 posiciones). Resolución de voltaje a valor digital.

    adc1_config_channel_atten(POT_PIN, ADC_ATTEN_DB_11); // (Pin del potenciómetro, rango máximo de voltaje que puede medir)
}   // Atenuación, reducir amplitud de señal, se expresa en decibeles

// Configuración LED PWM
static void configuracion_led() {
    ledc_timer_config_t parametros_pwm = { // Estructura con los parámetros para configurar el PWM
        .duty_resolution = LEDC_RES,       // Resolución del PWM
        .freq_hz = 500,                    // Frecuencia del PWM
        .speed_mode = LEDC_MODE,           // Modo de velocidad
        .timer_num = LEDC_TIMER,           // Selecciona qué timer interno (0-3) usará este canal PWM
    };
    ledc_timer_config(&parametros_pwm); // Se guarda configuración del PWM en el ESP32

    ledc_channel_config_t canal_pwm = { // Estructura con los parámetros para configurar el canal PWM
        .channel = LEDC_CHANNEL,        // Selecciona el canal interno que usará el PWM (0-7)
        .duty = 0,                      // LED empieza apagado (ciclo de trabajo inicial 0). Va de 0 a 255 (8 bits). 
        .gpio_num = LED_PIN,            // Pin físico por donde sale la señal PWM
        .speed_mode = LEDC_MODE,        // Modo de velocidad del PWM
        .timer_sel = LEDC_TIMER,        // Selecciona qué timer interno (0-3) usará este canal PWM
    };
    ledc_channel_config(&canal_pwm);    // Se guarda configuración del canal PWM en el ESP32
}

// Variables compartidas
static int lectura_pot = 0; // Guarda la lectura del potenciómetro (0-4095)
static int tiempo_encendido; // Representación del tiempo que el LED se mantiene encendido (0-255)

// Se ejecuta cada vez que la función del timer termina su periodo (callback)
static void TimerCallback(TimerHandle_t xTimer) { // Los parámetros no se usan pero no se pueden eliminar
    lectura_pot = adc1_get_raw(POT_PIN); // Convierte señal analógica a digital (0-4095)
    if (controlador_tarea != NULL) { // Si la tarea está creada (es decir, no es NULL) entra al if
        xTaskNotifyGive(controlador_tarea); // Notificar a la tarea que corresponde al handle
    }
}

// Tarea procesadora: espera notificación, procesa lectura e imprime valor
void Tarea_procesadora(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // La tarea se bloquea hasta recibir notificación
        int tiempo_encendido = (lectura_pot * 255) / 4095;   // Convertir la lectura del potenciómetro a un valor entre 0 y 255
        // duty = (Lectura del potenciómetro * valor máximo del LED) / valor máximo del potenciómetro
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, tiempo_encendido); // Cambia el valor interno dependiendo del cálculo de tiempo encendido.
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL); // Aplica los cambios internos en el hardware (Se actualiza)
        ESP_LOGI(TAG, "ADC: %d = LED duty: %d", lectura_pot, tiempo_encendido); // Imprime en la consola el valor del ADC y el duty
    }
}

void app_main(void) {
    configuracion_adc(); // Función para configurar el ADC
    configuracion_led(); // Función para configurar el LED con PWM

    // Crear tarea procesadora
    xTaskCreate(Tarea_procesadora, "Procesador", 2048, NULL, 2, &controlador_tarea);
    // Función que usa la tarea
    // Nombre de la tarea
    // Mmemoria asignada a la tarea para ejecutarse correctamente
    // Parámetros de entrada (no se usan, por eso es NULL)
    // Prioridad de la tarea
    // Dirección del handle de la tarea (para notificaciones). Handle puede suspender, reanudar, eliminar tarea principal.

    // Crear timer de 1 segundo
    TimerHandle_t timer = xTimerCreate("TimerADC", pdMS_TO_TICKS(1000), pdTRUE, NULL, TimerCallback);
    // Nombre del timer
    // Periodo del timer (pdMS_TO_TICKS convierte ms a ticks)
    // pdTRUE = el timer se repite, pdFALSE = el timer es de un solo disparo
    // Parámetro de entrada (no se usa, por eso es NULL)
    // Función callback que se ejecuta cuando el timer termina su periodo
    xTimerStart(timer, 0); // Inicia el timer (0 = no espera)
}
