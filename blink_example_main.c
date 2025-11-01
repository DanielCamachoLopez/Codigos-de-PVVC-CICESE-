#include <stdio.h> // Librería estándar de entrada/salida
#include "freertos/FreeRTOS.h" // Libreria principal para FreeRTOS
#include "freertos/task.h" // Permite crear y controlar tareas
#include "driver/gpio.h" // Librería para manejar pines de entrada y salida

#include "driver/adc.h" // Libreria para usar el ADC 
#include "driver/ledc.h" // LED Controller, permite controlar el LED con PWM
#include "esp_log.h" // Permite imprimir mensajes en la consola
#include "esp_timer.h" // Librerpia para crear timers



#define POT_PIN      ADC1_CHANNEL_6   // D34: Pin donde esta el potenciómetro (ADC1_6)
#define LED_PIN      4                // D4: Pin donde esta el LED y de donde sale la señal PWM
#define LEDC_CHANNEL LEDC_CHANNEL_0  // Selección del canal interno para señal PWM (0-7 espacios disponibles) 
#define LEDC_TIMER   LEDC_TIMER_0 // Selecciona el timer interno que usará la señal PWM (0-3 espacios disponibles). Configuración. 
#define LEDC_MODE    LEDC_LOW_SPEED_MODE // Selecciona el modo de velocidad (Alta o baja)
#define LEDC_RES     LEDC_TIMER_8_BIT // Define los niveles de brillo del LED (1-20 bits). En este caso se usaron 8 bits (0-255)
#define TAG          "POT_LED"


// Configuración del ADC1
static void configuracion_adc() {
    // Función para resolución del ADC, dentro recibe ese dato.
    adc1_config_width(ADC_WIDTH_BIT_12); // Define la precisión del potenciómetro (4096 posiciones). Resolución de voltaje a valor digital.

    adc1_config_channel_atten(POT_PIN, ADC_ATTEN_DB_11); // (Pin del potenciómetro, rango máximo de voltaje que puede medir)
}   // Atenuación, reducir amplitud de señal, se expresa en decibeles

// Configuración del LED con PWM
static void configuracion_led(){
    ledc_timer_config_t parametros_pwm = {  // Estructura con los parámetros para configurar el PWM
        .duty_resolution = LEDC_RES,        // Resolución del PWM
        .freq_hz = 500,                     // Frecuencia del PWM
        .speed_mode = LEDC_MODE,            // Modo de velocidad
        .timer_num = LEDC_TIMER,            // Selecciona qué timer interno (0-3) usará este canal PWM
    };
    ledc_timer_config(&parametros_pwm); // Se guarda configuración del PWM en el ESP32

    ledc_channel_config_t canal_PWM = {    // Estructura con los parámetros para configurar el canal PWM
        .channel    = LEDC_CHANNEL,        // Selecciona el canal interno que usará el PWM (0-7)
        .duty       = 0,                   // LED empieza apagado (ciclo de trabajo inicial 0). Va de 0 a 255 (8 bits). 
        .gpio_num   = LED_PIN,             // Pin físico por donde sale la señal PWM
        .speed_mode = LEDC_MODE,           // Modo de velocidad del PWM
        .timer_sel  = LEDC_TIMER,          // Selecciona qué timer interno (0-3) usará este canal PWM
    };
    ledc_channel_config(&canal_PWM);       // Se guarda configuración del canal PWM en el ESP32
}


static int lectura_pot; // Guarda la lectura del potenciómetro (0-4095)
static int tiempo_encendido; // Representación del tiempo que el LED se mantiene encendido (0-255)



// Esta función se ejecuta cada vez que el timer termina su period (callback)
static void repetidor_timer(void* arg) {
    lectura_pot = adc1_get_raw(POT_PIN); // Lee el valor del potenciómetro (0-4095)
    tiempo_encendido = (lectura_pot * 255) / 4095; // Se convierte la lectura del potenciómetro a un valor entre 0 y 255
    // tiempo_encendido = (Lectura del potenciómetro * valor máximo del LED) / valor máximo del potenciómetro

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, tiempo_encendido); // Cambia el valor interno dependiendo del cálculo de tiempo encendido.
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL); // Aplica los cambios internos en el hardware (Se actualiza)

    ESP_LOGI("POT_LED", "Potenciometro: %d = LED duty: %d", lectura_pot, tiempo_encendido);
}

void app_main(void) {
    configuracion_adc(); // Función para configurar el ADC
    configuracion_led(); // Función para configurar el LED con PWM

    const esp_timer_create_args_t timer_blink = { // Crear estructura para crear temporizador (timer)
        .callback = repetidor_timer, // Callback manda a llamar a la función una vez que el timer se termina
    };

    esp_timer_handle_t periodic_timer; // Llave que permite iniciar y detener el timer
    esp_timer_create(&timer_blink, &periodic_timer); // Crea el timer con la estructura creada y la "llave" del timer
    esp_timer_start_periodic(periodic_timer, 50000); // Arranca timer con duración de 50000 microsegundos
}