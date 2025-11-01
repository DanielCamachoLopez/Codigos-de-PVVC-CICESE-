#include "freertos/FreeRTOS.h" // Libreria principal para FreeRTOS
#include "freertos/task.h" // Permite crear y controlar tareas
#include "driver/gpio.h" // Librería para manejar pines de entrada y salida

#define LED_GPIO  GPIO_NUM_23 // El LED se conecta al GPIO23
#define BTN_GPIO  GPIO_NUM_18 // El botón se conecta al GPIO18

void app_main(void) {
    // LED como salida
    gpio_reset_pin(LED_GPIO); // Elimina cponfiguraciones previas (Lo regresa a su estado por defecto))
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); // Configura el pin del LED (GPIO23) como salida

    // Botón como entrada con pull-up interno
    gpio_config_t btn_config = {  // Estructura para configurar el pin del botón
        // gpio_reset_pin(BTN_GPIO);
        // gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);
        .pin_bit_mask = (1ULL << BTN_GPIO), // Unsigned Long Long. Sirve asignar varias configuraciones a la vez. ESP32 tiene mas de 32.
        .mode = GPIO_MODE_INPUT, // Configura ese pin como entrada digital, es decir, no mandará voltaje, solo 1s y 0s.
        .pull_up_en = GPIO_PULLUP_ENABLE,     // Resistencia pull-up interna activada. Garantiza que sin presionar el botón este en 1.
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Resistencia pull-down interna desactivada.
        .intr_type = GPIO_INTR_DISABLE // Desactiva interrupciones en ese pin para que no se ejecute ninguna función cuando cambie estado.
    };
    gpio_config(&btn_config); // Se guarda la configuración del botón en el ESP32

    while (1) {
        int estado = gpio_get_level(BTN_GPIO); // Lee el estado lógico del botón. 1 = no presionado, 0 = presionado

        if (estado == 0) { // Si el botón está presionado
            gpio_set_level(LED_GPIO, 1);   // presionado = LED ON
        } else { // Si el botón no está presionado
            gpio_set_level(LED_GPIO, 0);   // no presionado = LED OFF
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Convierte milisegundos a ticks y espera ese tiempo antes de la siguiente lectura
    }
}
