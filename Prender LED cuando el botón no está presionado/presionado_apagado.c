#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_GPIO  GPIO_NUM_23
#define BTN_GPIO  GPIO_NUM_18

void app_main(void) {
    // LED como salida
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
 
    // Botón como entrada con pull-up interno
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,     // pull-up activado
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_config);

    while (1) {
        int estado = gpio_get_level(BTN_GPIO); // // Lee el estado lógico del botón. 1 = presionado, 0 = no presionado

        if (estado == 1) { // Si el botón está presionado
            gpio_set_level(LED_GPIO, 0);   // LED OFF
        } else { // Si el botón no está presionado
            gpio_set_level(LED_GPIO, 1);   // LED ON
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Convierte milisegundos a ticks y espera ese tiempo antes de la siguiente lectura
    }
}