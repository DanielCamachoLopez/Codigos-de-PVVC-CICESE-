// 1. Uso de las funciones xTaskCreate y xTaskCreatePinnedToCore.
// Realizar un código que mantenga encienda-apague un led cada 500ms y encienda-apague un led cada 200ms. 
// Para este caso no se debe de usar el timer.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Definición de pines
#define LED1 GPIO_NUM_2   // Se asigna al LED interno del ESP32
#define LED2 GPIO_NUM_4   // Se asigna a LED externo conectado al pin GPIO4

void Tarea_led1(void *pvParameters) { // Tarea de FreeRTOS que se le asigna al LED 1
    int led_estado = 0;  // Variable para guardar el estado del LED
    while (1) { // Mantener esta tarea en un bucle infinito
        led_estado = !led_estado;              // Primero cambia estado de la variable. 0 = apagado, 1 = encendido
        gpio_set_level(LED1, led_estado);    // Luego se aplica el estado al pin
        vTaskDelay(pdMS_TO_TICKS(500));    // Convierte 500 milisegundos en ticks y espera esa cantidad de tiempo
    }
}

// Tarea para LED2 (200ms)
void Tarea_led2(void *tarea_parametros) { // Tarea de FreeRTOS que se le asigna al LED 2
    int led_estado = 0; 
    while (1) {
        led_estado = !led_estado;
        gpio_set_level(LED2, led_estado); // Se aplica el estado al pin
        vTaskDelay(pdMS_TO_TICKS(200)); // Convierte 200 milisegundos en ticks y espera esa cantidad de tiempo
    }
}

void app_main(void) {

    gpio_reset_pin(LED1); // Resetea el pin a su estado por defecto
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT); // Configura el pin como salida

    gpio_reset_pin(LED2); // Resetea el pin a su estado por defecto
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT); // Configura el pin como salida

    // Crear tareas en núcleos separados
    xTaskCreatePinnedToCore(Tarea_led1, "LED1", 1024, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(Tarea_led2, "LED2", 1024, NULL, 1, NULL, 1);
    // Tarea a realizar
    // Nombre de tarea
    // Memria asignada a la tarea para ejecutarse correctamente
    // Parámetro que recibe la tarea (NULL si no se usa)
    // Prioridad de la tarea
    // Handle de la tarea (NULL si no se usa)
    // Núcleo en el que se ejecuta la tarea (0 o 1 porque solo tiene 2 núcleos)
    }
