// 3. Uso de la función vTaskDelete
// Realizar un código con las siguientes 2 tareas
// Tarea 1: Hace parpadear un led 5 veces (una vez cada 100ms) y después se elimina a si misma.
// Tarea 2 (Tarea principal): Crea la tarea 1 cada 2 segundos.


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED GPIO_NUM_2   // Usaremos el LED integrado en GPIO2

static const char *TAG = "ELIMINADA_CREADA"; // Etiqueta para el Log

// Tarea 1: parpadea 5 veces y se elimina a sí misma
void Tarea_led_parpadear(void *pvParameters) {
    for (int i = 0; i < 5; i++) { // Ciclo que solo se repite 5 veces
        gpio_set_level(LED, 1);               // Encender LED
        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar 100 ms (convertidos en ticks)
        gpio_set_level(LED, 0);               // Apagar LED
        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar 100 ms (convertidos en ticks)
        ESP_LOGI(TAG, "Parpadeo %d/5", i + 1); // Log del parpadeo actual
    }
    ESP_LOGI(TAG, "Tarea de parpadeo completada. Eliminándose..."); // Avisa que el ciclo se repitió 5 veces y se elñiminará
    vTaskDelete(NULL); // Se elimina a sí misma
}

// Tarea 2: Tarea principal que crea la tarea 1 cada 2 segundos
void creacion_tarea(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Creando nueva tarea de parpadeo..."); // Indica que está a punto de crear una nueva tarea
        xTaskCreate(Tarea_led_parpadear, "Parpadeo", 1024, NULL, 1, NULL); // 
        // La función que ejecutará la tarea
        // Nombre de la tarea
        // Memoria asignada a la tarea para ejecutarse correctamente
        // Parámetros que se le pasan a la tarea (NULL en este caso)
        // Prioridad de la tarea (1 en este caso)
        // Puntero para almacenar el handle de la tarea (NULL si no se necesita)
        vTaskDelay(pdMS_TO_TICKS(2000)); // Esperar 2000 ms (convertidos en ticks)
    }
}

void app_main(void) {
    // Configurar LED como salidas
    gpio_reset_pin(LED); // Pin del LED a su estado por defecto
    gpio_set_direction(LED, GPIO_MODE_OUTPUT); // Configurar pin como salida

    // Crear la tarea principal
    xTaskCreate(creacion_tarea, "Principal", 2048, NULL, 1, NULL);
}
