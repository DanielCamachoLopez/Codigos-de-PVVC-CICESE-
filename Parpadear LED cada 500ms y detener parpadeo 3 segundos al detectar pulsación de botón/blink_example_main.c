// 2. Uso de las funciones vTaskSuspend y vTaskResume
// Realizar un código tenga una 2 tareas
// Tarea 1: dedicada a hacer parpadear un led cada 500ms
// Tarea 2: dedicada a la deteccion y reaccion ante la pulsasion de un boton. Al presionar el botón debe de suspender la tarea 1 y reanudarla 3s despues.


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED GPIO_NUM_23     // Pin del LED
#define BUTTON GPIO_NUM_4   // Pind el botón

static const char *TAG = "SUSPENDE_RESUME"; // Etiqueta para el log
TaskHandle_t controlador_tarea = NULL; // Necesario para suspenderla o reandarla
// Inicialmente no apuntamos a ninguna tarea, solo se define el "handle"para usarlo después

// Tarea 1: LED parpadea cada 500ms
void Tarea_led(void *pvParameters) { // Creamos tarea del LED
    int led_estado = 0; // Estado inicial del LED
    while (1) {
        led_estado = !led_estado; // Cambia el estado del LED
        gpio_set_level(LED, led_estado); // Actualiza el pin del LED
        vTaskDelay(pdMS_TO_TICKS(500)); // Cambia 500 microsegundos por tiempo en ticks
    }
}

// Tarea 2: Detecta botón y suspende/reanuda
void Tarea_boton(void *pvParameters) { // Creamos tarea del botón
    while (1) {
        if (gpio_get_level(BUTTON) == 0) { // Lee el estado de un GPIO, en este caso el botón (pulsado = 0, suelto = 1)
            ESP_LOGI(TAG, "Botón presionado... Suspendiendo LED"); // Indica que el botón se detectó
            vTaskSuspend(controlador_tarea); // Suspende la tarea del LED temporalemente 
            vTaskDelay(pdMS_TO_TICKS(3000)); // Espera 3 segundos (3000 ms convertidos a ticks)
            ESP_LOGI(TAG, "Reanudando LED"); // Avisa que se reanudará la tarea del LED
            vTaskResume(controlador_tarea); // Reanuda la tarea del LED
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Un poco de tiempo antes de inicar con el siguiente ciclo
    }
}

void app_main(void) {
    gpio_reset_pin(LED); // Configura el pin del LED por default
    gpio_set_direction(LED, GPIO_MODE_OUTPUT); // Configura el pin del LED como salida

    gpio_reset_pin(BUTTON); // Configura el pin del botón por default
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT); // Configura el pin del botón como entrada
    gpio_pullup_en(BUTTON); // Habilita la configuración pull-up interna del pin del botón

    xTaskCreate(Tarea_led, "LED", 1024, NULL, 1, &controlador_tarea); // Crear tarea
    // Funcion que ejecutará la tarea
    // Nombre de la tarea
    // Memoria asignada a la tarea para ejecutarse correctamente
    // Parámetros que se le pasan a la tarea (si no hay, NULL)
    // Prioridad de la tarea (0 es la más baja, 23 la más alta)
    // Puntero a la tarea

    xTaskCreate(Tarea_boton, "Button", 2048, NULL, 2, NULL);
    // Todos los parámetros son iguales a la tarea del LED, excepto:
    // Memoria asignada (mayor, porque debe de detectar el botón constantemente)
    // Prioridad (mayor, para que pueda detectar el botón incluso si el LED está parpadeando)
    // Puntero a la tarea (no es necesario, ya que no se va a suspender o reanudar esta tarea)
}
