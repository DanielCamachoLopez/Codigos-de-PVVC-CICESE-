// 5. Mutex - Uso de xSemaphoreCreateMutex, xSemaphoreTake y xSemaphoreGive.
// Realizar un código con 2 funciones.
// Caso 1 
// Ambas funciones deben de imprimir un texto de 50 caracteres (texto diferente para cada tarea).
// Las funciones deben de iniciar al mismo tiempo e imprimir el texto cada segundo.
// Caso 2
// Hacer las mismas funciones que en el caso 1 pero integrando el mutex.



#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "CASO1"; // Etiqueta para logs

// Texto de cada tarea (50 caracteres)
const char texto1[] = "TAREA 1: Este es un mensaje largo de 50 caracteres";
const char texto2[] = "TAREA 2: Este es otro mensaje de 50 caracteres!!!!";

// Tarea 1
void Tarea1(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "%s", texto1); // Imprime el texto 1 en la consola
        vTaskDelay(pdMS_TO_TICKS(1000)); // Retardo de 1 segundo (convertido a ticks)
    }
}

// Tarea 2
void Tarea2(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "%s", texto2); // Imprime el texto 2 en la consola
        vTaskDelay(pdMS_TO_TICKS(1000)); // Retardo de 1 segundo (convertido a ticks)
    }
}

void app_main(void) {
    // Crear las tareas
    xTaskCreate(Tarea1, "Tarea1", 2048, NULL, 1, NULL); 
    xTaskCreate(Tarea2, "Tarea2", 2048, NULL, 1, NULL);

    //Para ambas fucniones:
    // - Tarea1 y Tarea2 son los nombres de las funciones que implementan las tareas
    // - "Tarea1" y "Tarea2" son los nombres descriptivos de las tareas
    // - 2048 es la memoria asignada a la tarea para ejecutarse correctamente
    // - NULL indica que no se pasan parámetros a las tareas
    // - 1 es la prioridad de las tareas (ambas tienen la misma prioridad)
    // - NULL indica que no se necesita un handle para las tareas

}
