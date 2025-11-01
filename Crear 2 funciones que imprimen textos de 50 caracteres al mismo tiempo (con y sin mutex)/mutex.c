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
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "CASO2"; // Etiqueta para logs

// Texto de cada tarea
const char texto1[] = "TAREA 1: Mensaje largo de 50 caracteres con MUTEX!";
const char texto2[] = "TAREA 2: Otro mensaje de 50 caracteres con MUTEX!!";

// Definición del mutex
SemaphoreHandle_t mutex; 
// Mutex es como un semáforo que permite que solo una tarea acceda a un recurso compartido a la vez. Es un handle en sí mismo

// Tarea 1
void Tarea1(void *pvParameters) {
    while (1) {
        // Con mutex, solo una tarea puede entrar a la sección a la vez
        // xSemaphoreTake trata de tomar el mutex, esperando indefinidamente si está ocupado
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) { // Tomar mutex
            // hanlde del mutex, tiempo indefinido para entrar. Si lo toma, entra y cierra el acceso
            ESP_LOGI(TAG, "%s", texto1); // iMprime el texto 1 en la consola
            xSemaphoreGive(mutex); // Libera el mutex para que otro pueda usarlo
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Retardo de 1 segundo (convertido a ticks)
    }
}

// Tarea 2
void Tarea2(void *pvParameters) {
    // Se hace exactamente lo mismo que en la tarea 1 pero con el texto 2
    while (1) {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) { // Tomar mutex
            ESP_LOGI(TAG, "%s", texto2);
            xSemaphoreGive(mutex); // Liberar mutex
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    // Implementación del mutex
    mutex = xSemaphoreCreateMutex(); // Se crea el mutex

    // Crear las tareas
    xTaskCreate(Tarea1, "Tarea1", 2048, NULL, 1, NULL);
    xTaskCreate(Tarea2, "Tarea2", 2048, NULL, 1, NULL);

    // Para ambas funciones:
    // - Tarea1 y Tarea2 son los nombres de las funciones que implementan las tareas
    // - "Tarea1" y "Tarea2" son los nombres descriptivos de las tareas
    // - 2048 es la memoria asignada a la tarea para ejecutarse correctamente
    // - NULL indica que no se pasan parámetros a las tareas      
    // - 1 es la prioridad de las tareas (ambas tienen la misma prioridad)
    // - NULL indica que no se necesita un handle para las tareas
}
