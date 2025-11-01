// 6. Colas para comunicación - Uso de xQueueCreate, xQueueSend y xQueueReceive.
// Realizar un código con 2 funciones:
// Tarea 1: Al presionar un botón manda un valor (que va incrementando cada que se presiona el botón) por la cola.
// Tarea 2: Cada segundo revisa si hay un mensaje en la cola y si hay imprime el mensaje.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define BUTTON_PIN 4   // Pin donde conectas el botón
static const char *TAG = "COLA_EJEMPLO"; // Etiqueta para logs

QueueHandle_t Queue;       // Tipo de dato que representa la cola
int contador = 0;            // Contador global, cada vez que se presiona el botón se incrementa

// Tarea 1: Lee el botón y envía valor a la cola
void Tarea_boton(void *pvParameters) {
    int led_estado = 1;  // Inicia la variable de estado del botón en no presionado (1)
    while (1) {
        int estado = gpio_get_level(BUTTON_PIN); // Leer estado del botón, 0 = presionado, 1 = no presionado

        // Función que evita que se cuente más de una vez por cada pulsación
        // El botón está en 1 popr defecto y al presionarlo pasa a 0, es decir, detecta el 1 y 0 una sola vez y entra.
        // También evita que cuando se mantenga presionado el botón, siga contando.
        if (led_estado == 1 && estado == 0) { 
            contador++; // El botón se presiona y aumenta el contador

            // Intenta enviar valor del contador, dependiendo del resultado muestra un log u otro
            if (xQueueSend(Queue, &contador, pdMS_TO_TICKS(10)) != pdPASS) {  // Función que envía a la cola el valor del contador
                // Cola a la que se envía el dato, dirección del dato a enviar, tiempo máximo de espera para enviar
                ESP_LOGW(TAG, "No se pudo enviar a la cola"); // Si no se pudo enviar el dato a la cola
            } else {
                ESP_LOGI(TAG, "Valor enviado a la cola: %d", contador); // Si se envió correctamente el dato a la cola
            }
        }
        led_estado = estado;
        vTaskDelay(pdMS_TO_TICKS(50));  // Espera un momento emtre iteraciones
    }
}

// Tarea 2: Recibe valor de la cola y lo imprime
void Tarea_imprimir(void *pvParameters) {
    int valor_recibido; // Esta variable almacenará el valor recibido de la cola
    while (1) {
        if (xQueueReceive(Queue, &valor_recibido, pdMS_TO_TICKS(1)) == pdTRUE) { // Esta función intenta sacar un valor de la cola
            // La cola de la que se recibe el dato, dirección de donde se almacena el dato recibido, tiempo máximo de espera para recibir
            ESP_LOGI(TAG, "Valor recibido de la cola: %d", valor_recibido); // Si se recibió correctamente el dato de la cola
        } else {
            ESP_LOGI(TAG, "No hay mensaje en la cola"); // Si no se recibió ningún dato de la cola en el tiempo especificado
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

void app_main(void) {
    // Configurar botón
    gpio_reset_pin(BUTTON_PIN); // Resetea el pin del botón
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT); // Configura el pin del botón como entrada
    gpio_pullup_en(BUTTON_PIN);  // Configuración pull-up activada. No presionado = 1, presionado = 0   

    // Definir la cola con capacidad para 5 enteros
    Queue = xQueueCreate(5, sizeof(int)); // Capacidad de la cola, tamaño de cada elemento

    // Crear las tareas
    xTaskCreate(Tarea_boton, "Tarea_boton", 2048, NULL, 2, NULL);
    xTaskCreate(Tarea_imprimir, "Tarea_imprimir", 2048, NULL, 1, NULL);

    // Para ambas funciones:
    // Función que utiliza la tarea
    // Nombre de la tarea
    // Memoria asignada a la tarea para ejecutarse correctamente
    // Parámetros de la tarea (NULL si no se usan)
    // Prioridad de la tarea (mayor número = mayor prioridad)
    // Puntero a la tarea (NULL si no se necesita)
}
