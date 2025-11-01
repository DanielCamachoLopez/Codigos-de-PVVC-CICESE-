#include <stdio.h> // Librería estándar de C
#include <string.h> // Librería para manejo de strings
#include "freertos/FreeRTOS.h" // Librería estandar de FreeRTOS
#include "freertos/task.h" // Librería para manejo de tareas en FreeRTOS
#include "esp_wifi.h" // Librería que contiene las funciones de WiFi
#include "esp_event.h" // Librería para manejo de eventos - notificaciones 
#include "esp_log.h" // Librería para el log
#include "nvs_flash.h" // Librería para manejo de la memoria flash
#include "lwip/sockets.h" // Librería para manejo de TCP
#include "driver/gpio.h" // Librería para control de pines GPIO

#define BUTTON_PIN 4 // Asignar pin 4 del ESP32 al botón
#define TCP_PORT 3333 // Puerto al que se tiene que conectar el cliente para establecer conexión
#define MAX_CLIENTS 1 // Cantidad máxima de clientes a los que puede conectarse
#define TAG "TCP_BUTTON" // Etiqueta para los mensajes del log 

// Inicializa el ESP32 como AP (que cree su propia red WIFI), y prepara el ESP32 para que el wifi pueda funcionar
void wifi_ap(void)
{
    // ESP_ERROR_CHECK devuele si hubo un error o no
    ESP_ERROR_CHECK(nvs_flash_init()); // Inicializa la memoria flash (no volátil) del ESP32 para guardar y recuperar datos persistentes, como configuraciones de WiFi.
    ESP_ERROR_CHECK(esp_netif_init()); // Prepara al ESP32 para manejar conexiones de red y comunicación TCP/IP.
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Activa el “motor de notificaciones” que permite al ESP32 procesar eventos del WiFi y la red mientras el programa corre. 

    esp_netif_create_default_wifi_ap(); // Prepara una red para sí misma configurándose como Acces Point

    // A la variable cfg se le asigna el tipo de dato específico para guardar toda la configuración inicial del wifi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // A esa variable se le asignan configuraciones internas predeterminadas seguras
    // DEFAULT controla prioridades, espacios de memoria, como manejar los eventos, colas de menajes, etc.

    // Se inicializa el stack del WIFI para darle al ESP32 el espacio de memoria necesario para procesar eventos y tareas correctamente
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Se le da configuración de "cfg" al ESP32 (la DEFAULT)

    wifi_config_t ap_config = { // Tipo de dato que guarda toda la configuración del servidor y del cliente
        .ap = { // En este caso queremos configurar un AP (Acces Point) para que cree su propia red
            .ssid = "ESP32_AP", // Nombre de la red WIFI 
            .ssid_len = strlen("ESP32_AP"), // El nombre lo guarda caracter por caracter para mostrarlo como red en dispositivos
            .channel = 1, // Canal en el que operará el Acces Point
            .max_connection = MAX_CLIENTS, // Define el número máximo de clientes que pueden conectarse a este AP
            .authmode = WIFI_AUTH_WPA2_PSK, // Tipo de autenticación de red (en este caso WPA2 es con contraseña)
            .password = "12345678" // Contraseña con longitud mínima de 8 caracteres
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP)); // Indica al ESP32 que trabajará en modo Acces Point (red propia)
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config)); // Aplica todas las configuraciones que se definieron en ap_config
    ESP_ERROR_CHECK(esp_wifi_start()); // Arranca el WIFI del ESP32 con el modo y configuración definidos arriba

    ESP_LOGI(TAG, "Red AP creada: SSID=%s", ap_config.ap.ssid); // Mensaje informativo de que se creó la red
}

// Tarea TCP para manejar al cliente y botón
void tcp_tarea_servidor(void *pvParameters)
{
    // Socket permite crear un canal o "tubo" de comunicación entre este ESP32 y otro dispositivo (el otro ESP32)

    // Crea un socket TCP/IP (canal de comunicación) y guarda su descriptor (identificador de recurso) en la variable socket_esp32
    int socket_esp32 = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    // Función que crea el socket
    // Se usará IPV4 como protocolo de riecciones
    // Indica que el socket será de tipo stream, es decir, TCP (orientado a conexión, confiable y garantiza que los datos lleguen en orden)
    // Indica el protocolo de internet que se usará (En este caso, IP indica que se usará el establecido arriba: TCP)

    if (socket_esp32 < 0) { // Comprueba si la creación del socket fue exitosa o no
        ESP_LOGE(TAG, "Error creando socket"); // Informa sobre ello
        vTaskDelete(NULL); // Elimina la tarea
        return; // Sale de esta función
    }

    // Aquí el socket indica por que "tunel" estará escuchando. Si algún dispositivo quiere hablar con él, que se comunique por ese tunel
    struct sockaddr_in tunel_server; // Estructura "tunel" que contendrá la dirección y puerto donde el socket escuchará
    tunel_server.sin_family = AF_INET; // Indica que el socket solo usará direcciones IPV4

    // htons convierte el número de puerto del formato de la máquina (host) al formato de red (big-endian)
    tunel_server.sin_port = htons(TCP_PORT); // Indica el puerto específico por el cual el socket escuchará
    tunel_server.sin_addr.s_addr = htonl(INADDR_ANY); // Indica que escuchará cualquiera de las IPs disponibles del ESP32

    // Intenta asociar el socket creado con una IP y un puerto específico (tunel)
    // Para esto, bind toma los siguientes parámetros:
    // Canal que se creó para llevar a cabo la comunicación (socket). Caparazón vacío.
    // IP y puerto por donde el socket escuchará (configuraciones del tunel). Se llena el caparazón. En sockaddr se guarda configuración.
    // Espacio necesario que ocupa la estructura. Espacio necesario para que quepa bien y se desenvuelva correctamente dentro del caparazón.
    if (bind(socket_esp32, (struct sockaddr *)&tunel_server, sizeof(tunel_server)) < 0) {

        ESP_LOGE(TAG, "Error en bind()"); // Indica en caso de que no se pudiera asociar
        close(socket_esp32); // Cierra el socket previamente creado para liberar recursos del sistema
        vTaskDelete(NULL); // Elimina la tarea actual
        return; // Sale de esta función
    }

    // listen intenta activar el modo servidor TCP en el socket, lo que le permite aceptar conexiones entrantes de clientes
    // listen tiene como parámetros: socket ya asociado a un puerto, número de clientes que puede haber en cola (esperando)
    if (listen(socket_esp32, MAX_CLIENTS) < 0) { // Si devuelve 0 está listo, si es -1 algo falló
        ESP_LOGE(TAG, "Error en listen()"); // En caso de que falle lo informa
        close(socket_esp32); // Cierra el socket previamente creado para liberar recursos del sistema
        vTaskDelete(NULL); // Elimina la tarea actual
        return; // Sale de la función
    }

    ESP_LOGI(TAG, "Servidor TCP escuchando en puerto %d", TCP_PORT); // Informa si el servidor está listo y en qué puerto

    while (1) {
        struct sockaddr_in tunel_client; // Estructura que almacenará la IP y puerto del cliente que se conecte

        // socklen_t es el tipo de dato entero que se usa específicamente para almacenar tamaños de estructuras de direcciones de socket.
        socklen_t tunel_client_len = sizeof(tunel_client); // Guarda el tamaño de la estructura del cliente en bytes

        // client_sock guardará el descriptor, es decir, el identificador de dicha conexión con ese cliente
        // accept es la función que acepta las conexiones entrantes en este servidor TCP
        // Canal de escucha para clientes que quieren conectarse (el socket creado que puse a "escuhcar" en un puerto e IP)
        // La dirección donde accept guardará la información del cliente que se conecte (IP y puerto)
        // Dirección de la variable que indica el tamaño de la estructura de cliente
        int client_socket = accept(socket_esp32, (struct sockaddr *)&tunel_client, &tunel_client_len);
        
        if (client_socket < 0) { // Si el valor que devuelve accept es 0 está correcto, si es -1 entonces algo falló 
            ESP_LOGE(TAG, "Error en accept()"); // Avisa que accept tuvo un error al aceptar al cliente
            continue; // Hace que se salte lo que queda del ciclo y se dirige al inicio de este 
        }

        ESP_LOGI(TAG, "Cliente conectadooooooooooo"); // Si todo ocurrió como debería, el cliente se abrá conectado

        int estado_anterior = 1; // Suponemos botón no presionado
        while (1) {
            int estado_actual = gpio_get_level(BUTTON_PIN); // Lee el estado actual del botón y lo guarda

            // Detectar cambio de estado
            if (estado_actual != estado_anterior) { // Si el botón estaba anteriormente "no presionado", y ahora esta en "presionado", entra
                estado_anterior = estado_actual; // Cambio importante para asegurarse de que el botón deje de presionarse antes de entrar

                if (estado_actual == 0) { // Si esta presionado
                    char msg[] = "ON\n"; // Crea mensaje de encendido
                    send(client_socket, msg, strlen(msg), 0); // Envía ese mensaje al cliente 
                    // send envía el mensaje:
                    // Descriptor del socket que representa la conexión con el cliente. "Tubo por el que se enviará el mensaje"
                    // Tamaño de datos a enviar en bytes
                    // Opciones especiales de envío (como envío urgente, no bloquear, etc), en este caso es 0 porque no los necesitamos

                } else { // Si no esta presionado
                    char msg[] = "OFF\n"; // Crea mensaje de apagado
                    send(client_socket, msg, strlen(msg), 0); // Envía ese mensaje al cliente
                    // send envía el mensaje:
                    // Descriptor del socket que representa la conexión con el cliente. "Tubo por el que se enviará el mensaje"
                    // Tamaño de datos a enviar en bytes
                    // Opciones especiales de envío (como envío urgente, no bloquear, etc), en este caso es 0 porque no los necesitamos

                }
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Pequeña pausa entre lecturas para evitar un descanso
        }

        close(client_socket); // Cierra la conexión TCP con el cliente actual. Libera recursos y deja listo al servidor para nuevas conexiones.
    }
}

void app_main(void)
{
    // Configurar GPIO del botón como input con pull-up
    gpio_config_t boton = {
        .pin_bit_mask = 1ULL << BUTTON_PIN, // Indica los pines del ESP32 a configurar
        .mode = GPIO_MODE_INPUT, // Modo de operación del pin (en este caso, modo entrada)
        .pull_up_en = GPIO_PULLUP_ENABLE, // Se activa la resistencia interna pull up
        .pull_down_en = GPIO_PULLDOWN_DISABLE // Se desactiva la resistencia interna pull down
    };
    gpio_config(&boton); // Esa configuración se aplica al ESP32

    wifi_ap(); // Llama a función que inicializa el ESP32 como Acces Point
    xTaskCreate(tcp_tarea_servidor, "tcp_tarea_servidor", 4096, NULL, 5, NULL);
    // Función de la tarea
    // Nombre de la tarea
    // Espacio de memoria en bytes que esa tarea tendrá para ejecutarse correctamente
    // Parámetros de la tarea (no se ocupan aquí)
    // // Nivel de prioridad de la tarea 
    // Handle o identificador de la tarea (no se necesita aquí)
}
