CLIENTE: #include <string.h> // Librería estándar de C
#include "freertos/FreeRTOS.h" // Librería estandar de FreeRTOS
#include "freertos/task.h" // Librería para manejo de tareas en FreeRTOS
#include "esp_wifi.h" // Librería que contiene las funciones de WiFi
#include "esp_log.h" // Librería para el log
#include "esp_event.h" // Librería para manejo de eventos - notificaciones 
#include "esp_system.h" // Permite al cliente reiniciar el ESP32 para recuperarse de errores y reconectarse automáticamente al WiFi y usar otras funciones del sistema
#include "esp_netif.h" // Para inicializar la red y obtener IP al conectarse como cliente WiFi.
#include "nvs_flash.h" // Librería para manejo de la memoria flash
#include "driver/gpio.h" // Librería para control de pines GPIO
#include "lwip/sockets.h" // Librería para manejo de TCP
#include "esp_err.h" // // Códigos de error para verificar resultados de funciones

#define WIFI_SSID "ESP32_AP"      // Nombre de la red WiFi a la que se conectará el ESP32
#define WIFI_PASS "12345678"      // Contraseña del AP del servidor
#define SERVER_IP "192.168.4.1"   // Dirección IP del servidor al que el cliente se conectará vía TCP
#define PORT 3333                 // Puerto TCP usado para la comunicación con el servidor
#define LED_GPIO 2                // Pin del ESP32 que usará el LED

static const char *TAG = "TCP_LED"; // Etiqueta para el log

// El Event Group no ejecuta tareas, sirve para coordinar su orden.
// Una tarea marca un bit cuando termina un evento, y otras tareas esperan ese bit antes de continuar.
static EventGroupHandle_t evento_grupal; // Manejador de grupo de eventos para coordinar tareas usando bits de evento
const int wifi_conectado_bit = BIT0; // Se usa el bit 0, el cual indicará cuando hay conexión wifi, y se llamará wifi_conectado_bit

// Función manejadora de eventos WiFi, que reacciona a los eventos específicos definidos en el handler
// Puntero a datos adicionales que se pueden pasar al handler, en este caso no se necesita
// Indica la categoría del evento, qué tipo de evento se esta manejando (WIFI, IP, por ejemplo)
// Identifica al evento específico dentro de la categoría (una especificación dentro de la categoría)
// Puntero hacia el evento. Permite acceder a datos específicos de ese evento para tomar decisiones
static void wifi_evento_handler(void* arg, esp_event_base_t categoria_evento, int32_t evento_id, void* evento_datos) {
    if (categoria_evento == WIFI_EVENT) { // Si el evento está relacionado con wifi entra aquí

        if (evento_id == WIFI_EVENT_STA_START) { // Si el modo STA del wifi ya fue activado, entra
            esp_wifi_connect(); // Intenta conectar el ESP32 al Acces Point configurado

        } else if (evento_id == WIFI_EVENT_STA_DISCONNECTED) { // Si el ESP32 se desconecta del AP, entra
            ESP_LOGW(TAG, "Desconectado del AP, intentando reconectar..."); // Avisa que se desconectó y se intenta reconectar
            esp_wifi_connect(); // Intenta conectar el ESP32 al Acces Point configurado
        }

    // Entra si: es un evento relacionado con IP, indica que la conexión wifi fué exitosa y el ESP32 cliente recibió su IP (de su "red")    
    } else if (categoria_evento == IP_EVENT && evento_id == IP_EVENT_STA_GOT_IP) {

        // Marca el bit wifi_conectado_bit en el grupo de eventos para indicale a las otras tatreas que la conexión WiFi ya está lista y pueden continuar
        xEventGroupSetBits(evento_grupal, wifi_conectado_bit);

        // Aquí se quiere sacar el IP que el AP le dió al ESP32 cliente
        // evento es un puntero a la estructura completa que contiene varios datos de red, incluyendo la IP
        // // El cast (ip_event_got_ip_t*) interpreta los datos genéricos de evento_datos como esta estructura
        // Cast en este caso le dice: "estos datos genéricos (evento_datos) deben tratarse como si fueran de tipo ip_event_got_ip_t
        ip_event_got_ip_t* evento = (ip_event_got_ip_t*) evento_datos;

        // Mensaje con qué IP se conectó al wifi
        // IPSTR es como la plantilla: %d.%d.%d.%d, donde irán los cuatro números de la IP
        // IP2STR(&evento->ip_info.ip) es el “rescatador”: toma la IP interna del ESP32 y extrae los cuatro octetos para llenar la plantilla
        ESP_LOGI(TAG, "Conectado a WiFi con IP: " IPSTR, IP2STR(&evento->ip_info.ip));
    }
}

// Inicialización wifi en modo STA
void wifi_sta(void) {

    // ESP_ERROR_CHECK devuele si hubo un error o no
    ESP_ERROR_CHECK(nvs_flash_init()); // Inicializa la memoria no volátil del ESP32 para guardar y recuperar datos persistentes, como configuraciones de WiFi.
    ESP_ERROR_CHECK(esp_netif_init()); // Prepara al ESP32 para manejar conexiones de red y comunicación TCP/IP.

    // Activa el “motor de notificaciones” que permite al ESP32 procesar eventos del WiFi y la red mientras el programa corre. 
    ESP_ERROR_CHECK(esp_event_loop_create_default()); 

    esp_netif_create_default_wifi_sta();  // Prepara la red en modo Station (cliente).

    // A la variable cfg se le asigna el tipo de dato específico para guardar toda la configuración inicial del wifi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // A esa variable se le asignan configuraciones internas predeterminadas seguras
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // DEFAULT controla prioridades, espacios de memoria, como manejar los eventos, colas de menajes, etc.
    // Inicializa el hardware y softwaredel ESP32 para que fucnione con wifi

    evento_grupal = xEventGroupCreate(); // Crea el manejador de eventos para sincronizar tareas entre sí mediante eventos

    // Aquí se registran los handlers, es decir, las funciones que se ejecutarán automáticamente cuando ocurran ciertos eventos del sistema
    // esp_event_handler_instance_register asocia el evento con la función
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, // Se manejarán eventos de tipo wifi
                                                        ESP_EVENT_ANY_ID, // Escucha todos los eventos de wifi y ejecutará handler
                                                        &wifi_evento_handler, // Función que se ejecutará cuando ocurra el evento
                                                        NULL, // No se pasan datos adicionales a la función (puntero a datos)
                                                        NULL)); // No es necesario guardar el handler porque tampoco será eliminado

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, // Se manejarán eventos de tipo IP
                                                        IP_EVENT_STA_GOT_IP, // Hanlder solo se ejecutará cuando se consiga el IP del AP
                                                        &wifi_evento_handler, // Función que se ejecutará cuando ocurra el evento
                                                        NULL, // No se pasan datos adicionales a la función (puntero a datos)
                                                        NULL)); // No es necesario guardar el handler porque tampoco será eliminado



    wifi_config_t wifi_config = { // Estructura que guarda la configuración de wifi en el ESP32. Prepara datos para establecer conexión
        .sta = { // Indica que se está configurando en modo Station (cliente)
            .ssid = WIFI_SSID, // Nombre de la red wifi a la que quieres conectarte
            .password = WIFI_PASS, // Contraseña de esa red wifi
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Define que el ESP32 funcionará en modo Station

    // Indica que al modo Station se le asigna la configuración que se definió arriba
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); // Se enciende el software y hardware wifi del ESP32 con las configuraciones establecidas

    ESP_LOGI(TAG, "Iniciando WiFi STA..."); // Indica que el proceso de conexión wifi en modo Station ha comenzado
    
    // Esperar hasta que ciertos eventos de Event Group se realicen (se marquen)
    xEventGroupWaitBits(evento_grupal, wifi_conectado_bit, pdFALSE, pdTRUE, portMAX_DELAY);
    // Event Group en donde están los eventos que deben suceder para continuar
    // El bit que indica que la conexión wifi ya está lista y el ESP32 ya tomó la IP del AP (evento que tiene que marcarse para continuar)
    // Indica si quiero borrar el bit cuando se marque (en este caso no quiero, FALSE)
    // Indica si quiero esperar todos los bits especificados (aunque en este caso solo hay uno, igualmente ponemos TRUE)
    // Tiempo máximo de espera para que este evento suceda (en este caso tiempo indefinido)
}

// Tarea TCP cliente para recibir mensajes y encender el LED
void tcp_tarea_cliente(void *pvParameters) {
    char buffer[16]; // Buffer donde se guardarán los datos recibidos del servidor TCP, con capacidad de 16 bytes
    int socket_esp32; // Variable que guardará el descriptor del socket (canal)
    struct sockaddr_in tunel_server; // Estructura "tunel" que contiene la dirección del servidor TCP. Canal por el que debo "hablarle" al servidor

    gpio_reset_pin(LED_GPIO); // Reinicia la configuración del pin del LED
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); // Define el pin del LED como salida 

    while (1) {
        // En esta variable se guarda el descriptor, es decir, el identificador del "canal" por el que se comunican (dirección del servidor)
        socket_esp32 = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        // Función que crea el socket
        // Se usará IPV4 como protocolo de diecciones
        // Indica que el socket será de tipo stream, es decir, TCP (orientado a conexión, confiable y garantiza que los datos lleguen en orden)
        // Indica el protocolo de internet que se usará (En este caso, IP indica que se usará el establecido arriba: TCP)

        if (socket_esp32 < 0) { // Comprueba si la creación del socket fue exitosa o no
            ESP_LOGE(TAG, "Error creando socket"); // Informa sobre ello
            vTaskDelay(pdMS_TO_TICKS(2000)); // Pausa la tarea por 2000 milisegundos (2 segundos) antes de volver a intentar
            continue; // Sale de esta función
        }

        tunel_server.sin_family = AF_INET; // Indica que el socket solo usará direcciones IPV4

        // htons convierte el número de puerto del formato de la máquina (host) al formato de red (big-endian)
        tunel_server.sin_port = htons(PORT); // Indica el puerto específico por el cual el socket escuchará

        // Esta función convierte la IP de texto a binario porque los sockets solo entienden direcciones en binario
        inet_pton(AF_INET, SERVER_IP, &tunel_server.sin_addr.s_addr);
        // Tipo de dirección (IPV4)
        // Dirección de la IP en formato de texto
        // Puntero a la ubicación donde se guardará la IP convertida en binario

        // Intenta conectar el socket cliente (socket_esp32) a un server (tunel_server)
        if (connect(socket_esp32, (struct sockaddr *)&tunel_server, sizeof(tunel_server)) != 0) { // Si es 0 sigue, si no entra al if
        // Identificador del canal de comunicación que quieres conectar
        // Indica a qué servidor y puerto debe conectarse el socket
        // Tamaño de la dirección para que connect() sepa cuánto leer exactamente para conseguir la dirección completa en bytes.

            ESP_LOGI(TAG, "Error conectando al servidor, reintentando..."); // Informa que la conexión TCP falló
            close(socket_esp32); // Libera los recursos usados por el socket que falló 
            vTaskDelay(pdMS_TO_TICKS(2000)); // Espera 2 segundos antes de volver a intentar la conexión
            continue; // Salta al inicio del while(1).
        }

        ESP_LOGI(TAG, "Conectado al servidor"); // Imprime que la conexión con el servidor fue exitosa

        int len; // Variable para guardar cuántos bytes se reciben en cada lectura del socket
        // recv sirve para recibir datos que el servidor (u otro dispositivo) envía a tu ESP32 a través de un socket y guardalo en buffer
        // Si se recibe 0 quiere decir que la conexión se cerró, si recibe menos de 0 es que hubo un error
        while ((len = recv(socket_esp32, buffer, sizeof(buffer) - 1, 0)) > 0) {
        // Socket desde el cual se quieren recibir los datos (identificador del canal TCP o canal abierto al server)
        // Memoria donde se guardan los bytes recibidos (literalmente lo que mandó el server, caracter por caracter)
        // Cantidad máxima de bytes a leer quitando el final (\0) para evitar datos de más
        // Flags: Operaciones especiales, en este caso no se necesitan

            buffer[len] = 0;  // Se cierra el string con el 0 (que es \0) para completar el mensaje recibido
            if (strcmp(buffer, "ON\n") == 0) {  // Compara dos strings. Si ambos coinciden es 0 y entra
                gpio_set_level(LED_GPIO, 1); // Se enciende el LED
                ESP_LOGI(TAG, "LED encendido!!!"); // El mensaje indica que se enciende el LED en el log
            } else { // Si los strings no son iguales
                gpio_set_level(LED_GPIO, 0); // Se apaga el LED
                ESP_LOGI(TAG, "LED apagado!!!"); // El mensaje indica que se apaga el LED en el log
            }
        }

        // Esto pasa si ya no se reciben bytes del servidor o si los valores que llegan son negativos
        close(socket_esp32); // Se cierra el socket. Libera los recursos usados por el socket
        ESP_LOGI(TAG, "Desconectado del servidor, reintentando..."); // Indica que se deconectó y que volverá a intentar conectarse
        vTaskDelay(pdMS_TO_TICKS(2000)); // Pequeña pausa antes de intentar conectarse en el while anterior
    }
}

void app_main(void) {
    wifi_sta(); // Inicializa el wifi en modo Station (cliente)
    xTaskCreate(tcp_tarea_cliente, "tcp_tarea_cliente", 4096, NULL, 5, NULL); // Crea la tarea principal del cliente
    // Función de la tarea
    // Nombre de la tarea
    // Espacio de memoria en bytes que esa tarea tendrá para ejecutarse correctamente
    // Parámetros de la tarea (no se ocupan aquí)
    // Nivel de prioridad de la tarea 
    // Handle o identificador de la tarea (no se necesita aquí)
}
