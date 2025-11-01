#include "bmp180.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMP180";

// BMP180 registers
#define BMP180_REG_AC1       0xAA
#define BMP180_REG_CONTROL   0xF4
#define BMP180_REG_DATA      0xF6

// BMP180 commands
#define BMP180_CMD_TEMPERATURE 0x2E
#define BMP180_CMD_PRESSURE    0x34

static esp_err_t bmp180_read_calibration_data(bmp180_dev_t *dev) {
    uint8_t data[22];
    
    // Read all calibration registers
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP180_REG_AC1, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 22, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert calibration data
    dev->cal_data.ac1 = (data[0] << 8) | data[1];
    dev->cal_data.ac2 = (data[2] << 8) | data[3];
    dev->cal_data.ac3 = (data[4] << 8) | data[5];
    dev->cal_data.ac4 = (data[6] << 8) | data[7];
    dev->cal_data.ac5 = (data[8] << 8) | data[9];
    dev->cal_data.ac6 = (data[10] << 8) | data[11];
    dev->cal_data.b1 = (data[12] << 8) | data[13];
    dev->cal_data.b2 = (data[14] << 8) | data[15];
    dev->cal_data.mb = (data[16] << 8) | data[17];
    dev->cal_data.mc = (data[18] << 8) | data[19];
    dev->cal_data.md = (data[20] << 8) | data[21];
    
    return ESP_OK;
}

// Funcipon para leer temperatura cruda
static esp_err_t bmp180_read_raw_temperature(bmp180_dev_t *dev, int32_t *raw_temp) {
    // Inicia lectura de temperatura
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP180_REG_CONTROL, true);
    i2c_master_write_byte(cmd, BMP180_CMD_TEMPERATURE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for conversion (4.5ms)
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Read temperature
    uint8_t data[2];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP180_REG_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    *raw_temp = (data[0] << 8) | data[1];
    return ESP_OK;
}

// Inicialización de la estructura del sensor
esp_err_t bmp180_init_desc(bmp180_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
    dev->i2c_port = port;
    dev->i2c_addr = BMP180_DEVICE_ADDRESS;
    return ESP_OK;
}

// Inicialización completa del sensor
esp_err_t bmp180_init(bmp180_dev_t *dev) {
    // Read calibration data
    esp_err_t ret = bmp180_read_calibration_data(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // // Mostrar datos de calibración en logs
    ESP_LOGI(TAG, "=== CALIBRATION DATA ===");
    ESP_LOGI(TAG, "AC1: %d", dev->cal_data.ac1);
    ESP_LOGI(TAG, "AC2: %d", dev->cal_data.ac2);
    ESP_LOGI(TAG, "AC3: %d", dev->cal_data.ac3);
    ESP_LOGI(TAG, "AC4: %u", dev->cal_data.ac4);
    ESP_LOGI(TAG, "AC5: %u", dev->cal_data.ac5);
    ESP_LOGI(TAG, "AC6: %u", dev->cal_data.ac6);
    ESP_LOGI(TAG, "B1: %d", dev->cal_data.b1);
    ESP_LOGI(TAG, "B2: %d", dev->cal_data.b2);
    ESP_LOGI(TAG, "MB: %d", dev->cal_data.mb);
    ESP_LOGI(TAG, "MC: %d", dev->cal_data.mc);
    ESP_LOGI(TAG, "MD: %d", dev->cal_data.md);
    
    ESP_LOGI(TAG, "BMP180 initialized successfully");
    return ESP_OK;
}

// Función para leer presión cruda
static esp_err_t bmp180_read_raw_pressure(bmp180_dev_t *dev, int32_t *raw_pressure, bmp180_mode_t oss) {
    uint8_t pressure_cmd = BMP180_CMD_PRESSURE + (oss << 6);
    
    // Inicia lectura de presión
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP180_REG_CONTROL, true);
    i2c_master_write_byte(cmd, pressure_cmd, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for conversion (based on oversampling)
    uint32_t delay_ms;
    switch (oss) {
        case BMP180_MODE_ULTRA_LOW_POWER: delay_ms = 5; break;
        case BMP180_MODE_STANDARD: delay_ms = 8; break;
        case BMP180_MODE_HIGH_RESOLUTION: delay_ms = 14; break;
        case BMP180_MODE_ULTRA_HIGH_RESOLUTION: delay_ms = 26; break;
        default: delay_ms = 8;
    }
    vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    
    // Leer presión cruda
    uint8_t data[3];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BMP180_REG_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(dev->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    *raw_pressure = ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - oss);
    return ESP_OK;
}

esp_err_t bmp180_measure(bmp180_dev_t *dev, float *temperature, uint32_t *pressure, bmp180_mode_t oss) {
    int32_t raw_temp, raw_pressure;
    
    // Leer temperatura
    esp_err_t ret = bmp180_read_raw_temperature(dev, &raw_temp);
    if (ret != ESP_OK) return ret;
    
    // Leer presión
    ret = bmp180_read_raw_pressure(dev, &raw_pressure, oss);
    if (ret != ESP_OK) return ret;
    
    // Calcular temperatura
    int32_t ut = raw_temp;
    int32_t x1, x2, b5;
    
    x1 = (ut - dev->cal_data.ac6) * dev->cal_data.ac5 >> 15;
    x2 = (dev->cal_data.mc << 11) / (x1 + dev->cal_data.md);
    b5 = x1 + x2;
    
    float temp_calculada = (b5 + 8) / 160.0;
    
    float OFFSET_TEMPERATURA = -7.1f;
    *temperature = temp_calculada + OFFSET_TEMPERATURA;
    
    // Calcular presión real
    int32_t up = raw_pressure;
    int32_t b6 = b5 - 4000;
    
    // Calculate B3
    x1 = (dev->cal_data.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (dev->cal_data.ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)dev->cal_data.ac1 * 4 + x3) << oss) + 2) >> 2;
    
    // Calculate B4
    x1 = (dev->cal_data.ac3 * b6) >> 13;
    x2 = (dev->cal_data.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)dev->cal_data.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    
    uint32_t b7 = ((uint32_t)(up - b3) * (50000 >> oss));
    
    int32_t p;
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }
    
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);
    
    static bool primera_lectura = true;
    static int32_t OFFSET_PRESION = 0;
    
    if (primera_lectura) {
        
        int32_t PRESION_OBJETIVO = 101400; 
        OFFSET_PRESION = PRESION_OBJETIVO - p;
        primera_lectura = false;
        ESP_LOGW(TAG, "Offset presión calculado: +%ld hPa", OFFSET_PRESION/100);
    }
    
    // Aplicar offset fijo a todas las lecturas
    p = p + OFFSET_PRESION;
    
    *pressure = (uint32_t)p;
    
    ESP_LOGI(TAG, "Temp: %.1f°C, Presión: %lu Pa (%.1f hPa)", 
             *temperature, *pressure, *pressure / 100.0);
    
    return ESP_OK;
}