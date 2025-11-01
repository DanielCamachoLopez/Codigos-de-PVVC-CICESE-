#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP180_DEVICE_ADDRESS 0x77

// Estructura para definir los modos de operación del BMP180
typedef enum {
    BMP180_MODE_ULTRA_LOW_POWER = 0,
    BMP180_MODE_STANDARD = 1,
    BMP180_MODE_HIGH_RESOLUTION = 2,
    BMP180_MODE_ULTRA_HIGH_RESOLUTION = 3
} bmp180_mode_t;

// Estructura para alojar los datos de calibración del BMP180
typedef struct {
    int16_t ac1, ac2, ac3;
    uint16_t ac4, ac5, ac6;
    int16_t b1, b2;
    int16_t mb, mc, md;
    int32_t b5;
} bmp180_calibration_data_t;

// Estructura para definir el descriptor
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    bmp180_calibration_data_t cal_data;
} bmp180_dev_t;

/**
 * @brief Initialize BMP180 descriptor
 */
esp_err_t bmp180_init_desc(bmp180_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Initialize BMP180 sensor
 */
esp_err_t bmp180_init(bmp180_dev_t *dev);

/**
 * @brief Read temperature and pressure from BMP180
 */
esp_err_t bmp180_measure(bmp180_dev_t *dev, float *temperature, uint32_t *pressure, bmp180_mode_t oss);

#ifdef __cplusplus
}
#endif

#endif /* BMP180_H */