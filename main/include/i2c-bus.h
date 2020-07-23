#ifndef INCLUDE_I2C_BUS_H_
#define INCLUDE_I2C_BUS_H_

#include <inttypes.h>
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//! <I2C configuration structure
typedef struct {
    i2c_port_t port;
    i2c_config_t cfg;
    uint8_t address;
} i2c_cfg;

//! <I2C device structure with callbacks
typedef struct {
    esp_err_t(*eep_write16)(i2c_cfg *cfg, uint16_t reg, uint8_t *data, size_t sz);
    esp_err_t(*eep_read16)(i2c_cfg *cfg, uint16_t reg, uint8_t *data, size_t sz);
    esp_err_t(*bmp_read8)(i2c_cfg *cfg, uint8_t reg, uint8_t *data, size_t sz);
    esp_err_t(*bmp_write8)(i2c_cfg *cfg, uint8_t reg, uint8_t *data, size_t sz);
#ifdef DEBUG
    esp_err_t(*i2c_scan)(i2c_cfg *cfg);
#endif /* DEBUG */
} i2c_dev;

esp_err_t i2c_init(i2c_dev *dev);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INCLUDE_I2C_BUS_H_ */
