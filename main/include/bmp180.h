#ifndef INCLUDE_BMP180_H_
#define INCLUDE_BMP180_H_

#include <inttypes.h>
#include <stdbool.h>
#include <esp_err.h>
#include "i2c-bus.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//! <Enumeration for conversion mode
enum {
    OSS_MODE_NORMAL = 0,    //! <Default mode, normal power consumption
    OSS_MODE_ULP = 3,       //! <Ultra-low power consumption mode
    /* SENTINEL */
    OSS_MODE_MAX
};
typedef uint8_t oss_mode_t;

//! <Data structure for BMP180 sensor
typedef struct {
    int32_t temperature;
    int32_t pressure;
} bmp180;

esp_err_t bmp180_fetch_data(oss_mode_t mode, bmp180 *ptr);
esp_err_t bmp180_init(i2c_dev *device, i2c_cfg *config);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INCLUDE_BMP180_H_ */
