#include <inttypes.h>
#include <string.h>
#include <esp_err.h>
#include "i2c-bus.h"
#include "bmp180.h"

//! <BMP180 configuration and control registers
#define CONTROL_REG                 0xF4
#define CONTROL_OUTPUT_REG          0xF6
#define READ_UNC_TEMPERATURE_REG    0x2E
#define READ_UNC_PRESSURE_REG       0x34
//! <BMP180 calibration registers
#define CALIBRATION_REG_AC1         0xAA
#define CALIBRATION_REG_AC2         0xAC
#define CALIBRATION_REG_AC3         0xAE
#define CALIBRATION_REG_AC4         0xB0
#define CALIBRATION_REG_AC5         0xB2
#define CALIBRATION_REG_AC6         0xB4
#define CALIBRATION_REG_B1          0xB6
#define CALIBRATION_REG_B2          0xB8
#define CALIBRATION_REG_MB          0xBA
#define CALIBRATION_REG_MC          0xBC
#define CALIBRATION_REG_MD          0xBE

//! <BMP180 calibration data structure definition
typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} calibration_data;

static calibration_data cal_data;               //! <Calibration data storage
static i2c_dev *dev = NULL;                     //! <Pointer to I2C device stucture with callbacks
static i2c_cfg *cfg = NULL;                     //! <Pointer to I2C configuration structure
static uint8_t _rx_buff[3];                     //! <BMP180 receive buffer

/**
 * Read calibration data
 * @brief Read calibaration data from calibration registers
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __read_calibration_data(void) {
    esp_err_t ret = 0;

    memset(_rx_buff, 0, 3);
    memset(&cal_data, 0, sizeof(calibration_data));

    //! <Reg AC1
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_AC1, _rx_buff, 2);
    cal_data.ac1 = ((int16_t)_rx_buff[0] << 8) | ((int16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "AC1: %i\n", (int16_t)cal_data.ac1);
#endif /* DEBUG */

    //! <Reg AC2
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_AC2, _rx_buff, 2);
    cal_data.ac2 = ((int16_t)_rx_buff[0] << 8) | ((int16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "AC2: %i\n", (int16_t)cal_data.ac2);
#endif /* DEBUG */

    //! <Reg AC3
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_AC3, _rx_buff, 2);
    cal_data.ac3 = ((int16_t)_rx_buff[0] << 8) | ((int16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "AC3: %i\n", (int16_t)cal_data.ac3);
#endif /* DEBUG */

    //! <Reg AC4
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_AC4, _rx_buff, 2);
    cal_data.ac4 = ((uint16_t)_rx_buff[0] << 8) | ((uint16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "AC4: %u\n", (uint16_t)cal_data.ac4);
#endif /* DEBUG */

    //! <Reg AC5
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_AC5, _rx_buff, 2);
    cal_data.ac5 = ((uint16_t)_rx_buff[0] << 8) | ((uint16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "AC5: %u\n", (uint16_t)cal_data.ac5);
#endif /* DEBUG */

    //! <Reg AC6
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_AC6, _rx_buff, 2);
    cal_data.ac6 = ((uint16_t)_rx_buff[0] << 8) | ((uint16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "AC6: %u\n", (uint16_t)cal_data.ac6);
#endif /* DEBUG */

    //! <Reg B1
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_B1, _rx_buff, 2);
    cal_data.b1 = ((int16_t)_rx_buff[0] << 8) | ((int16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "B1: %i\n", (int16_t)cal_data.b1);
#endif /* DEBUG */

    //! <Reg B2
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_B2, _rx_buff, 2);
    cal_data.b2 = ((int16_t)_rx_buff[0] << 8) | ((int16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "B2: %i\n", (int16_t)cal_data.b2);
#endif /* DEBUG */

    //! <Reg MB
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_MB, _rx_buff, 2);
    cal_data.mb = ((int16_t)_rx_buff[0] << 8) | ((int16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "MB: %i\n", (int16_t)cal_data.mb);
#endif /* DEBUG */

    //! <Reg MC
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_MC, _rx_buff, 2);
    cal_data.mc = ((int16_t)_rx_buff[0] << 8) | ((int16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "MC: %i\n", (int16_t)cal_data.mc);
#endif /* DEBUG */

    //! <Reg MD
    ret = dev->bmp_read8(cfg, CALIBRATION_REG_MD, _rx_buff, 2);
    cal_data.md = ((int16_t)_rx_buff[0] << 8) | ((int16_t)_rx_buff[1] & 0xff);
#ifdef DEBUG
    fprintf(stdout, "MD: %i\n", (int16_t)cal_data.md);
#endif /* DEBUG */

    return ret;
}

/**
 * Read uncompensated temperature
 * @brief Read calculated uncompensated temperature (UT) from the sensor's registers
 * @param ut_val pointer to a variable where UT value will be put
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __read_uncompensated_temperature(int32_t *ut_val) {
    esp_err_t ret = 0;
    uint8_t cmd = READ_UNC_TEMPERATURE_REG;

    if (ut_val == NULL) {
        return ESP_FAIL;
    }

    ret = dev->bmp_write8(cfg, CONTROL_REG, &cmd, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
    ret = dev->bmp_read8(cfg, CONTROL_OUTPUT_REG, _rx_buff, 2);
    *ut_val = ((int32_t)_rx_buff[0] << 8) | ((int32_t)_rx_buff[1] & 0xff);

    return ret;
}

/**
 * Read uncompensated pressure
 * @brief Read calculated uncompensated pressure (UP) from the sensor's registers
 * @param mode conversion mode
 * @param up_val pointer to a variable where UP value will be put
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __read_uncompensated_pressure(oss_mode_t mode, uint32_t *up_val) {
    esp_err_t ret = 0;
    uint8_t cmd = (READ_UNC_PRESSURE_REG + (mode << 6));

    if (up_val == NULL) {
        return ESP_FAIL;
    }

    ret = dev->bmp_write8(cfg, CONTROL_REG, &cmd, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = dev->bmp_read8(cfg, CONTROL_OUTPUT_REG, _rx_buff, 3);
    *up_val = (((uint32_t)_rx_buff[0] << 16) | ((uint32_t)_rx_buff[1] << 8) |
            ((uint32_t)_rx_buff[2] & 0xff)) >> (8 - mode);
    return ret;
}

/**
 * BMP180 fetch data
 * @brief Get and calculated true temperature/pressure from the sensor
 * @param mode conversion mode
 * @param ptr pointer to a structure which will contatin calculated values
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bmp180_fetch_data(oss_mode_t mode, bmp180 *ptr) {
    esp_err_t ret = 0;
    int32_t x1 = 0;
    int32_t x2 = 0;
    int32_t x3 = 0;
    int32_t b3 = 0;
    uint32_t b4 = 0;
    int32_t b5 = 0;
    int32_t b6 = 0;
    uint32_t b7 = 0;
    uint32_t up = 0;
    int32_t ut = 0;

    if (ptr == NULL) {
        return ESP_FAIL;
    }

    memset(ptr, 0, sizeof(bmp180));

    //! <Get uncompensated temperature value
    ret = __read_uncompensated_temperature(&ut);
    if (ret != ESP_OK) {
        return ret;
    }
#ifdef DEBUG
    fprintf(stdout, "Uncompensated temperature: %i\n", ut);
#endif /* DEBUG */

    //! <Get uncompensated pressure value
    ret = __read_uncompensated_pressure(mode, &up);
    if (ret != ESP_OK) {
        return ret;
    }
#ifdef DEBUG
    fprintf(stdout, "Uncompensated pressure: %u\n", up);
#endif /* DEBUG */

    //! <Get real temperature
    x1 = ((ut - cal_data.ac6) * cal_data.ac5) >> 15;
    x2 = (cal_data.mc << 11) / (x1 + cal_data.md);
    b5 = x1 + x2;
    ptr->temperature = (b5 + 8) / 16;

    //! <Get real pressure
    b6 = b5 - 4000;
    x1 = (cal_data.b2 * (b6 * (b6 >> 12))) >> 11;
    x2 = (cal_data.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((cal_data.ac1 * 4 + x3) << mode) + 2) >> 2;
    x1 = (cal_data.ac3 * b6) >> 13;
    x2 = (cal_data.b1 * (b6 * b6 >> 12) >> 16);
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (cal_data.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    b7 = ((uint32_t)up - b3) * (50000 >> mode);
    if (b7 < 0x80000000) {
        ptr->pressure = (b7 * 2) / b4;
    } else {
        ptr->pressure = (b7 / b4) * 2;
    }

    x1 = (ptr->pressure >> 8) * (ptr->pressure >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * ptr->pressure) >> 16;
    ptr->pressure = ptr->pressure + ((x1 + x2 + 3791) >> 4);

    return ret;
}

/**
 * BMP180 init
 * @brief Initialize BMP180
 * @param device pointer to a I2C device structure with callbacks
 * @param config pointer to a I2C configuration structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t bmp180_init(i2c_dev *device, i2c_cfg *config) {
    if ((device == NULL) && (config == NULL)) {
        return ESP_FAIL;
    }

    dev = device;
    cfg = config;

    return __read_calibration_data();
}
