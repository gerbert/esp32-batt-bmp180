#include <string.h>
#include "driver/i2c.h"
#include "i2c-bus.h"

#define I2C_MASTER_TX_BUF_DISABLE 0 //! <I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 //! <I2C master doesn't need buffer
#define WRITE_BIT I2C_MASTER_WRITE  //! <I2C master write
#define READ_BIT I2C_MASTER_READ    //! <I2C master read
#define ACK_CHECK_EN 0x1            //! <I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0           //! <I2C master will not check ack from slave
#define ACK_VAL 0x0                 //! <I2C ack value
#define NACK_VAL 0x1                //! <I2C nack value

/**
 * I2C read (1 byte data)
 * @brief Read 1 byte data from I2C device
 * @param cfg I2C configuration structure
 * @param reg I2C device register to read from
 * @param data pointer to a data array to read to
 * @param sz number of bytes to be read
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __i2c_read8(i2c_cfg *cfg, uint8_t reg, uint8_t *data, size_t sz) {
    esp_err_t ret = 0;

    if (cfg == NULL) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: no config params provided\n", __func__, __LINE__);
#endif /* DEBUG */
        return ESP_FAIL;
    }

    ret = i2c_param_config(cfg->port, &cfg->cfg);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: param config failed\n", __func__, __LINE__);
#endif /* DEBUG */
        return ret;
    }

    ret = i2c_driver_install(cfg->port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE,
            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: driver install failed\n", __func__, __LINE__);
#endif /* DEBUG */
        return ret;
    }

    if (data != NULL) {
        memset(data, 0, sz);
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg->address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg->address << 1) | READ_BIT, ACK_CHECK_EN);

    if (sz > 1) {
        i2c_master_read(cmd, data, sz - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, (data + sz - 1), NACK_VAL);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(cfg->port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    i2c_driver_delete(cfg->port);

    return ret;
}

/**
 * I2C write (1 byte data)
 * @brief Write 1 byte data to I2C device
 * @param cfg I2C configuration structure
 * @param reg I2C device register to write to
 * @param data pointer to a data array to read from
 * @param sz number of bytes to be written
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __i2c_write8(i2c_cfg *cfg, uint8_t reg, uint8_t *data, size_t sz) {
    esp_err_t ret = 0;
    size_t i = 0;

    if (cfg == NULL) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: no config params provided\n", __func__, __LINE__);
#endif /* DEBUG */
        return ESP_FAIL;
    }

    ret = i2c_param_config(cfg->port, &cfg->cfg);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: param config failed\n", __func__, __LINE__);
#endif /* DEBUG */
        return ret;
    }

    ret = i2c_driver_install(cfg->port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE,
            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: driver install failed\n", __func__, __LINE__);
#endif /* DEBUG */
        return ret;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg->address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

    for (i = 0; i < sz; i++) {
        i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
    }

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(cfg->port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    i2c_driver_delete(cfg->port);

    return ret;
}

/**
 * EEPROM read block
 * @brief Read the data from EEPROM
 * @param cfg I2C configuration structure
 * @param reg I2C device register to read from (16 bit, auto increments)
 * @param data pointer to a data array to read to
 * @param sz number of bytes to be read
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __eep_read_block(i2c_cfg *cfg, uint16_t reg, uint8_t *data, uint8_t sz) {
    esp_err_t ret = 0;

    if (cfg == NULL) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: no config params provided\n", __func__, __LINE__);
#endif /* DEBUG */
        return ESP_FAIL;
    }

    ret = i2c_param_config(cfg->port, &cfg->cfg);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: param config failed\n", __func__, __LINE__);
#endif /* DEBUG */
        return ret;
    }

    ret = i2c_driver_install(cfg->port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE,
            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: driver install failed\n", __func__, __LINE__);
#endif /* DEBUG */
        return ret;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg->address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (reg >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (reg & 0xff), ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg->address << 1) | READ_BIT, ACK_CHECK_EN);

    if (sz > 1) {
        i2c_master_read(cmd, data, (sz - 1), ACK_VAL);
    }
    i2c_master_read_byte(cmd, &data[sz - 1], NACK_VAL);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(cfg->port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    i2c_driver_delete(cfg->port);

    return ret;
}

/**
 * EEPROM write block
 * @brief Write the data to the EEPROM
 * @param cfg I2C configuration structure
 * @param reg I2C device register to write to (16 bit, auto increments)
 * @param data pointer to a data array to read from
 * @param sz number of bytes to be written
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __eep_write_block(i2c_cfg *cfg, uint16_t reg, uint8_t *data, uint8_t sz) {
    esp_err_t ret = 0;

    if (cfg == NULL) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: no config params provided\n", __func__, __LINE__);
#endif /* DEBUG */
        return ESP_FAIL;
    }

    ret = i2c_param_config(cfg->port, &cfg->cfg);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: param config failed\n", __func__, __LINE__);
#endif /* DEBUG */
        return ret;
    }

    ret = i2c_driver_install(cfg->port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE,
            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "%s [%u]: driver install failed\n", __func__, __LINE__);
#endif /* DEBUG */
        return ret;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg->address << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(reg >> 8), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, (uint8_t)(reg & 0xff), ACK_CHECK_EN);
    i2c_master_write(cmd, data, sz, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    vTaskDelay(1);

    ret = i2c_master_cmd_begin(cfg->port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    i2c_driver_delete(cfg->port);

    return ret;
}

/**
 * EEPROM write (16 bit data)
 * @brief Write the data to the EEPROM
 * @param cfg I2C configuration structure
 * @param reg I2C device register to write to (16 bit, auto increments)
 * @param data pointer to a data array to read from
 * @param sz number of bytes to be written
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __eep_write16(i2c_cfg *cfg, uint16_t reg, uint8_t *data, size_t sz) {
    esp_err_t ret = 0;
    uint16_t blocks = 0;
    uint8_t diff = 0;

    blocks = sz / 64;
    diff = (sz - (blocks * 64));
    blocks += ((diff) > 0 ? 1 : 0);

    for (uint16_t j = 0; j < blocks; j++) {
        if (j == (blocks - 1)) {
            ret = __eep_write_block(cfg, (reg + j) * 64, &data[j * 64], diff);
        } else {
            ret = __eep_write_block(cfg, (reg + j) * 64, &data[j * 64], 64);
        }
        if (ret < 0) {
#ifdef DEBUG
            fprintf(stderr, "%s [%u]: byte write failed at block %u\n",
                    __func__, __LINE__, reg + j);
#endif /* DEBUG */
            return ret;
        }
    }

    //! Protection delay
    vTaskDelay(1);
    return ESP_OK;
}

/**
 * EEPROM read (16 bit data)
 * @brief Read the data from the EEPROM
 * @param cfg I2C configuration structure
 * @param reg I2C device register to read from (16 bit, auto increments)
 * @param data pointer to a data array to read to
 * @param sz number of bytes to be read
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __eep_read16(i2c_cfg *cfg, uint16_t reg, uint8_t *data, size_t sz) {
    esp_err_t ret = 0;
    uint16_t blocks = 0;
    uint8_t diff = 0;

    blocks = sz / 64;
    diff = (sz - (blocks * 64));
    blocks += ((diff) > 0 ? 1 : 0);

    for (uint16_t j = 0; j < blocks; j++) {
        if (j == (blocks - 1)) {
            ret = __eep_read_block(cfg, (reg + j) * 64, &data[j * 64], diff);
        } else {
            ret = __eep_read_block(cfg, (reg + j) * 64, &data[j * 64], 64);
        }
        if (ret < 0) {
#ifdef DEBUG
            fprintf(stderr, "%s [%u]: byte read failed at block %u\n",
                    __func__, __LINE__, reg + j);
#endif /* DEBUG */
            return ret;
        }
    }

    //! Protection delay
    vTaskDelay(1);
    return ESP_OK;
}


#ifdef DEBUG
/**
 * I2C scan
 * @brief Scan I2C bus and get all devices that will respond.
 *        Prints device's address to the debug output
 * @param cfg I2C configuration structure
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t __i2c_scan(i2c_cfg *cfg) {
    i2c_cfg test_cfg;
    esp_err_t ret = 0;
    uint8_t i = 0;
    uint8_t data = 0x00;

    if (cfg == NULL) {
        return ESP_FAIL;
    }

    memset(&test_cfg, 0, sizeof(i2c_cfg));
    memcpy(&test_cfg, cfg, sizeof(i2c_cfg));

    fflush(stdout);

    for (i = 0; i < 128; i++) {
        test_cfg.address = i;
        ret = __i2c_write8(&test_cfg, 0x00, &data, 1);
        if (ret == ESP_OK) {
            fprintf(stdout, "Found device at 0x%02x address\n", i);
        }
    }

    return ESP_OK;
}
#endif /* DEBUG */

/**
 * I2C init
 * @brief Initialize I2C callbacks
 * @param dev I2C device structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t i2c_init(i2c_dev *dev) {
    if (dev == NULL) {
        return ESP_FAIL;
    }

    dev->eep_read16 = __eep_read16;
    dev->eep_write16 = __eep_write16;
    dev->bmp_read8 = __i2c_read8;
    dev->bmp_write8 = __i2c_write8;
#ifdef DEBUG
    dev->i2c_scan = __i2c_scan;
#endif /* DEBUG */

    return ESP_OK;
}
