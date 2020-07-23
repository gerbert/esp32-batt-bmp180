#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <limits.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_adc_cal.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_pm.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "soc/uart_struct.h"
#include "global.h"
#include "con_params.h"
#include "sensor.h"
#include "i2c-bus.h"
#include "bmp180.h"

#define SLEEP_SEC_DEFAULT       15                                  //! <Default deep sleep time interval
#define SLEEP_MS(x)             (uint64_t)((x) * 1000UL)            //! <Macro for calculating sleep interval in milliseconds
#define SLEEP_SEC(x)            (uint64_t)((x) * SLEEP_MS(1000UL))  //! <Macro for calculating sleep interval in seconds
#define SLEEP_MIN(x)            (uint64_t)((x) * SLEEP_SEC(60UL))   //! <Macro for calculating sleep interval in minutes
#define UART_PORT               UART_NUM_0                          //! <UART0
#define RESISTANCE_R1_KOHM      22                                  //! <22 kOhm
#define RESISTANCE_R2_KOHM      22                                  //! <22 kOhm
#define DEFAULT_VREF            (3300 / ((RESISTANCE_R1_KOHM + \
                                RESISTANCE_R2_KOHM) /          \
                                RESISTANCE_R2_KOHM))                //! <VRef is not configured
#define BATTERY_SENSE           ADC1_CHANNEL_7                      //! <GPIO35
#define WIFI_LISTEN_INTERVAL    3                                   //! <WiFi beacon wait interval
#define EEPROM_STORAGE_ADDRESS  0x50                                //! <EEPROM I2C bus address
#define BMP180_ADDRESS          0x77                                //! <BMP180 I2C bus address

//! <Enumeration for Rx states
typedef enum {
    RX_WAIT_START_FLAG = 0,//!< RX_WAIT_START_FLAG
    RX_WAIT_HDR,           //!< RX_WAIT_HDR
    RX_WAIT_COMPLETE,      //!< RX_WAIT_COMPLETE
    /* SENTINEL */
    RX_WAIT_MAX            //!< RX_WAIT_MAX
} transmission_state_t;

static transmission_msg rxBuffer;                                   //! <Receive buffer
static i2c_dev dev;                                                 //! <I2C device generic structure
static i2c_cfg eeprom_cfg;                                          //! <I2C configuration data for EEPROM
static i2c_cfg bmp180_cfg;                                          //! <I2C configuration data for BMP180
static wifi_config_t wifi_config;                                   //! <WiFi configuration structure
static ap_client_cfg ap_config;                                     //! <Device configuration
static TaskHandle_t xUARTRxTask_h;                                  //! <Handle for UART task
static QueueHandle_t uart0_queue;                                   //! <UART queue handle
static transmission_state_t rxState = RX_WAIT_START_FLAG;           //! <Default state for Rx state machine
static uint8_t *rxBufferPtr = NULL;                                 //! <Pointer to a Rx buffer
static bool rxBufferDone = false;                                   //! <Buffer ready flag
static bool enUartRxTask = false;                                   //! <UART enable/disable flag
static bool client_configured = false;                              //! <Client configuration ready flag
static uint64_t sleep_interval = SLEEP_SEC(SLEEP_SEC_DEFAULT);      //! <Deep sleep interval variable

/**
 * Reset Rx buffer
 * @brief Clear receive buffer, reset receive buffer pointer and
 *        set Rx control flag and state
 */
static void reset_rx_buffer(void) {
    memset(&rxBuffer, 0, sizeof(transmission_msg));
    rxBufferPtr = rxBuffer.buffer;
    rxBufferDone = false;
    rxState = RX_WAIT_START_FLAG;
}

/**
 * UART transmit
 * @brief Transmit binary data through the UART line
 * @param data pointer to binary data
 * @param sz size of data to be transmitted
 * @return number of bytes written
 */
static int uart_tx(uint8_t *data, uint16_t sz) {
    return uart_write_bytes(UART_PORT, (const char *)data, sz);
}

/**
 * Confirm Rx
 * @brief Send confirmation message on reception with a status
 * @param status type of confirmation message
 */
static void confirm_rx(message_type_t status) {
    transmission_msg tx_msg;
    msg_header *header = NULL;

    memset(&tx_msg, 0, sizeof(transmission_msg));
    header = &tx_msg.msg.header;
    header->magic = TRANSMISSION_MAGIC;
    header->m_type = status;
    header->sz = sizeof(msg_header) + TRANSMISSION_CRC_SZ;

    //! <Transmit confirmation
    uart_tx(tx_msg.buffer, set_crc(&tx_msg));
}

/**
 * Send heartbeat
 * @brief Send device status (sensor data, voltage, system stats)
 */
static void send_heartbeat(void) {
    struct sockaddr_in dest;
    transmission_msg msg;
    msg_header *hdr_ptr = NULL;
    wbmp_sens sensor;
    bmp180 sensor_data;
    esp32_bbmp_sens *sensor_ptr = NULL;
    int sock = -1;
    int err = -1;
    uint8_t sz = 0;

    memset(&sensor, 0, sizeof(wbmp_sens));
    sensor_ptr = &sensor.msg.sensor;

    sensor.msg.id = 0;
    sensor.msg.s_type = ID_SENSOR_1;

    sensor_ptr->free_heap = esp_get_free_heap_size();
    sensor_ptr->voltage = (uint16_t)((4096 / DEFAULT_VREF) * adc1_get_raw(BATTERY_SENSE));

    memset(&sensor_data, 0, sizeof(bmp180));
    bmp180_fetch_data(OSS_MODE_NORMAL, &sensor_data);
    sensor_ptr->temperature = sensor_data.temperature;
    sensor_ptr->pressure = sensor_data.pressure;

    //! <Prepare message
    memset(&msg, 0, sizeof(transmission_msg));
    hdr_ptr = &msg.msg.header;
    hdr_ptr->magic = TRANSMISSION_MAGIC;
    hdr_ptr->m_type = MT_HEARTBEAT;
    hdr_ptr->sz = sizeof(msg_header) + sizeof(wbmp_sens) + TRANSMISSION_CRC_SZ;
    memcpy(msg.msg.payload, sensor.buffer, ARRAY_SZ(sensor.buffer));
    sz = set_crc(&msg);

    //! <Setup socket and transmit the data
    memset(&dest, 0, sizeof(struct sockaddr_in));
    dest.sin_addr.s_addr = ap_config.ip_address;
    dest.sin_port = htons(ap_config.port);
    dest.sin_family = AF_INET;
    sock = socket(PF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
#ifdef DEBUG
        fprintf(stderr, "Unable to initialize a socket\n");
#endif /* DEBUG */
        return;
    }

    err = connect(sock, (struct sockaddr *)&dest, sizeof(struct sockaddr_in));
    if (err != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "Unable to connect to a socket. Error %i\n", err);
#endif /* DEBUG */

#ifdef DEBUG_NO_SLEEP
        esp_wifi_disconnect();
        esp_wifi_stop();
        esp_restart();
#endif /* DEBUG_NO_SLEEP */
        return;
    }

    err = send(sock, msg.buffer, sz, 0);
#ifdef DEBUG
    if (err < 0) {
        fprintf(stderr, "Unable to send heartbeat\n");
    } else {
        fprintf(stdout, "Sent %i bytes\n", err);
    }
#endif /* DEBUG */

    // Protection delay to allow data to be sent
    vTaskDelay(15 / portTICK_PERIOD_MS);
    // Terminate socket connection gracefully and close the socket
    closesocket(sock);
}

/**
 * Event handler
 * @brief WiFi event handler callback
 * @param ctx WiFi context
 * @param event system event type
 * @return ESP_OK on success (always)
 */
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
#ifdef DEBUG_NO_SLEEP
        while (true) {
            //! <Send sensor heartbeat
            send_heartbeat();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
#else
        //! <Send sensor heartbeat
        send_heartbeat();
        //! <Disconnect from WiFi and stop it
        esp_wifi_disconnect();
        esp_wifi_stop();
        //! <Go to sleep
        esp_deep_sleep(sleep_interval);
#endif /* DEBUG_NO_SLEEP */
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    default:
        break;
    }

    return ESP_OK;
}

/**
 * Init I2C bus
 * @brief Initialize I2C bus on the device
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t init_i2c_bus(void) {
    esp_err_t ret = 0;
    i2c_config_t *ptr = NULL;

    //! <Empty callback structure
    memset(&dev, 0, sizeof(i2c_dev));
    //! <Empty eeprom I2C configuration
    memset(&eeprom_cfg, 0, sizeof(i2c_cfg));

    eeprom_cfg.port = I2C_NUM_0;
    eeprom_cfg.address = EEPROM_STORAGE_ADDRESS;

    ptr = &eeprom_cfg.cfg;
    ptr->mode = I2C_MODE_MASTER;
    ptr->sda_io_num = GPIO_NUM_19;
    ptr->scl_io_num = GPIO_NUM_18;
    ptr->sda_pullup_en = GPIO_PULLUP_ENABLE;
    ptr->scl_pullup_en = GPIO_PULLUP_ENABLE;
    ptr->master.clk_speed = 400000;

    //! <Empty BMP180 I2C configuration
    memset(&bmp180_cfg, 0, sizeof(i2c_cfg));

    bmp180_cfg.port = I2C_NUM_0;
    bmp180_cfg.address = BMP180_ADDRESS;

    ptr = &bmp180_cfg.cfg;
    ptr->mode = I2C_MODE_MASTER;
    ptr->sda_io_num = GPIO_NUM_19;
    ptr->scl_io_num = GPIO_NUM_18;
    ptr->sda_pullup_en = GPIO_PULLUP_ENABLE;
    ptr->scl_pullup_en = GPIO_PULLUP_ENABLE;
    ptr->master.clk_speed = 400000;

    //! <Initialize callbacks
    ret = i2c_init(&dev);
    if (ret != ESP_OK) {
#ifdef DEBUG
        fprintf(stderr, "Unable to initialize I2C bus!\n");
#endif /* DEBUG */
    }

    return ret;
}

/**
 * Set client params
 * @brief Sets device params either from EEPROM storage, or
 *        based on receieved through UART data
 * @param update if false, EEPROM will be used, otherwise the
 *        data, that has been received from UART line during
 *        configuration
 */
static void set_client_params(bool update) {
    client_setup *setup_data = (client_setup *)rxBuffer.msg.payload;
    ap_client_cfg *cfg = (ap_client_cfg *)&setup_data->msg.cfg;
    esp_err_t ret = 0;

    //! <First of all, disconnect from WiFi, if
    //! <we were connected before
    esp_wifi_disconnect();
    esp_wifi_stop();

    //! <Check eeprom first
    if (!update) {
        memset(setup_data, 0, sizeof(client_setup));
        ret = dev.eep_read16(&eeprom_cfg, 0, setup_data->buffer,
                ARRAY_SZ(setup_data->buffer));
        if (ret != ESP_OK) {
#ifdef DEBUG
            fprintf(stderr, "Unable to read EEPROM\n");
#endif /* DEBUG */
            client_configured = false;
            return;
        }

        if ((!strlen((const char *)cfg->ap_name)) && (!strlen((const char *)cfg->ap_psk)) &&
                (cfg->ip_address == 0) && (cfg->port == 0)) {
#ifdef DEBUG
            fprintf(stdout, "AP settings not found\n");
#endif /* DEBUG */
            client_configured = false;
            return;
        }
    } else {
        if ((!strlen((const char *)cfg->ap_name)) && (!strlen((const char *)cfg->ap_psk)) &&
                (cfg->ip_address == 0) && (cfg->port == 0)) {
#ifdef DEBUG
            fprintf(stdout, "AP settings not found\n");
#endif /* DEBUG */
            client_configured = false;
            return;
        }
        ret = dev.eep_write16(&eeprom_cfg, 0, setup_data->buffer,
                ARRAY_SZ(setup_data->buffer));
        if (ret != ESP_OK) {
#ifdef DEBUG
            fprintf(stderr, "Failed to save data to EEPROM. Setting won't be restored on next reboot\n");
#endif /* DEBUG */
            confirm_rx(MT_EEPROM_ERR);
        } else {
#ifdef DEBUG
            fprintf(stdout, "AP setting saved\n");
#endif /* DEBUG */
            confirm_rx(MT_CLIENT_CFG_SAVED);
        }
    }

    //! <Update configuration
    memcpy(wifi_config.sta.ssid, cfg->ap_name, ARRAY_SZ(cfg->ap_name));
    memcpy(wifi_config.sta.password, cfg->ap_psk, ARRAY_SZ(cfg->ap_psk));
    wifi_config.sta.listen_interval = WIFI_LISTEN_INTERVAL;
    //! <Update configuration
    memset(&ap_config, 0, sizeof(ap_client_cfg));
    memcpy(&ap_config, cfg, sizeof(ap_client_cfg));

    //! <Set device sleep interval
    if ((cfg->sleep_interval_sec >= SLEEP_SEC_DEFAULT) && (cfg->sleep_interval_sec < USHRT_MAX)) {
        sleep_interval = SLEEP_SEC(cfg->sleep_interval_sec);
    } else {
        sleep_interval = SLEEP_SEC(SLEEP_SEC_DEFAULT);
    }

    //! <Set configured flag to true
    client_configured = true;
}

/**
 * Message handler task
 * @brief Callback used during configuration on
 *        initial stage after the device was booted
 *        and configuration timeout didn't yet ended.
 */
static void message_hdlr_task(void) {
    msg_header *hdr = (msg_header *)&rxBuffer.msg.header;

    switch (hdr->m_type) {
    case MT_CLIENT_CFG:
        client_configured = false;
        confirm_rx(MT_ACK);
        set_client_params(true);
        reset_rx_buffer();
        break;
    }
}

/**
 * UART Rx task
 * @brief UART Rx callback
 * @param ptr additional data which can be passed to the
 *        callback (not used)
 */
static void uart_rx_task(void __attribute__ ((unused)) *ptr) {
    uint8_t sz = 0;
    uint16_t rxCounter = 0;
    uint8_t data = 0;

    while (enUartRxTask) {
        if (rxBufferDone) {
            continue;
        }

        sz = uart_read_bytes(UART_PORT, &data, 1, 10 / portTICK_PERIOD_MS);
        if (sz) {
            switch (rxState) {
            case RX_WAIT_START_FLAG:
                if (data == TRANSMISSION_MAGIC) {
                    reset_rx_buffer();
                    *(rxBufferPtr++) = data;
                    rxCounter = sizeof(msg_header) - 1;
                    rxState = RX_WAIT_HDR;
                }
                break;
            case RX_WAIT_HDR:
                if (rxCounter--) {
                    *(rxBufferPtr++) = data;
                }

                if (!rxCounter) {
                    rxCounter = align_msg(&rxBuffer);
                    if (rxCounter <= (uint16_t)sizeof(rxBuffer.buffer)) {
                        rxCounter -= sizeof(msg_header);
                        rxState = RX_WAIT_COMPLETE;
                    } else {
                        reset_rx_buffer();
                    }
                }
                break;
            case RX_WAIT_COMPLETE:
                if (rxCounter--) {
                    *(rxBufferPtr++) = data;
                }

                if (!rxCounter) {
                    if (verify_crc((transmission_msg *)&rxBuffer)) {
                        rxBufferDone = true;
                        message_hdlr_task();
                        rxState = RX_WAIT_MAX;
                    } else {
                        confirm_rx(MT_CRC_ERR);
                        reset_rx_buffer();
                    }
                }
                break;
            case RX_WAIT_MAX:
                break;
            default:
                break;
            }
        } else {
            if ((rxState != RX_WAIT_START_FLAG) &&
                    (rxState != RX_WAIT_MAX)) {
                reset_rx_buffer();
            }
        }
    }

    //! <Correctly terminate the task
    if (xUARTRxTask_h != NULL) {
        vTaskDelete(xUARTRxTask_h);
    }
}

/**
 * Trigger function
 * @brief This function checks whether the client was
 *        configured and, depending on it either starts
 *        WiFi and sends a heartbeat or reboots the device.
 */
static void trigger_task(void) {

    //! <Start handlers, required for message transmission
    if (client_configured) {
        //! <Configure WiFi client and connect to the AP
        tcpip_adapter_init();
        ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

        wifi_init_config_t sta_cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&sta_cfg));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
    } else {
        esp_restart();
    }
}

//! <UART configuration structure
static uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
};

/**
 * Main application
 * @brief Main application
 */
void app_main(void) {
    //! <Grace period to receive settings before running main task
    TickType_t xSettingsDelay = 10000 / portTICK_PERIOD_MS;
    //! <Delay before rebooting the device on critical error
    TickType_t xRebootDelay = 2000 / portTICK_PERIOD_MS;
    esp_err_t ret = 0;

    //! <Enable UART RX task
    enUartRxTask = true;
    //! <Client is not configured by-default
    client_configured = false;

    //! <Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //! <Set UART parameters
    uart_param_config(UART_PORT, &uart_config);
    //! <Install UART driver and get the queue
    uart_driver_install(UART_PORT, (512 << 1), (512 << 1),
            10, &uart0_queue, 0);

    //! <UART pins use default configuration
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //! Set-up ADC subsystem
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_SENSE, ADC_ATTEN_DB_11);

    //! <Empty receive buffer
    reset_rx_buffer();

    //! <Init I2C bus and prepare configuration data
    //! <for all devices connected to it
    ret = init_i2c_bus();
    if (ret != ESP_OK) {
        fprintf(stderr, "Unable to init I2C bus\n");
        vTaskDelay(xRebootDelay);
        esp_restart();
    }

    //! <Init BMP180 sensor
    ret = bmp180_init(&dev, &bmp180_cfg);
    if (ret != ESP_OK) {
        fprintf(stderr, "Unable to init BMP180 sensor\n");
        vTaskDelay(xRebootDelay);
        esp_restart();
    }

    //! <Create RX task
    xTaskCreate(uart_rx_task, "uart_rx_task", 2048,
            NULL, 12, &xUARTRxTask_h);

    //! <Wait
    vTaskDelay(xSettingsDelay);
    //! <Stop UART task and free resources
    enUartRxTask = false;

    if (!client_configured) {
        //! <Try to set client parameters
        set_client_params(false);
    }

    //! <Run sequence function
    trigger_task();
}
