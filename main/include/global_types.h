#ifndef INCLUDE_GLOBAL_TYPES_H_
#define INCLUDE_GLOBAL_TYPES_H_

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//! <Message types
enum {
    MT_CLIENT_CFG = 0x01,
    MT_CLIENT_CFG_SAVED,
    MT_ACK,
    MT_NACK,
    MT_CRC_ERR,
    MT_EEPROM_ERR,
    MT_STATUS,
    MT_HEARTBEAT,
    /* SENTINEL */
    MT_MAX
};

//! <Sensor types
enum {
    ID_SENSOR_1 = 0x00,
    /* SENTINEL */
    ID_SENSOR_MAX
};

typedef uint32_t checksum_t;
typedef uint8_t magic_t;
typedef uint8_t client_id_t;
typedef uint8_t message_type_t;
typedef uint8_t sensor_type_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INCLUDE_GLOBAL_TYPES_H_ */
