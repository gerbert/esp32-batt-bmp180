#ifndef INCLUDE_SENSOR_H_
#define INCLUDE_SENSOR_H_

#include <inttypes.h>
#include <stdbool.h>
#include "global_types.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#pragma pack(push, 1)
//! <ESP32 sensor data (WiFi/Battery/BMP180)
typedef struct {
    uint16_t voltage;
    int32_t temperature;
    int32_t pressure;
    uint32_t free_heap;
} esp32_bbmp_sens;

typedef union {
    struct _wbmp_sens {
        client_id_t id;
        sensor_type_t s_type;
        esp32_bbmp_sens sensor;
    } msg;
    uint8_t buffer[sizeof(struct _wbmp_sens)];
} wbmp_sens;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INCLUDE_SENSOR_H_ */
