#ifndef INCLUDE_CON_PARAMS_H_
#define INCLUDE_CON_PARAMS_H_

#include <inttypes.h>
#include <stdbool.h>
#include "global_types.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#pragma pack(push, 1)
//! <Client configuration
typedef struct {
    uint8_t ap_name[32];
    uint8_t ap_psk[64];
    client_id_t id;
    uint32_t ip_address;
    uint16_t port;
    uint16_t sleep_interval_sec;
} ap_client_cfg;

typedef union {
    struct _client_setup {
        ap_client_cfg cfg;
    } msg;
    uint8_t buffer[sizeof(struct _client_setup)];
} client_setup;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INCLUDE_CON_PARAMS_H_ */
