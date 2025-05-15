#ifndef __SRV_BARO_H
#define __SRV_BARO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef struct
{
    bool (*init)(void);
    bool (*sample)(void);
    void (*get)(void);
} SrvBaro_TypeDef;

extern SrvBaro_TypeDef SrvBaro;

#ifdef __cplusplus
}
#endif
#endif
