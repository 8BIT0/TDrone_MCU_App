#ifndef __SRV_MAG_H__
#define __SRV_MAG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{
    bool (*init)(void);
    bool (*sample)(void);
    bool (*calib)(void);
} SrvMag_TypeDef;

extern SrvMag_TypeDef SrvMag;

#ifdef __cplusplus
}
#endif

#endif