#ifndef __STORAGE_DEV_PORT_H
#define __STORAGE_DEV_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define To_StorageDevObj_Ptr(x) ((StorageDevObj_TypeDef *)x)

typedef enum
{
    Storage_Chip_None = 0,
    Storage_ChipType_W25Qxx,
    Storage_ChipType_All,
} Storage_FlashChipType_List;

/* hadware flash chip info */
typedef struct
{
    Storage_FlashChipType_List chip_type;

    uint32_t start_addr;
    uint32_t total_size;

    uint32_t page_num;
    uint32_t page_size;

    uint32_t bank_num;
    uint32_t bank_size;

    uint32_t block_num;
    uint32_t block_size;

    uint32_t sector_num;
    uint32_t sector_size;

    void *obj;
    void *api;
} StorageDevObj_TypeDef;

typedef struct
{
    bool (*set)(StorageDevObj_TypeDef *ext_dev);
    bool (*init)(StorageDevObj_TypeDef *ext_dev, uint32_t *p_code);

    /* directly write data to physical section */
    bool (*write_phy_sec)(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint32_t len);
    bool (*read_phy_sec)(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint32_t len);
    bool (*erase_phy_sec)(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint32_t len);

    bool (*param_write)(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint8_t *p_data, uint32_t len);
    bool (*param_read)(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint8_t *p_data, uint32_t len);
    bool (*param_erase)(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t len);
} StorageDevApi_TypeDef;

extern StorageDevApi_TypeDef StorageDev;

#ifdef __cplusplus
}
#endif

#endif
