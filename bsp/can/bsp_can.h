#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "fdcan.h"

#define CAN_MX_REGISTER_CNT 16
#define DEVICE_CAN_CNT 3

typedef struct _
{
    FDCAN_HandleTypeDef *can_handle;
    FDCAN_TxHeaderTypeDef txconf;
    uint32_t tx_id;
    uint32_t tx_mailbox;
    uint8_t tx_buff[8];
    uint8_t rx_buff[8];
    uint32_t rx_id;
    uint8_t rx_len;
    void (*can_module_callback)(struct _ *);
    void *id;
} CANInstance;

typedef struct
{
    FDCAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;

typedef struct
{
    uint8_t tx_free_level;
    uint32_t hal_error;
    uint32_t esr; /* FDCAN error counter register snapshot. */
    uint32_t tsr; /* FDCAN TX FIFO/queue status register snapshot. */
    uint32_t msr; /* FDCAN protocol status register snapshot. */
    uint32_t state;
} CAN_Debug_Info_s;

CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *instance, uint8_t length);
uint8_t CANTransmit(CANInstance *instance, float timeout);
void CANGetDebugInfo(FDCAN_HandleTypeDef *hfdcan, CAN_Debug_Info_s *out);
void CANSetAutoRetransmission(FDCAN_HandleTypeDef *hfdcan, uint8_t enable);
void CANPollRecover(FDCAN_HandleTypeDef *hfdcan);
void CANPollRecoverAll(void);

/* H7 FDCAN has no bxCAN ABOM bit. Bus-off recovery is done in software. */
void CANSetAutoBusOff(FDCAN_HandleTypeDef *hfdcan, uint8_t enable);
void CANSetMode(FDCAN_HandleTypeDef *hfdcan, uint32_t mode);

#endif
