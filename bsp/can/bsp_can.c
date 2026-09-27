#include "bsp_can.h"

#include "bsp_dwt.h"
#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

#define CAN_BUS_COUNT 3
#define CAN_BUSOFF_RETRY_MIN_MS 50.0f
#define CAN_STATUS_LOG_MIN_MS 200.0f

static CANInstance *can_instance[CAN_MX_REGISTER_CNT];
static uint8_t instance_count;

typedef struct
{
    volatile uint8_t recover_request;
    volatile uint8_t warning_request;
    volatile uint32_t psr;
    volatile uint32_t ecr;
    volatile uint32_t recover_count;
    volatile uint32_t warning_count;
    float last_recover_ms;
    float last_status_log_ms;
} CANBusHealth_s;

static CANBusHealth_s can_health[CAN_BUS_COUNT];

static FDCAN_HandleTypeDef *CANHandleByIndex(uint8_t index)
{
    switch (index)
    {
    case 0:
        return &hfdcan1;
    case 1:
        return &hfdcan2;
    case 2:
        return &hfdcan3;
    default:
        return NULL;
    }
}

static int8_t CANIndexByHandle(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1)
        return 0;
    if (hfdcan == &hfdcan2)
        return 1;
    if (hfdcan == &hfdcan3)
        return 2;
    return -1;
}

static uint32_t CANLengthToDlc(uint8_t length)
{
    static const uint32_t dlc[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
    };
    return dlc[length];
}

static uint8_t CANDlcToLength(uint32_t dlc)
{
    return (uint8_t)(dlc & 0x0FU);
}

static uint8_t CANBusIsOff(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL || hfdcan->Instance == NULL)
        return 0;
    /* Need both: BO means the controller is bus-off, INIT means it is still
     * waiting for software to start the 128*11 recessive-bit recovery. */
    return ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U) &&
           ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0U);
}

static HAL_StatusTypeDef CANConfigureReceive(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = 0,
        .FilterID2 = 0,
    };

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
        return HAL_ERROR;

    return HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE);
}

static HAL_StatusTypeDef CANEnableNotifications(FDCAN_HandleTypeDef *hfdcan)
{
    return HAL_FDCAN_ActivateNotification(hfdcan,
                                          FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                              FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                              FDCAN_IT_BUS_OFF |
                                              FDCAN_IT_ERROR_WARNING |
                                              FDCAN_IT_ERROR_PASSIVE,
                                          0);
}

static HAL_StatusTypeDef CANStartBus(FDCAN_HandleTypeDef *hfdcan)
{
    if (CANConfigureReceive(hfdcan) != HAL_OK)
        return HAL_ERROR;
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
        return HAL_ERROR;
    return CANEnableNotifications(hfdcan);
}

static void CANLeaveBusOff(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL)
        return;

    /*
     * H7 FDCAN sets CCCR.INIT on bus-off and stays there until software
     * clears it. HAL_FDCAN_Start() only runs from READY, so force that
     * state rather than waiting for a full HAL_FDCAN_Init().
     */
    if (hfdcan->State == HAL_FDCAN_STATE_BUSY)
        (void)HAL_FDCAN_Stop(hfdcan);

    if (hfdcan->State != HAL_FDCAN_STATE_READY)
        hfdcan->State = HAL_FDCAN_STATE_READY;

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
        hfdcan->State = HAL_FDCAN_STATE_BUSY;
    }

    hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
    (void)CANEnableNotifications(hfdcan);
}

static void CANRecoverHandle(FDCAN_HandleTypeDef *hfdcan, uint8_t from_isr)
{
    const int8_t index = CANIndexByHandle(hfdcan);
    const float now_ms = DWT_GetTimeline_ms();
    CANBusHealth_s *health;

    if (index < 0)
        return;

    health = &can_health[index];
    if ((now_ms - health->last_recover_ms) < CAN_BUSOFF_RETRY_MIN_MS)
        return;

    health->last_recover_ms = now_ms;
    health->psr = hfdcan->Instance->PSR;
    health->ecr = hfdcan->Instance->ECR;
    health->recover_count++;
    health->recover_request = 1;

    if (from_isr)
    {
        /* Start the ISO 11898 recovery sequence immediately. */
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
        hfdcan->ErrorCode = HAL_FDCAN_ERROR_NONE;
        if (hfdcan->State != HAL_FDCAN_STATE_BUSY)
            hfdcan->State = HAL_FDCAN_STATE_BUSY;
        return;
    }

    CANLeaveBusOff(hfdcan);
}

static void CANLogPendingHealth(uint8_t index)
{
    CANBusHealth_s *health = &can_health[index];
    const float now_ms = DWT_GetTimeline_ms();
    const uint32_t psr = health->psr;
    const uint32_t ecr = health->ecr;

    if (!health->recover_request && !health->warning_request)
        return;
    if ((now_ms - health->last_status_log_ms) < CAN_STATUS_LOG_MIN_MS)
        return;

    health->last_status_log_ms = now_ms;

    if (health->recover_request)
    {
        health->recover_request = 0;
        LOGERROR("[bsp_can] FDCAN%u bus-off recover count=%lu PSR=0x%08lx ECR=0x%08lx TEC=%lu REC=%lu LEC=%lu",
                 (unsigned)(index + 1u),
                 (unsigned long)health->recover_count,
                 (unsigned long)psr,
                 (unsigned long)ecr,
                 (unsigned long)(ecr & 0xFFu),
                 (unsigned long)((ecr >> 8) & 0x7Fu),
                 (unsigned long)(psr & 0x7u));
    }

    if (health->warning_request)
    {
        health->warning_request = 0;
        LOGWARNING("[bsp_can] FDCAN%u error status count=%lu PSR=0x%08lx ECR=0x%08lx TEC=%lu REC=%lu",
                   (unsigned)(index + 1u),
                   (unsigned long)health->warning_count,
                   (unsigned long)psr,
                   (unsigned long)ecr,
                   (unsigned long)(ecr & 0xFFu),
                   (unsigned long)((ecr >> 8) & 0x7Fu));
    }
}

void CANPollRecover(FDCAN_HandleTypeDef *hfdcan)
{
    const int8_t index = CANIndexByHandle(hfdcan);

    if (index < 0)
        return;

    if (CANBusIsOff(hfdcan))
        CANRecoverHandle(hfdcan, 0);

    CANLogPendingHealth((uint8_t)index);
}

void CANPollRecoverAll(void)
{
    for (uint8_t i = 0; i < CAN_BUS_COUNT; ++i)
        CANPollRecover(CANHandleByIndex(i));
}

static void CANServiceInit(void)
{
    if (CANStartBus(&hfdcan1) != HAL_OK ||
        CANStartBus(&hfdcan2) != HAL_OK ||
        CANStartBus(&hfdcan3) != HAL_OK)
    {
        LOGERROR("[bsp_can] failed to start one or more FDCAN buses");
    }
    else
    {
        LOGINFO("[bsp_can] FDCAN1/2/3 service initialized");
    }
}

static void CANRestart(FDCAN_HandleTypeDef *hfdcan)
{
    (void)HAL_FDCAN_Stop(hfdcan);
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK || CANStartBus(hfdcan) != HAL_OK)
        LOGERROR("[bsp_can] FDCAN restart failed");
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (config == NULL || config->can_handle == NULL || config->rx_id > 0x7FFU || config->tx_id > 0x7FFU)
    {
        LOGERROR("[bsp_can] invalid registration config");
        return NULL;
    }

    if (instance_count == 0U)
        CANServiceInit();

    if (instance_count >= CAN_MX_REGISTER_CNT)
    {
        LOGERROR("[bsp_can] CAN instance count exceeded %u", CAN_MX_REGISTER_CNT);
        return NULL;
    }

    for (uint8_t i = 0; i < instance_count; i++)
    {
        if (can_instance[i]->rx_id == config->rx_id &&
            can_instance[i]->can_handle == config->can_handle)
        {
            LOGERROR("[bsp_can] duplicate RX ID 0x%03lx", (unsigned long)config->rx_id);
            return NULL;
        }
    }

    CANInstance *instance = malloc(sizeof(*instance));
    if (instance == NULL)
    {
        LOGERROR("[bsp_can] allocation failed");
        return NULL;
    }
    memset(instance, 0, sizeof(*instance));

    instance->txconf.Identifier = config->tx_id;
    instance->txconf.IdType = FDCAN_STANDARD_ID;
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8;
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;
    instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker = 0;
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    can_instance[instance_count++] = instance;
    return instance;
}

uint8_t CANTransmit(CANInstance *instance, float timeout)
{
    static uint32_t busy_count;
    float start;

    if (instance == NULL || instance->can_handle == NULL)
        return 0;

    CANPollRecover(instance->can_handle);

    start = DWT_GetTimeline_ms();
    while (HAL_FDCAN_GetTxFifoFreeLevel(instance->can_handle) == 0U)
    {
        CANPollRecover(instance->can_handle);
        if ((DWT_GetTimeline_ms() - start) > timeout)
        {
            LOGWARNING("[bsp_can] FDCAN TX FIFO full, count=%lu PSR=0x%08lx",
                       (unsigned long)busy_count++,
                       (unsigned long)instance->can_handle->Instance->PSR);
            return 0;
        }
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(instance->can_handle,
                                      &instance->txconf,
                                      instance->tx_buff) != HAL_OK)
    {
        CANPollRecover(instance->can_handle);
        LOGWARNING("[bsp_can] failed to enqueue FDCAN frame, count=%lu", (unsigned long)busy_count++);
        return 0;
    }
    return 1;
}

void CANGetDebugInfo(FDCAN_HandleTypeDef *hfdcan, CAN_Debug_Info_s *out)
{
    if (hfdcan == NULL || out == NULL)
        return;

    out->tx_free_level = (uint8_t)HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
    out->hal_error = HAL_FDCAN_GetError(hfdcan);
    out->state = (uint32_t)HAL_FDCAN_GetState(hfdcan);
    out->esr = hfdcan->Instance->ECR;
    out->tsr = hfdcan->Instance->TXFQS;
    out->msr = hfdcan->Instance->PSR;
}

void CANSetAutoRetransmission(FDCAN_HandleTypeDef *hfdcan, uint8_t enable)
{
    if (hfdcan == NULL)
        return;
    hfdcan->Init.AutoRetransmission = enable ? ENABLE : DISABLE;
    CANRestart(hfdcan);
}

void CANSetAutoBusOff(FDCAN_HandleTypeDef *hfdcan, uint8_t enable)
{
    (void)hfdcan;
    (void)enable;
    LOGINFO("[bsp_can] FDCAN bus-off recovery is always enabled in software");
}

void CANSetMode(FDCAN_HandleTypeDef *hfdcan, uint32_t mode)
{
    if (hfdcan == NULL)
        return;
    hfdcan->Init.Mode = mode;
    CANRestart(hfdcan);
}

void CANSetDLC(CANInstance *instance, uint8_t length)
{
    if (instance == NULL || length == 0U || length > 8U)
    {
        LOGERROR("[bsp_can] invalid classic CAN payload length");
        return;
    }
    instance->txconf.DataLength = CANLengthToDlc(length);
}

static void CANFIFO0Callback(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
            return;

        uint8_t rx_length = CANDlcToLength(rx_header.DataLength);
        if (rx_length > sizeof(rx_data))
            rx_length = sizeof(rx_data);

        for (uint8_t i = 0; i < instance_count; i++)
        {
            CANInstance *instance = can_instance[i];
            if (instance->can_handle == hfdcan && instance->rx_id == rx_header.Identifier)
            {
                instance->rx_len = rx_length;
                memcpy(instance->rx_buff, rx_data, rx_length);
                if (instance->can_module_callback != NULL)
                    instance->can_module_callback(instance);
            }
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
{
    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
        CANFIFO0Callback(hfdcan);

    if ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        const int8_t index = CANIndexByHandle(hfdcan);
        if (index >= 0)
        {
            can_health[index].psr = hfdcan->Instance->PSR;
            can_health[index].ecr = hfdcan->Instance->ECR;
            can_health[index].warning_count++;
            can_health[index].warning_request = 1;
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    const int8_t index = CANIndexByHandle(hfdcan);
    CANBusHealth_s *health;

    if (index < 0)
        return;

    health = &can_health[index];
    health->psr = hfdcan->Instance->PSR;
    health->ecr = hfdcan->Instance->ECR;

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U &&
        (hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U)
    {
        CANRecoverHandle(hfdcan, 1);
        return;
    }

    if ((ErrorStatusITs & (FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE)) != 0U)
    {
        health->warning_count++;
        health->warning_request = 1;
    }
}
