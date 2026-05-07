#pragma once

/* Includes ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

/* USER INCLUDE BEGIN */
/* USER INCLUDE END */

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ----------------------------------------------------------- */
/* PC控制模式 */
typedef enum {
    PC_MODE_RC = 0,
    PC_MODE_PC = 1,
} PC_ControlMode_t;

/* 通信命令 */
typedef enum {
    PC_CMD_HEARTBEAT = 0x01,
    PC_CMD_CHASSIS = 0x10,
    PC_CMD_POLE = 0x11,
} PC_CMD_t;

/* 反馈命令 */
typedef enum {
    PC_FEEDBACK_HEARTBEAT = 0x81,
    PC_FEEDBACK_CHASSIS = 0x90,
    PC_FEEDBACK_POLE = 0x91,
    PC_FEEDBACK_STATUS = 0xA0,
} PC_FeedbackCMD_t;

/* 帧头 */
#define PC_PROTOCOL_FRAME_HEADER_0 (0xAA)
#define PC_PROTOCOL_FRAME_HEADER_1 (0x55)
#define PC_PROTOCOL_MAX_PAYLOAD_SIZE (64)

/* Chassis命令 */
typedef struct {
    float vx;
    float vy;
    float wz;
} PC_ChassisCMD_t;

/* Pole命令 */
typedef struct {
    uint8_t mode;
    float lift[2];
} PC_PoleCMD_t;

/* Arm命令 */
typedef struct {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
} PC_ArmCMD_t;

/* Rod命令 */
typedef struct {
    uint8_t mode;
} PC_RodCMD_t;

/* Chassis反馈 */
typedef struct {
    float vx;
    float vy;
    float wz;
} PC_ChassisFeedback_t;

/* Pole反馈 */
typedef struct {
    float lift[2];
} PC_PoleFeedback_t;

/* Arm反馈 */
typedef struct {
    float x;
    float y;
    float z;
    float pitch;
    float yaw;
} PC_ArmFeedback_t;

/* Rod反馈 */
typedef struct {
    uint8_t mode;
} PC_RodFeedback_t;

/* Status反馈 */
typedef struct {
    uint8_t online;
    uint32_t recv_count;
    float cpu_temp;
} PC_StatusFeedback_t;

/* 命令数据 */
typedef struct {
    PC_ChassisCMD_t chassis;
    PC_PoleCMD_t pole;
    PC_ArmCMD_t arm;
    PC_RodCMD_t rod;
} PC_CMD_Data_t;

/* 反馈数据 */
typedef struct {
    PC_ChassisFeedback_t chassis;
    PC_PoleFeedback_t pole;
    PC_ArmFeedback_t arm;
    PC_RodFeedback_t rod;
    PC_StatusFeedback_t status;
} PC_FeedbackData_t;

/* PC协议结构体 */
typedef struct {
    PC_ControlMode_t control_mode;
    bool heartbeat_valid;
    uint32_t last_heartbeat_tick;
    PC_CMD_Data_t cmd;
    PC_FeedbackData_t feedback;
} PC_Protocol_t;

/* PC通信结构体 */
typedef struct {
    PC_Protocol_t protocol;
    uint32_t last_recv_time;
    uint32_t recv_count;
    uint32_t error_count;
    bool online;
} PC_Comm_t;

/* Exported function prototypes --------------------------------------------- */
void PC_Protocol_Init(PC_Protocol_t *proto);
void PC_Protocol_Update(PC_Protocol_t *proto);
int8_t PC_Protocol_ParseFrame(PC_Protocol_t *proto, const uint8_t *data, uint16_t size);
uint16_t PC_Protocol_BuildFrame(PC_Protocol_t *proto, uint8_t cmd, uint8_t *tx_buf, uint16_t buf_size);
bool PC_Protocol_IsPCControlMode(const PC_Protocol_t *proto);
bool PC_Protocol_IsHeartbeatValid(const PC_Protocol_t *proto);
const PC_ChassisCMD_t* PC_Protocol_GetChassisCMD(const PC_Protocol_t *proto);
const PC_PoleCMD_t* PC_Protocol_GetPoleCMD(const PC_Protocol_t *proto);
const PC_ArmCMD_t* PC_Protocol_GetArmCMD(const PC_Protocol_t *proto);
const PC_RodCMD_t* PC_Protocol_GetRodCMD(const PC_Protocol_t *proto);
void PC_Protocol_SetChassisFeedback(PC_Protocol_t *proto, const PC_ChassisFeedback_t *fb);
void PC_Protocol_SetPoleFeedback(PC_Protocol_t *proto, const PC_PoleFeedback_t *fb);
void PC_Protocol_SetArmFeedback(PC_Protocol_t *proto, const PC_ArmFeedback_t *fb);
void PC_Protocol_SetRodFeedback(PC_Protocol_t *proto, const PC_RodFeedback_t *fb);
void PC_Protocol_SetStatusFeedback(PC_Protocol_t *proto, const PC_StatusFeedback_t *fb);

/* PC通信管理函数 */
bool PC_Comm_Init(PC_Comm_t *comm);
bool PC_Comm_IsOnline(const PC_Comm_t *comm);
void PC_Comm_Process(PC_Comm_t *comm, uint32_t now_ms);
PC_Comm_t* PC_Comm_GetInstance(void);

/* USER FUNCTION BEGIN */
/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif
