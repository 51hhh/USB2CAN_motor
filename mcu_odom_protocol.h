#ifndef ODOM_PROTOCOL_H
#define ODOM_PROTOCOL_H

#include <stdint.h>

/* ---------- 编译宏：协议模式切换 ---------- */
/* 1 = 二进制 ODOM 协议, 0 = VOFA CSV 调试模式 */
#ifndef ODOM_BINARY_MODE
#define ODOM_BINARY_MODE 1
#endif

/* ---------- 帧头 & 版本 ---------- */
#define ODOM_FRAME_HEADER_0  0xAA
#define ODOM_FRAME_HEADER_1  0x55
#define ODOM_FRAME_VERSION   0x01

/* ---------- 消息类型 ---------- */
#define ODOM_MSG_POSE                  0x01   /* 24B payload, MCU -> Host */
#define ODOM_MSG_STATE                 0x02   /* 36B payload, MCU -> Host */
#define ODOM_MSG_STATUS                0x10   /* 12B payload, MCU -> Host */
#define ODOM_MSG_TIME_SYNC_REQ         0x20   /*  8B payload, Host -> MCU */
#define ODOM_MSG_TIME_SYNC_RESP        0x21   /* 16B payload, MCU -> Host */
#define ODOM_MSG_SET_LOCAL_ORIGIN      0x30   /* 16B payload, Host -> MCU */
#define ODOM_MSG_SET_LOCAL_ORIGIN_ACK  0x31   /*  8B payload, MCU -> Host */

/* ---------- 帧结构常量 ---------- */
#define ODOM_FRAME_OVERHEAD  9  /* header(2)+ver(1)+type(1)+seq(1)+len(2)+crc(2) */
#define ODOM_POSE_PAYLOAD_LEN              24
#define ODOM_STATE_PAYLOAD_LEN             36
#define ODOM_STATUS_PAYLOAD_LEN            12
#define ODOM_TIME_SYNC_REQ_PAYLOAD_LEN      8
#define ODOM_TIME_SYNC_RESP_PAYLOAD_LEN    16
#define ODOM_SET_ORIGIN_PAYLOAD_LEN        16
#define ODOM_SET_ORIGIN_ACK_PAYLOAD_LEN     8

#define ODOM_POSE_FRAME_LEN              (ODOM_FRAME_OVERHEAD + ODOM_POSE_PAYLOAD_LEN)              /* 33 */
#define ODOM_STATE_FRAME_LEN             (ODOM_FRAME_OVERHEAD + ODOM_STATE_PAYLOAD_LEN)             /* 45 */
#define ODOM_STATUS_FRAME_LEN            (ODOM_FRAME_OVERHEAD + ODOM_STATUS_PAYLOAD_LEN)            /* 21 */
#define ODOM_TIME_SYNC_RESP_FRAME_LEN    (ODOM_FRAME_OVERHEAD + ODOM_TIME_SYNC_RESP_PAYLOAD_LEN)    /* 25 */
#define ODOM_SET_ORIGIN_ACK_FRAME_LEN    (ODOM_FRAME_OVERHEAD + ODOM_SET_ORIGIN_ACK_PAYLOAD_LEN)    /* 17 */
/* 上位机最长一帧 = SET_LOCAL_ORIGIN = 25 字节 */
#define ODOM_UPSTREAM_MAX_FRAME_LEN      (ODOM_FRAME_OVERHEAD + ODOM_SET_ORIGIN_PAYLOAD_LEN)        /* 25 */

/* ---------- status_bits 定义 ---------- */
#define ODOM_STATUS_ENC_VALID    (1u << 0)
#define ODOM_STATUS_IMU_VALID    (1u << 1)
#define ODOM_STATUS_YAW_VALID    (1u << 2)
#define ODOM_STATUS_POS_VALID    (1u << 3)
#define ODOM_STATUS_VEL_VALID    (1u << 4)
#define ODOM_STATUS_RELOCATE     (1u << 5)
#define ODOM_STATUS_TIME_SYNC    (1u << 6)
#define ODOM_STATUS_DEGRADED     (1u << 7)

/* ---------- quality 等级 ---------- */
#define ODOM_QUALITY_UNAVAIL  0
#define ODOM_QUALITY_DEGRADED 1
#define ODOM_QUALITY_NORMAL   2
#define ODOM_QUALITY_HIGH     3

/* ---------- Payload 结构体 (packed) ---------- */
#pragma pack(push, 1)

typedef struct {
    uint64_t t_sample_us;
    float    x;
    float    y;
    float    yaw;
    uint16_t status_bits;
    uint8_t  quality;
    uint8_t  reserved;
} OdomPosePayload_t;

typedef struct {
    uint64_t t_sample_us;
    float    x;
    float    y;
    float    yaw;
    float    vx;
    float    vy;
    float    wz;
    uint16_t status_bits;
    uint8_t  quality;
    uint8_t  reserved;
} OdomStatePayload_t;

typedef struct {
    uint64_t t_sample_us;
    uint16_t status_bits;
    uint8_t  quality;
    uint8_t  link_state;
} OdomStatusPayload_t;

typedef struct {
    uint64_t host_time_us;
} OdomTimeSyncReqPayload_t;

typedef struct {
    uint64_t echoed_host_time_us;
    uint64_t mcu_time_us;
} OdomTimeSyncRespPayload_t;

typedef struct {
    float    x;
    float    y;
    float    yaw;
    uint8_t  flags;        /* bit0=reset_xy, bit1=reset_yaw, 0 时上位机一般会发 0 表示重置全部 */
    uint8_t  reserved[3];
} OdomSetLocalOriginPayload_t;

typedef struct {
    uint16_t acked_seq;     /* 回显请求 seq (低 8 位有意义) */
    uint16_t result_code;   /* 0 = 成功 */
    uint32_t event_counter; /* 累计成功归零次数 */
} OdomSetLocalOriginAckPayload_t;

#pragma pack(pop)

/* ---------- 上行帧解析结果 ---------- */
typedef struct {
    uint8_t        msg_type;
    uint8_t        seq;
    uint16_t       payload_len;
    const uint8_t *payload;   /* 指向原缓冲区的指针 */
} OdomUpstreamFrame_t;

/* ---------- YAW unwrap 状态 ---------- */
typedef struct {
    float prev_deg;       /* 上一次 YAW_ANGLE (度, [-180,+180]) */
    float continuous_rad; /* 连续弧度输出 */
    uint8_t initialized;
} YawUnwrap_t;

/* ---------- 函数声明 ---------- */

/* CRC-16/CCITT (初始值 0xFFFF, 多项式 0x1021) */
uint16_t odom_crc16(const uint8_t *data, uint16_t len);

/* YAW unwrap：输入度 [-180,+180]，输出连续弧度 */
float odom_yaw_unwrap(YawUnwrap_t *state, float yaw_deg);

/* 组帧：返回帧总长度，buf 需 >= ODOM_STATE_FRAME_LEN */
uint16_t odom_pack_pose(uint8_t *buf, uint8_t seq, const OdomPosePayload_t *payload);
uint16_t odom_pack_state(uint8_t *buf, uint8_t seq, const OdomStatePayload_t *payload);
uint16_t odom_pack_status(uint8_t *buf, uint8_t seq, const OdomStatusPayload_t *payload);
uint16_t odom_pack_time_sync_resp(uint8_t *buf, uint8_t seq, const OdomTimeSyncRespPayload_t *payload);
uint16_t odom_pack_set_origin_ack(uint8_t *buf, uint8_t seq, const OdomSetLocalOriginAckPayload_t *payload);

/* 上行帧解析：在 [data, data+len) 中扫描第一帧合法的 0xAA 0x55 帧。
 * 返回值: 1=成功并填充 out_frame; 0=没有找到完整合法帧 (CRC 错或长度不足)。
 */
int odom_parse_upstream(const uint8_t *data, uint16_t len, OdomUpstreamFrame_t *out_frame);

/* 全局 ISR tick 计数器 (每 TIM11 中断 +1, 50us/tick) */
extern volatile uint64_t odom_isr_tick;

/* 全局 unwrap 状态 */
extern YawUnwrap_t g_yaw_unwrap;

#endif /* ODOM_PROTOCOL_H */
