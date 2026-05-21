#include "odom_protocol.h"
#include <string.h>

/* ---------- 全局变量 ---------- */
volatile uint64_t odom_isr_tick = 0;
YawUnwrap_t g_yaw_unwrap = {0.0f, 0.0f, 0};

/* ---------- CRC-16/CCITT ---------- */
/* 多项式 0x1021, 初始值 0xFFFF, 无反转 */
uint16_t odom_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ---------- YAW unwrap ---------- */
float odom_yaw_unwrap(YawUnwrap_t *state, float yaw_deg)
{
    if (!state->initialized) {
        state->prev_deg = yaw_deg;
        state->continuous_rad = yaw_deg * 0.0174532925f;
        state->initialized = 1;
        return state->continuous_rad;
    }

    float delta = yaw_deg - state->prev_deg;
    /* 处理 ±180° 翻转 */
    if (delta > 180.0f)
        delta -= 360.0f;
    else if (delta < -180.0f)
        delta += 360.0f;

    state->continuous_rad += delta * 0.0174532925f;
    state->prev_deg = yaw_deg;
    return state->continuous_rad;
}

/* ---------- 内部：写帧头 + payload + CRC ---------- */
static uint16_t odom_pack_frame(uint8_t *buf, uint8_t msg_type, uint8_t seq,
                                const void *payload, uint16_t payload_len)
{
    uint16_t idx = 0;

    /* Header */
    buf[idx++] = ODOM_FRAME_HEADER_0;
    buf[idx++] = ODOM_FRAME_HEADER_1;
    /* Version */
    buf[idx++] = ODOM_FRAME_VERSION;
    /* Type */
    buf[idx++] = msg_type;
    /* Sequence */
    buf[idx++] = seq;
    /* Length (little-endian) */
    buf[idx++] = (uint8_t)(payload_len & 0xFF);
    buf[idx++] = (uint8_t)((payload_len >> 8) & 0xFF);
    /* Payload */
    memcpy(&buf[idx], payload, payload_len);
    idx += payload_len;
    /* CRC16 over [Version .. Payload] */
    uint16_t crc = odom_crc16(&buf[2], idx - 2);
    buf[idx++] = (uint8_t)(crc & 0xFF);
    buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    return idx;
}

/* ---------- 公开打包接口 ---------- */
uint16_t odom_pack_pose(uint8_t *buf, uint8_t seq, const OdomPosePayload_t *payload)
{
    return odom_pack_frame(buf, ODOM_MSG_POSE, seq, payload, ODOM_POSE_PAYLOAD_LEN);
}

uint16_t odom_pack_state(uint8_t *buf, uint8_t seq, const OdomStatePayload_t *payload)
{
    return odom_pack_frame(buf, ODOM_MSG_STATE, seq, payload, ODOM_STATE_PAYLOAD_LEN);
}

uint16_t odom_pack_status(uint8_t *buf, uint8_t seq, const OdomStatusPayload_t *payload)
{
    return odom_pack_frame(buf, ODOM_MSG_STATUS, seq, payload, ODOM_STATUS_PAYLOAD_LEN);
}

uint16_t odom_pack_time_sync_resp(uint8_t *buf, uint8_t seq, const OdomTimeSyncRespPayload_t *payload)
{
    return odom_pack_frame(buf, ODOM_MSG_TIME_SYNC_RESP, seq, payload, ODOM_TIME_SYNC_RESP_PAYLOAD_LEN);
}

uint16_t odom_pack_set_origin_ack(uint8_t *buf, uint8_t seq, const OdomSetLocalOriginAckPayload_t *payload)
{
    return odom_pack_frame(buf, ODOM_MSG_SET_LOCAL_ORIGIN_ACK, seq, payload, ODOM_SET_ORIGIN_ACK_PAYLOAD_LEN);
}

/* ---------- 上行帧解析 ---------- *
 * 协议：AA 55 ver(0x01) type seq payloadLen(LE16) payload CRC16(LE)
 * CRC 范围: byte[2] .. payload 末尾 (与下行一致)
 * 在缓冲中找第一个完整合法帧；若 CRC 错或长度不够，返回 0。
 */
int odom_parse_upstream(const uint8_t *data, uint16_t len, OdomUpstreamFrame_t *out_frame)
{
    if (data == 0 || out_frame == 0) return 0;

    uint16_t i = 0;
    /* 至少要 9 字节 (overhead) 才能解析出 payload_len */
    while (i + ODOM_FRAME_OVERHEAD <= len) {
        if (data[i] != ODOM_FRAME_HEADER_0 || data[i + 1] != ODOM_FRAME_HEADER_1) {
            i++;
            continue;
        }
        if (data[i + 2] != ODOM_FRAME_VERSION) {
            i++;
            continue;
        }
        uint8_t  msg_type    = data[i + 3];
        uint8_t  seq         = data[i + 4];
        uint16_t payload_len = (uint16_t)data[i + 5] | ((uint16_t)data[i + 6] << 8);

        /* 合法 payload_len 上限 = 上行最长 (16) */
        if (payload_len > ODOM_SET_ORIGIN_PAYLOAD_LEN) {
            i++;
            continue;
        }
        uint16_t total = (uint16_t)ODOM_FRAME_OVERHEAD + payload_len;
        if (i + total > len) {
            /* 帧不完整，等待更多数据；直接返回 0 */
            return 0;
        }

        /* CRC 范围 = ver..payload 末尾 = 1+1+1+2+payload_len = 5+payload_len 字节 */
        uint16_t crc_calc = odom_crc16(&data[i + 2], (uint16_t)(5 + payload_len));
        uint16_t crc_recv = (uint16_t)data[i + 7 + payload_len]
                          | ((uint16_t)data[i + 8 + payload_len] << 8);
        if (crc_calc != crc_recv) {
            i++;  /* CRC 错，跳一字节继续找 */
            continue;
        }

        out_frame->msg_type    = msg_type;
        out_frame->seq         = seq;
        out_frame->payload_len = payload_len;
        out_frame->payload     = &data[i + 7];
        return 1;
    }
    return 0;
}
