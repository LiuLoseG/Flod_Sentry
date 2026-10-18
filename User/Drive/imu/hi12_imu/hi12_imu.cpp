/**
 * @file    hi12_imu.cpp
 * @brief   HiPNUC HI12 系列 IMU 协议解析
 * @note    基于官方 hipnuc_dec.c 逐字节状态机 + CRC16-CCITT
 *
 * 协议帧:
 *  | SYNC1 | SYNC2 | LEN_L | LEN_H | CRC16_L | CRC16_H |  DATA...  |
 *  | 0x5A  | 0xA5  |  2B payload len (LE)  | CRC16 over all above   |
 *
 *  CH_HDR_SIZE = 6 (SYNC2 + LEN2 + CRC162)
 *  CRC16 多项式 0x1021, 初值 0
 *  计算范围: SYNC(2) + LEN(2) + DATA(payload_len)
 */

#include "hi12_imu.h"

volatile hi12_gyro hi12_data;

/* ===========================================================================
 * CRC16-CCITT 查表 (poly=0x1021, init=0, 非反射, MSB优先)
 *
 * 表生成逻辑:
 *   for (i = 0; i < 256; i++) {
 *       uint16_t c = (uint16_t)i << 8;
 *       for (int j = 0; j < 8; j++)
 *           c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
 *       table[i] = c;
 *   }
 * =========================================================================== */
static const uint16_t crc16_ccitt_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/**
 * @brief CRC16-CCITT 查表法 (poly=0x1021, init=0, 非反射, MSB优先)
 * @param crc  当前 CRC 值指针 (初值 0, 结果写回)
 * @param buf  数据缓冲区
 * @param len  数据长度
 * @note  与逐位法结果完全一致, 速度约 8~10x
 *        查表公式: crc = (crc << 8) ^ table[(crc >> 8) ^ byte]
 */
void hi12_gyro::crc16_ccitt(uint16_t *crc, const uint8_t *buf, uint32_t len)
{
    uint16_t c = *crc;
    for (uint32_t j = 0; j < len; ++j) {
        c = (c << 8) ^ crc16_ccitt_table[(c >> 8) ^ buf[j]];
    }
    *crc = c;
}

/* ===========================================================================
 * 逐字节输入 — 状态机核心 (参照官方 hipnuc_input)
 *
 * 状态:
 *   nbyte==0  搜索 SYNC, 匹配后 nbyte=2
 *   nbyte 2~5 填充帧头 (LEN + CRC16)
 *   nbyte==6  提取 payload_len, 校验合法性
 *   nbyte >6  填充数据载荷
 *   nbyte >= CH_HDR_SIZE + payload_len  收完一帧 → CRC16 → 解析
 *
 * @return 1=解码成功  0=等待更多数据  -1=错误(已自动复位)
 * =========================================================================== */
int8_t hi12_gyro::feed_byte(uint8_t data) volatile
{
    /* ---- 搜索 SYNC ---- */
    if (nbyte == 0) {
        raw_buf[0] = raw_buf[1];
        raw_buf[1] = data;
        if (raw_buf[0] != CHSYNC1 || raw_buf[1] != CHSYNC2)
            return 0;
        nbyte = 2;
        return 0;
    }

    /* ---- 接收字节 ---- */
    raw_buf[nbyte++] = data;

    /* ---- 帧头收齐: 提取载荷长度 ---- */
    if (nbyte == CH_HDR_SIZE) {
        payload_len = (int)read_u16((const uint8_t *)&raw_buf[2]);
        if (payload_len > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE)) {
            nbyte = 0;
            return -1;
        }
    }

    /* ---- 等待载荷收完 ---- */
    if (nbyte < CH_HDR_SIZE || nbyte < (payload_len + CH_HDR_SIZE))
        return 0;

    /* ---- 帧完整, 复位并解码 ---- */
    nbyte = 0;
    return decode();
}

/* ===========================================================================
 * 帧解码 — CRC16 校验 → 解析载荷
 * =========================================================================== */
int8_t hi12_gyro::decode() volatile
{
    uint16_t crc = 0;

    /* CRC over SYNC(2) + LEN(2) = raw_buf[0..3] */
    crc16_ccitt(&crc, (const uint8_t *)raw_buf, CH_HDR_SIZE - 2);
    /* CRC over DATA = raw_buf[CH_HDR_SIZE .. ] */
    crc16_ccitt(&crc, (const uint8_t *)raw_buf + CH_HDR_SIZE, payload_len);

    if (crc != read_u16((const uint8_t *)raw_buf + (CH_HDR_SIZE - 2)))
        return -1;

    parse_payload();
    return 1;
}

/* ===========================================================================
 * 载荷解析 — 遍历 tagged 子包 (参照官方 parse_data)
 * =========================================================================== */
void hi12_gyro::parse_payload() volatile
{
    const uint8_t *p = (const uint8_t *)raw_buf + CH_HDR_SIZE;
    int ofs = 0;

    /* 清除上次标签 */
    hi91.tag = 0;
    hi81.tag = 0;
    hi83.tag = 0;

    while (ofs < payload_len) {
        switch (p[ofs]) {

        case HIPNUC_ID_HI91:
            memcpy((void *)&hi91, p + ofs, sizeof(hi91_t));
            ofs += sizeof(hi91_t);
            break;

        case HIPNUC_ID_HI81:
            memcpy((void *)&hi81, p + ofs, sizeof(hi81_t));
            ofs += sizeof(hi81_t);
            break;

        case HIPNUC_ID_HI83: {
            /* 固定头: tag(1) + main_status(2) + ins_status(1) + bitmap(4) */
            hi83.tag         = HIPNUC_ID_HI83;
            hi83.main_status = read_u16(p + ofs + 1);
            hi83.ins_status  = p[ofs + 3];
            hi83.data_bitmap = read_u32(p + ofs + 4);

            int idx = ofs + 8;
            uint32_t bm = hi83.data_bitmap;

            if (bm & HI83_BMAP_ACC_B)        { hi83.acc_b[0] = read_f32(p+idx+ 0); hi83.acc_b[1] = read_f32(p+idx+ 4); hi83.acc_b[2] = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_GYR_B)        { hi83.gyr_b[0] = read_f32(p+idx+ 0); hi83.gyr_b[1] = read_f32(p+idx+ 4); hi83.gyr_b[2] = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_MAG_B)        { hi83.mag_b[0] = read_f32(p+idx+ 0); hi83.mag_b[1] = read_f32(p+idx+ 4); hi83.mag_b[2] = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_RPY)          { hi83.rpy[0]   = read_f32(p+idx+ 0); hi83.rpy[1]   = read_f32(p+idx+ 4); hi83.rpy[2]   = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_QUAT)         { hi83.quat[0]  = read_f32(p+idx+ 0); hi83.quat[1]  = read_f32(p+idx+ 4); hi83.quat[2]  = read_f32(p+idx+ 8); hi83.quat[3]  = read_f32(p+idx+12); idx += 16; }
            if (bm & HI83_BMAP_SYSTEM_TIME)  { hi83.system_time_us = read_u64(p + idx); idx += 8; }
            if (bm & HI83_BMAP_UTC)          { hi83.utc.year=p[idx+0]; hi83.utc.month=p[idx+1]; hi83.utc.day=p[idx+2]; hi83.utc.hour=p[idx+3]; hi83.utc.min=p[idx+4]; hi83.utc.sec_ms=read_u16(p+idx+5); hi83.utc.rev=p[idx+7]; idx += 8; }
            if (bm & HI83_BMAP_AIR_PRESSURE) { hi83.air_pressure     = read_f32(p + idx); idx += 4; }
            if (bm & HI83_BMAP_TEMPERATURE)  { hi83.temperature      = read_f32(p + idx); idx += 4; }
            if (bm & HI83_BMAP_INCLINATION)  { hi83.inclination[0]   = read_f32(p+idx+ 0); hi83.inclination[1]   = read_f32(p+idx+ 4); hi83.inclination[2]   = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_HSS)          { hi83.hss[0]           = read_f32(p+idx+ 0); hi83.hss[1]           = read_f32(p+idx+ 4); hi83.hss[2]           = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_HSS_FRQ)      { hi83.hss_frq[0]       = read_f32(p+idx+ 0); hi83.hss_frq[1]       = read_f32(p+idx+ 4); hi83.hss_frq[2]       = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_VEL_ENU)      { hi83.vel_enu[0]       = read_f32(p+idx+ 0); hi83.vel_enu[1]       = read_f32(p+idx+ 4); hi83.vel_enu[2]       = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_ACC_ENU)      { hi83.acc_enu[0]       = read_f32(p+idx+ 0); hi83.acc_enu[1]       = read_f32(p+idx+ 4); hi83.acc_enu[2]       = read_f32(p+idx+ 8); idx += 12; }
            if (bm & HI83_BMAP_INS_LON_LAT_MSL) { hi83.ins_lon_lat_msl[0]=read_f64(p+idx+ 0); hi83.ins_lon_lat_msl[1]=read_f64(p+idx+ 8); hi83.ins_lon_lat_msl[2]=read_f64(p+idx+16); idx += 24; }
            if (bm & HI83_BMAP_GNSS_QUALITY_NV){ hi83.solq_pos=p[idx+0]; hi83.nv_pos=p[idx+1]; hi83.solq_heading=p[idx+2]; hi83.nv_heading=p[idx+3]; idx += 4; }
            if (bm & HI83_BMAP_OD_SPEED)     { hi83.od_speed          = read_f32(p + idx); idx += 4; }
            if (bm & HI83_BMAP_UNDULATION)   { hi83.undulation        = read_f32(p + idx); idx += 4; }
            if (bm & HI83_BMAP_DIFF_AGE)     { hi83.diff_age          = read_f32(p + idx); idx += 4; }
            if (bm & HI83_BMAP_NODE_ID)      { hi83.node.node_id      = p[idx+0]; hi83.node.reserved[0]=p[idx+1]; hi83.node.reserved[1]=p[idx+2]; hi83.node.reserved[2]=p[idx+3]; idx += 4; }
            if (bm & HI83_BMAP_GNSS_LON_LAT_MSL){ hi83.gnss_lon_lat_msl[0]=read_f64(p+idx+ 0); hi83.gnss_lon_lat_msl[1]=read_f64(p+idx+ 8); hi83.gnss_lon_lat_msl[2]=read_f64(p+idx+16); idx += 24; }
            if (bm & HI83_BMAP_GNSS_VEL)     { hi83.gnss_vel[0]       = read_f32(p+idx+ 0); hi83.gnss_vel[1]       = read_f32(p+idx+ 4); hi83.gnss_vel[2]       = read_f32(p+idx+ 8); idx += 12; }

            ofs = idx;
            break;
        }

        default:
            ++ofs;
            break;
        }
    }
}

/* ===========================================================================
 * 公开接口
 * =========================================================================== */

/**
 * @brief 初始化
 */
void hi12_gyro::init(uart_manger_object *uart_manager) volatile
{
    hi12_uart_mgr = uart_manager;
    nbyte         = 0;
    payload_len   = 0;
    memset((void *)raw_buf, 0, sizeof(raw_buf));
    memset((void *)&hi91,   0, sizeof(hi91));
    memset((void *)&hi81,   0, sizeof(hi81));
    memset((void *)&hi83,   0, sizeof(hi83));
}

/**
 * @brief DMA 双缓冲回调入口 — 逐字节送入状态机
 */
void hi12_gyro::parse_buffer(uint8_t *buf, uint16_t len) volatile
{
    if (!buf || !len) return;
    for (uint16_t i = 0; i < len; ++i)
        feed_byte(buf[i]);
}

/**
 * @brief UART 回调
 */
void hi12_gyro_uart_callback(uint8_t *buf, uint16_t len)
{
    hi12_data.parse_buffer(buf, len);
}

void h12_imu_init(uart_manger_object *uart_manager)
{
    hi12_data.init(uart_manager);
}