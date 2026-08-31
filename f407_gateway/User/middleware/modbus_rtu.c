#include "modbus_rtu.h"

/* ----------------------------- CRC16 ----------------------------- */
uint16_t mb_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)buf[i];
        for (int b = 0; b < 8; b++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;   /* 线上先发低字节，再发高字节 */
}

/* ------------------------- 请求帧构建 ------------------------- */
uint16_t mb_build_read_req(uint8_t *out, uint8_t addr, uint16_t reg, uint16_t count)
{
    out[0] = addr;
    out[1] = MB_FC_READ_HOLDING;
    out[2] = (reg >> 8) & 0xFF;
    out[3] = reg & 0xFF;
    out[4] = (count >> 8) & 0xFF;
    out[5] = count & 0xFF;
    uint16_t crc = mb_crc16(out, 6);
    out[6] = crc & 0xFF;
    out[7] = (crc >> 8) & 0xFF;
    return 8;
}

uint16_t mb_build_write_single_req(uint8_t *out, uint8_t addr, uint16_t reg, uint16_t val)
{
    out[0] = addr;
    out[1] = MB_FC_WRITE_SINGLE;
    out[2] = (reg >> 8) & 0xFF;
    out[3] = reg & 0xFF;
    out[4] = (val >> 8) & 0xFF;
    out[5] = val & 0xFF;
    uint16_t crc = mb_crc16(out, 6);
    out[6] = crc & 0xFF;
    out[7] = (crc >> 8) & 0xFF;
    return 8;
}

uint16_t mb_build_write_multi_req(uint8_t *out, uint8_t addr, uint16_t reg,
                                  uint16_t count, const uint16_t *vals)
{
    uint16_t idx = 0;
    out[idx++] = addr;
    out[idx++] = MB_FC_WRITE_MULTI;
    out[idx++] = (reg >> 8) & 0xFF;
    out[idx++] = reg & 0xFF;
    out[idx++] = (count >> 8) & 0xFF;
    out[idx++] = count & 0xFF;
    out[idx++] = (uint8_t)(count * 2);   /* 字节数 */
    for (uint16_t i = 0; i < count; i++)
    {
        out[idx++] = (vals[i] >> 8) & 0xFF;
        out[idx++] = vals[i] & 0xFF;
    }
    uint16_t crc = mb_crc16(out, idx);
    out[idx++] = crc & 0xFF;
    out[idx++] = (crc >> 8) & 0xFF;
    return idx;
}

/* ------------------------- 帧校验/解析 ------------------------- */
int mb_check_frame(const uint8_t *frame, uint16_t len)
{
    if (len < 4) return 0;
    uint16_t crc = mb_crc16(frame, len - 2);
    uint16_t recv = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    return (crc == recv) ? 1 : 0;
}

int mb_parse_read_resp(const uint8_t *frame, uint16_t len,
                       uint16_t *out_regs, uint16_t max_regs)
{
    if (len < 5) return -1;
    if (frame[1] == (MB_FC_READ_HOLDING | 0x80)) return -2;  /* 异常 */
    if (frame[1] != MB_FC_READ_HOLDING) return -3;

    uint8_t nbytes = frame[2];
    uint16_t nregs = nbytes / 2;
    if (nregs > max_regs) nregs = max_regs;

    uint16_t idx = 3;
    for (uint16_t i = 0; i < nregs; i++)
    {
        out_regs[i] = ((uint16_t)frame[idx] << 8) | frame[idx + 1];
        idx += 2;
    }
    return (int)nregs;
}

/* --------------------------- 从站处理 --------------------------- */
uint16_t mb_slave_handle(const uint8_t *req, uint16_t req_len,
                         uint8_t *resp, uint16_t resp_max,
                         uint8_t my_addr,
                         uint16_t (*read_reg)(uint16_t reg),
                         int (*write_reg)(uint16_t reg, uint16_t val))
{
    if (req_len < 4) return 0;

    uint8_t addr = req[0];
    uint8_t fc   = req[1];

    /* 地址过滤：非本机地址一律不响应（避免总线上其他从站被抢答） */
    if (addr != my_addr) return 0;

    uint16_t idx = 0;
    uint16_t crc = 0;

    if (fc == MB_FC_READ_HOLDING)
    {
        uint16_t reg   = ((uint16_t)req[2] << 8) | req[3];
        uint16_t count = ((uint16_t)req[4] << 8) | req[5];
        if (count == 0 || count > MB_MAX_REGS) goto except_illegal_data;

        resp[idx++] = addr;
        resp[idx++] = fc;
        resp[idx++] = (uint8_t)(count * 2);
        for (uint16_t i = 0; i < count; i++)
        {
            uint16_t v = read_reg(reg + i);
            resp[idx++] = (v >> 8) & 0xFF;
            resp[idx++] = v & 0xFF;
        }
        goto append_crc;
    }
    else if (fc == MB_FC_WRITE_SINGLE)
    {
        uint16_t reg = ((uint16_t)req[2] << 8) | req[3];
        uint16_t val = ((uint16_t)req[4] << 8) | req[5];
        if (write_reg && write_reg(reg, val) != 0) goto except_illegal_addr;
        /* 响应 = 回显请求（0x06 标准） */
        for (int i = 0; i < 6; i++) resp[idx++] = req[i];
        goto append_crc;
    }
    else if (fc == MB_FC_WRITE_MULTI)
    {
        uint16_t reg   = ((uint16_t)req[2] << 8) | req[3];
        uint16_t count = ((uint16_t)req[4] << 8) | req[5];
        uint8_t  bc    = req[6];
        if (count == 0 || count > MB_MAX_REGS || bc != count * 2) goto except_illegal_data;
        uint16_t p = 7;
        for (uint16_t i = 0; i < count; i++)
        {
            uint16_t v = ((uint16_t)req[p] << 8) | req[p + 1];
            p += 2;
            if (write_reg && write_reg(reg + i, v) != 0) goto except_illegal_addr;
        }
        /* 响应：addr fc reg_h reg_l count_h count_l crc */
        resp[idx++] = addr;
        resp[idx++] = fc;
        resp[idx++] = (reg >> 8) & 0xFF;
        resp[idx++] = reg & 0xFF;
        resp[idx++] = (count >> 8) & 0xFF;
        resp[idx++] = count & 0xFF;
        goto append_crc;
    }
    else
    {
        /* 不支持的功能码 -> 异常 0x01 (illegal function) */
        resp[idx++] = addr;
        resp[idx++] = (uint8_t)(fc | 0x80);
        resp[idx++] = 0x01;
        goto append_crc;
    }

except_illegal_data:
    resp[idx++] = addr;
    resp[idx++] = (uint8_t)(fc | 0x80);
    resp[idx++] = 0x03;   /* illegal data value */
    goto append_crc;

except_illegal_addr:
    resp[idx++] = addr;
    resp[idx++] = (uint8_t)(fc | 0x80);
    resp[idx++] = 0x02;   /* illegal data address */
    /* fall through */

append_crc:
    crc = mb_crc16(resp, idx);
    resp[idx++] = crc & 0xFF;
    resp[idx++] = (crc >> 8) & 0xFF;
    return idx;
}
