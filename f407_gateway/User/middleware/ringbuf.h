#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdint.h>

/* 轻量串口接收环形缓冲。
 * 与 F103 端已验证方案一致：RXNE 中断每收 1 字节入队，主循环查询。
 * 解决 HAL_UART_Receive 在高速多字节场景下的丢字节问题。 */
typedef struct
{
    uint8_t  buf[64];
    volatile uint16_t head;   /* 写指针（中断中推进） */
    volatile uint16_t tail;   /* 读指针（主循环中推进） */
} ringbuf_t;

static inline void ringbuf_init(ringbuf_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
}

static inline uint8_t ringbuf_full(const ringbuf_t *rb)
{
    return ((rb->head + 1) % 64) == rb->tail;
}

static inline void ringbuf_put(ringbuf_t *rb, uint8_t c)
{
    if (ringbuf_full(rb)) return;   /* 满则丢弃，避免覆盖未处理数据 */
    rb->buf[rb->head] = c;
    rb->head = (rb->head + 1) % 64;
}

static inline int ringbuf_get(ringbuf_t *rb, uint8_t *c)
{
    if (rb->head == rb->tail) return 0;
    *c = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % 64;
    return 1;
}

static inline uint16_t ringbuf_avail(const ringbuf_t *rb)
{
    return (rb->head - rb->tail + 64) % 64;
}

#endif /* RINGBUF_H */
