#include "modbus_rtu.h"

static char atox(char c)
{
    if (c <= '9') return c - '0';   /* 0-9 */
    else return c - '7';            /* A-F */
}

static char xtoa(char c)
{
    if (c <= (char)9) return c + '0';   /* 0-9 */
    else return c + '7';                /* A-F */
}

int modbus_rtu_init(struct modbus_rtu *ctx, struct uart *uart)
{
    ctx->uart = uart;
    ctx->state = MODBUS_RTU_STATE_START;
    ctx->tick = 0;

    ctx->c = (char)0;
    ctx->count = (size_t)0;
    ctx->lrc = 0;

    return 0;
}

static int read_start(struct modbus_rtu *ctx)
{
    char c = (char)0;

    if (uart_read(ctx->uart, &c, sizeof(c)) != -EAGAIN && c == ':') {
        ctx->state = MODBUS_RTU_STATE_ADDR_FUNC;
        ctx->tick = picoRTOS_get_tick();
        ctx->count = (size_t)4;
        return 0;
    }

    return -EAGAIN;
}

static int read_addr_func(struct modbus_rtu *ctx, /*@partial@*/ struct modbus_rtu_frame *frame)
{
    char c = (char)0;

    /* timeout */
    picoRTOS_assert((picoRTOS_get_tick() - ctx->tick) < MODBUS_RTU_TIMEOUT,
                    ctx->state = MODBUS_RTU_STATE_START; return -EPIPE);

    /* wait for 4 bytes of data */
    if (uart_read(ctx->uart, &c, sizeof(c)) != -EAGAIN) {
        switch (--ctx->count) {
        case 3:
            frame->address = (uint8_t)atox(c) << 4;
            break;
        case 2:
            frame->address |= (uint8_t)atox(c);
            break;
        case 1:
            frame->function = (uint8_t)atox(c) << 4;
            break;
        case 0:
            frame->function |= (uint8_t)atox(c);
            ctx->state = MODBUS_RTU_STATE_DATA;
            ctx->data_state = MODBUS_RTU_DATA_STATE_H;
            /* LRC */
            ctx->lrc = frame->address + frame->function;
            break;
        }
        return 0;
    }

    return -EAGAIN;
}

static int read_data(struct modbus_rtu *ctx, /*@partial@*/ struct modbus_rtu_frame *frame)
{
    char c = (char)0;

    /* buffer overflow */
    picoRTOS_assert(ctx->count < (size_t)MODBUS_RTU_DATA_COUNT,
                    ctx->state = MODBUS_RTU_STATE_START; return -EAGAIN);
    /* timeout */
    picoRTOS_assert((picoRTOS_get_tick() - ctx->tick) < MODBUS_RTU_TIMEOUT,
                    ctx->state = MODBUS_RTU_STATE_START; return -EPIPE);

    if (uart_read(ctx->uart, &c, sizeof(c)) != -EAGAIN) {
        /* CR/LF */
        if (c == '\r') {
            ctx->state = MODBUS_RTU_STATE_END;
            return 0;
        }
        /* DATA */
        switch (ctx->data_state) {
        case MODBUS_RTU_DATA_STATE_H:
            ctx->c = (uint8_t)atox(c) << 4;
            ctx->data_state = MODBUS_RTU_DATA_STATE_L;
            break;

        case MODBUS_RTU_DATA_STATE_L:
            ctx->c |= (uint8_t)atox(c);
            frame->data[ctx->count++] = ctx->c;
            ctx->data_state = MODBUS_RTU_DATA_STATE_H;
            /* lrc */
            ctx->lrc += ctx->c;
            break;

        default:
            ctx->state = MODBUS_RTU_STATE_START;
            picoRTOS_break();
            /*@notreached@*/ return -EIO;
        }

        /* back to main loop */
        return 0;
    }

    return -EAGAIN;
}

static int read_end(struct modbus_rtu *ctx)
{
    char c = (char)0;

    /* timeout */
    picoRTOS_assert((picoRTOS_get_tick() - ctx->tick) < MODBUS_RTU_TIMEOUT,
                    ctx->state = MODBUS_RTU_STATE_START; return -EPIPE);

    if (uart_read(ctx->uart, &c, sizeof(c)) != -EAGAIN) {
        /* ignore last char */
        ctx->state = MODBUS_RTU_STATE_START;
        /* check LRC */
#ifndef RX_IGNORE_LRC
        if (ctx->lrc != 0)
            return -ENOENT;
#endif

        /* return count - lrc */
        return (int)ctx->count - 1;
    }

    return -EAGAIN;
}

int modbus_rtu_read(struct modbus_rtu *ctx, /*@out@*/ struct modbus_rtu_frame *frame)
{
    int res = 0;

    for (;;) {
        switch (ctx->state) {
        case MODBUS_RTU_STATE_START: res = read_start(ctx); break;
        case MODBUS_RTU_STATE_ADDR_FUNC: res = read_addr_func(ctx, frame); break;
        case MODBUS_RTU_STATE_DATA: res = read_data(ctx, frame); break;
        case MODBUS_RTU_STATE_END: res = read_end(ctx); break;
        default:
            picoRTOS_break();
            /*@notreached@*/ return -EIO;
        }

        if (res != 0)
            break;
    }

    return res;
}

static int write_start(struct modbus_rtu *ctx)
{
    if (uart_write(ctx->uart, ":", (size_t)1) != -EAGAIN) {
        ctx->state = MODBUS_RTU_STATE_ADDR_FUNC;
        ctx->count = (size_t)4;
        return 0;
    }

    return -EAGAIN;
}

static int write_addr_func(struct modbus_rtu *ctx, const struct modbus_rtu_frame *frame)
{
    char c;

    switch (ctx->count) {
    case 4: c = xtoa((char)(frame->address >> 4)); break;
    case 3: c = xtoa((char)(frame->address & 0xf)); break;
    case 2: c = xtoa((char)(frame->function >> 4)); break;
    case 1: c = xtoa((char)(frame->function & 0xf)); break;
    default:
        ctx->state = MODBUS_RTU_STATE_START;
        picoRTOS_break();
        /*@notreached@*/ return -EIO;
    }

    if (uart_write(ctx->uart, &c, sizeof(c)) != -EAGAIN) {
        /* -> DATA */
        if (--ctx->count == 0) {
            ctx->state = MODBUS_RTU_STATE_DATA;
            ctx->data_state = MODBUS_RTU_DATA_STATE_H;
            /* lrc */
            ctx->lrc = frame->address + frame->function;
        }
        /* back to main loop */
        return 0;
    }

    return -EAGAIN;
}

static int write_data(struct modbus_rtu *ctx, const struct modbus_rtu_frame *frame, size_t n)
{
    picoRTOS_assert(n > 0, return -EINVAL);

    char c;
    uint8_t data = frame->data[ctx->count];

    /* DATA */
    switch (ctx->data_state) {
    case MODBUS_RTU_DATA_STATE_H:
        c = xtoa((char)(data >> 4));
        if (uart_write(ctx->uart, &c, sizeof(c)) != -EAGAIN) {
            ctx->data_state = MODBUS_RTU_DATA_STATE_L;
            return 0;
        }
        break;

    case MODBUS_RTU_DATA_STATE_L:
        c = xtoa((char)(data & 0xf));
        if (uart_write(ctx->uart, &c, sizeof(c)) != -EAGAIN) {
            /* LRC */
            ctx->lrc += data;
            /* Last byte */
            if (++ctx->count == n) {
                ctx->state = MODBUS_RTU_STATE_END;
                ctx->count = (size_t)4;
                ctx->lrc = (uint8_t)-(char)ctx->lrc;
            }
            /* next c */
            ctx->data_state = MODBUS_RTU_DATA_STATE_H;
            return 0;
        }
        break;

    default:
        ctx->state = MODBUS_RTU_STATE_START;
        picoRTOS_break();
        /*@notreached @*/ return -EIO;
    }

    return -EAGAIN;
}

static int write_end(struct modbus_rtu *ctx, size_t n)
{
    char c;

    switch (ctx->count) {
    case 4: c = xtoa((char)(ctx->lrc >> 4)); break;
    case 3: c = xtoa((char)(ctx->lrc & 0xf)); break;
    case 2: c = '\r'; break;
    case 1: c = '\n'; break;
    default:
        ctx->state = MODBUS_RTU_STATE_START;
        picoRTOS_break();
        /*@notreached@*/ return -EIO;
    }

    if (uart_write(ctx->uart, &c, sizeof(c)) != -EAGAIN) {
        if (--ctx->count == 0) {
            ctx->state = MODBUS_RTU_STATE_START;
            return (int)n;
        }
        /* back to main loop */
        return 0;
    }

    return -EAGAIN;
}

int modbus_rtu_write(struct modbus_rtu *ctx, const struct modbus_rtu_frame *frame, size_t n)
{
    int res = 0;

    for (;;) {
        switch (ctx->state) {
        case MODBUS_RTU_STATE_START: res = write_start(ctx); break;
        case MODBUS_RTU_STATE_ADDR_FUNC: res = write_addr_func(ctx, frame); break;
        case MODBUS_RTU_STATE_DATA: res = write_data(ctx, frame, n); break;
        case MODBUS_RTU_STATE_END: res = write_end(ctx, n); break;
        default:
            picoRTOS_break();
            /*@notreached@*/ return -EIO;
        }

        if (res != 0)
            break;
    }

    return res;
}
