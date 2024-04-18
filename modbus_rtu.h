#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

/**
 * @file modbus_rtu.h
 * @brief The main picoRTOS MODBUS RTU library header
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* picoRTOS includes */
#include "uart.h"
#include "picoRTOS.h"

/*
 * MODBUS_RTU, only ASCII framing support
 */

typedef enum {
    MODBUS_RTU_STATE_START,
    MODBUS_RTU_STATE_ADDR_FUNC,
    MODBUS_RTU_STATE_FUNCTION,
    MODBUS_RTU_STATE_DATA,
    MODBUS_RTU_STATE_END,
    MODBUS_RTU_STATE_COUNT
} modbus_rtu_state_t;

#define MODBUS_RTU_TIMEOUT PICORTOS_DELAY_SEC(1)

/*
 * You might want to use -DMODBUS_RTU_BUFLEN=
 * in your CFLAGS to reduce momory consumption
 * on very small systems
 */
#ifndef MODBUS_RTU_BUFLEN
# define MODBUS_RTU_BUFLEN 256
#endif

typedef enum {
    MODBUS_RTU_DATA_STATE_H,
    MODBUS_RTU_DATA_STATE_L,
    MODBUS_RTU_DATA_STATE_COUNT
} modbus_rtu_data_state_t;

/*
 * main object
 */
struct modbus_rtu {
    /*@temp@*/ struct uart *uart;
    modbus_rtu_state_t state;
    picoRTOS_tick_t tick;
    /* buffering */
    uint8_t c;
    size_t count;
    modbus_rtu_data_state_t data_state;
    /* checksum */
    uint8_t lrc;
};

#define MODBUS_RTU_DATA_COUNT (MODBUS_RTU_BUFLEN - 4)

struct modbus_rtu_frame {
    uint8_t address;
    uint8_t function;
    uint8_t data[MODBUS_RTU_DATA_COUNT];
};

/**
 * @function modbus_rtu_init
 * @brief Initializes a MODBUS RTU object/context
 * @param ctx The MODBUS RTU context to init
 * @param uart The UART to read from / write to
 * @return Always 0
 */
int modbus_rtu_init(/*@out@*/ struct modbus_rtu *ctx, struct uart *uart);

/**
 * @function modbus_rtu_read
 * @brief read a MODBUS RTU frame
 * @param ctx The current MODBUS RTU object/context
 * @param frame A MODBUS RTU frame to receive data to
 * @return The size (in bytes) of the frame data if read is successful
 *         -EAGAIN if no data is available
 *         -EPIPE if we reached timeout
 *         -ENOENT if LRC is erroneous
 *         -EIO if some internal error occurred
 */
int modbus_rtu_read(struct modbus_rtu *ctx, /*@out@*/ struct modbus_rtu_frame *frame);

/**
 * @function modbus_rtu_write
 * @brief write a MODBUS RTU frame
 * @param ctx The current MODBUS RTU object/context
 * @param frame A MODBUS RTU frame to send
 * @return The size (in bytes) of the frame data if write is successful
 *         -EINVAL if n < 1
 *         -EAGAIN if no data can be written
 *         -EIO if some internal error occurred
 */
int modbus_rtu_write(struct modbus_rtu *ctx, const struct modbus_rtu_frame *frame, size_t n);

#endif
