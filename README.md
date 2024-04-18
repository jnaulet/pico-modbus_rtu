# pico-MODBUS RTU

## Presentation

This is a small library to add support for MODBUS RTU (ASCII) to picoRTOS (available here: https://github.com/jnaulet/OpenPicoRTOS)

## How to use

To include this library in your project, just add this to your makefile:

```make
-include /path/to/pico-modbus_rtu/Makefile.in
```

In your project:

```c
#include "modbus_rtu.h"

    /* ... */

    static struct modbus_rtu modbus_tx;
    static struct modbus_rtu_frame frame;
 
    (void)modbus_rtu_init(&modbus_tx, ctx->UART);

    frame.address = (uint8_t)BAR_ADDR;
    frame.function = (uint8_t)FUNC_FOO;

    (void)modbus_rtu_write(&modbus, &frame, (size_t)PICO_PDU_PKT_LEN);
```
