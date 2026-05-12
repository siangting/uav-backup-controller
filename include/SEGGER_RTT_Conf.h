#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

// ===== Buffer sizes =====
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS             2
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS           2

#define BUFFER_SIZE_UP                            1024
#define BUFFER_SIZE_DOWN                          16

// ===== Mode =====
#define SEGGER_RTT_PRINTF_BUFFER_SIZE             64

// ===== Behavior =====
// 0 = no block skip (best for RT)
// 1 = block if full
#define SEGGER_RTT_MODE_DEFAULT SEGGER_RTT_MODE_NO_BLOCK_SKIP

#endif
