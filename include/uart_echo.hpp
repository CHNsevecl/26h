#pragma once

// ============================================================================
// UART 回显/测试模块
// ----------------------------------------------------------------------------
// 封装了 UART 的初始化、字符串发送，以及主服务循环（逐字节打印、
// 换行/超时整条回显、空闲时周期发送 "hallo"）。
// main 只需要调用 uart_echo_init() 和 uart_echo_service() 即可。
// ============================================================================

// ============= 可配置参数（修改后请重新编译） =============
// UART总线选择: uart0 或 uart1
#define DEFAULT_UART_ID uart1
#define DEFAULT_UART_BAUD_RATE 115200

// 当前默认使用 UART1: TX=8, RX=9
// 可根据需要修改为其他有效引脚
#define DEFAULT_UART_TX_PIN 8
#define DEFAULT_UART_RX_PIN 9
// =========================================================

#include <vector>
#include "hardware/timer.h"
#include <string>
#include "hardware/uart.h"

class UART {
private:
    uart_inst_t *UART_ID;
    uint UART_BAUD_RATE;
    uint UART_TX_PIN;
    uint UART_RX_PIN;
public:
    UART(uart_inst_t *id = DEFAULT_UART_ID, uint baud = DEFAULT_UART_BAUD_RATE, uint tx = DEFAULT_UART_TX_PIN, uint rx = DEFAULT_UART_RX_PIN)
        : UART_ID(id),
          UART_BAUD_RATE(baud),
          UART_TX_PIN(tx),
          UART_RX_PIN(rx)
    {}

    // 初始化 UART（硬件配置 + 打印启动信息），内部会调用 uart_init()
    void uart_echo_init();

    // 通过 UART 发送字节（不包含结尾的 '\0'）
    void uart_echo_send_byte(std::vector<uint8_t> byte, int len);

    // 通过 UART 接收字节（不包含结尾的 '\0'）
    std::vector<uint8_t> uart_echo_receive_byte(int len = 8, uint32_t timeout_ms = 10);

    // 通过 UART 接收字符串（不包含结尾的 '\0'）
    std::string uart_echo_receive_string(uint32_t timeout_ms = 10);


    // 通过 UART 发送字符串（不包含结尾的 '\0'）
    void uart_echo_send_string(const char *str);

    // 主服务：处理一次接收/回显/超时/心跳（不含延时，建议主循环每 10ms 调用一次）
    void uart_echo_service();
};

