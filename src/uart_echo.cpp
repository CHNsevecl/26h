#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "uart_echo.hpp"

// ============= 内部常量与状态（仅本文件可见） =============
#define BUFFER_SIZE 256

static char rx_buffer[BUFFER_SIZE];          // 接收缓冲区
static uint rx_index = 0;                    // 缓冲区中有效字节数
static uint32_t last_rx_time_ms = 0;         // 最近一次收到字节的时刻（ms）
static uint32_t last_send_time_us = 0;       // 最近一次发送的时刻（us）
static uint32_t last_report_time_us = 0;     // 最近一次状态打印的时刻（us）

static const uint32_t RX_TIMEOUT_MS = 500;          // 500ms 无新数据则按整条消息处理
static const uint32_t HALLO_INTERVAL_US = 1000000;  // 空闲时每 1s 发送一次 hallo
static const uint32_t STATUS_INTERVAL_US = 5000000; // 每 5s 打印一次状态
// =========================================================

// 初始化UART
void UART::uart_echo_init() {
    // 初始化UART接口
    uart_init(UART_ID, UART_BAUD_RATE);

    // 设置TX和RX引脚
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // 禁用流控
    uart_set_hw_flow(UART_ID, false, false);

    // 设置数据格式: 8位数据, 1位停止位, 无奇偶校验
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    // ---- 启动信息 ----
    printf("\n=== UART 测试程序 ===\n");
    printf("UART%d TX=%d RX=%d BAUD=%d\n",
           UART_ID == uart0 ? 0 : 1, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
    printf("[UART 初始化完成]\n");
    printf("[配置: UART%d, 波特率=%d, TX引脚=%d, RX引脚=%d]\n",
           UART_ID == uart0 ? 0 : 1, UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    printf("=== UART 检测程序已启动 ===\n");
    printf("收到任意字节都会先打印，再在换行或超时后整条回显。\n");
    printf("没有收到数据时，每 1 秒发送一次 hallo。\n\n");

    last_send_time_us = time_us_32();
    last_report_time_us = time_us_32();
}

/**
 * 发送字符串
 * @param str 要发送的字符串（不包含结尾的 '\0'）
 * @warning 该函数不会自动添加换行符，如果需要换行，请在字符串中手动添加 '\n' 或 '\r\n'。
 */
void UART::uart_echo_send_string(const char *str) {
    for (int i = 0; str[i] != '\0'; i++) {
        uart_putc(UART_ID, str[i]);
    }
}

void UART::uart_echo_send_byte(std::vector<uint8_t> byte, int len) {
    for (int i = 0; i < len; i++) {
        uart_putc(UART_ID, byte[i]);
    }
}

std::vector<uint8_t> UART::uart_echo_receive_byte(int len,uint32_t timeout_ms) {
    std::vector<uint8_t> received_bytes;
    received_bytes.reserve(len); // 预分配内存，提高效率
    
    uint64_t start_time = time_us_64();  // 使用64位避免溢出
    uint64_t timeout_us = (uint64_t)timeout_ms * 1000;
    
    while (received_bytes.size() < len) {
        // 1. 检查超时
        if (time_us_64() - start_time > timeout_us) {
            printf("UART接收超时，已接收 %zu 字节\n", received_bytes.size());
            break; // 超时退出，返回已接收的数据
        }
        
        // 2. 检查是否有数据可读（非阻塞）
        if (uart_is_readable(UART_ID)) {
            uint8_t byte = uart_getc(UART_ID);
            received_bytes.push_back(byte);
        } else {
            // 3. 没有数据时短暂休眠，避免CPU空转
            sleep_us(100); // 100微秒轮询间隔
        }
    }
    
    return received_bytes;
}

std::string UART::uart_echo_receive_string(uint32_t timeout_ms) {
    std::string received_str = "null";
    std::vector<uint8_t> received_bytes;
    received_bytes.reserve(256); // 预分配内存，提高效率

    uint64_t start_time = time_us_64();  // 使用64位避免溢出
    uint64_t timeout_us = (uint64_t)timeout_ms * 1000;


    while (time_us_64() - start_time < timeout_us)
    {
        if (uart_is_readable(UART_ID)) {
            uint8_t byte = uart_getc(UART_ID);
            if(byte == '\n'){
                received_str = std::string(received_bytes.begin(), received_bytes.end());
                //received_bytes.begin()迭代器起始位
                //received_bytes.end()迭代器终止位，received_bytes.back()表示最后一个值
                break; // 收到换行符，结束接收
            }
            received_bytes.push_back(byte);

        } else {
            // 3. 没有数据时短暂休眠，避免CPU空转
            sleep_us(100); // 100微秒轮询间隔
        } 
    }
    
    return received_str; // 超时未接收到任何数据，返回默认字符串
}

// 主服务：每调用一次处理一轮接收/回显/心跳/状态（逻辑与原 main 循环体一致）
void UART::uart_echo_service() {
    uint32_t now_us = time_us_32();
    uint32_t now_ms = now_us / 1000;

    // ---- 读取并处理当前可读的所有字节 ----
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        last_rx_time_ms = now_ms;

        printf("[RX 0x%02X '%c']\n", (unsigned char)c, (c >= 32 && c <= 126) ? c : '.');

        if (c == '\n' || c == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                printf("[收到完整消息]: %s\n", rx_buffer);
                uart_echo_send_string(rx_buffer);
                uart_echo_send_string("\r\n");
                rx_index = 0;
                last_send_time_us = now_us;
            }
        } else if (rx_index < BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = c;
        } else {
            rx_index = 0;
            printf("[RX 缓冲区溢出，已清空]\n");
        }
    }

    // ---- 超时未收完整条消息，按整条处理并回显 ----
    if (rx_index > 0 && (now_ms - last_rx_time_ms) > RX_TIMEOUT_MS) {
        rx_buffer[rx_index] = '\0';
        printf("[收到超时消息]: %s\n", rx_buffer);
        uart_echo_send_string(rx_buffer);
        uart_echo_send_string("\r\n");
        rx_index = 0;
        last_send_time_us = now_us;
    }

    // ---- 空闲时周期发送 hallo ----
    if (rx_index == 0 && (now_us - last_send_time_us) >= HALLO_INTERVAL_US) {
        uart_echo_send_string("hallo\r\n");
        printf("[发送]: hallo\n");
        last_send_time_us = now_us;
    }

    // ---- 周期状态报告 ----
    if ((now_us - last_report_time_us) >= STATUS_INTERVAL_US) {
        printf("[状态] RX空闲中\n");
        last_report_time_us = now_us;
    }
}
