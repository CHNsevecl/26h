#ifndef QD4310_HPP
#define QD4310_HPP

//硬件库
#include "pico/stdlib.h"
//串口库
#include "uart_echo.hpp"
//向量库
#include <vector>
//字节库
#include <cstdint>
#include <iostream>
//
#include <cmath>
//延时库
#include <chrono>
#include <thread>

#define UART_QD uart0
#define BAUD_RATE_QD 115200
#define UART_TX_PIN_QD 0
#define UART_RX_PIN_QD 1
#define PI 3.14159265358979323846

/*!
 * \brief QD4310 控制模式枚举
 */
typedef enum {
    QD4310_MODE_REPORT    = 0x00, /*!< 报告模式 */
    QD4310_MODE_ENABLE    = 0x01, /*!< 电机使能 */
    QD4310_MODE_DISABLE   = 0x02, /*!< 电机失能 */
    QD4310_MODE_TORQUE    = 0x03,  /*!< 力矩控制模式 */
    QD4310_MODE_SPEED     = 0x04, /*!< 速度控制模式 */
    QD4310_MODE_ANGLE     = 0x05, /*!< 角度控制模式 */
    QD4310_MODE_LOWSPEED  = 0x06, /*!< 低速模式 */
    QD4310_MODE_STEPANGLE = 0x07  /*!< 角度步进控制 */
} QD4310_ControlMode_t;

struct QD4310_Feedback {
    uint8_t status;         /*!< 电机状态 */
    double elc_current;   /*!< 电机位置 */
    uint16_t speed;         /*!< 电机速度 */
    double angle;         /*!< 电机角度 */
};


class QD4310 {
private:
    
public:
    UART uart_qd;
    QD4310()
    :uart_qd(UART_QD, BAUD_RATE_QD, UART_TX_PIN_QD, UART_RX_PIN_QD)
    {
        uart_qd.uart_echo_init();
    }
    QD4310_Feedback feedback;
    void QD4310_Contol(uint8_t addr ,uint8_t Control_mode ,uint16_t Control_quantity);
    uint16_t rad (double angle);
    uint8_t CRC8(const std::vector<uint8_t>& data);
    uint16_t angle_to_raw(double theta_deg);
};

#endif