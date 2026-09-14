#include "QD4310.hpp"
#include <algorithm>

/*!
 * \brief 控制QD4310
 * \param addr 地址
 * \param Control_mode 控制模式，详见 #QD4310_ControlMode_t
 * \param Control_quantity 控制量   
 * 
 */
void QD4310::QD4310_Contol(uint8_t addr ,uint8_t Control_mode ,uint16_t Control_quantity) {
    std::vector<uint8_t> command;
    command.reserve(5);
    command.push_back(addr); //地址
    command.push_back(Control_mode); //控制模式
    command.push_back(Control_quantity & 0xFF); //命令字
    command.push_back(Control_quantity >> 8);
    command.push_back(CRC8(command)); //CRC8校验码
    // std::reverse(command.begin(), command.end());
    uart_qd.uart_echo_send_byte(command, command.size());
    uart_qd.uart_echo_flush(); //清空接收缓冲区，避免接收到上次的残留数据
    sleep_ms(1); //等待电机响应

    if(Control_mode == QD4310_MODE_REPORT){
        std::vector<uint8_t> datas = uart_qd.uart_echo_receive_byte(10, 100);
        if(datas.size() == 10){
            uint8_t CRC8_Byte = datas[9];
            datas.pop_back();
            if(CRC8(datas) == CRC8_Byte){
                feedback.status = datas[1];
                feedback.elc_current = double((int16_t)(datas[3] | (datas[4] << 8)))/32767*10.0;
                feedback.speed = (uint16_t)(datas[5] | (datas[6] << 8));
                feedback.angle = (double)(uint16_t)(datas[7] | (datas[8] << 8)) / 65535.0 * 360.0;
            }
            else{
                std::cout << "CRC8校验失败" << std::endl;
            }
        }
        else{
            std::cout << "接收数据错误" << std::endl;
        }
    }
    sleep_ms(1); //等待电机响应
}

/**
 * @brief 计算CRC-8校验码（多项式 x^8 + x^2 + x + 1）
 * @param data 数据向量，约定 data[0] 为CRC占位字节（不参与计算）
 * @return CRC-8校验码
 * @note 计算结果应存入 data[0]
 */
uint8_t QD4310::CRC8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0x00;
    for (int i = 0; i < data.size(); ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; // 多项式 x^8 + x^2 + x + 1
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint16_t QD4310::rad(double angle_rad) {
    if(angle_rad > 2.0 * M_PI) {
        angle_rad = std::fmod(angle_rad, 2.0 * M_PI);
    }
    else if(angle_rad < 0.0) {
        if(angle_rad < -2.0 * M_PI) {
            angle_rad = std::fmod(angle_rad, 2.0 * M_PI);
        }
        angle_rad = std::fmod(angle_rad, 2.0 * M_PI) + 2.0 * M_PI;
    }

    return static_cast<uint16_t>(angle_rad /(2*PI)*65535);
}

uint16_t QD4310::angle_to_raw(double theta_deg) {
    // 1. 把角度归一到 [-180, 180)
    double t = std::fmod(theta_deg + 180.0, 360.0);
    if (t < 0) t += 360.0;
    t -= 180.0;

    // 2. 映射到 [0, 65535]
    double v = (t + 180.0) / 360.0 * 65535.0;
    v = std::clamp(v, 0.0, 65535.0);
    return static_cast<uint16_t>(std::lround(v));
}