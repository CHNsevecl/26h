#pragma once
#include <cstdint>
#include <cstddef>
#include "uart_echo.hpp"

// ======================= 输出模式选择 =======================
// BMI270_OUTPUT_MODE:
//   0 = RAW  原始数据 (不过滤, 不补偿零偏)
//   1 = 滤波数据 (零偏补偿 + EMA 低通)
//   2 = VOFA+ 姿态 (FireWater: roll,pitch,yaw, 配 cube 3D 控件)
//   3 = VOFA+ 加速度波形 (FireWater: ax,ay,az, 配 waveform 控件)
#define BMI270_OUTPUT_MODE  3   // ← 切换这里: 0=raw, 1=filtered, 2=姿态, 3=加速度波形
#define BMI270_SERIAL_CHOSEN 1  // 0=USB CDC, 1=UART0 (BMI270_OUTPUT_MODE=2/3 时生效)

#define BMI270_angle_unit 0  // 0=rad, 1=deg (BMI270_OUTPUT_MODE=2 时生效)

// 滤波参数 (BMI270_OUTPUT_MODE=1 时生效)
#define BMI270_FILTER_ALPHA 0.5f  // EMA 低通系数: 0~1, 越小滤波越强(响应越慢)

// 姿态解算参数 (BMI270_OUTPUT_MODE=2 时生效)
#define BMI270_ATT_KP       0.98f  // 互补滤波: 陀螺仪权重 (加速度权重 = 1-KP)
#define BMI270_ATT_DT       0.01f  // 姿态更新周期 (秒), 与 main 循环间隔一致

// 陀螺仪零偏 (单位 dps, 静止时各轴读数均值) —— 通过静止采样获得
#define BMI270_GYR_BIAS_X  0.24f
#define BMI270_GYR_BIAS_Y  0.04f
#define BMI270_GYR_BIAS_Z  -0.63f

// 加速度零偏 (单位 mg, 平放静止时: 均值 - 理论重力 (0,0,-1000))
#define BMI270_ACC_BIAS_X  4.8f
#define BMI270_ACC_BIAS_Y  -6.6f
#define BMI270_ACC_BIAS_Z  -19.0f

//UART 配置
//uart0: TX=0, RX=1
#define UART_ID0 uart0
#define UART_BAUD_RATE0 115200
#define UART_TX_PIN0 0
#define UART_RX_PIN0 1
//uart1: TX=8, RX=9
#define UART_ID1 uart1
#define UART_BAUD_RATE1 115200
#define UART_TX_PIN1 8
#define UART_RX_PIN1 9

/*
 * BMI270 六轴 IMU 驱动类 (I2C) —— Raspberry Pi Pico 2 (RP2350)
 *
 * 接线:
 *   SDA -> GP16 (I2C0), SCL -> GP17 (I2C0), VDD -> 3V3, GND -> GND
 *   SDO 接 GND -> 地址 0x68
 *
 * 用法:
 *   BMI270 imu;
 *   if (!imu.begin()) {  // 初始化失败, 检查接线
 *       ...
 *   }
 *   float acc[3], gyr[3], temp;
 *   imu.read(acc, gyr, &temp);
 */
class BMI270 {
private:
#if BMI270_SERIAL_CHOSEN != 0
    UART uart_;  // UART 输出 (BMI270_OUTPUT_MODE=2/3 时使用)
#endif
public:
    // 原始传感器数据 (16 位有符号 LSB)
    BMI270()
    #if BMI270_SERIAL_CHOSEN == 2
        : uart_(UART_ID1, UART_BAUD_RATE1, UART_TX_PIN1, UART_RX_PIN1)
    #elif BMI270_SERIAL_CHOSEN == 1
        : uart_(UART_ID0, UART_BAUD_RATE0, UART_TX_PIN0, UART_RX_PIN0)
    #endif
    {
        #if BMI270_SERIAL_CHOSEN != 0
        uart_.uart_echo_init();
        #endif
    }

    struct RawData {
        int16_t ax, ay, az;   // 加速度
        int16_t gx, gy, gz;   // 陀螺仪
        int16_t temp;         // 温度
    };

    // 初始化: I2C 初始化 + 加载 config file + 配置传感器 (100Hz, ±4g, ±2000dps)
    //   sda/scl: Pico GPIO 引脚号 (默认 GP16/GP17, I2C0)
    //   addr   : I2C 地址, 0 = 自动探测 0x68/0x69
    // 返回 true 表示成功
    bool begin(uint8_t sda = 16, uint8_t scl = 17, uint8_t addr = 0);

    // 读取原始数据 (LSB), 成功返回 true
    bool read_raw(RawData &d);

    // 一步读取并换算物理量 (输出按 BMI270_OUTPUT_MODE 决定 raw 或滤波)
    //   acc_mg[3]  : 加速度 (单位 mg)
    //   gyr_dps[3] : 角速度 (单位 dps)
    //   temp_c     : 温度 (单位 °C)
    bool read(float acc_mg[3], float gyr_dps[3], float *temp_c);

    // 姿态解算 (互补滤波, BMI270_OUTPUT_MODE=2 用)
    //   dt: 两次调用间隔 (秒), 建议用实际测得的时间
    bool update_attitude(float dt);

    // 获取欧拉角 (单位度): roll 绕X / pitch 绕Y / yaw 绕Z
    void get_euler(float &roll, float &pitch, float &yaw) const;

    // 打印一帧数据 (格式由 BMI270_OUTPUT_MODE 决定), 调用一次打印一次
    void print_state();

    // 打印一次性启动信息 (模式 2/3 为 FireWater 协议时自动静默)
    void print_header();

    // 换算系数 (灵敏度) —— 标准 BMI270 值
    static constexpr float ACC_LSB_PER_G   = 8192.0f;
    static constexpr float GYR_LSB_PER_DPS = 16.384f;

    uint8_t addr()    const { return addr_; }
    uint8_t chip_id() const { return chip_id_; }

private:
    uint8_t addr_ = 0;
    uint8_t chip_id_ = 0;

    // 滤波状态 (BMI270_OUTPUT_MODE=1 时使用)
    float acc_f_[3] = { 0, 0, 0 };
    float gyr_f_[3] = { 0, 0, 0 };
    bool filter_inited_ = false;

    // 姿态状态 (BMI270_OUTPUT_MODE=2 时使用)
    float roll_ = 0, pitch_ = 0, yaw_ = 0;
    bool att_inited_ = false;
    uint64_t last_t_ = 0;   // 姿态积分上次时间戳 (us), print_state 里实测 dt 用

    bool init_sensor();
    bool load_config();

    bool write_reg(uint8_t reg, uint8_t value);
    bool read_reg(uint8_t reg, uint8_t &value);
    bool read_regs(uint8_t reg, uint8_t *buf, size_t len);
};
