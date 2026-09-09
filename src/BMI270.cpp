/*
 * BMI270 驱动类实现 —— I2C (Raspberry Pi Pico 2 / RP2350)
 *
 * 初始化流程 (按 Bosch 官方 BMI270_SensorAPI):
 *   软复位 -> 加载 8KB config file -> 使能 acc/gyr/temp -> 配置 100Hz
 *
 * 灵敏度校准说明:
 *   本模块实测静止时 |a| = 6833 LSB (±4g 配置), 而数据手册标称 8192 LSB/g.
 *   官方 Bosch 驱动跑同一模块也得到相同结果, 说明不是代码问题, 而是模块
 *   芯片的增益与标称不符 (疑为非正品芯片或替代型号, 实际量程约 ±4.8g).
 *   因此用实测值 6833 LSB/g 校准, 使静止时 |a| ≈ 1.00g.
 *   如果换正品模块且数据正常(静止 |a| = 1.00g), 把 BMI270::ACC_LSB_PER_G 改回 8192.
 */

#include "../include/BMI270.hpp"
#include "../include/bmi270_config.h"

#include <cmath>
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ------------------------- 寄存器定义 -------------------------
#define REG_CHIP_ID         0x00    // 期望值 0x24
#define REG_ERR_REG         0x02    // bit0 = internal_error
#define REG_INTERNAL_STATUS 0x21    // 低4位 = config load 状态
#define REG_ACC_DATA        0x0C    // 6 字节: ACC_X_LSB..ACC_Z_MSB
#define REG_GYR_DATA        0x12    // 6 字节: GYR_X_LSB..GYR_Z_MSB
#define REG_TEMP_DATA       0x22    // 2 字节 (0x22=LSB, 0x23=MSB)
#define REG_ACC_CONF        0x40
#define REG_ACC_RANGE       0x41
#define REG_GYR_CONF        0x42
#define REG_GYR_RANGE       0x43
#define REG_PWR_CONF        0x7C    // bit2 = adv_power_save
#define REG_PWR_CTRL        0x7D    // bit0 acc, bit1 gyr, bit2 temp
#define REG_CMD             0x7E

// config file 加载专用寄存器
#define REG_INIT_CTRL       0x59    // bit0 = conf_load_en
#define REG_INIT_ADDR_0     0x5B
#define REG_INIT_ADDR_1     0x5C
#define REG_INIT_DATA       0x5E

#define CHIP_ID_EXPECTED    0x24
#define CMD_SOFT_RESET      0xB6
#define CONFIG_LOAD_SUCCESS 0x01    // INTERNAL_STATUS 低4位 == 1 表示加载成功

#define ACC_RANGE_4G        0x01    // ±4g
#define GYR_RANGE_2000DPS   0x00    // ±2000dps
#define I2C_BAUD            400 * 1000

// ------------------------- I2C 基础读写 -------------------------
bool BMI270::write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_write_blocking(i2c0, addr_, buf, 2, false) == 2;
}

bool BMI270::read_reg(uint8_t reg, uint8_t &value) {
    if (i2c_write_blocking(i2c0, addr_, &reg, 1, false) != 1) return false;
    return i2c_read_blocking(i2c0, addr_, &value, 1, false) == 1;
}

bool BMI270::read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_blocking(i2c0, addr_, &reg, 1, false) != 1) return false;
    return i2c_read_blocking(i2c0, addr_, buf, len, false) == (int)len;
}

// ------------------------- config file 加载 -------------------------
// 参考 Bosch 官方 bmi2.c: upload_file() + set_config_load() + write_config_file()
bool BMI270::load_config() {
    // 1) 关闭高级省电模式 (否则后续写入可能不生效)
    if (!write_reg(REG_PWR_CONF, 0x00)) return false;
    sleep_ms(1);

    // 2) 禁用 config load (INIT_CTRL bit0 = 0)
    uint8_t ctrl = 0;
    if (!read_reg(REG_INIT_CTRL, ctrl)) return false;
    ctrl &= ~0x01;
    if (!write_reg(REG_INIT_CTRL, ctrl)) return false;

    // 3) 分块上传 config file (每块 32 字节)
    for (uint16_t idx = 0; idx < sizeof(BMI270_CONFIG_FILE); idx += 32) {
        uint8_t addr_hi = (uint8_t)((idx / 2) >> 4);
        uint8_t addr_lo = (uint8_t)((idx / 2) & 0x0F);
        uint8_t addr_buf[2] = { addr_lo, addr_hi };

        // 写目标地址 (INIT_ADDR_0/1), 再写 32 字节数据到 INIT_DATA
        if (!write_reg(REG_INIT_ADDR_0, addr_buf[0])) return false;
        if (!write_reg(REG_INIT_ADDR_1, addr_buf[1])) return false;
        uint8_t tmp[33] = { 0 };
        tmp[0] = REG_INIT_DATA;
        for (int i = 0; i < 32; i++) tmp[1 + i] = BMI270_CONFIG_FILE[idx + i];
        if (i2c_write_blocking(i2c0, addr_, tmp, 33, false) != 33) return false;
    }

    // 4) 使能 config load (INIT_CTRL bit0 = 1)
    if (!read_reg(REG_INIT_CTRL, ctrl)) return false;
    ctrl |= 0x01;
    if (!write_reg(REG_INIT_CTRL, ctrl)) return false;

    // 5) 轮询 INTERNAL_STATUS 低4位 == CONFIG_LOAD_SUCCESS, 最多 ~100ms
    for (int i = 0; i < 100; i++) {
        uint8_t st = 0;
        if (read_reg(REG_INTERNAL_STATUS, st) && ((st & 0x0F) == CONFIG_LOAD_SUCCESS))
            return true;
        sleep_ms(1);
    }
    return false;
}

// ------------------------- 传感器初始化 -------------------------
bool BMI270::init_sensor() {
    // 1) 软复位, 等待 2ms (官方库在软复位后立即加载 config file)
    if (!write_reg(REG_CMD, CMD_SOFT_RESET)) return false;
    sleep_ms(2);

    // 2) 加载 config file (关键步骤! 不加载则 acc/gyr 无有效输出)
    if (!load_config()) return false;

    // 3) 使能 加速度+陀螺仪+温度
    if (!write_reg(REG_PWR_CTRL, 0x07)) return false;

    // 4) 加速度: 100Hz 输出, ±4g  (0xA8 = 官方性能模式: filter_perf + bwp=normal + odr=100Hz)
    if (!write_reg(REG_ACC_CONF, 0xA8)) return false;
    if (!write_reg(REG_ACC_RANGE, ACC_RANGE_4G)) return false;

    // 5) 陀螺仪: 100Hz 输出, ±2000dps (0xE8 = filter_perf + noise_perf + bwp=normal + odr=100Hz)
    if (!write_reg(REG_GYR_CONF, 0xE8)) return false;
    if (!write_reg(REG_GYR_RANGE, GYR_RANGE_2000DPS)) return false;

    // 6) 等待第一批数据
    sleep_ms(100);
    return true;
}

// ------------------------- 初始化入口 -------------------------
bool BMI270::begin(uint8_t sda, uint8_t scl, uint8_t addr) { 
    // 初始化 I2C0
    i2c_init(i2c0, I2C_BAUD);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    // 确定探测地址
    uint8_t candidates[2];
    uint8_t n = 0;
    if (addr != 0) {
        candidates[n++] = addr;
    } else {
        candidates[n++] = 0x68;
        candidates[n++] = 0x69;
    }

    // 探测: 读 CHIP_ID 确认
    for (uint8_t i = 0; i < n; i++) {
        uint8_t id = 0;
        addr_ = candidates[i];
        if (read_reg(REG_CHIP_ID, id) && id == CHIP_ID_EXPECTED) {
            chip_id_ = id;
            break;
        }
        addr_ = 0;
    }
    if (addr_ == 0) return false;

    // 传感器初始化
    return init_sensor();
}

// ------------------------- 数据读取 -------------------------
bool BMI270::read_raw(RawData &d) {
    uint8_t acc[6], gyr[6], tmp[2];
    if (!read_regs(REG_ACC_DATA, acc, 6)) return false;
    if (!read_regs(REG_GYR_DATA, gyr, 6)) return false;
    if (!read_regs(REG_TEMP_DATA, tmp, 2)) return false;

    d.ax = (int16_t)(acc[0] | (acc[1] << 8));
    d.ay = (int16_t)(acc[2] | (acc[3] << 8));
    d.az = (int16_t)(acc[4] | (acc[5] << 8));
    d.gx = (int16_t)(gyr[0] | (gyr[1] << 8));
    d.gy = (int16_t)(gyr[2] | (gyr[3] << 8));
    d.gz = (int16_t)(gyr[4] | (gyr[5] << 8));
    d.temp = (int16_t)(tmp[0] | (tmp[1] << 8));
    return true;
}

bool BMI270::read(float acc_mg[3], float gyr_dps[3], float *temp_c) {
    RawData d;
    if (!read_raw(d)) return false;

    float ax = d.ax / (ACC_LSB_PER_G / 1000.0f);
    float ay = d.ay / (ACC_LSB_PER_G / 1000.0f);
    float az = d.az / (ACC_LSB_PER_G / 1000.0f);
    float gx = d.gx / GYR_LSB_PER_DPS;
    float gy = d.gy / GYR_LSB_PER_DPS;
    float gz = d.gz / GYR_LSB_PER_DPS;

#if BMI270_OUTPUT_MODE == 1
    // ---- 滤波模式: 零偏补偿 + EMA 低通 ----
    // 1) 零偏补偿
    ax -= BMI270_ACC_BIAS_X; ay -= BMI270_ACC_BIAS_Y; az -= BMI270_ACC_BIAS_Z;
    gx -= BMI270_GYR_BIAS_X; gy -= BMI270_GYR_BIAS_Y; gz -= BMI270_GYR_BIAS_Z;

    // 2) EMA 低通滤波
    if (!filter_inited_) {
        acc_f_[0] = ax; acc_f_[1] = ay; acc_f_[2] = az;
        gyr_f_[0] = gx; gyr_f_[1] = gy; gyr_f_[2] = gz;
        filter_inited_ = true;
    }
    acc_f_[0] = BMI270_FILTER_ALPHA * ax + (1.0f - BMI270_FILTER_ALPHA) * acc_f_[0];
    acc_f_[1] = BMI270_FILTER_ALPHA * ay + (1.0f - BMI270_FILTER_ALPHA) * acc_f_[1];
    acc_f_[2] = BMI270_FILTER_ALPHA * az + (1.0f - BMI270_FILTER_ALPHA) * acc_f_[2];
    gyr_f_[0] = BMI270_FILTER_ALPHA * gx + (1.0f - BMI270_FILTER_ALPHA) * gyr_f_[0];
    gyr_f_[1] = BMI270_FILTER_ALPHA * gy + (1.0f - BMI270_FILTER_ALPHA) * gyr_f_[1];
    gyr_f_[2] = BMI270_FILTER_ALPHA * gz + (1.0f - BMI270_FILTER_ALPHA) * gyr_f_[2];

    ax = acc_f_[0]; ay = acc_f_[1]; az = acc_f_[2];
    gx = gyr_f_[0]; gy = gyr_f_[1]; gz = gyr_f_[2];
#else
    // ---- RAW 模式: 原样输出 (不加零偏补偿) ----
    (void)0;
#endif

    acc_mg[0] = ax;
    acc_mg[1] = ay;
    acc_mg[2] = az;
    gyr_dps[0] = gx;
    gyr_dps[1] = gy;
    gyr_dps[2] = gz;
    *temp_c = d.temp / 512.0f + 23.0f;
    return true;
}

// ------------------------- 姿态解算 (互补滤波) -------------------------
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool BMI270::update_attitude(float dt) {
    RawData d;
    if (!read_raw(d)) return false;

    // 换算物理量 (减零偏)
    float ax = d.ax / (ACC_LSB_PER_G / 1000.0f) - BMI270_ACC_BIAS_X;
    float ay = d.ay / (ACC_LSB_PER_G / 1000.0f) - BMI270_ACC_BIAS_Y;
    float az = d.az / (ACC_LSB_PER_G / 1000.0f) - BMI270_ACC_BIAS_Z;
    float gx = d.gx / GYR_LSB_PER_DPS - BMI270_GYR_BIAS_X;
    float gy = d.gy / GYR_LSB_PER_DPS - BMI270_GYR_BIAS_Y;
    float gz = d.gz / GYR_LSB_PER_DPS - BMI270_GYR_BIAS_Z;

    // 加速度计估算倾角 (度): 重力方向投影
    float acc_roll  = atan2f(ay, az) * 180.0f / (float)M_PI;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;

    // 首次调用: 用加速度角度初始化, 避免开机瞬间姿态跳变
    if (!att_inited_) {
        roll_  = acc_roll;
        pitch_ = acc_pitch;
        yaw_   = 0.0f;
        att_inited_ = true;
        return true;
    }

    // 互补滤波: 陀螺仪积分(快) + 加速度修正(不漂移)
    roll_  = BMI270_ATT_KP * (roll_  + gx * dt) + (1.0f - BMI270_ATT_KP) * acc_roll;
    pitch_ = BMI270_ATT_KP * (pitch_ + gy * dt) + (1.0f - BMI270_ATT_KP) * acc_pitch;
    yaw_  += gz * dt;   // yaw 无绝对参考, 纯积分 (静止时会缓慢漂移)

    return true;
}

void BMI270::get_euler(float &roll, float &pitch, float &yaw) const {
     #if BMI270_angle_unit == 0 
        roll  = roll_  / 180.0f * (float)M_PI;
        pitch = pitch_ / 180.0f * (float)M_PI;
        yaw   = yaw_   / 180.0f * (float)M_PI;
    #else
        roll  = roll_ ;
        pitch = pitch_;
        yaw   = yaw_;
    #endif
}

// ------------------------- 数据输出 -------------------------
// 打印一帧数据, 格式由 BMI270_OUTPUT_MODE 决定
void BMI270::print_state() {
#if BMI270_OUTPUT_MODE == 2
    // ---- VOFA+ 姿态: "roll,pitch,yaw\n" (度) ----
    // 用实际时间差作为积分步长, 调用间隔随意
    uint64_t now = time_us_64();
    float dt = (last_t_ == 0) ? BMI270_ATT_DT : (float)(now - last_t_) / 1000000.0f;
    last_t_ = now;

    update_attitude(dt);
    float roll, pitch, yaw;
    get_euler(roll, pitch, yaw);
    #if BMI270_SERIAL_CHOSEN == 0
        printf("%.3f,%.3f,%.3f\n", roll, pitch, yaw);
    #elif BMI270_SERIAL_CHOSEN == 1 || BMI270_SERIAL_CHOSEN == 2
        char buf[64];
        snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f\n", roll, pitch, yaw);
        uart_.uart_echo_send_string(buf);
    #endif

#elif BMI270_OUTPUT_MODE == 3
    // ---- VOFA+ 加速度波形: "ax,ay,az\n" (mg) ----
    float acc_mg[3], gyr_dps[3], temp_c;
    if (read(acc_mg, gyr_dps, &temp_c)) {
        #if BMI270_SERIAL_CHOSEN == 0
            printf("%.1f,%.1f,%.1f\n", acc_mg[0], acc_mg[1], acc_mg[2]);
        #elif BMI270_SERIAL_CHOSEN == 1 || BMI270_SERIAL_CHOSEN == 2
            char buf[64];
            snprintf(buf, sizeof(buf), "%.1f,%.1f,%.1f\n", acc_mg[0], acc_mg[1], acc_mg[2]);
            uart_.uart_echo_send_string(buf);
        #endif
    }
#else
    // ---- RAW / 滤波模式: 文本输出 ----
    float acc_mg[3], gyr_dps[3], temp_c;
    if (read(acc_mg, gyr_dps, &temp_c)) {
        printf("acc(mg): %7.1f %7.1f %7.1f | gyr(dps): %8.2f %8.2f %8.2f | temp: %5.2f C\n",
               acc_mg[0], acc_mg[1], acc_mg[2],
               gyr_dps[0], gyr_dps[1], gyr_dps[2],
               temp_c);
    } else {
        printf("[err] I2C read failed\n");
    }
#endif
}

// 打印一次性启动信息 (模式 2/3 为 FireWater 协议时自动静默, 避免干扰解析)
void BMI270::print_header() {
#if BMI270_OUTPUT_MODE == 2 || BMI270_OUTPUT_MODE == 3
    (void)0;
#else
    printf("\n===== BMI270 Test on Pico 2 =====\n");
    printf("[ok] BMI270 found at 0x%02X, chip_id = 0x%02X\n", addr_, chip_id_);
    printf("[ok] init done. streaming accel/gyro/temp...\n\n");
#endif
}
