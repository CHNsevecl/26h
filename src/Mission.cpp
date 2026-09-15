#include "Mission.hpp"
#include <math.h>



// 任务1：每秒打印一次
void vTask1(void *pvParameters) {
    taskParams1* params = (taskParams1 *)pvParameters;
    params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ENABLE, 0);  // 使用 QD4310 对象
    while (1) {
        double theta = (44.7213*(*(params->distance)/100.0) + 28.4449*(*(params->speed)/100.0))/(2.0*M_PI);  // 计算角度;
        params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ANGLE, params->qd4310->rad(theta));  // 使用 QD4310 对象
        // std::cout << "speed: " << *(params->speed) << std::endl;
        vTaskDelay(pdMS_TO_TICKS(33)); //不加延时，任务1会一直占用锁，导致2无法正常运行
        
    }
}

// 任务2：每2秒打印一次
void vTask2(void *pvParameters) {
    // int taskID = *(int *)pvParameters;
    taskParams2* params = (taskParams2 *)pvParameters;
    UART& uart_ = params->uart;
    double* distance = params->distance;
    double* speed = params->speed;
    Field fields[] = {
        {"distance", FieldType::DOUBLE, distance},
        {"speed", FieldType::DOUBLE, speed},
    };
    while (1) {
        std::string str = uart_.uart_echo_receive_string(1000); // 1秒超时
        if(!parse_fields(str, fields, 2)) {
            // 解析失败，将距离和速度设为0
            if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) {
                std::cout << str << std::endl;
                *distance = 0.0;
                *speed = 0.0;
                xSemaphoreGive(xPrintMutex);              // 释放互斥量
            }
        }   
        // std::cout << "Received: " << str << ", Distance: " << *distance << std::endl;
        vTaskDelay(pdMS_TO_TICKS(33));
    }
}

void vTask3(void *pvParameters) {
    taskParams3* params = (taskParams3 *)pvParameters;
    BMI270* bmi270 = params->bmi270;
    float* accx = params->accx;
    float copy_accx = 0.0f; 

    while (1) {
        bmi270->read(bmi270->acc_mg, bmi270->gyr_dps, &bmi270->temp_c); // 只读取加速度数据，忽略陀螺仪和温度
        if(bmi270->acc_mg[0] < 35.0f && bmi270->acc_mg[0] > -10.0f) { // 如果加速度小于20mg或大于50mg，则认为是噪声
            *accx = 0.0f; // 如果加速度小于50mg，则认为是噪声，设为0
            copy_accx = 0.0f; // 重置拷贝的加速度值
        }
        else{
            *accx = bmi270->acc_mg[0]; // 获取 x 轴加速度
            copy_accx = *accx; // 拷贝当前加速度值
        }
        

        if(xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) { // 如果拷贝的加速度值不为0，则打印
            if(copy_accx != 0.0f) {
                std::cout << "accx: " << copy_accx << std::endl; // 打印 x 轴加速度
            }
            xSemaphoreGive(xPrintMutex);              // 释放互斥量
        }
        
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}