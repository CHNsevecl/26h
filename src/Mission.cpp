#include "Mission.hpp"
#include <math.h>



// 任务1：每秒打印一次
void vTask1(void *pvParameters) {
    taskParams1* params = (taskParams1 *)pvParameters;
    params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ENABLE, 0);  // 使用 QD4310 对象
    // params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ANGLE, params->qd4310->rad(0.0));  // 使用 QD4310 对象
    while (1) {
        // std::cout << "distance: " << *(params->distance) << ", speed: " << *(params->speed) << std::endl;
        double theta = (44.7213*(*(params->distance)/100.0) + 28.4449*(*(params->speed)/100.0))/(2.0*M_PI);  // 计算角度;
        params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ANGLE, params->qd4310->rad(theta));  // 使用 QD4310 对象
        // std::cout << "theta: " << theta << std::endl;
        // std::cout << "distance: " << *(params->distance) << std::endl;
        std::cout << "speed: " << *(params->speed) << std::endl;
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


// if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) {
        //     xSemaphoreGive(xPrintMutex);              // 释放互斥量
        // }