#include "Mission.hpp"
#include <math.h>



// 任务1：每秒打印一次
void vTask1(void *pvParameters) {
    taskParams1* params = (taskParams1 *)pvParameters;
    params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ENABLE, 0);  // 使用 QD4310 对象
    params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ANGLE, params->qd4310->rad(0.0));  // 使用 QD4310 对象
    while (1) {
        if(*(params->distance) > 50){
            params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ANGLE, params->qd4310->rad(0.3));  // 使用 QD4310 对象
        }
        else if(*(params->distance) < -50){
            params->qd4310->QD4310_Contol(0x01, QD4310_MODE_ANGLE, params->qd4310->rad(-0.3));  // 使用 QD4310 对象
        }
        
        std::cout << "distance: " << *(params->distance) << std::endl;
        vTaskDelay(pdMS_TO_TICKS(1)); //不加延时，任务1会一直占用锁，导致2无法正常运行
        
    }
}

// 任务2：每2秒打印一次
void vTask2(void *pvParameters) {
    // int taskID = *(int *)pvParameters;
    taskParams2* params = (taskParams2 *)pvParameters;
    UART& uart_ = params->uart;
    int* distance = params->distance;
    while (1) {
        // if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) {
        //     printf("Task %d is running on core %d\n", taskID, get_core_num());
        //     xSemaphoreGive(xPrintMutex);
        // }
        // vTaskDelay(pdMS_TO_TICKS(500));

        std::string str = uart_.uart_echo_receive_string(1000); // 1秒超时
        if(!parse_int(str, *distance)) {
            if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) {
                std::cout << "Failed to parse distance from string: " << str << std::endl;
                xSemaphoreGive(xPrintMutex);              // 释放互斥量
            }
        }   
        // std::cout << "Received: " << str << ", Distance: " << *distance << std::endl;

    }
}


// if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) {
        //     xSemaphoreGive(xPrintMutex);              // 释放互斥量
        // }