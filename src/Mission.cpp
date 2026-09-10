#include "Mission.hpp"

// 任务1：每秒打印一次
void vTask1(void *pvParameters) {
    std::cout << "123" << std::endl;
    while (1) {
        if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) {
            ((BMI270 *)pvParameters)->print_state();  // 这里的 printf 被保护
            // 
            xSemaphoreGive(xPrintMutex);              // 释放互斥量
        }
        vTaskDelay(pdMS_TO_TICKS(1)); //不加延时，任务1会一直占用锁，导致2无法正常运行
        
    }
}

// 任务2：每2秒打印一次
void vTask2(void *pvParameters) {
    // int taskID = *(int *)pvParameters;
    UART uart_ = *(UART *)pvParameters;
    while (1) {
        // if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) {
        //     printf("Task %d is running on core %d\n", taskID, get_core_num());
        //     xSemaphoreGive(xPrintMutex);
        // }
        // vTaskDelay(pdMS_TO_TICKS(500));

        std::string str = uart_.uart_echo_receive_string(1000); // 1秒超时
        std::cout << "Received: " << str << std::endl;

    }
}
