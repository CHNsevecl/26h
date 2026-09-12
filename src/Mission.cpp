#include "Mission.hpp"
#include <math.h>

// 任务1：每秒打印一次
void vTask1(void *pvParameters) {
    std::cout << "123" << std::endl;
    BMI270 imu = *(BMI270 *)pvParameters;
    while (1) {
        if (xSemaphoreTake(xPrintMutex, portMAX_DELAY) == pdTRUE) {
            // imu.print_state();  // 这里的 printf 被保护
            imu.update_attitude(0.001f); // 这里的 printf 被保护
            // std::cout << "acc: " << imu.acc_f_[0] << ", " << imu.acc_f_[1] << ", " << imu.acc_f_[2] << std::endl;
            // std::cout << "gyr: " << imu.gyr_f_[0] << ", " << imu.gyr_f_[1] << ", " << imu.gyr_f_[2] << std::endl;
            // printf("%.3f,%.3f,%.3f\n", imu.AngleGyrop[0]/180.0f*3.141592, imu.AngleGyrop[1]/180.0f*3.141592, imu.AngleGyrop[2]/180.0f*3.141592);
            // printf("%.3f,%.3f,%.3f\n", imu.gyr_f_[0], imu.gyr_f_[1], imu.gyr_f_[2]);
            printf("%.3f,%.3f,%.3f\n",imu.Angle[0]/180.0*3.141592, imu.Angle[1]/180.0*3.141592, imu.Angle[2]/180.0*3.141592);
        
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
        // std::cout << "Received: " << str << std::endl;

    }
}
