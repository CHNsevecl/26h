#include "Mission.hpp"


int main() {
    sleep_ms(1);
    stdio_init_all();

    while(!stdio_usb_connected()) {
        sleep_ms(100);
    }

    

    BMI270 imu;
    if (!imu.begin()) {
        printf("[fatal] BMI270 init failed! check wiring: SDA->GP16, SCL->GP17\n");
        while (true) tight_loop_contents();
    }

    int id2 = 2;

    TaskHandle_t task1Handle = NULL;
    TaskHandle_t task2Handle = NULL;

    xPrintMutex = xSemaphoreCreateMutex();
    
    xTaskCreate(vTask1, "Task 1", 256, &imu, 1, &task1Handle);
    xTaskCreate(vTask2, "Task 2", 256, &id2, 1, &task2Handle);

    vTaskCoreAffinitySet(task2Handle, (1 << 1));
    
    vTaskStartScheduler();
    
    while (1) {
        tight_loop_contents();
    }
}