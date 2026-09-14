#include "Mission.hpp"


int main() {
    sleep_ms(1);
    stdio_init_all();

    // while(!stdio_usb_connected()) {
    //     sleep_ms(100);
    // }

    UART uart(uart1, 115200, 8, 9); // 使用 UART1，波特率 115200，TX=GP8，RX=GP9
    uart.uart_echo_init();

    QD4310 qd4310;



    double distance = 0.0; // 初始化距离变量
    double speed = 0.0; // 初始化速度变量

    static taskParams1 params1;            // 用 static，别 new
    params1.distance = &distance;       // 指向真实 int
    params1.speed = &speed;             // 指向真实 double
    params1.qd4310 = &qd4310;            // 指向真实 QD4310

    static taskParams2 params2;            // 用 static，别 new
    params2.distance = &distance;       // 指向真实 int
    params2.speed = &speed;             // 指向真实 double
    params2.uart = uart;              // 拷贝一份 UART 进去


    TaskHandle_t task1Handle = NULL;
    TaskHandle_t task2Handle = NULL;

    xPrintMutex = xSemaphoreCreateMutex();
    
    xTaskCreate(vTask1, "Task 1", 256, &params1, 1, &task1Handle);
    xTaskCreate(vTask2, "Task 2", 256, &params2, 1, &task2Handle);

    // vTaskCoreAffinitySet(task2Handle, (1 << 0));
    
    vTaskStartScheduler();
    
    while (1) {
        tight_loop_contents();
    }
}