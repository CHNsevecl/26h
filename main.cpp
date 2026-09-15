#include "Mission.hpp"


int main() {
    sleep_ms(1);
    stdio_init_all();

    while(!stdio_usb_connected()) {
        sleep_ms(100);
    }

    UART uart(uart1, 115200, 8, 9); // 使用 UART1，波特率 115200，TX=GP8，RX=GP9
    uart.uart_echo_init();

    QD4310 qd4310;
    BMI270 bmi270;
    if (!bmi270.begin()) {     // 默认 SDA=GP16, SCL=GP17, 自动探测 0x68/0x69
        printf("[fatal] BMI270 init failed! check wiring: SDA->GP16, SCL->GP17\n");
        while (true) tight_loop_contents();
    }



    double distance = 0.0; // 初始化距离变量
    double speed = 0.0; // 初始化速度变量
    float accx = 0.0; // 初始化加速度变量

    static taskParams1 params1;            // 用 static，别 new
    params1.distance = &distance;       // 指向真实 int
    params1.speed = &speed;             // 指向真实 double
    params1.qd4310 = &qd4310;            // 指向真实 QD4310

    static taskParams2 params2;            // 用 static，别 new
    params2.distance = &distance;       // 指向真实 int
    params2.speed = &speed;             // 指向真实 double
    params2.uart = uart;              // 拷贝一份 UART 进去

    static taskParams3 params3;            // 用 static，别 new
    params3.accx = &accx;       // 初始化 accx
    params3.bmi270 = &bmi270;            // 指向真实 BMI270


    TaskHandle_t task1Handle = NULL;
    TaskHandle_t task2Handle = NULL;
    TaskHandle_t task3Handle = NULL;

    xPrintMutex = xSemaphoreCreateMutex();
    
    xTaskCreate(vTask1, "Task 1", 256, &params1, 1, &task1Handle);
    xTaskCreate(vTask2, "Task 2", 256, &params2, 1, &task2Handle);
    xTaskCreate(vTask3, "Task 3", 256, &params3, 1, &task3Handle);

    // vTaskCoreAffinitySet(task2Handle, (1 << 0));
    
    vTaskStartScheduler();
    
    while (1) {
        tight_loop_contents();
    }
}