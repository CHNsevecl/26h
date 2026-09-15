#ifndef Mission_hpp
#define Mission_hpp 

#include <stdio.h>
#include <iostream>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "uart_echo.hpp"
#include "BMI270.hpp"
#include "QD4310.hpp"

inline SemaphoreHandle_t xPrintMutex = NULL;  // 定义互斥量句柄

struct taskParams1 {
    double* distance;
    double* speed;
    QD4310* qd4310;
};
struct taskParams2 {
    double* distance;
    double* speed;
    UART uart;
};

struct taskParams3 {
    BMI270* bmi270;
    float* accx;
};

void vTask1(void *pvParameters);
void vTask2(void *pvParameters);
void vTask3(void *pvParameters);

#endif /* Mission_hpp */