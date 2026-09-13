#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <queue>
#include <mutex> 
#include "opencv4/opencv2/opencv.hpp"
#include "uart.hpp"
#include "Myhailo.hpp"
// #include "QD4310.hpp"
#include <condition_variable>

// ============ 全局变量 ============
std::queue<int> g_queue;
std::mutex g_mutex;
std::condition_variable g_cv;
bool g_stop = false;

void mission1(){
    std::string pipeline = 
    "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a ! "
    "video/x-raw, format=NV12, width=1280, height=480 ! " // 传感器输出
    "videoconvert ! "                                   // 硬件加速转换
    "video/x-raw, format=BGR ! "                        // 直接输出 BGR
    "appsink drop=true max-buffers=1";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return;
    }

    std::optional<HailoContext> hailo_context = Hailo_init(hef_path);
    if (!hailo_context) {
        std::cerr << "Failed to initialize Hailo context" << std::endl;
        return;
    }

    while (true) {
        cv::Mat frame;
        cap >> frame; // Capture a new frame
        if (frame.empty()) {
            std::cerr << "Failed to capture frame" << std::endl;
            break;
        }

        frame = Stream_process(frame, target_w, target_h); // Resize and letterbox the frame
        std::optional<std::vector<Detection>> detections_opt = ParseDetections(*hailo_context, target_w, target_h, frame, class_names);
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

        if (!detections_opt) {
            std::cerr << "Error: Failed to parse detections." << std::endl;
            break;
        }
        const auto& detections = *detections_opt;

        for (const auto& det : detections) {
            if(det.label == "Steel_Ball" && det.score < 1.0f) {
                cv::rectangle(frame, det.upper, det.lower, cv::Scalar(0, 255, 0), 2); // 在检测到的画布位置画一个绿色矩形框
                cv::putText(frame,                                      // 图像
                    det.label + " " + cv::format("%.2f", det.score),                    // 文本内容
                    cv::Point(det.upper.x, std::max(0, det.upper.y - 10)),       // 文本位置
                    cv::FONT_HERSHEY_SIMPLEX,                                   // 字体类型
                    0.5,                                                        // 字体大小
                    cv::Scalar(0, 255, 0),                                      // 颜色（BGR）
                    1                                                           // 线条粗细
                );

                int d = ((det.upper.x + det.lower.x) / 2) - 320; // 将检测到的钢球的中心点x坐标入队
                std::cout << "Steel_Ball detected at x: " << d << std::endl;
                {
                    std::lock_guard<std::mutex> lock(g_mutex);
                    g_queue.push(d); // 将检测到的钢球的中心点x坐标入队
                }
                g_cv.notify_one();  // 通知消费者

                continue; // 如果检测到钢球，跳过管道检测
            }
        }

        // Display the captured frame
        cv::imshow("Camera Feed", frame);
        if (cv::waitKey(1) == 27) { // Exit on 'ESC' key
            break;
        }
    }
}

void mission2() {
    UART uart("/dev/ttyAMA0", 115200);
    if (!uart.open()) {
        std::cerr << "Failed to open UART" << std::endl;
        return;
    }

    while (true) {
        std::unique_lock<std::mutex> lock(g_mutex);
        g_cv.wait(lock, [] { return !g_queue.empty() || g_stop; });

        if (g_stop && g_queue.empty()) {
            break;
        }

        int data = g_queue.front();
        g_queue.pop();
        lock.unlock();

        
        std::string data_to_send = std::to_string(data);
        // data_to_send.insert(0, "x: ");
        if (!uart.send(data_to_send)) {
            std::cerr << "Failed to send data over UART" << std::endl;
            break;
        }
    }

    uart.close();
}

int main(){
    setenv("DISPLAY",":0", 1);

    std::thread mission1_thread(mission1);
    std::thread mission2_thread(mission2);

    mission1_thread.join();
    mission2_thread.join();

    return 0;
}