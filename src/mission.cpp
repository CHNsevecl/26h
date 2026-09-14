#include "mission.hpp"


// ============ 全局变量 ============
std::queue<double> g_queue;
std::mutex g_mutex;
std::condition_variable g_cv;
std::mutex t_mutex;
std::condition_variable t_cv;
bool g_stop = false;
bool cap_runing = false;
double target_point = 5.0f; // 目标点的x坐标，单位为厘米
double d_cm = 0.0f;

void mission1(){
    std::string pipeline = 
    "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a ! "
    "video/x-raw, format=NV12, width=1680, height=480 ! " // 传感器输出
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

    cv::Mat frame_rubbish;
    int warmupFrames = 30;   // 按摄像头/分辨率调整
    for (int i = 0; i < warmupFrames; ++i) {
        if (!cap.read(frame_rubbish)) {
            std::cerr << "Warmup failed at frame " << i << std::endl;
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(t_mutex);
        cap_runing = true;
    }

    double pipe_length = 25.0f;
    double length_per_pixel = 0.060532687651f; 

    while (true) {
        cv::Mat frame;
        cap >> frame; // Capture a new frame
        if (frame.empty()) {
            std::cerr << "Failed to capture frame" << std::endl;
            break;
        }

        if(warmupFrames > 0) {
            --warmupFrames;
            continue; // Skip processing during warmup
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
            if(det.label == "pipe" && det.score < 1.0f) {
                cv::rectangle(frame, det.upper, det.lower, cv::Scalar(0, 255, 0), 2); // 在检测到的画布位置画一个绿色矩形框
                cv::putText(frame,                                      // 图像
                    det.label + " " + cv::format("%.2f", det.score),                    // 文本内容
                    cv::Point(det.upper.x, std::max(0, det.upper.y - 10)),       // 文本位置
                    cv::FONT_HERSHEY_SIMPLEX,                                   // 字体类型
                    0.5,                                                        // 字体大小
                    cv::Scalar(0, 255, 0),                                      // 颜色（BGR）
                    1                                                           // 线条粗细
                );
                length_per_pixel = pipe_length / (det.lower.x - det.upper.x); // 计算每个像素对应的实际长度
            }

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

                double d = ((det.upper.x + det.lower.x) / 2) - (320+target_point/length_per_pixel); // 将检测到的钢球的中心点x坐标入队

                {
                    std::lock_guard<std::mutex> lock(t_mutex);
                    d_cm = d * length_per_pixel; // 将像素距离转换为厘米距离
                }
                
                
                {
                    std::lock_guard<std::mutex> lock(g_mutex);
                    g_queue.push(d_cm); // 将检测到的钢球的中心点x坐标入队
                }
                g_cv.notify_one();  // 通知消费者

                // continue; // 如果检测到钢球，跳过管道检测
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
    double last_data = 0.0;
    std::chrono::steady_clock::time_point last_send_time = std::chrono::steady_clock::now();
    double speed = 0.0;

    UART uart("/dev/ttyAMA0", 115200);
    if (!uart.open()) {
        std::cerr << "Failed to open UART" << std::endl;
        return;
    }
    
   
    while(!cap_runing) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    while (true) {
        std::unique_lock<std::mutex> lock(g_mutex);
        g_cv.wait(lock, [] { return !g_queue.empty() || g_stop; });

        if (g_stop && g_queue.empty()) {
            break;
        }

        double data = g_queue.front();
        g_queue.pop();
        lock.unlock();
        if(last_data != 0.0f) {
        speed = (data - last_data) / std::chrono::duration<double>(std::chrono::steady_clock::now() - last_send_time).count();
        }
        last_send_time = std::chrono::steady_clock::now();
        last_data = data;

        
        std::string data_to_send = std::format("distance:{:.1f} speed:{:.1f}", data, speed);
        std::cout << data_to_send << std::endl;
        if (!uart.send(data_to_send)) {
            std::cerr << "Failed to send data over UART" << std::endl;
            break;
        }

        
    }
    

    uart.close();
}

void mission3(){
    // {
    //     std::lock_guard<std::mutex> lock(t_mutex);
        while(!cap_runing) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    // }
    
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 等待，确保mission1和mission2已经开始运行

    while(1){
        double local_d_cm;
        // {
            // std::lock_guard<std::mutex> lock(t_mutex);
            local_d_cm = d_cm; // 读取共享变量d_cm
        // }

        if(local_d_cm < -0.5f) {
            std::cout << "Steel ball is to the left of the target point." << std::endl;
        } else if(local_d_cm > 0.5f) {
            std::cout << "Steel ball is to the right of the target point." << std::endl;
        } else {
            std::cout << "Steel ball is at the target point." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 每0.5秒检查一次
    }
}