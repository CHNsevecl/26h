#include "mission.hpp"

// ============ 全局变量定义 ============
std::queue<double> g_queue;
std::queue<double> t_queue;
std::mutex g_mutex;
std::condition_variable g_cv;
std::mutex t_mutex;
std::condition_variable t_cv;
std::atomic<bool> g_stop{false};
std::atomic<bool> cap_runing{false};
double target_point = 5.0;   // 初始目标 5cm

// ============================================================
// mission1：视觉线程
//   1. 检测钢球，算出 d_cm
//   2. push 到 g_queue（给 mission2 发 UART）
//   3. push 到 t_queue（给 mission3 做状态机）
// ============================================================
void mission1() {
    std::string pipeline =
        "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a ! "
        "video/x-raw, format=NV12, width=1680, height=480 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return;
    }

    std::optional<HailoContext> hailo_context = Hailo_init(hef_path);
    if (!hailo_context) {
        std::cerr << "Failed to initialize Hailo context" << std::endl;
        return;
    }

    cv::Mat frame_rubbish;
    int warmupFrames = 30;
    for (int i = 0; i < warmupFrames; ++i) {
        if (!cap.read(frame_rubbish)) {
            std::cerr << "Warmup failed at frame " << i << std::endl;
            return;
        }
    }

    cap_runing.store(true, std::memory_order_release);

    double pipe_length = 25.0;
    double length_per_pixel = 0.060532687651;

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Failed to capture frame" << std::endl;
            break;
        }

        if (warmupFrames > 0) {
            --warmupFrames;
            continue;
        }

        frame = Stream_process(frame, target_w, target_h);
        std::optional<std::vector<Detection>> detections_opt =
            ParseDetections(*hailo_context, target_w, target_h, frame, class_names);
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

        if (!detections_opt) {
            std::cerr << "Error: Failed to parse detections." << std::endl;
            break;
        }
        const auto& detections = *detections_opt;

        for (const auto& det : detections) {
            if (det.label == "pipe" && det.score < 1.0f) {
                cv::rectangle(frame, det.upper, det.lower, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame,
                            det.label + " " + cv::format("%.2f", det.score),
                            cv::Point(det.upper.x, std::max(0, det.upper.y - 10)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 255, 0), 1);
                length_per_pixel = pipe_length / (det.lower.x - det.upper.x);
            }

            if (det.label == "Steel_Ball" && det.score < 1.0f) {
                cv::rectangle(frame, det.upper, det.lower, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame,
                            det.label + " " + cv::format("%.2f", det.score),
                            cv::Point(det.upper.x, std::max(0, det.upper.y - 10)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 255, 0), 1);

                // 读 target_point（加锁）
                double local_tp;
                {
                    std::lock_guard<std::mutex> lock(t_mutex);
                    local_tp = target_point;
                }

                double ball_center_x = (det.upper.x + det.lower.x) / 2.0;
                double d_pixel = ball_center_x - (320.0 + local_tp / length_per_pixel);
                double local_d_cm = d_pixel * length_per_pixel;

                // 同时推入两个队列（各自锁保护）
                {
                    std::lock_guard<std::mutex> lock(g_mutex);
                    g_queue.push(local_d_cm);
                }
                {
                    std::lock_guard<std::mutex> lock(t_mutex);
                    t_queue.push(local_d_cm);
                }

                g_cv.notify_one();   // 通知 mission2
                t_cv.notify_one();   // 通知 mission3
            }
        }

        cv::imshow("Camera Feed", frame);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    // 退出前通知所有等待者
    g_stop.store(true, std::memory_order_release);
    g_cv.notify_all();
    t_cv.notify_all();
}

// ============================================================
// mission2：UART 发送线程
//   从 g_queue 拿 d_cm，计算速度，发 UART
// ============================================================
void mission2() {
    double last_data = 0.0;
    auto last_send_time = std::chrono::steady_clock::now();
    double speed = 0.0;

    UART uart("/dev/ttyAMA0", 115200);
    if (!uart.open()) {
        std::cerr << "Failed to open UART" << std::endl;
        return;
    }

    // 等相机启动
    while (!cap_runing.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    while (true) {
        std::unique_lock<std::mutex> lock(g_mutex);
        g_cv.wait(lock, [] {
            return !g_queue.empty() || g_stop.load(std::memory_order_acquire);
        });

        if (g_stop.load(std::memory_order_acquire) && g_queue.empty()) {
            break;
        }

        double data = g_queue.front();
        g_queue.pop();
        lock.unlock();

        if (last_data != 0.0) {
            double dt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_send_time).count();
            if (dt > 0.0) {
                speed = (data - last_data) / dt;
            }
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

// ============================================================
// mission3：状态机线程
//   从 t_queue 拿 d_cm，根据 target_point 做状态切换
// ============================================================
void mission3() {
    using namespace std::chrono;

    // 等相机启动
    while (!cap_runing.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(milliseconds(100));
    }

    std::this_thread::sleep_for(seconds(1));

    // 状态机：第一阶段目标 5，第二阶段目标 -5，第三阶段归 0
    auto phase_start = steady_clock::now();
    double current_target = 5.0;   // 本地记录当前阶段目标，避免反复读 target_point
    const double TOL = 0.5;        // 误差阈值 cm
    const double STABLE_TIME = 1.0; // 稳定时间秒

    while (true) {
        double local_d_cm;
        {
            std::unique_lock<std::mutex> lock(t_mutex);
            t_cv.wait(lock, [] {
                return !t_queue.empty() || g_stop.load(std::memory_order_acquire);
            });

            if (g_stop.load(std::memory_order_acquire) && t_queue.empty()) {
                break;
            }

            local_d_cm = t_queue.front();
            t_queue.pop();
        }

        // 状态机判断
        double err = std::abs(local_d_cm);

        if (err < TOL) {
            double elapsed = duration<double>(steady_clock::now() - phase_start).count();
            if (elapsed >= STABLE_TIME) {
                // 切换目标
                if (std::abs(current_target - 5.0) < 0.5) {
                    current_target = -5.0;
                } else if (std::abs(current_target + 5.0) < 0.5) {
                    current_target = 0.0;
                } else {
                    // 已完成，可以 break 或继续
                    break;
                }

                {
                    std::lock_guard<std::mutex> lock(t_mutex);
                    target_point = current_target;
                }

                phase_start = steady_clock::now();   // 重新计时
                std::cout << "[mission3] target -> " << current_target << std::endl;
            }
        } else {
            // 误差超标，重新计时
            phase_start = steady_clock::now();
        }
    }
}