/// @file ugv_serial_driver.cpp
/// @brief UGV 바퀴/팬틸트/LED 시리얼 드라이버 + 센서 피드백 구현
///
/// ## 통신 흐름
///
/// ### 명령 전송 (cmd_vel → 바퀴 모터)
/// ```
/// [ROS2 /cmd_vel]  →  [UgvDriverNode]  →  [UgvSerialDriver::set_velocity()]
///                                              ↓ T:13 JSON
///                                          [ESP32 → 모터]
/// ```
///
/// ### 센서 피드백 (ESP32 → ROS2 토픽)
/// ```
/// [ESP32 T:1001 연속 전송]  →  [feedback_loop() 스레드]  →  [UgvDriverNode]
///                                                              ↓
///                                                    [/imu/data, /voltage, /odom/odom_raw]
/// ```
///
/// Linux에서 같은 fd에 대한 read()와 write()는 독립적인 커널 버퍼를 사용하므로
/// 피드백 스레드의 read와 명령 전송의 write가 동시에 안전하게 동작합니다.

#include "ugv_cpp_nodes/ugv_serial_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace ugv {

UgvSerialDriver::UgvSerialDriver(const std::string& port, int baud_rate)
    : serial_(port, baud_rate) {}

UgvSerialDriver::~UgvSerialDriver() {
    feedback_running_ = false;
    if (feedback_thread_.joinable()) {
        feedback_thread_.join();
    }
}

bool UgvSerialDriver::connect() {
    return serial_.open();
}

void UgvSerialDriver::disconnect() {
    feedback_running_ = false;
    if (feedback_thread_.joinable()) {
        feedback_thread_.join();
    }
    serial_.close();
}

bool UgvSerialDriver::is_connected() const {
    return serial_.is_open();
}

// === 센서 피드백 ===

void UgvSerialDriver::enable_feedback() {
    // ESP32에 연속 피드백 모드 활성화 명령 전송
    send_command(R"({"T":131,"cmd":1})");

    // 피드백 읽기 스레드 시작
    feedback_running_ = true;
    feedback_thread_ = std::thread(&UgvSerialDriver::feedback_loop, this);
}

std::optional<UgvFeedback> UgvSerialDriver::get_feedback() {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    auto fb = latest_feedback_;
    latest_feedback_.reset();
    return fb;
}

void UgvSerialDriver::feedback_loop() {
    while (feedback_running_) {
        // read_line은 뮤텍스 없이 호출 — rx 버퍼는 tx와 독립적
        std::string line = serial_.read_line(200);
        if (line.empty()) continue;

        auto fb = parse_feedback(line);
        if (fb) {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            latest_feedback_ = fb;
        }
    }
}

std::optional<UgvFeedback> UgvSerialDriver::parse_feedback(const std::string& line) {
    // 불완전한 JSON 라인 거부: { 로 시작하고 } 로 끝나야 함
    // 시리얼 읽기에서 라인이 잘리면 필드 값이 깨질 수 있음
    if (line.empty() || line.front() != '{' || line.back() != '}') return std::nullopt;

    // T:1001이 포함되지 않은 라인은 무시
    if (line.find("1001") == std::string::npos) return std::nullopt;

    // JSON 키에서 숫자 값을 추출하는 람다.
    // "key": 패턴이 {나 , 뒤에 올 때만 매칭하여
    // "p"가 "temp" 안에서 잘못 매칭되는 것을 방지합니다.
    auto extract = [&](const char* key) -> std::optional<double> {
        char pattern[32];
        std::snprintf(pattern, sizeof(pattern), "\"%s\":", key);

        size_t pos = 0;
        while (true) {
            pos = line.find(pattern, pos);
            if (pos == std::string::npos) return std::nullopt;
            // 키 앞이 { 또는 , 인지 확인 (완전한 키 매칭)
            if (pos == 0 || line[pos - 1] == '{' || line[pos - 1] == ',') {
                break;
            }
            pos += std::strlen(pattern);
        }
        pos += std::strlen(pattern);
        while (pos < line.size() && line[pos] == ' ') pos++;

        char* end;
        double val = std::strtod(line.c_str() + pos, &end);
        if (end == line.c_str() + pos) return std::nullopt;
        return val;
    };

    auto t = extract("T");
    if (!t || static_cast<int>(*t) != 1001) return std::nullopt;

    // 필수 필드 추출 — 하나라도 없으면 깨진 라인이므로 거부
    auto r = extract("r");
    auto p = extract("p");
    auto y = extract("y");
    auto v = extract("v");
    if (!r || !p || !y || !v) return std::nullopt;

    UgvFeedback fb;
    fb.r = *r;
    fb.p = *p;
    fb.y = *y;
    fb.v = *v;

    auto L = extract("L"); if (L) fb.L = static_cast<int>(*L);
    auto R = extract("R"); if (R) fb.R = static_cast<int>(*R);
    auto temp = extract("temp"); if (temp) fb.temp = *temp;

    return fb;
}

/// 바퀴 속도 명령을 ESP32에 전송합니다 (T:13).
///
/// ESP32 펌웨어는 이 값을 받아 4개 바퀴의 PWM을 계산합니다 (차동 구동).
/// snprintf로 JSON을 직접 조립하여 Python json.dumps() 대비
/// 메모리 할당과 연산 오버헤드를 줄입니다.
bool UgvSerialDriver::set_velocity(double linear, double angular) {
    if (!is_connected()) return false;

    // snprintf: 사전 할당된 버퍼에 직접 포맷팅 → 힙 할당 최소화
    // R"(...)": C++ raw string literal — 이스케이프 없이 JSON 작성 가능
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  R"({"T":"13","X":%.4f,"Z":%.4f})",
                  linear, angular);
    send_command(buf);
    return true;
}

/// 팬틸트 카메라 서보 명령을 전송합니다 (T:134).
///
/// ROS2에서는 각도를 라디안(rad)으로 다루지만,
/// ESP32 팬틸트 서보는 도(degree) 단위를 사용합니다.
/// 라디안→도 변환은 ROS2 노드(UgvDriverNode)에서 수행합니다.
bool UgvSerialDriver::set_pan_tilt(double x_deg, double y_deg, int sx, int sy) {
    if (!is_connected()) return false;

    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  R"({"T":134,"X":%.4f,"Y":%.4f,"SX":%d,"SY":%d})",
                  x_deg, y_deg, sx, sy);
    send_command(buf);
    return true;
}

/// LED 제어 명령을 전송합니다 (T:132).
/// IO4, IO5는 ESP32의 GPIO 핀 번호입니다.
bool UgvSerialDriver::set_led(double io4, double io5) {
    if (!is_connected()) return false;

    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  R"({"T":132,"IO4":%.1f,"IO5":%.1f})",
                  io4, io5);
    send_command(buf);
    return true;
}

/// JSON 명령 문자열을 시리얼 포트로 전송합니다.
///
/// 뮤텍스(lock_guard)로 보호되어, 여러 ROS2 콜백
/// (cmd_vel, joint_states, led_ctrl)이 동시에 호출되어도
/// 시리얼 데이터가 섞이지 않습니다.
///
/// 모든 명령은 개행 문자(\n)로 끝나야 합니다.
/// ESP32는 \n을 받으면 JSON 파싱을 시작합니다.
void UgvSerialDriver::send_command(const std::string& json) {
    std::lock_guard<std::mutex> lock(serial_.mutex());
    serial_.write_bytes(json + "\n");
}

}  // namespace ugv
