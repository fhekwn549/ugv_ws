/// @file ugv_serial_driver.cpp
/// @brief UGV 바퀴/팬틸트/LED 시리얼 드라이버 구현
///
/// ## 통신 흐름 (cmd_vel → 바퀴 모터)
///
/// ```
/// [ROS2 /cmd_vel 토픽]
///       ↓ (UgvDriverNode가 수신)
/// [angular_scale 적용, 최소 각속도 보정]
///       ↓
/// [UgvSerialDriver::set_velocity()]
///       ↓
/// [시리얼 전송: {"T":"13","X":0.5,"Z":0.2}\n]
///       ↓
/// [ESP32 → 모터 PWM 출력 → 바퀴 회전]
/// ```
///
/// 이 과정은 단방향이며, ESP32는 명령 수신 즉시 모터를 제어합니다.
/// 응답을 기다리지 않으므로 지연이 최소화됩니다.

#include "ugv_cpp_nodes/ugv_serial_driver.hpp"

#include <cstdio>

namespace ugv {

UgvSerialDriver::UgvSerialDriver(const std::string& port, int baud_rate)
    : serial_(port, baud_rate) {}

bool UgvSerialDriver::connect() {
    return serial_.open();
}

void UgvSerialDriver::disconnect() {
    serial_.close();
}

bool UgvSerialDriver::is_connected() const {
    return serial_.is_open();
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
