/// @file ugv_serial_driver.hpp
/// @brief UGV 바퀴/팬틸트/LED 시리얼 프로토콜 드라이버 (순수 C++, ROS2 의존성 없음)
///
/// ## UGV 바퀴 제어란?
/// UGV(Unmanned Ground Vehicle)는 4개의 메카넘 바퀴로 이동합니다.
/// 바퀴 모터는 별도의 ESP32가 제어하며, RPi의 /dev/ttyAMA0 (UART)으로 연결됩니다.
/// (Jetson의 경우 /dev/ttyTHS1)
///
/// ## ESP32 JSON 프로토콜 (단방향 — PC→ESP32만)
/// 바퀴 제어는 응답을 읽을 필요가 없습니다 (write-only).
/// ESP32가 명령을 받으면 즉시 모터를 구동합니다.
///
/// ### 명령 종류
/// | T 값 | 용도              | 예시                                           |
/// |------|-------------------|------------------------------------------------|
/// | 13   | 모터 속도 설정    | {"T":"13","X":0.5,"Z":0.2}                     |
/// | 132  | LED 제어          | {"T":132,"IO4":1,"IO5":0}                      |
/// | 134  | 팬틸트 제어       | {"T":134,"X":45.0,"Y":-10.0,"SX":600,"SY":600}|
///
/// ### T:13 (모터 속도)
/// - X: 전진/후진 선속도 (m/s, 양수=전진, 음수=후진)
/// - Z: 회전 각속도 (rad/s, 양수=반시계, 음수=시계)
///
/// ### T:134 (팬틸트 카메라)
/// - X: 수평 회전 각도 (도, degree)
/// - Y: 수직 회전 각도 (도, degree)
/// - SX, SY: 서보 속도 (0~1000, 기본값 600)
///
/// ### T:132 (LED)
/// - IO4, IO5: GPIO 핀 상태 (0 또는 1)

#pragma once

#include "ugv_cpp_nodes/serial_driver.hpp"

#include <string>

namespace ugv {

/// UGV 바퀴/팬틸트/LED를 ESP32 시리얼로 제어하는 드라이버.
///
/// RoArmSerialDriver와 달리 단방향(write-only) 통신입니다.
/// ESP32에 명령을 보내기만 하고 응답을 기다리지 않습니다.
///
/// ## 왜 RoArmSerialDriver와 분리되어 있는가?
/// 1. 물리적으로 다른 시리얼 포트 사용 (ttyAMA0 vs ttyUSB0)
/// 2. 다른 ESP32 펌웨어 (바퀴용 vs 팔용)
/// 3. 통신 패턴이 다름 (write-only vs request-response)
class UgvSerialDriver {
public:
    /// @brief 드라이버를 생성합니다 (아직 연결하지 않음).
    /// @param port 시리얼 포트 (RPi: "/dev/ttyAMA0", Jetson: "/dev/ttyTHS1")
    /// @param baud_rate 보드레이트 (기본값: 115200)
    explicit UgvSerialDriver(const std::string& port = "/dev/ttyAMA0",
                              int baud_rate = 115200);

    /// @brief 시리얼 포트를 열고 ESP32와 연결합니다.
    bool connect();

    /// @brief 시리얼 포트를 닫습니다.
    void disconnect();

    /// @brief 연결 상태를 확인합니다.
    bool is_connected() const;

    /// @brief 바퀴 속도를 설정합니다 (T:13).
    /// @param linear 선속도 (m/s, 양수=전진, 음수=후진)
    /// @param angular 각속도 (rad/s, 양수=반시계, 음수=시계)
    ///
    /// angular_scale은 이 드라이버가 아닌 ROS2 노드(UgvDriverNode)에서 적용합니다.
    /// 이 함수는 스케일링된 최종 값을 받습니다.
    bool set_velocity(double linear, double angular);

    /// @brief 팬틸트 카메라 각도를 설정합니다 (T:134).
    /// @param x_deg 수평 회전 (도, degree)
    /// @param y_deg 수직 회전 (도, degree)
    /// @param sx 수평 서보 속도 (0~1000, 기본값 600)
    /// @param sy 수직 서보 속도 (0~1000, 기본값 600)
    bool set_pan_tilt(double x_deg, double y_deg, int sx = 600, int sy = 600);

    /// @brief LED를 제어합니다 (T:132).
    /// @param io4 GPIO4 핀 상태 (0.0=끔, 1.0=켬)
    /// @param io5 GPIO5 핀 상태 (0.0=끔, 1.0=켬)
    bool set_led(double io4, double io5);

private:
    SerialDriver serial_;  ///< 저수준 시리얼 포트 드라이버

    /// @brief JSON 명령을 전송합니다. 뮤텍스로 보호됩니다.
    void send_command(const std::string& json);
};

}  // namespace ugv
