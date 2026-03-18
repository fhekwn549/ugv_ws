/// @file roarm_serial_driver.hpp
/// @brief RoArm-M2 로봇 팔 시리얼 프로토콜 드라이버 (순수 C++, ROS2 의존성 없음)
///
/// ## RoArm-M2란?
/// Waveshare RoArm-M2는 4축(base, shoulder, elbow, hand) 로봇 팔입니다.
/// ESP32 마이크로컨트롤러가 내장되어 있으며, 시리얼(UART)을 통해 JSON 명령을 주고받습니다.
/// RPi에서는 /dev/ttyUSB0 (USB-Serial 변환기)로 연결됩니다.
///
/// ## ESP32 JSON 프로토콜
/// 모든 명령/응답은 JSON 형식이며, "T" 필드로 명령 종류를 구분합니다.
///
/// ### 명령 종류
/// | T 값 | 용도                  | 방향        | 예시                                              |
/// |------|-----------------------|-------------|---------------------------------------------------|
/// | 102  | 관절 이동             | PC → ESP32  | {"T":102,"base":0,"shoulder":-1.6,...}             |
/// | 105  | 관절 피드백 요청      | PC → ESP32  | {"T":105}                                         |
/// | 106  | 그리퍼 제어           | PC → ESP32  | {"T":106,"cmd":3.0,"spd":0,"acc":0}               |
/// | 210  | 토크 활성화/비활성화  | PC → ESP32  | {"T":210,"cmd":1}                                 |
///
/// ### T:105 응답 형식 (ESP32 → PC)
/// ESP32는 명령을 에코(echo)한 후, 실제 응답을 보냅니다:
/// ```
/// {"T":105}\r\n          ← 에코 (우리가 보낸 명령을 그대로 돌려줌)
/// \r\n                   ← 빈 줄
/// {"T":"1051","b":0.1,"s":-1.5,"e":3.1,"t":2.9,"x":100,"y":200,"z":300}\r\n
///                        ↑ 실제 응답 (T가 문자열 "1051"임에 주의!)
/// ```
/// - b: base 관절 각도 (rad)
/// - s: shoulder 관절 각도 (rad)
/// - e: elbow 관절 각도 (rad)
/// - t: hand(gripper) 관절 각도 (rad)
/// - x, y, z: 엔드이펙터 위치 (mm)

#pragma once

#include "ugv_cpp_nodes/serial_driver.hpp"

#include <string>
#include <vector>
#include <optional>
#include <map>
#include <mutex>

namespace ugv {

/// T:105 응답에서 파싱된 로봇 팔 관절 피드백 데이터.
/// 모든 각도는 라디안 단위, 위치는 mm 단위입니다.
struct ArmFeedback {
    double base = 0.0;      ///< base 관절 각도 (rad) — 좌우 회전
    double shoulder = 0.0;  ///< shoulder 관절 각도 (rad) — 위아래 1번
    double elbow = 0.0;     ///< elbow 관절 각도 (rad) — 위아래 2번
    double hand = 0.0;      ///< hand/gripper 관절 각도 (rad) — 집게
    double x = 0.0;         ///< 엔드이펙터 X 위치 (mm)
    double y = 0.0;         ///< 엔드이펙터 Y 위치 (mm)
    double z = 0.0;         ///< 엔드이펙터 Z 위치 (mm)
};

/// RoArm-M2 ESP32와 시리얼 통신하는 드라이버.
///
/// 이 클래스는 ROS2에 의존하지 않으며, 순수 C++로 작성되어
/// 단위 테스트와 재사용이 용이합니다.
///
/// ## 스레드 안전성
/// 모든 시리얼 I/O는 내부 뮤텍스로 보호됩니다.
/// 여러 ROS2 콜백에서 동시에 호출해도 안전합니다.
class RoArmSerialDriver {
public:
    /// @brief 드라이버를 생성합니다 (아직 연결하지 않음).
    /// @param port 시리얼 포트 (RPi 기본값: "/dev/ttyUSB0")
    /// @param baud_rate 보드레이트 (기본값: 115200)
    explicit RoArmSerialDriver(const std::string& port = "/dev/ttyUSB0",
                                int baud_rate = 115200);

    /// @brief 시리얼 포트를 열고 ESP32와 연결합니다.
    /// @return 성공 시 true
    bool connect();

    /// @brief 시리얼 포트를 닫습니다.
    void disconnect();

    /// @brief 연결 상태를 확인합니다.
    bool is_connected() const;

    /// @brief 토크(모터 전원)를 켜거나 끕니다 (T:210).
    /// 토크가 꺼지면 팔이 중력에 의해 아래로 떨어지므로,
    /// 보통 노드 시작 시 enable, 종료 시 disable 합니다.
    /// @param enable true=토크 켜기, false=토크 끄기
    bool set_torque(bool enable);

    /// @brief 4개 관절을 동시에 이동합니다 (T:102).
    /// @param base base 관절 목표 각도 (rad) — 좌우 회전
    /// @param shoulder shoulder 관절 목표 각도 (rad)
    /// @param elbow elbow 관절 목표 각도 (rad)
    /// @param hand hand(gripper) 관절 목표 각도 (rad)
    /// @param spd 속도 (0=기본속도)
    /// @param acc 가속도 (0~20, 기본값 10)
    bool move_joints(double base, double shoulder, double elbow, double hand,
                     int spd = 0, int acc = 10);

    /// @brief 그리퍼를 제어합니다 (T:106).
    /// @param value 그리퍼 위치 (rad)
    /// @param spd 속도 (0=기본)
    /// @param acc 가속도 (0=기본)
    bool set_gripper(double value, int spd = 0, int acc = 0);

    /// @brief ESP32에 현재 관절 위치를 요청합니다 (T:105).
    /// @return 성공 시 ArmFeedback 구조체, 실패/타임아웃 시 nullopt
    std::optional<ArmFeedback> get_feedback();

private:
    SerialDriver serial_;  ///< 저수준 시리얼 포트 드라이버

    /// @brief JSON 명령을 전송합니다 (응답 읽지 않음).
    /// 뮤텍스로 보호됩니다.
    void send_command(const std::string& json);

    /// @brief JSON 명령을 전송하고 응답을 읽습니다.
    /// ESP32의 에코를 건너뛰고 실제 응답만 반환합니다.
    /// 뮤텍스로 보호됩니다.
    std::string send_query(const std::string& json);

    /// @brief key-value 쌍들로 JSON 문자열을 생성합니다.
    /// 값이 숫자이면 따옴표 없이, 문자열이면 따옴표를 붙여서 생성합니다.
    /// 예: [("T","102"),("base","0.5")] → {"T":102,"base":0.5}
    static std::string build_json(const std::vector<std::pair<std::string, std::string>>& fields);

    /// @brief JSON 문자열에서 특정 키의 숫자 값을 파싱합니다.
    /// 예: parse_json_double({"b":0.1,"s":-1.5}, "b") → 0.1
    static std::optional<double> parse_json_double(const std::string& json, const std::string& key);

    /// @brief JSON 문자열에서 특정 키의 문자열 값을 파싱합니다.
    /// 예: parse_json_string({"T":"1051"}, "T") → "1051"
    static std::optional<std::string> parse_json_string(const std::string& json, const std::string& key);
};

}  // namespace ugv
