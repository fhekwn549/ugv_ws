/// @file roarm_serial_driver.cpp
/// @brief RoArm-M2 시리얼 드라이버 구현
///
/// ## 통신 흐름 (T:105 피드백 요청 예시)
///
/// ```
/// [C++ 드라이버]                          [ESP32]
///     |                                      |
///     |--- {"T":105}\n ---------------------->|  1) 명령 전송
///     |                                      |
///     |<-- {"T":105}\r\n --------------------|  2) 에코 (무시해야 함)
///     |<-- \r\n ------------------------------|  3) 빈 줄 (무시)
///     |<-- {"T":"1051","b":0.1,...}\r\n ------|  4) 실제 응답 (이것만 사용)
/// ```
///
/// send_query() 함수가 이 에코 처리를 담당합니다.

#include "ugv_cpp_nodes/roarm_serial_driver.hpp"

#include <sstream>
#include <cmath>
#include <cstring>
#include <algorithm>

namespace ugv {

RoArmSerialDriver::RoArmSerialDriver(const std::string& port, int baud_rate)
    : serial_(port, baud_rate) {}

bool RoArmSerialDriver::connect() {
    return serial_.open();
}

void RoArmSerialDriver::disconnect() {
    serial_.close();
}

bool RoArmSerialDriver::is_connected() const {
    return serial_.is_open();
}

/// 토크(모터 전원)를 켜거나 끕니다.
/// 토크 OFF 상태에서는 팔이 중력에 의해 자유 낙하합니다.
bool RoArmSerialDriver::set_torque(bool enable) {
    if (!is_connected()) return false;
    std::string json = build_json({
        {"T", "210"},
        {"cmd", enable ? "1" : "0"}
    });
    send_command(json);
    return true;
}

/// 4개 관절을 목표 각도(rad)로 이동시킵니다.
/// ESP32가 내부적으로 PID 제어를 수행합니다.
bool RoArmSerialDriver::move_joints(double base, double shoulder, double elbow,
                                     double hand, int spd, int acc) {
    if (!is_connected()) return false;

    // 소수점 4자리까지 포맷팅 (ESP32가 인식하는 정밀도)
    auto fmt = [](double v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.4f", v);
        return std::string(buf);
    };

    std::string json = build_json({
        {"T", "102"},
        {"base", fmt(base)},
        {"shoulder", fmt(shoulder)},
        {"elbow", fmt(elbow)},
        {"hand", fmt(hand)},
        {"spd", std::to_string(spd)},    // 0 = 기본 속도
        {"acc", std::to_string(acc)}      // 가속도 (0~20)
    });
    send_command(json);
    return true;
}

/// 그리퍼(집게)를 제어합니다.
/// T:106은 그리퍼 전용 명령으로, T:102와 별개로 작동합니다.
bool RoArmSerialDriver::set_gripper(double value, int spd, int acc) {
    if (!is_connected()) return false;

    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.4f", value);

    std::string json = build_json({
        {"T", "106"},
        {"cmd", std::string(buf)},
        {"spd", std::to_string(spd)},
        {"acc", std::to_string(acc)}
    });
    send_command(json);
    return true;
}

/// ESP32에 현재 관절 위치를 요청합니다 (T:105).
///
/// 응답의 T 필드는 문자열 "1051"입니다 (숫자 1051이 아님).
/// 이것은 ESP32 펌웨어의 특이사항입니다.
std::optional<ArmFeedback> RoArmSerialDriver::get_feedback() {
    if (!is_connected()) return std::nullopt;

    std::string json = build_json({{"T", "105"}});
    std::string response = send_query(json);

    if (response.empty()) return std::nullopt;

    // T:105 응답의 T 필드 검증
    // ESP32가 "1051"을 문자열 또는 숫자로 보낼 수 있으므로 두 가지 모두 확인
    auto t_val = parse_json_string(response, "T");
    if (!t_val.has_value() || t_val.value() != "1051") {
        auto t_num = parse_json_double(response, "T");
        if (!t_num.has_value() || static_cast<int>(t_num.value()) != 1051) {
            return std::nullopt;  // 예상하지 못한 응답 형식
        }
    }

    // 각 관절 각도를 파싱합니다
    ArmFeedback fb;
    auto b = parse_json_double(response, "b");
    auto s = parse_json_double(response, "s");
    auto e = parse_json_double(response, "e");
    auto t = parse_json_double(response, "t");
    auto x = parse_json_double(response, "x");
    auto y = parse_json_double(response, "y");
    auto z = parse_json_double(response, "z");

    if (b) fb.base = b.value();
    if (s) fb.shoulder = s.value();
    if (e) fb.elbow = e.value();
    if (t) fb.hand = t.value();
    if (x) fb.x = x.value();
    if (y) fb.y = y.value();
    if (z) fb.z = z.value();

    return fb;
}

/// JSON 명령을 ESP32에 전송합니다 (응답을 기다리지 않음).
/// 모든 명령은 개행 문자(\n)로 끝나야 합니다 — ESP32가 이것으로 명령 끝을 인식합니다.
void RoArmSerialDriver::send_command(const std::string& json) {
    std::lock_guard<std::mutex> lock(serial_.mutex());
    serial_.write_bytes(json + "\n");
}

/// JSON 명령을 전송하고 응답을 읽습니다.
///
/// ESP32의 에코(echo) 동작을 처리합니다:
/// 1. 수신 버퍼를 비움 (이전 잔여 데이터 제거)
/// 2. 명령 전송
/// 3. 최대 5줄을 읽으면서 에코와 빈 줄을 건너뜀
/// 4. T 필드가 "1051"인 줄을 찾으면 그것이 실제 응답
///
/// 에코 구분법:
/// - 에코의 T 값: 105 (우리가 보낸 것과 동일한 숫자)
/// - 응답의 T 값: "1051" (문자열, 다른 값)
std::string RoArmSerialDriver::send_query(const std::string& json) {
    std::lock_guard<std::mutex> lock(serial_.mutex());
    serial_.flush_input();       // 이전 잔여 데이터 제거
    serial_.write_bytes(json + "\n");

    // 최대 5줄을 읽으면서 실제 응답을 찾습니다
    for (int i = 0; i < 5; ++i) {
        std::string line = serial_.read_line(500);  // 500ms 타임아웃
        if (line.empty()) break;  // 타임아웃 — 더 이상 데이터 없음

        // JSON의 "T" 필드가 있는 줄만 검사합니다
        if (line.find("\"T\"") != std::string::npos || line.find("\"T\":") != std::string::npos) {
            auto t_str = parse_json_string(line, "T");
            auto t_num = parse_json_double(line, "T");

            // T가 "1051" 문자열이면 → 실제 응답
            if (t_str.has_value() && t_str.value() == "1051") {
                return line;
            }
            // T가 숫자 105이면 → 에코 (건너뜀)
            if (t_num.has_value() && static_cast<int>(t_num.value()) == 105) {
                continue;
            }
            // 그 외 → 알 수 없는 응답이지만 반환
            return line;
        }
    }

    return {};  // 응답을 찾지 못함
}

/// key-value 쌍 목록을 JSON 문자열로 변환합니다.
///
/// 값이 숫자(정수 또는 소수)이면 따옴표 없이 출력합니다:
///   {"T":102,"base":0.5000}
/// 값이 문자열이면 따옴표를 붙입니다:
///   {"T":"1051"}
///
/// 외부 JSON 라이브러리(nlohmann/json 등)를 사용하지 않는 이유:
/// - 의존성 최소화 (RPi에 추가 패키지 설치 불필요)
/// - ESP32 프로토콜이 단순하여 간단한 문자열 조합으로 충분
std::string RoArmSerialDriver::build_json(
    const std::vector<std::pair<std::string, std::string>>& fields) {
    std::string result = "{";
    for (size_t i = 0; i < fields.size(); ++i) {
        if (i > 0) result += ",";
        result += "\"" + fields[i].first + "\":";

        const auto& val = fields[i].second;

        // 값이 숫자인지 판별: 숫자(0-9), 마이너스(-), 소수점(.)만 포함
        bool is_numeric = !val.empty();
        bool has_dot = false;
        for (size_t j = 0; j < val.size(); ++j) {
            char c = val[j];
            if (c == '.') {
                has_dot = true;
            } else if (c == '-' && j == 0) {
                continue;  // 첫 글자 마이너스는 허용
            } else if (!std::isdigit(static_cast<unsigned char>(c))) {
                is_numeric = false;
                break;
            }
        }
        (void)has_dot;  // 컴파일러 경고 방지 (has_dot은 판별에만 사용)

        if (is_numeric) {
            result += val;               // 숫자: 따옴표 없이 {"T":102}
        } else {
            result += "\"" + val + "\""; // 문자열: 따옴표 포함 {"T":"1051"}
        }
    }
    result += "}";
    return result;
}

/// JSON 문자열에서 특정 키의 숫자 값을 파싱합니다.
/// 외부 JSON 라이브러리 없이 문자열 검색으로 구현합니다.
///
/// 예: parse_json_double(R"({"b":0.1,"s":-1.5})", "b") → 0.1
///
/// 값이 따옴표로 둘러싸인 문자열이면 nullopt을 반환합니다.
std::optional<double> RoArmSerialDriver::parse_json_double(
    const std::string& json, const std::string& key) {
    // "key": 패턴을 찾습니다
    std::string search = "\"" + key + "\":";
    auto pos = json.find(search);
    if (pos == std::string::npos) {
        // 콜론 뒤에 공백이 있는 경우도 시도
        search = "\"" + key + "\": ";
        pos = json.find(search);
        if (pos == std::string::npos) return std::nullopt;
    }

    pos += search.size();
    // 값 앞의 공백 건너뛰기
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) ++pos;

    if (pos >= json.size()) return std::nullopt;

    // 따옴표로 시작하면 문자열 값이므로 숫자가 아님
    if (json[pos] == '"') return std::nullopt;

    try {
        size_t end_pos;
        double val = std::stod(json.substr(pos), &end_pos);
        return val;
    } catch (...) {
        return std::nullopt;  // 파싱 실패
    }
}

/// JSON 문자열에서 특정 키의 문자열 값을 파싱합니다.
///
/// 예: parse_json_string(R"({"T":"1051"})", "T") → "1051"
///
/// 값이 따옴표로 둘러싸이지 않은 숫자이면 nullopt을 반환합니다.
std::optional<std::string> RoArmSerialDriver::parse_json_string(
    const std::string& json, const std::string& key) {
    std::string search = "\"" + key + "\":";
    auto pos = json.find(search);
    if (pos == std::string::npos) {
        search = "\"" + key + "\": ";
        pos = json.find(search);
        if (pos == std::string::npos) return std::nullopt;
    }

    pos += search.size();
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) ++pos;

    // 따옴표로 시작해야 문자열 값
    if (pos >= json.size() || json[pos] != '"') return std::nullopt;

    ++pos;  // 여는 따옴표 건너뛰기
    auto end = json.find('"', pos);  // 닫는 따옴표 찾기
    if (end == std::string::npos) return std::nullopt;

    return json.substr(pos, end - pos);
}

}  // namespace ugv
