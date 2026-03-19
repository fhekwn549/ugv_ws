/// @file serial_driver.hpp
/// @brief 저수준 POSIX 시리얼 포트 드라이버 (순수 C++, ROS2 의존성 없음)
///
/// 이 클래스는 Linux의 termios API를 사용하여 시리얼 포트(UART)를 제어합니다.
/// UGV 로봇의 ESP32 마이크로컨트롤러와 통신하기 위한 기반 클래스입니다.
///
/// ## 왜 순수 C++인가?
/// - ROS2에 의존하지 않으므로 단위 테스트가 쉽고, 다른 프로젝트에서도 재사용 가능
/// - Python의 pyserial 대비 GIL(Global Interpreter Lock) 경합 없이 진정한 멀티스레딩 지원
/// - 가비지 컬렉션 없이 예측 가능한 타이밍 보장
///
/// ## 사용 예시
/// ```cpp
/// ugv::SerialDriver serial("/dev/ttyUSB0", 115200);
/// if (serial.open()) {
///     serial.write_bytes("{\"T\":105}\n");
///     std::string response = serial.read_line(500);  // 500ms 타임아웃
///     serial.close();
/// }
/// ```
///
/// ## 스레드 안전성
/// mutex()를 통해 뮤텍스에 접근할 수 있으며, 상위 드라이버(RoArmSerialDriver 등)에서
/// std::lock_guard로 시리얼 읽기/쓰기를 보호합니다.

#pragma once

#include <string>
#include <mutex>
#include <optional>

namespace ugv {

/// POSIX 시리얼 포트를 제어하는 저수준 드라이버.
///
/// 내부적으로 Linux의 file descriptor(fd)와 termios 구조체를 사용하여
/// 시리얼 포트를 열고, 데이터를 읽고 쓸 수 있습니다.
/// 이 클래스는 복사할 수 없습니다 (파일 디스크립터 소유권 때문).
class SerialDriver {
public:
    /// @brief 시리얼 드라이버를 생성합니다 (아직 포트를 열지는 않음).
    /// @param port 시리얼 포트 장치 경로 (예: "/dev/ttyUSB0")
    /// @param baud_rate 통신 속도 (기본값: 115200 bps)
    SerialDriver(const std::string& port = "/dev/ttyUSB0", int baud_rate = 115200);

    /// @brief 소멸자. 열려 있는 포트를 자동으로 닫습니다.
    ~SerialDriver();

    // 파일 디스크립터를 소유하므로 복사 금지
    SerialDriver(const SerialDriver&) = delete;
    SerialDriver& operator=(const SerialDriver&) = delete;

    /// @brief 시리얼 포트를 엽니다 (8N1, 하드웨어 흐름 제어 없음).
    /// @return 성공 시 true, 이미 열려 있으면 true, 실패 시 false
    bool open();

    /// @brief 시리얼 포트를 닫습니다.
    void close();

    /// @brief 포트가 열려 있는지 확인합니다.
    bool is_open() const;

    /// @brief 시리얼 포트에 문자열 데이터를 씁니다.
    /// @param data 전송할 바이트 데이터 (문자열 형태)
    /// @return 전송된 바이트 수, 오류 시 -1 (이때 is_open()이 false로 변경됨)
    int write_bytes(const std::string& data);

    /// @brief 시리얼 포트에서 한 줄(\n까지)을 읽습니다.
    /// @param timeout_ms 타임아웃 (밀리초, 기본값 500ms)
    /// @return 읽은 문자열 (개행/캐리지 리턴 제거됨), 타임아웃 시 빈 문자열
    std::string read_line(int timeout_ms = 500);

    /// @brief 수신 버퍼를 비웁니다 (오래된 데이터 제거).
    void flush_input();

    /// @brief 외부에서 시리얼 I/O를 보호하기 위한 뮤텍스 접근자.
    /// 상위 드라이버에서 std::lock_guard로 사용합니다.
    std::mutex& mutex() { return mutex_; }

private:
    std::string port_;       ///< 시리얼 포트 경로 (예: "/dev/ttyUSB0")
    int baud_rate_;          ///< 통신 속도 (bps)
    int fd_ = -1;            ///< POSIX 파일 디스크립터 (-1이면 닫혀 있음)
    bool opened_ = false;    ///< 포트 열림 상태
    std::mutex mutex_;       ///< 스레드 안전 보장용 뮤텍스

    // 버퍼드 읽기 (Python ReadLine 클래스와 동일한 방식)
    static constexpr size_t READ_BUF_SIZE = 1024;
    char read_buf_[READ_BUF_SIZE];
    size_t buf_start_ = 0;  ///< 버퍼에서 아직 처리 안 된 데이터 시작
    size_t buf_end_ = 0;    ///< 버퍼에서 유효 데이터 끝

    /// @brief 정수 보드레이트를 termios speed_t 상수로 변환합니다.
    /// @param baud 정수 보드레이트 (예: 115200)
    /// @return 대응하는 termios 상수 (예: B115200)
    static int baud_to_speed(int baud);
};

}  // namespace ugv
