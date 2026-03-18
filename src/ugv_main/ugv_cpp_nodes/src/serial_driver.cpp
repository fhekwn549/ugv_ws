/// @file serial_driver.cpp
/// @brief SerialDriver 구현 — POSIX termios 기반 시리얼 포트 제어
///
/// ## 핵심 개념
/// Linux에서 시리얼 포트(/dev/ttyUSB0 등)는 일반 파일처럼 열고(open),
/// 읽고(read), 쓸(write) 수 있습니다. termios 구조체로 보드레이트, 패리티,
/// 스톱비트 등 통신 설정을 제어합니다.
///
/// ## 시리얼 통신 기본 설정 (8N1)
/// - 8비트 데이터 (CS8)
/// - No 패리티 (~PARENB)
/// - 1 스톱비트 (~CSTOPB)
/// - 하드웨어 흐름 제어 없음 (~CRTSCTS)
/// 이 설정은 ESP32의 기본 UART 설정과 일치합니다.

#include "ugv_cpp_nodes/serial_driver.hpp"

#include <fcntl.h>        // open(), O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>      // struct termios, tcgetattr, tcsetattr, cfsetispeed, cfsetospeed
#include <unistd.h>       // read(), write(), close()
#include <sys/select.h>   // select() — 타임아웃 기반 읽기에 사용
#include <cstring>

namespace ugv {

SerialDriver::SerialDriver(const std::string& port, int baud_rate)
    : port_(port), baud_rate_(baud_rate) {}

SerialDriver::~SerialDriver() {
    close();
}

/// 정수 보드레이트를 Linux termios 상수로 변환합니다.
/// ESP32는 보통 115200 bps를 사용합니다.
int SerialDriver::baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;  // 알 수 없는 값이면 115200으로 기본 설정
    }
}

bool SerialDriver::open() {
    if (opened_) return true;  // 이미 열려 있으면 성공으로 반환

    // O_RDWR    : 읽기+쓰기 모드
    // O_NOCTTY  : 이 포트를 프로세스의 제어 터미널로 만들지 않음
    // O_NONBLOCK: 열기 시 블로킹하지 않음 (이후에 해제)
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    // O_NONBLOCK을 해제하여, 이후 read()가 데이터가 올 때까지 대기하도록 설정
    // (타임아웃은 select()로 별도 제어)
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    // 현재 시리얼 포트 설정을 가져옵니다
    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 보드레이트 설정 (입력/출력 동일)
    speed_t speed = baud_to_speed(baud_rate_);
    cfsetispeed(&tty, speed);  // 입력 보드레이트
    cfsetospeed(&tty, speed);  // 출력 보드레이트

    // === 8N1 설정 (가장 일반적인 시리얼 통신 설정) ===
    tty.c_cflag &= ~PARENB;    // 패리티 비트 없음 (No parity)
    tty.c_cflag &= ~CSTOPB;    // 스톱비트 1개 (1 stop bit)
    tty.c_cflag &= ~CSIZE;     // 데이터 비트 크기 초기화
    tty.c_cflag |= CS8;        // 데이터 비트 8개 (8 data bits)
    tty.c_cflag &= ~CRTSCTS;   // 하드웨어 흐름 제어 비활성화 (No RTS/CTS)
    tty.c_cflag |= CREAD | CLOCAL;  // 수신 활성화, 모뎀 상태 무시

    // === Raw 모드 설정 ===
    // 터미널 모드가 아닌 원시 바이트 모드로 동작합니다.
    // 이렇게 하면 ESP32가 보내는 JSON 데이터를 변조 없이 그대로 받을 수 있습니다.
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 라인 편집/에코/시그널 비활성화
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);           // 소프트웨어 흐름 제어 비활성화
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;  // 출력 후처리 비활성화 (보내는 데이터 변환 없음)

    // === 읽기 타임아웃 설정 ===
    // VMIN=0, VTIME=1 → 데이터가 없으면 100ms 후 즉시 반환
    // 이 설정은 select()와 함께 사용되어 정밀한 타임아웃을 제공합니다.
    tty.c_cc[VMIN] = 0;    // 최소 읽기 바이트 수: 0 (즉시 반환 가능)
    tty.c_cc[VTIME] = 1;   // 읽기 타임아웃: 100ms (0.1초 단위)

    // 설정 즉시 적용 (TCSANOW = 지금 바로 적용)
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 설정 변경 전 쌓여있던 오래된 데이터를 모두 버립니다
    tcflush(fd_, TCIOFLUSH);
    opened_ = true;
    return true;
}

void SerialDriver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    opened_ = false;
}

bool SerialDriver::is_open() const {
    return opened_;
}

int SerialDriver::write_bytes(const std::string& data) {
    if (!opened_) return -1;

    // POSIX write(): 파일 디스크립터에 데이터를 씁니다
    ssize_t n = ::write(fd_, data.c_str(), data.size());
    if (n <= 0) {
        // 쓰기 실패 → 포트 연결이 끊긴 것으로 간주
        opened_ = false;
        return -1;
    }
    return static_cast<int>(n);
}

/// 시리얼 포트에서 한 줄을 읽습니다.
///
/// 작동 방식:
/// 1. select()로 데이터가 올 때까지 대기 (타임아웃 있음)
/// 2. 한 바이트씩 읽으면서 개행 문자(\n)를 만나면 반환
/// 3. 캐리지 리턴(\r)은 무시
/// 4. 타임아웃 초과 시 빈 문자열 반환
///
/// ESP32의 응답 형식: "{"T":"1051","b":0.1,...}\r\n"
/// 이 함수는 \r\n을 제거하고 JSON 부분만 반환합니다.
std::string SerialDriver::read_line(int timeout_ms) {
    if (!opened_) return {};

    std::string result;
    result.reserve(256);  // 메모리 재할당 최소화를 위해 256바이트 미리 확보

    auto deadline_us = timeout_ms * 1000;  // 밀리초 → 마이크로초 변환
    int elapsed = 0;

    while (elapsed < deadline_us) {
        // select(): 파일 디스크립터에 읽을 데이터가 있을 때까지 대기
        // 타임아웃을 설정하여 무한 대기를 방지합니다
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);

        struct timeval tv;
        int remaining = deadline_us - elapsed;
        tv.tv_sec = remaining / 1000000;
        tv.tv_usec = remaining % 1000000;

        int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) break;  // 타임아웃(0) 또는 에러(-1)

        // 한 바이트 읽기
        char c;
        ssize_t n = ::read(fd_, &c, 1);
        if (n <= 0) break;  // 읽기 실패

        if (c == '\n') {
            return result;  // 개행 = 한 줄 완성
        }
        if (c != '\r') {
            result += c;    // 캐리지 리턴은 무시, 나머지 문자는 추가
        }

        // 경과 시간 근사치 (115200bps에서 1바이트 ≈ 약 0.087ms)
        elapsed += 1000;  // ~1ms per char
    }

    return result;  // 타임아웃 — 불완전한 줄 또는 빈 문자열 반환
}

/// 수신 버퍼를 비웁니다.
/// 새 쿼리를 보내기 전에 이전 응답의 잔여 데이터를 제거하는 데 사용합니다.
void SerialDriver::flush_input() {
    if (fd_ >= 0) {
        tcflush(fd_, TCIFLUSH);  // TCIFLUSH = 입력 버퍼만 비움
    }
}

}  // namespace ugv
