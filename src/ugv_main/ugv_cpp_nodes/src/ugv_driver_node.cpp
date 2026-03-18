/// @file ugv_driver_node.cpp
/// @brief UGV 바퀴/팬틸트/LED ROS2 컨트롤러 노드 (C++)
///
/// ## 이 노드의 역할
/// ROS2의 /cmd_vel 토픽을 받아 UGV의 ESP32에 모터 명령을 보내는 노드입니다.
/// Python의 ugv_driver.py를 1:1 대체하며, 동일한 토픽/파라미터를 사용합니다.
///
/// ## 아키텍처
/// ```
/// [Nav2 / teleop]           [이 노드 (C++)]              [ESP32]
///       |                          |                         |
///       |-- /cmd_vel ------------->|                         |
///       |                          |-- T:13 (JSON) -------->|→ 바퀴 모터
///       |                          |                         |
/// [teleop_all.py]                  |                         |
///       |-- /ugv/joint_states ---->|                         |
///       |                          |-- T:134 (JSON) ------->|→ 팬틸트 서보
///       |                          |                         |
///       |-- /ugv/led_ctrl -------->|                         |
///       |                          |-- T:132 (JSON) ------->|→ LED
///       |                          |                         |
/// [ugv_bringup]                    |                         |
///       |-- /voltage ------------->|                         |
///       |                          |-- (경고 로그만 출력)    |
/// ```
///
/// ## 구독 토픽
/// - cmd_vel (geometry_msgs/Twist): 이동 명령 (선속도, 각속도)
/// - ugv/joint_states (sensor_msgs/JointState): 팬틸트 카메라 관절 명령
/// - ugv/led_ctrl (std_msgs/Float32MultiArray): LED 제어 [IO4, IO5]
/// - voltage (std_msgs/Float32): 배터리 전압 (저전압 경고용)
///
/// ## 파라미터
/// - serial_port: 자동 감지 (RPi: /dev/ttyAMA0, Jetson: /dev/ttyTHS1)
/// - baud_rate: 115200
/// - angular_scale: 2.5 (스키드-스티어 마찰 보정 계수)
///
/// ## angular_scale이란?
/// 스키드-스티어(skid-steer) 방식의 UGV는 바퀴를 미끄러뜨려서 회전합니다.
/// 이론적 각속도와 실제 각속도 사이에 차이가 발생하므로,
/// 보정 계수(기본값 2.5)를 곱하여 실제 회전을 목표에 맞춥니다.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "ugv_cpp_nodes/ugv_serial_driver.hpp"

#include <memory>
#include <string>
#include <cmath>
#include <fstream>

namespace {

/// Jetson 플랫폼 감지.
///
/// /ugv_jetson 마커 파일이 존재하면 Jetson으로 판단합니다.
/// RPi와 Jetson은 시리얼 포트가 다릅니다:
/// - RPi:    /dev/ttyAMA0 (내장 UART)
/// - Jetson: /dev/ttyTHS1 (Tegra 고속 UART)
bool is_jetson() {
    std::ifstream f("/ugv_jetson");
    return f.good();
}

}  // namespace

/// UGV 바퀴/팬틸트/LED ROS2 컨트롤러 노드.
///
/// cmd_vel(이동 명령)을 받아 ESP32에 모터 속도를 전달하고,
/// 팬틸트 카메라와 LED도 제어합니다.
class UgvDriverNode : public rclcpp::Node {
public:
    UgvDriverNode() : Node("ugv_driver") {
        // === 시리얼 포트 자동 감지 ===
        std::string default_port = is_jetson() ? "/dev/ttyTHS1" : "/dev/ttyAMA0";

        // === ROS2 파라미터 선언 ===
        this->declare_parameter("serial_port", default_port);
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("angular_scale", 2.5);

        auto port = this->get_parameter("serial_port").as_string();
        auto baud = this->get_parameter("baud_rate").as_int();
        angular_scale_ = this->get_parameter("angular_scale").as_double();

        // === 시리얼 드라이버 초기화 ===
        driver_ = std::make_unique<ugv::UgvSerialDriver>(port, baud);

        if (!driver_->connect()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            throw std::runtime_error("Serial port open failed");
        }
        RCLCPP_INFO(this->get_logger(), "Serial opened: %s @ %ld", port.c_str(), baud);

        // === 구독자(Subscriber) 설정 ===

        // /cmd_vel: Nav2 또는 텔레옵에서 발행하는 이동 명령
        // 선속도(linear.x)와 각속도(angular.z)를 포함
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&UgvDriverNode::cmd_vel_callback, this, std::placeholders::_1));

        // /ugv/joint_states: 팬틸트 카메라 관절 위치 (rad)
        // teleop_all.py에서 키보드로 팬틸트를 조작할 때 발행
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "ugv/joint_states", 10,
            std::bind(&UgvDriverNode::joint_states_callback, this, std::placeholders::_1));

        // /ugv/led_ctrl: LED 제어 배열 [IO4, IO5]
        led_ctrl_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "ugv/led_ctrl", 10,
            std::bind(&UgvDriverNode::led_ctrl_callback, this, std::placeholders::_1));

        // /voltage: 배터리 전압 (ugv_bringup 노드에서 발행)
        // 저전압 시 경고 로그만 출력 (Python 버전의 aplay 비프음 대신)
        voltage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "voltage", 10,
            std::bind(&UgvDriverNode::voltage_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "UGV driver ready (C++), angular_scale=%.2f", angular_scale_);
    }

    ~UgvDriverNode() override {
        if (driver_) {
            // 노드 종료 시 바퀴를 정지시킵니다 (안전)
            driver_->set_velocity(0.0, 0.0);
            driver_->disconnect();
        }
    }

private:
    /// /cmd_vel 콜백 — 이동 명령을 ESP32에 전달합니다.
    ///
    /// angular_scale을 적용하여 스키드-스티어 마찰을 보정합니다.
    /// 또한, 정지 상태에서 극소 각속도는 최소 임계값(0.2)으로 올립니다.
    /// 이유: 너무 작은 각속도로는 마찰 때문에 바퀴가 돌지 않기 때문입니다.
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear = msg->linear.x;
        double angular = msg->angular.z * angular_scale_;

        // 정지 상태에서 최소 각속도 임계값 적용
        // (선속도=0일 때 제자리 회전에 필요한 최소 토크)
        if (linear == 0.0) {
            if (angular > 0.0 && angular < 0.2) angular = 0.2;
            else if (angular < 0.0 && angular > -0.2) angular = -0.2;
        }

        driver_->set_velocity(linear, angular);
    }

    /// /ugv/joint_states 콜백 — 팬틸트 카메라 제어.
    ///
    /// ROS2 관절 위치는 라디안(rad)이지만,
    /// ESP32 팬틸트 서보는 도(degree) 단위를 사용합니다.
    /// 여기서 rad → deg 변환을 수행합니다.
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 팬틸트 관절 인덱스 찾기
        int x_idx = -1, y_idx = -1;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "pt_base_link_to_pt_link1") x_idx = static_cast<int>(i);  // 수평
            if (msg->name[i] == "pt_link1_to_pt_link2") y_idx = static_cast<int>(i);       // 수직
        }

        if (x_idx < 0 || y_idx < 0) return;  // 팬틸트 관절이 없는 메시지
        if (static_cast<size_t>(x_idx) >= msg->position.size() ||
            static_cast<size_t>(y_idx) >= msg->position.size()) return;

        // 라디안 → 도 변환
        double x_deg = msg->position[x_idx] * 180.0 / M_PI;
        double y_deg = msg->position[y_idx] * 180.0 / M_PI;

        driver_->set_pan_tilt(x_deg, y_deg, 600, 600);
    }

    /// /ugv/led_ctrl 콜백 — LED 제어.
    /// Float32MultiArray에서 [IO4, IO5] 값을 추출합니다.
    void led_ctrl_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;  // 데이터 부족
        driver_->set_led(msg->data[0], msg->data[1]);
    }

    /// /voltage 콜백 — 배터리 저전압 경고.
    ///
    /// Python 버전에서는 aplay로 wav 파일을 재생했지만,
    /// C++ 버전에서는 ROS2 로그 경고로 대체합니다.
    /// RCLCPP_WARN_THROTTLE: 10초에 한 번만 경고 (로그 폭주 방지)
    void voltage_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        double voltage = msg->data;

        if (voltage > 0.1 && voltage < 9.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                  "Low battery voltage: %.2f V", voltage);
        }
    }

    // === 멤버 변수 ===
    std::unique_ptr<ugv::UgvSerialDriver> driver_;  ///< ESP32 시리얼 드라이버

    // ROS2 구독자
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr led_ctrl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_sub_;

    double angular_scale_ = 2.5;  ///< 스키드-스티어 마찰 보정 계수
};

/// 노드 진입점.
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UgvDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
