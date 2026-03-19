/// @file ugv_driver_node.cpp
/// @brief UGV 통합 드라이버 노드 (C++) — 제어 + 센서 피드백
///
/// ## 이 노드의 역할
/// 1. ROS2 토픽(/cmd_vel 등)을 받아 ESP32에 모터/서보/LED 명령 전송
/// 2. ESP32의 센서 피드백(T:1001)을 읽어 IMU/인코더/전압 토픽 발행
///
/// Python의 ugv_driver.py + ugv_bringup.py를 하나의 C++ 노드로 통합합니다.
/// 시리얼 포트를 단일 프로세스가 전담하여 데이터 깨짐을 방지합니다.
///
/// ## 아키텍처
/// ```
/// [ESP32 바퀴 보드] <── /dev/ttyAMA0 ──> [이 노드 (C++)]
///       │                                       │
///       │ ←── T:13/132/134 (명령) ──────────────┤← /cmd_vel, /ugv/joint_states, /ugv/led_ctrl
///       │                                       │
///       │ ──→ T:1001 (센서 피드백) ─────────────┤→ /imu/data, /odom/odom_raw, /voltage
/// ```

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "ugv_cpp_nodes/ugv_serial_driver.hpp"

#include <memory>
#include <string>
#include <cmath>
#include <fstream>

namespace {

bool is_jetson() {
    std::ifstream f("/ugv_jetson");
    return f.good();
}

/// Roll/Pitch/Yaw (radians) → Quaternion (x, y, z, w) 변환.
void rpy_to_quaternion(double roll, double pitch, double yaw,
                       double& qx, double& qy, double& qz, double& qw) {
    double cr = std::cos(roll / 2.0);
    double sr = std::sin(roll / 2.0);
    double cp = std::cos(pitch / 2.0);
    double sp = std::sin(pitch / 2.0);
    double cy = std::cos(yaw / 2.0);
    double sy = std::sin(yaw / 2.0);

    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
    qw = cr * cp * cy + sr * sp * sy;
}

}  // namespace

/// UGV 통합 드라이버 노드 — 제어 명령 수신 + 센서 피드백 발행.
class UgvDriverNode : public rclcpp::Node {
public:
    UgvDriverNode() : Node("ugv_driver") {
        // === 파라미터 ===
        std::string default_port = is_jetson() ? "/dev/ttyTHS1" : "/dev/ttyAMA0";
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

        // === 제어 구독자 ===
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&UgvDriverNode::cmd_vel_callback, this, std::placeholders::_1));

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "ugv/joint_states", 10,
            std::bind(&UgvDriverNode::joint_states_callback, this, std::placeholders::_1));

        led_ctrl_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "ugv/led_ctrl", 10,
            std::bind(&UgvDriverNode::led_ctrl_callback, this, std::placeholders::_1));

        // === 센서 퍼블리셔 ===
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
        odom_raw_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("odom/odom_raw", 100);
        voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("voltage", 50);

        // === 센서 피드백 활성화 ===
        driver_->enable_feedback();
        RCLCPP_INFO(this->get_logger(), "ESP32 feedback enabled");

        // 피드백 폴링 타이머 (50ms = 20Hz)
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&UgvDriverNode::feedback_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "UGV driver ready (C++), angular_scale=%.2f", angular_scale_);
    }

    ~UgvDriverNode() override {
        if (driver_) {
            driver_->set_velocity(0.0, 0.0);
            driver_->disconnect();
        }
    }

private:
    // === 제어 콜백 ===

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear = msg->linear.x;
        double angular = msg->angular.z * angular_scale_;

        if (linear == 0.0) {
            if (angular > 0.0 && angular < 0.2) angular = 0.2;
            else if (angular < 0.0 && angular > -0.2) angular = -0.2;
        }

        driver_->set_velocity(linear, angular);
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        int x_idx = -1, y_idx = -1;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "pt_base_link_to_pt_link1") x_idx = static_cast<int>(i);
            if (msg->name[i] == "pt_link1_to_pt_link2") y_idx = static_cast<int>(i);
        }

        if (x_idx < 0 || y_idx < 0) return;
        if (static_cast<size_t>(x_idx) >= msg->position.size() ||
            static_cast<size_t>(y_idx) >= msg->position.size()) return;

        double x_deg = msg->position[x_idx] * 180.0 / M_PI;
        double y_deg = msg->position[y_idx] * 180.0 / M_PI;

        driver_->set_pan_tilt(x_deg, y_deg, 600, 600);
    }

    void led_ctrl_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        driver_->set_led(msg->data[0], msg->data[1]);
    }

    // === 센서 피드백 ===

    void feedback_timer_callback() {
        auto fb = driver_->get_feedback();
        if (fb) {
            last_good_feedback_ = *fb;  // 정상 데이터 캐시
            has_feedback_ = true;
        }

        // 마지막 정상 값으로 항상 발행 (Python ugv_bringup과 동일한 동작)
        if (!has_feedback_) return;

        publish_imu(last_good_feedback_);
        publish_odom_raw(last_good_feedback_);
        publish_voltage(last_good_feedback_);
    }

    void publish_imu(const ugv::UgvFeedback& fb) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "base_imu_link";

        double roll = fb.r * M_PI / 180.0;
        double pitch = fb.p * M_PI / 180.0;
        double yaw = fb.y * M_PI / 180.0;

        rpy_to_quaternion(roll, pitch, yaw,
                          msg.orientation.x, msg.orientation.y,
                          msg.orientation.z, msg.orientation.w);

        imu_pub_->publish(msg);
    }

    void publish_odom_raw(const ugv::UgvFeedback& fb) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {static_cast<float>(fb.L) / 100.0f,
                    static_cast<float>(fb.R) / 100.0f};
        odom_raw_pub_->publish(msg);
    }

    void publish_voltage(const ugv::UgvFeedback& fb) {
        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(fb.v);
        voltage_pub_->publish(msg);

        // 저전압 경고
        if (fb.v > 0.1 && fb.v < 9.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                  "Low battery voltage: %.2f V", fb.v);
        }
    }

    // === 멤버 변수 ===
    std::unique_ptr<ugv::UgvSerialDriver> driver_;

    // 제어 구독자
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr led_ctrl_sub_;

    // 센서 퍼블리셔
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr odom_raw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;

    // 피드백 타이머
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    double angular_scale_ = 2.5;

    // 마지막 정상 피드백 캐시 (깨진 라인 무시, 안정적 발행)
    ugv::UgvFeedback last_good_feedback_;
    bool has_feedback_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UgvDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
