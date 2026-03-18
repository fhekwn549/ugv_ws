/// @file roarm_driver_node.cpp
/// @brief RoArm-M2 로봇 팔 ROS2 컨트롤러 노드 (C++)
///
/// ## 이 노드의 역할
/// RoArm-M2 로봇 팔을 ROS2 토픽으로 제어할 수 있게 해주는 노드입니다.
/// Python의 roarm_driver.py를 1:1 대체하며, 동일한 토픽/파라미터를 사용합니다.
///
/// ## 아키텍처
/// ```
/// [teleop_all.py]                    [이 노드 (C++)]               [ESP32]
///       |                                   |                          |
///       |-- /arm_controller/joint_trajectory -->|                      |
///       |                                   |-- T:102 (JSON) -------->|
///       |-- /roarm/gripper_cmd ------------>|                          |
///       |                                   |-- T:106 (JSON) -------->|
///       |                                   |                          |
///       |                                   |<-- T:105 응답 (5Hz) ----|
///       |<-- /joint_states -----------------|                          |
/// ```
///
/// ## 구독 토픽
/// - /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory)
///   → 팔 관절 목표 각도. teleop_all.py 또는 MoveIt에서 발행
/// - /roarm/gripper_cmd (std_msgs/Float64)
///   → 그리퍼 목표 위치 (rad)
///
/// ## 발행 토픽
/// - /joint_states (sensor_msgs/JointState)
///   → 팔 관절 + 바퀴 관절 현재 위치. robot_state_publisher가 TF로 변환
///
/// ## 파라미터
/// - serial_port: ESP32 시리얼 포트 (기본값: /dev/ttyUSB0)
/// - baud_rate: 보드레이트 (기본값: 115200)
/// - feedback_rate: 피드백 주기 (Hz, 기본값: 5.0)
///
/// ## Python(roarm_driver.py) 대비 C++ 개선점
/// 1. 시리얼 I/O에서 GIL 경합 없음 → 예측 가능한 타이밍
/// 2. snprintf 기반 JSON 생성 → json.dumps() 대비 낮은 오버헤드
/// 3. std::mutex → threading.Lock 대비 진정한 멀티스레딩 지원
/// 4. 가비지 컬렉션 없음 → 일정한 반응 시간

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "ugv_cpp_nodes/roarm_serial_driver.hpp"

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>

namespace {

/// URDF 관절 이름 → ESP32 T:102 명령 필드 이름 매핑.
///
/// URDF에서 관절 이름은 "arm_base_link_to_arm_link1" 형식이고,
/// ESP32는 "base", "shoulder" 등 짧은 이름을 사용합니다.
///
/// 예: "arm_base_link_to_arm_link1" → "base" (T:102의 base 필드)
const std::map<std::string, std::string> JOINT_MAP = {
    {"arm_base_link_to_arm_link1", "base"},         // 좌우 회전 관절
    {"arm_link1_to_arm_link2", "shoulder"},          // 어깨 관절 (위아래 1)
    {"arm_link2_to_arm_link3", "elbow"},             // 팔꿈치 관절 (위아래 2)
    {"arm_link3_to_arm_gripper_link", "hand"},       // 손목/그리퍼 관절
};

/// ESP32 T:105 응답 필드 → URDF 관절 이름 매핑.
///
/// T:105 응답에서 "b", "s", "e", "t" 등 1글자 필드를
/// 풀 URDF 관절 이름으로 변환합니다.
const std::map<std::string, std::string> FEEDBACK_MAP = {
    {"b", "arm_base_link_to_arm_link1"},     // base
    {"s", "arm_link1_to_arm_link2"},         // shoulder
    {"e", "arm_link2_to_arm_link3"},         // elbow
    {"t", "arm_link3_to_arm_gripper_link"},  // hand (gripper)
};

/// 바퀴 관절 이름 목록.
///
/// UGV는 바퀴 인코더가 없어서 실제 위치를 알 수 없지만,
/// robot_state_publisher가 완전한 TF 트리를 만들려면
/// 모든 관절의 위치가 필요합니다.
/// 따라서 바퀴 관절은 항상 0.0으로 발행합니다.
const std::vector<std::string> WHEEL_JOINTS = {
    "left_up_wheel_link_joint",
    "left_down_wheel_link_joint",
    "right_up_wheel_link_joint",
    "right_down_wheel_link_joint",
};

}  // namespace

/// RoArm-M2 ROS2 컨트롤러 노드.
///
/// 이 노드는 RoArmSerialDriver(순수 C++)를 사용하여
/// ESP32와 시리얼 통신을 수행합니다.
/// ROS2 토픽을 통해 관절 명령을 받고, 피드백을 발행합니다.
class RoArmDriverNode : public rclcpp::Node {
public:
    RoArmDriverNode() : Node("roarm_driver") {
        // === ROS2 파라미터 선언 ===
        // launch 파일이나 명령줄에서 오버라이드 가능
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("feedback_rate", 5.0);  // Hz

        auto port = this->get_parameter("serial_port").as_string();
        auto baud = this->get_parameter("baud_rate").as_int();
        auto feedback_rate = this->get_parameter("feedback_rate").as_double();

        // === 시리얼 드라이버 초기화 ===
        driver_ = std::make_unique<ugv::RoArmSerialDriver>(port, baud);

        if (!driver_->connect()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            throw std::runtime_error("Serial port open failed");
        }
        RCLCPP_INFO(this->get_logger(), "Serial opened: %s @ %ld", port.c_str(), baud);

        // === 구독자(Subscriber) 설정 ===

        // 팔 관절 궤적 명령 (teleop_all.py 또는 MoveIt에서 발행)
        traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10,
            std::bind(&RoArmDriverNode::trajectory_callback, this, std::placeholders::_1));

        // 그리퍼 명령 (teleop_all.py에서 발행)
        gripper_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/roarm/gripper_cmd", 10,
            std::bind(&RoArmDriverNode::gripper_callback, this, std::placeholders::_1));

        // === 발행자(Publisher) 설정 ===

        // 관절 상태 발행 → robot_state_publisher가 TF 트리 생성에 사용
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // === 주기적 피드백 타이머 ===
        // feedback_rate Hz로 ESP32에 T:105를 보내고 /joint_states를 발행
        double period = 1.0 / feedback_rate;
        feedback_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&RoArmDriverNode::feedback_callback, this));

        // === 초기 상태 설정 ===

        // 마지막 그리퍼 ESP32 값 (홈 포즈의 hand 값)
        // T:102 명령에 항상 hand 값을 포함해야 토크가 유지됩니다
        last_gripper_esp32_ = 3.0;

        // 토크 활성화 (모터 전원 켜기)
        driver_->set_torque(true);

        // 홈 포즈로 이동 (팔 접기, 그리퍼 닫기)
        // base=0, shoulder=-1.6, elbow=3.2, hand=3 (모두 rad)
        driver_->move_joints(0.0, -1.6, 3.2, 3.0, 0, 10);
        RCLCPP_INFO(this->get_logger(), "RoArm-M2 home pose set (0, -1.6, 3.2, 3)");

        RCLCPP_INFO(this->get_logger(), "RoArm-M2 driver ready (C++)");
    }

    ~RoArmDriverNode() override {
        if (driver_) {
            driver_->disconnect();
        }
    }

private:
    /// /arm_controller/joint_trajectory 콜백.
    ///
    /// JointTrajectory 메시지의 첫 번째 포인트에서 관절 이름과 목표 위치를 추출하여
    /// ESP32에 T:102 명령을 보냅니다.
    ///
    /// 중요: hand(그리퍼) 값이 메시지에 없으면, 마지막으로 알고 있는 값을 사용합니다.
    /// 이렇게 하지 않으면 ESP32가 hand 관절의 토크를 해제하여 그리퍼가 풀릴 수 있습니다.
    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        if (msg->points.empty()) return;

        const auto& point = msg->points[0];  // 첫 번째 (유일한) 궤적 포인트

        // 각 관절의 목표 각도 (기본값은 0 또는 마지막 그리퍼 값)
        double base = 0.0, shoulder = 0.0, elbow = 0.0, hand = last_gripper_esp32_;
        bool has_base = false, has_shoulder = false, has_elbow = false, has_hand = false;

        // JointTrajectory에서 관절 이름과 위치를 매핑
        for (size_t i = 0; i < msg->joint_names.size() && i < point.positions.size(); ++i) {
            const auto& name = msg->joint_names[i];
            double urdf_angle = point.positions[i];
            // URDF ↔ ESP32 각도 변환 (현재는 1:1 동일, 스케일=1, 오프셋=0)
            double esp32_angle = urdf_angle;

            auto it = JOINT_MAP.find(name);
            if (it != JOINT_MAP.end()) {
                const auto& field = it->second;
                if (field == "base")     { base = esp32_angle; has_base = true; }
                if (field == "shoulder") { shoulder = esp32_angle; has_shoulder = true; }
                if (field == "elbow")    { elbow = esp32_angle; has_elbow = true; }
                if (field == "hand")     { hand = esp32_angle; has_hand = true; }
            }
        }

        // hand 값이 없으면 마지막 알려진 값 사용 (토크 유지)
        if (!has_hand) {
            hand = last_gripper_esp32_;
        }

        driver_->move_joints(
            has_base ? base : 0.0,
            has_shoulder ? shoulder : 0.0,
            has_elbow ? elbow : 0.0,
            hand,
            0, 10);  // spd=0(기본속도), acc=10
    }

    /// /roarm/gripper_cmd 콜백.
    ///
    /// 그리퍼 전용 T:106 명령을 ESP32에 보냅니다.
    /// 마지막 그리퍼 값을 저장하여 T:102 명령에서도 사용합니다.
    void gripper_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double esp32_val = msg->data;  // URDF→ESP32 변환 (현재 1:1)
        last_gripper_esp32_ = esp32_val;
        driver_->set_gripper(esp32_val, 0, 0);
    }

    /// 주기적 피드백 콜백 (기본 5Hz).
    ///
    /// ESP32에 T:105를 보내 현재 관절 위치를 받아오고,
    /// /joint_states 토픽으로 발행합니다.
    ///
    /// /joint_states는 robot_state_publisher가 구독하여
    /// 관절 위치를 TF 트리로 변환합니다 (RViz에서 로봇 모양 표시).
    ///
    /// 바퀴 관절 (4개)은 인코더가 없으므로 항상 0.0으로 발행합니다.
    /// 이것은 TF 트리를 완성하기 위한 것이며, 실제 바퀴 위치와 무관합니다.
    void feedback_callback() {
        auto fb = driver_->get_feedback();
        if (!fb.has_value()) return;  // 타임아웃 또는 파싱 실패

        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = this->get_clock()->now();

        const auto& f = fb.value();

        // 피드백 데이터를 JointState 메시지에 추가
        auto add_joint = [&](const std::string& field, double esp32_angle) {
            auto it = FEEDBACK_MAP.find(field);
            if (it != FEEDBACK_MAP.end()) {
                double urdf_angle = esp32_angle;  // ESP32→URDF 변환 (현재 1:1)
                js.name.push_back(it->second);
                js.position.push_back(urdf_angle);
            }
        };

        add_joint("b", f.base);
        add_joint("s", f.shoulder);
        add_joint("e", f.elbow);
        add_joint("t", f.hand);

        // 하드웨어 피드백에서 실제 그리퍼 위치 업데이트
        last_gripper_esp32_ = f.hand;

        // 바퀴 관절 (TF 트리 완성을 위해 0.0으로 발행)
        for (const auto& wheel : WHEEL_JOINTS) {
            js.name.push_back(wheel);
            js.position.push_back(0.0);
        }

        if (!js.name.empty()) {
            joint_state_pub_->publish(js);
        }
    }

    // === 멤버 변수 ===
    std::unique_ptr<ugv::RoArmSerialDriver> driver_;  ///< ESP32 시리얼 드라이버

    // ROS2 구독자/발행자/타이머
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    /// 마지막으로 알려진 그리퍼(hand) ESP32 값.
    /// T:102 명령에 항상 포함하여 토크를 유지합니다.
    double last_gripper_esp32_ = 3.0;
};

/// 노드 진입점.
/// rclcpp::init()으로 ROS2를 초기화하고,
/// rclcpp::spin()으로 콜백 루프를 시작합니다.
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoArmDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
