#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "nurirobot_msgs/msg/nurirobot_pos.hpp"
#include "nurirobot_msgs/msg/nurirobot_speed.hpp"
#include "nurirobot_msgs/msg/hc_control.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "config.hpp"
#include "protocol.hpp"

#include <functional>
#include <memory>

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h> // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
#include <asm/ioctls.h>
#include <asm/termbits.h>

#include <iomanip>
#include <sstream>
#include <cmath>
#include <cstring>
#include <cstdint>

using namespace std::chrono_literals;

class Nurirobot : public rclcpp::Node {
public:
    Nurirobot(); // 생성
    ~Nurirobot(); // 소멸
    
    void read(); // 컨트롤로 부터 데이터 읽기
    void cbFeedback();
    // void write(); // 컨트롤러로 데이터 전송

    // void feedbackCall(); // 피드백
    
 private:
    void protocol_recv (uint8_t c); // Function to recontruct serial packets coming from BLDC controller
    void feedbackCall(uint8_t id);
    void feedbackHCCall();

    /// @brief 스마트휠체어 제어 시작
    void commandRemoteStart();
    /// @brief 스마트휠체어 제어 종료
    void commandRemoteStop();

    /// @brief 위치 publish 포인트 
    rclcpp::Publisher<nurirobot_msgs::msg::NurirobotPos>::SharedPtr pos_pub_;
    /// @brief 속도 Publish 포인트
    rclcpp::Publisher<nurirobot_msgs::msg::NurirobotSpeed>::SharedPtr speed_pub_;

    rclcpp::Publisher<nurirobot_msgs::msg::NurirobotSpeed>::SharedPtr hc_speed_pub_;

    rclcpp::Publisher<nurirobot_msgs::msg::HCControl>::SharedPtr hc_ctrl_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr hc_joy_pub_;

    rclcpp::Publisher<nurirobot_msgs::msg::NurirobotPos>::SharedPtr left_pos_pub_;
    rclcpp::Publisher<nurirobot_msgs::msg::NurirobotPos>::SharedPtr right_pos_pub_;

    /// @brief 제어 신호 구독 
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr remote_sub_;
    /// @brief 속도 제어 구독
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmdTwist_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr rawdata_sub_;

    /// @brief 원격제어 여부
    /// @param msg 
    void setRemote_callback(std_msgs::msg::Bool::UniquePtr msg);

    /// @brief 속도 제어 메시지 수신
    /// @param msg 
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ROS2 Suscriber
    // TODO : this msg sends setpoints for 4 wheels, but we can control only 2 wheels
    // rclcpp::Subscription<wheel_msgs::msg::WheelSpeeds>::SharedPtr speeds_sub_;
    // Callback for subscriber
    // void setpoint_callback(wheel_msgs::msg::WheelSpeeds::UniquePtr msg); // Be careful with UniquePtr, in speeds_sub_ definition we remove it... TODO :  understand what UniquePtr is doing!

    // Hoverboard protocol variables
    int port_fd;
    unsigned int msg_len = 0;
    uint8_t prev_byte = 0; // uint8_t is nice to store bytes
    uint16_t start_frame = 0;
    uint8_t* p;
    FeedbackResponse msg;

    // 최초 외부 명령 모드 아님
    bool remote = true;

    // 최대 속도
    float max_lin_vel_x = MAX_LIN_VEL_X;
    // 최대 회전 속도
    float max_ang_vel_z = MAX_ANG_VEL_Z;
    // 휠간 거리
    float wheel_separation = WHEEL_SEPARATION;
    // 바퀴반지름
    float wheel_radius = WHEEL_RADIUS;

    void write_base_velocity(float x, float z);
    float convertRange(float input);
    const float DEADZONE = 50.0f;
    void byteMultiArrayCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);

    void timeCallback();
};