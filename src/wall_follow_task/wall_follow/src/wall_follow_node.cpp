#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        pub_wall = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        sub_wall = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, _1));
        pub_marker = this->create_publisher<visualization_msgs::msg::Marker>("/wall_follow_marker", 10);
    }

private:
    // PID 파라미터
    double kp = 3.5; 
    double kd = 0.5;
    double ki = 0.0;

    double angle_min;
    double angle_increment;
    int ranges_size;

    double prev_error = 0.0;
    double integral = 0.0;

    // 주행 설정
    double look_ahead_dist = 0.7; // 차가 미래에 도달할 위치 예측 거리 (너무 길면 유턴 시 일찍 꺾음)
    double desired_dist = 0.8;    // 왼쪽 벽으로부터 유지할 거리 [m]

    // ROS subscribers, publishers 선언
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_wall;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_wall;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;

    // RViz에 목표 지점을 시각화하는 함수
    void publish_lookahead_marker(double theta, double future_dist) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "ego_racecar/laser"; 
        marker.header.stamp = this->now();
        marker.ns = "lookahead";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 자동차 기준 목표 좌표 계산
        // x: 전방 방향 거리, y: 벽과의 거리(좌측)
        marker.pose.position.x = look_ahead_dist * cos(theta);
        marker.pose.position.y = future_dist; 
        marker.pose.position.z = 0.2;

        marker.scale.x = 0.15; marker.scale.y = 0.15; marker.scale.z = 0.15;
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;

        pub_marker->publish(marker);
    }

    double get_range(const std::vector<float>& range_data, double angle)
    {
        // 유효하지 않은 값 보정
        int index = static_cast<int>((angle - angle_min) / angle_increment);
        if (index < 0 || index >= ranges_size) return 10.0;

        double range = range_data[index];
        if (std::isnan(range) || std::isinf(range) || range < 0.1) return 10.0;
        return range;
    }

    double get_error(const std::vector<float>& range_data, double dist)
    {
        // 왼쪽 벽을 따라가기 위해 두 광선을 쏨 (0도: 정면, 90도: 왼쪽)
        double b_angle = M_PI / 2.0;            
        double a_angle = M_PI / 3.0;   
        double alpha = b_angle - a_angle;

        double b = get_range(range_data, b_angle);
        double a = get_range(range_data, a_angle);

        // 유턴/교차로 예외 처리
        // 왼쪽 벽이 너무 멀어지면(유턴 상황), 차가 길을 잃지 않게 강제로 왼쪽 에러를 발생
        if (b > 1.8) { 
            // 벽이 사라졌으니 더 적극적으로 왼쪽으로 꺾으라고 에러값을 크게 줍니다.
            return -1.0; // 음수 에러 -> PID 결과로 양수(좌측) 조향 발생
        }
        // 1. 벽과의 입사각 계산
        double theta = atan((a * cos(alpha) - b) / (a * sin(alpha)));
        
        // 2. 현재 실제 거리 계산
        double current_dist = b * cos(theta);

        // 3. 미래 예측 거리 계산
        double future_dist = current_dist + look_ahead_dist * sin(theta);

        // 에러 계산 직후 시각화 호출
        publish_lookahead_marker(theta, future_dist);

        // 유턴/교차로에서 벽이 갑자기 멀어질 때 에러 폭주 방지
        if (b > 1.5) {
            return prev_error * 0.5; // 에러를 감쇠시켜 부드럽게 주행
        }

        // 에러 = 목표 거리 - 미래 예측 거리
        return dist - future_dist;
    }

    void pid_control(double error, double velocity)
    {
        // PID 제어
        double derivative = error - prev_error;
        integral += error;

        // 조향각 계산 (에러 부호 주의: 왼쪽 벽 추종 시 에러가 +면 벽에 너무 가까운 것이므로 오른쪽(-) 조향)
        double steering_angle = -(kp * error + ki * integral + kd * derivative);
        
        // 조향각 제한
        double max_angle = 0.60; 
        steering_angle = std::clamp(steering_angle, -max_angle, max_angle);

        // 속도 조절 : 조향이 클수록 더 확실하게 감속하여 안정적
        if (std::abs(steering_angle) > 0.2) velocity = 1.5;
        else if (std::abs(steering_angle) > 0.1) velocity = 3.1;
        else velocity = 4.0; 
        
        // drive message 생성
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity;

        pub_wall->publish(drive_msg);
        prev_error = error;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        angle_min = scan_msg->angle_min;
        angle_increment = scan_msg->angle_increment;
        ranges_size = scan_msg->ranges.size();

        // 정면 감지 범위 설정: 오른쪽 1도(-1도) ~ 왼쪽 1도(+1도)
        double angle_from = -1.0 * M_PI / 180.0; 
        double angle_to   =  1.0 * M_PI / 180.0;

        // 각도에 대응하는 배열 인덱스 계산
        int start_idx = static_cast<int>((angle_from - angle_min) / angle_increment);
        int end_idx   = static_cast<int>((angle_to - angle_min) / angle_increment);
        
        // 인덱스 안전 범위 제한 (Clamping)
        start_idx = std::max(0, std::min(ranges_size - 1, start_idx));
        end_idx   = std::max(0, std::min(ranges_size - 1, end_idx));

        // 해당 범위 내에서 최소 거리 찾기
        double front_dist = 10.0;
        for (int i = start_idx; i <= end_idx; ++i) {
            float r = scan_msg->ranges[i];
            // 0.45m 이하는 자기 차체 노이즈이므로 무시 (시작 지점 문제 해결)
            if (r > 0.45 && !std::isnan(r) && !std::isinf(r)) {
                front_dist = std::min(front_dist, (double)r);
            }
        }

        double error = get_error(scan_msg->ranges, desired_dist);

        // 정면 충돌 방지 및 우회전 판단
        // 정면 벽이 1.0m 이내로 들어오면 우회전 준비, 0.8m 이내면 강제 우회전
        if (front_dist < 1.0) { 
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->now();
        
            if (front_dist < 0.8) {
            // 0.8m 이내: 더 긴급한 상황이므로 강하게 꺾고 더 감속
            drive_msg.drive.steering_angle = -0.7; // 더 큰 조향각 (강제 우회전)
            drive_msg.drive.speed = 0.9;           // 충돌 직전이므로 대폭 감속
            } else {
                // 0.8m ~ 1.0m 사이: 우회전 준비 및 완만한 회전
                drive_msg.drive.steering_angle = -0.4; 
                drive_msg.drive.speed = 1.3; 
            }
        
            pub_wall->publish(drive_msg);
        
            // PID의 이전 에러를 현재 에러로 갱신하여 복귀 시 튀는 것 방지
            prev_error = error; 
        }
        else {
            pid_control(error, 1.5); // 평소에는 기존 PID 제어
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}