#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class ReactiveFollowGap : public rclcpp::Node {

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // /drive 토픽으로 자동차를 움직이는 명령을 보낼 Publisher 생성
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        // /scan 토픽으로 들어오는 라이다 데이터를 받아올 Subscriber 생성
        // 데이터가 들어올 때마다 lidar_callback 함수 실행
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;


    void preprocess_lidar(std::vector<float>& ranges)
    {   
        // NaN, inf 값 제거, max range 제한
        const float MAX_RANGE = 10.0; // 인식할 최대 거리 (10m)

        for (size_t i = 0; i < ranges.size(); i++) {
            // 값이 없거나(NaN), 무한대(inf), 혹은 너무 먼 경우 MAX_RANGE로 통일
            if (std::isnan(ranges[i]) || std::isinf(ranges[i]) || ranges[i] > MAX_RANGE) {
                ranges[i] = MAX_RANGE;
            }
        }
        
        // 주변 값 5개의 평균으로 부드럽게 만들어 중간의 작은 튀는 값 제거
        std::vector<float> smoothed = ranges;
        int window_size = 5;
        int half = window_size / 2;

        for (size_t i = half; i < (int)ranges.size() - half; i++) {
            float sum = 0.0;
            for (int j = -half; j <= half; j++) {
                sum += ranges[i + j];
            }
            smoothed[i] = sum / window_size;
        }
        ranges = smoothed;
        
        return;
    }

    void find_max_gap(const std::vector<float>& ranges, int start_i, int end_i, int& gap_start, int& gap_end)
    {
        int max_len = 0;        // 가장 긴 구간의 길이
        int current_start = -1; // 현재 검사 중인 구간의 시작 인덱스
        int current_len = 0;    // 현재 검사 중인 구간의 길이

        gap_start = start_i;
        gap_end = start_i;

        // start_i부터 end_i까지 루프를 돌며 가장 큰 gap 탐색
        for (int i = start_i; i <= end_i; i++) {
            // 거리가 0.1m보다 크면 빈 공간으로 인식 (버블 처리된 곳은 0.0임)
            if (ranges[i] > 0.1) {
                if (current_len == 0) current_start = i;
                current_len++;
            }
            else {
                // 장애물을 만났으므로 현재까지의 빈 공간 길이를 체크
                if (current_len > max_len) {
                    max_len = current_len;
                    gap_start = current_start;
                    gap_end = i - 1;
                }
                current_len = 0;
            }
        }

        // 마지막까지 gap이 이어진 경우 처리
        if (current_len > max_len) {
            gap_start = current_start;
            gap_end = end_i;
        }
        
        return;
    }

    void find_best_point(const std::vector<float>& ranges, int gap_start, int gap_end, int& best_i)
    {   
        // Gap 안에서 가장 멀리 뚫려 있는 지점 찾기
        int furthest_i = gap_start;
        float max_dist = -1.0;
        for (int i = gap_start; i <= gap_end; i++) {
            if (ranges[i] > max_dist) {
                max_dist = ranges[i];
                furthest_i = i;
            }
        }
        // Gap의 정중앙 지점 계산
        int gap_center_i = (gap_start + gap_end) / 2;

        // 가장 먼 곳(60%)과 중앙(40%)을 섞어서 최종 목표 인덱스 결정
        // 이렇게 섞어야 벽에 너무 붙지 않고 안정적으로 주행
        best_i = (furthest_i * 0.6 + gap_center_i * 0.4);

        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // 원본 라이다 데이터 ranges 복사해서 가져오기 (scan_msg는 const라 수정 불가함. bubble 제거하려면 복사본 필요)
        std::vector<float> ranges = scan_msg->ranges;
        
        // 관심 영역(ROI) 설정 : 전방 약 160도
        int num_points = (int)ranges.size();
        int start_idx = num_points / 5;
        int end_idx = 4 * num_points / 5;
        
        // 데이터 전처리 (노이즈 제거)
        preprocess_lidar(ranges);

        // 안전 버블(Safety Bubble) 처리
        // 장애물 근처를 0으로 만들어 자동차가 아예 근처에 가지 못하게 함
        float bubble_radius = 0.25; // 자동차의 너비를 고려한 안전 반경 (25cm)
        float threshold = 1.5;      // 1.5m 이내 장애물만 버블 처리

        // 1.5m 이내의 모든 점을 검사하여 버블 생성
        std::vector<float> ranges_with_bubbles = ranges; 

        for (int i = start_idx; i <= end_idx; i++) {
            if (ranges[i] < threshold && ranges[i] > 0.0) {
                // 아크탄젠트를 이용해 장애물 거리에 따른 버블 인덱스 범위 계산
                int b_range = std::atan2(bubble_radius, ranges[i]) / scan_msg->angle_increment;
            
                // 주변 인덱스들을 0으로 만들어 '장애물이 있음'을 표시
                for (int j = i - b_range; j <= i + b_range; j++) {
                    if (j >= start_idx && j <= end_idx) {
                        ranges_with_bubbles[j] = 0.0; // 위험 구역은 0으로 덮어버림
                    }
                }
            }
        }
    
        ranges = ranges_with_bubbles; // 버블이 적용된 데이터를 이후 gap 탐색에 사용

        // 가장 긴 gap 찾기 (설정된 ROI 범위 내에서)
        int gap_start, gap_end;
        find_max_gap(ranges, start_idx, end_idx, gap_start, gap_end);
        
        // Gap 내에서 최적의 지점(Index) 찾기
        // 중앙점만 쓰면 안정적이지만 느리고, 먼 점만 쓰면 벽에 붙음. 섞어서 사용
        int best_i;
        find_best_point(ranges, gap_start, gap_end, best_i);

        // 인덱스를 실제 조향각으로 변환
        float angle = scan_msg->angle_min + best_i * scan_msg->angle_increment;
        angle = angle * 1.3; // 조향 반응성을 높이기 위한 보정치

        // 자동차 제어 메시지 생성 및 전송
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = angle;

        // 조향각에 따른 속도 제어 (급커브일수록 천천히)
        float abs_angle = std::abs(angle);
        if (abs_angle > 20.0 * M_PI / 180.0) {          // 20도 이상 꺾을 때
            drive_msg.drive.speed = 1.5; 
        } else if (abs_angle > 10.0 * M_PI / 180.0) {   // 10도 이상 꺾을 때
            drive_msg.drive.speed = 3.5;
        } else {                                        // 직진 구간
            drive_msg.drive.speed = 5.5;                // 직진 구간 가속
        }
        
        drive_pub->publish(drive_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}