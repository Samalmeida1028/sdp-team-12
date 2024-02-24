#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_demo_example_2/diffbot_system.hpp"

using namespace std::chrono_literals;

class ReadMotors : public rclcpp::Node {
    public:
        ReadMotors() : Node("read_motors"), count_(0) {
            publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_vels", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&ReadMotors::timer_callback, this));
        }

    private:
        void timer_callback() {
            auto message = std_msgs::msg::Float32MultiArray();

            for(int count = 0; count < 4; count++) {
                message.data[count] = (float) db.hw_velocities_[count];
            }
            
            publisher_->publish(message);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
        ros2_control_demo_example_2::DiffBotSystemHardware db;
        size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadMotors>());
    rclcpp::shutdown();
    return 0;
}