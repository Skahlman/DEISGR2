#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TelloControl : public rclcpp::Node
{
public:
    TelloControl() : Node("control")
    {
        publisher_land = this->create_publisher<std_msgs::msg::Empty>("land", 1);
        publisher_flip = this->create_publisher<std_msgs::msg::String>("flip", 1);
        publisher_takeoff = this->create_publisher<std_msgs::msg::Empty>("takeoff", 1);
        publisher_velocity = this->create_publisher<geometry_msgs::msg::Twist>("control", 1);
        publisher_emergency = this->create_publisher<std_msgs::msg::Empty>("emergency", 1);

        subscription_cmd_movement = this->create_subscription<std_msgs::msg::String>(
            "/cmd_movement", 10, std::bind(&TelloControl::cmdMovementCallback, this, std::placeholders::_1));

        subscription_detected_object = this->create_subscription<std_msgs::msg::String>(
            "/detected_object", 10, std::bind(&TelloControl::detectedObjectCallback, this, std::placeholders::_1));

        current_mode = MODE_MAPPING;
        last_mode_switch_time = now();
        last_action = ACTION_NONE;

        last_detection_time = now() - 10s;
        last_rotate_start = now();
        currently_forward = false;

        timer = this->create_wall_timer(100ms, std::bind(&TelloControl::timerCallback, this));
    }

private:
    enum Mode {
        MODE_MAPPING,
        MODE_CHARGING
    };

    enum Action {
        ACTION_NONE,
        ACTION_LAND,
        ACTION_TAKEOFF,
        ACTION_MOVE
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_velocity;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_takeoff;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_flip;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_land;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_emergency;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_cmd_movement;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_detected_object;

    rclcpp::TimerBase::SharedPtr timer;

    Mode current_mode;
    Action last_action;
    std::string last_movement_command;

    std::chrono::time_point<std::chrono::steady_clock> last_mode_switch_time;
    std::chrono::time_point<std::chrono::steady_clock> last_detection_time; 
    std::chrono::time_point<std::chrono::steady_clock> last_rotate_start;

    bool currently_forward;

    double speed = 0.5;
    double rotate_speed = 0.5;
    double mode_switch_interval = 10.0;
    double detection_timeout = 5.0;

    std::chrono::time_point<std::chrono::steady_clock> now()
    {
        return std::chrono::steady_clock::now();
    }

    void cmdMovementCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (current_mode == MODE_MAPPING) {
            last_movement_command = msg->data;
        }
    }

    void detectedObjectCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "similar_object_found") {
            last_detection_time = now();
        }
    }

    void changeMode()
    {
        if (current_mode == MODE_MAPPING) {
            current_mode = MODE_CHARGING;
            RCLCPP_INFO(this->get_logger(), "Switching to CHARGING mode");
            currently_forward = false;
            last_rotate_start = now();
        } else {
            current_mode = MODE_MAPPING;
            RCLCPP_INFO(this->get_logger(), "Switching to MAPPING mode");
        }
        last_mode_switch_time = now();
    }

    void performMapping()
    {
        if (!last_movement_command.empty()) {
            if (last_action == ACTION_LAND) {
                takeoff();
                last_action = ACTION_TAKEOFF;
            }

            geometry_msgs::msg::Twist msg;
            if (last_movement_command == "left") {
                msg.linear.x = -speed;
            } else if (last_movement_command == "right") {
                msg.linear.x = speed;
            } else if (last_movement_command == "forward") {
                msg.linear.y = speed;
            }

            publisher_velocity->publish(msg);
            last_action = ACTION_MOVE;
        }
    }

    void performCharging()
    {
        double time_since_detection = std::chrono::duration<double>(now() - last_detection_time).count();
        geometry_msgs::msg::Twist msg;

        if (time_since_detection < 1.0) {
            msg.linear.y = speed;
            currently_forward = true;
            publisher_velocity->publish(msg);
            last_action = ACTION_MOVE;
        } else {
            if (currently_forward) {
                currently_forward = false;
                last_rotate_start = now();
            }

            double rotating_time = std::chrono::duration<double>(now() - last_rotate_start).count();
            if (rotating_time > detection_timeout) {
                land();
                last_action = ACTION_LAND;
            } else {
                msg.angular.z = rotate_speed;
                publisher_velocity->publish(msg);
                last_action = ACTION_MOVE;
            }
        }
    }

    void takeoff()
    {
        RCLCPP_INFO(this->get_logger(), "Taking off");
        publisher_takeoff->publish(std_msgs::msg::Empty());
    }

    void land()
    {
        RCLCPP_INFO(this->get_logger(), "Landing");
        publisher_land->publish(std_msgs::msg::Empty());
    }

    void timerCallback()
    {
        double elapsed = std::chrono::duration<double>(now() - last_mode_switch_time).count();
        if (elapsed > mode_switch_interval) {
            changeMode();
        }

        if (current_mode == MODE_MAPPING) {
            performMapping();
        } else {
            performCharging();
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelloControl>());
    rclcpp::shutdown();
    return 0;
}
