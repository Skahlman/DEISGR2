#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

static const float CELL_SIZE = 0.1f;   // Each cell = 0.1m
static const int GRID_WIDTH = 30;     // 50m
static const int GRID_HEIGHT = 30;    // 50m
static const float MAP_ORIGIN_X = -1.5f; 
static const float MAP_ORIGIN_Y = -1.5f;

cv::Mat occupancyGrid(GRID_HEIGHT, GRID_WIDTH, CV_8S, cv::Scalar(-1));

bool WorldToMap(float x, float y, int &mx, int &my) {
    mx = static_cast<int>((x - MAP_ORIGIN_X) / CELL_SIZE);
    my = static_cast<int>((y - MAP_ORIGIN_Y) / CELL_SIZE);
    if (mx < 0 || mx >= GRID_WIDTH || my < 0 || my >= GRID_HEIGHT)
        return false;
    return true;
}

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode()
    : Node("path_planner_node")
    {
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/camera/pose", 10, std::bind(&PathPlannerNode::poseCallback, this, std::placeholders::_1));
        
        points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/orbslam/tracked_map_points", 10, std::bind(&PathPlannerNode::pointsCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/cmd_movement", 10);

        occupancy_grid_msg_.info.resolution = CELL_SIZE;
        occupancy_grid_msg_.info.width = GRID_WIDTH;
        occupancy_grid_msg_.info.height = GRID_HEIGHT;
        occupancy_grid_msg_.info.origin.position.x = MAP_ORIGIN_X;
        occupancy_grid_msg_.info.origin.position.y = MAP_ORIGIN_Y;
        occupancy_grid_msg_.header.frame_id = "map";
        occupancy_grid_msg_.data.resize(GRID_WIDTH * GRID_HEIGHT, -1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PathPlannerNode::planStep, this));
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
    }

    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float X = *iter_x;
            float Y = *iter_y;

            int mx, my;
            if (WorldToMap(X, Y, mx, my)) {
                occupancyGrid.at<int8_t>(my, mx) = 100;
            }
        }

        for (int r = 0; r < GRID_HEIGHT; r++) {
            for (int c = 0; c < GRID_WIDTH; c++) {
                occupancy_grid_msg_.data[r * GRID_WIDTH + c] = occupancyGrid.at<int8_t>(r, c);
            }
        }
        occupancy_grid_msg_.header.stamp = this->now();
    }

    void planStep() {

        // Extract robot position and orientation
        float rx = current_pose_.pose.position.x;
        float ry = current_pose_.pose.position.y;

        // Check one cell ahead in X direction
        float forward_x = rx + 0.5f; // Move 0.5m ahead
        float forward_y = ry;

        int mx, my;
        std_msgs::msg::String cmd;
        if (WorldToMap(forward_x, forward_y, mx, my)) {
            int8_t cell_val = occupancyGrid.at<int8_t>(my, mx);
            if (cell_val == -1 || cell_val == 0) {
                cmd.data = "forward";
            } else {
                cmd.data = "right";
            }
        } else {
            cmd.data = "left";
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped current_pose_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
