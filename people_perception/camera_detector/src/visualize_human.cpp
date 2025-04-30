#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <functional>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

#include <people_msgs/msg/detected_person_array.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

class HumanVisualizer : public rclcpp::Node
{
public:
    HumanVisualizer() : Node("human_visualizer")
    {
        RCLCPP_INFO(this->get_logger(), "Start visualize human position node");

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/camera/human_markers", 10);
        human_pos_array_sub_ = this->create_subscription<people_msgs::msg::DetectedPersonArray>(
            "/camera/detected_person_pos_array", 10,
            std::bind(&HumanVisualizer::listener_callback, this, _1));
    }

private:
    void listener_callback(const people_msgs::msg::DetectedPersonArray::SharedPtr msg)
    {
        marker_array_.markers.clear();  // Clear existing markers

        if (msg->persons.empty())
        {
            // If no humans detected, publish empty marker array
            marker_pub_->publish(marker_array_);
            return;
        }

        for (const auto &human : msg->persons)
        {
            create_markers(human, msg->header.frame_id);
        }

        marker_pub_->publish(marker_array_);
    }

    void create_markers(const people_msgs::msg::DetectedPerson &human, const std::string &frame_id)
    {
        // Get a unique color for this person
        auto color = get_unique_color(human.detection_id);

        // Create the head marker
        visualization_msgs::msg::Marker head_marker;
        head_marker.header.frame_id = frame_id;
        head_marker.header.stamp = this->get_clock()->now();
        head_marker.ns = "human";
        head_marker.id = human.detection_id * 100 + 0;  // Unique ID
        head_marker.type = visualization_msgs::msg::Marker::SPHERE;
        head_marker.action = visualization_msgs::msg::Marker::ADD;
        head_marker.pose.position.x = human.pose.pose.position.x;
        head_marker.pose.position.y = human.pose.pose.position.y;
        head_marker.pose.position.z = 1.2;  // Height for head
        head_marker.scale.x = 0.25;
        head_marker.scale.y = 0.25;
        head_marker.scale.z = 0.25;
        head_marker.color.a = 1.0;
        head_marker.color.r = color.r;
        head_marker.color.g = color.g;
        head_marker.color.b = color.b;
        marker_array_.markers.push_back(head_marker);

        // Create the body marker
        visualization_msgs::msg::Marker body_marker;
        body_marker.header.frame_id = frame_id;
        body_marker.header.stamp = this->get_clock()->now();
        body_marker.ns = "human";
        body_marker.id = human.detection_id * 100 + 1;  // Unique ID
        body_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        body_marker.action = visualization_msgs::msg::Marker::ADD;
        body_marker.pose.position.x = human.pose.pose.position.x;
        body_marker.pose.position.y = human.pose.pose.position.y;
        body_marker.pose.position.z = 0.6;  // Height for body
        body_marker.scale.x = 0.3;
        body_marker.scale.y = 0.3;
        body_marker.scale.z = 1.0;
        body_marker.color.a = 1.0;
        body_marker.color.r = color.r;
        body_marker.color.g = color.g;
        body_marker.color.b = color.b;
        marker_array_.markers.push_back(body_marker);

        // Create the text marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = frame_id;
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.ns = "human";
        text_marker.id = human.detection_id * 100 + 2;  // Unique ID
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = human.pose.pose.position.x;
        text_marker.pose.position.y = human.pose.pose.position.y;
        text_marker.pose.position.z = 1.6;  // Height for text
        text_marker.scale.z = 0.2;
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;  // White text
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.text = std::to_string(human.detection_id);
        marker_array_.markers.push_back(text_marker);
    }

    struct Color 
    {
        float r, g, b;
    };

    Color get_unique_color(uint64_t id) 
    {
        float r = static_cast<float>((id * 37) % 255) / 255.0f;
        float g = static_cast<float>((id * 73) % 255) / 255.0f;
        float b = static_cast<float>((id * 109) % 255) / 255.0f;

        return {r, g, b};
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<people_msgs::msg::DetectedPersonArray>::SharedPtr human_pos_array_sub_;
    visualization_msgs::msg::MarkerArray marker_array_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanVisualizer>());
    rclcpp::shutdown();
    return 0;
}
