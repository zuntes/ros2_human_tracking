#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <chrono>
#include <vector>
#include <stdbool.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "yolo_msgs/msg/track.hpp"
#include "yolo_msgs/msg/track_array.hpp"
#include "yolo_msgs/msg/point_array.hpp"

#include "people_msgs/msg/detected_person.hpp"
#include "people_msgs/msg/detected_person_array.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class PersonPosition : public rclcpp::Node
{
public:
    PersonPosition() : Node("compute_human_pos_node")
    {
        RCLCPP_INFO(this->get_logger(), "Start calculate persons position");

        // Params
        use_iqr_ = this->declare_parameter<bool>("use_iqr", true); // using the interquatile range to remove the outlier

        include_bbox_center_ = this->declare_parameter<bool>("include_bbox_center", false); // Use bbox center position

        use_mean_point_ = this->declare_parameter<bool>("use_mean_point", true); // Use the mean of all the keypoints to calculate human position (otherwise using median)

        keypoint_ids_ = this->declare_parameter<std::vector<int64_t>>("keypoint_id", std::vector<int64_t>{-1}); // Select the keypoints are used for calculate human position (-1 for all keypoints available)

        threshold_ = this->declare_parameter<float>("threshold", 0.25); // how far away to consider a point is a outliner (m)

        // Publisher
        person_pos_pub_ = this->create_publisher<people_msgs::msg::DetectedPersonArray>("/camera/detected_person_pos_array", 50);
        // marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/camera/visualize_people_marker", 10);

        // Subcriber
        track_array_sub_ = this->create_subscription<yolo_msgs::msg::TrackArray>(
            "/camera/track_3D_array", 50, std::bind(&PersonPosition::track_callback, this, _1));

    }

private:
    // Conpute human position
    void track_callback(const yolo_msgs::msg::TrackArray &tracks)
    {
        // RCLCPP_INFO(this->get_logger(), "Got track array");
        people_msgs::msg::DetectedPersonArray detected_persons;

        // Consider only human position in x,y,z-axis (No orientation)
        if (keypoint_ids_.size() == 1 && keypoint_ids_[0] == -1)
        {
            for (const auto &track : tracks.tracks)
            {
                people_msgs::msg::DetectedPerson human_pos;
                yolo_msgs::msg::PointArray point_array;
                geometry_msgs::msg::Point processed_point;

                if (include_bbox_center_)
                {
                    geometry_msgs::msg::Point bbox;
                    bbox.x = track.bbox3d.center.position.x;
                    bbox.y = track.bbox3d.center.position.y;
                    bbox.z = track.bbox3d.center.position.z;
                    point_array.points.push_back(bbox);
                }
                for (const auto &keypoint : track.keypoints3d.keypoints)
                {
                    geometry_msgs::msg::Point kp;
                    kp.x = keypoint.point.x;
                    kp.y = keypoint.point.y;
                    kp.z = keypoint.point.z;
                    point_array.points.push_back(kp);
                }

                processed_point = compute_human_position(point_array);
                human_pos.confidence = track.score;
                human_pos.detection_id = track.track_id;
                human_pos.pose.pose.position.x = processed_point.x;
                human_pos.pose.pose.position.y = processed_point.y;
                human_pos.pose.pose.position.z = processed_point.z;

                detected_persons.persons.push_back(human_pos);
            }
        }
        else
        {
            for (const auto &kp_id : keypoint_ids_)
            {
                for (const auto &track : tracks.tracks)
                {
                    people_msgs::msg::DetectedPerson human_pos;
                    yolo_msgs::msg::PointArray point_array;
                    geometry_msgs::msg::Point processed_point;

                    auto it = std::find_if(track.keypoints3d.keypoints.begin(), track.keypoints3d.keypoints.end(),
                                           [&kp_id](const yolo_msgs::msg::KeyPoint3D &kp)
                                           { return kp_id == kp.id; });

                    if (it != track.keypoints3d.keypoints.end())
                    {
                        for (const auto &keypoint : track.keypoints3d.keypoints)
                        {
                            if (keypoint.id == kp_id)
                            {
                                geometry_msgs::msg::Point kp;
                                kp.x = keypoint.point.x;
                                kp.y = keypoint.point.y;
                                kp.z = keypoint.point.z;
                                point_array.points.push_back(kp);
                            }
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "ID %ld not found in detection keypoints", kp_id);
                    }

                    if (include_bbox_center_)
                    {
                        geometry_msgs::msg::Point bbox;
                        bbox.x = track.bbox3d.center.position.x;
                        bbox.y = track.bbox3d.center.position.y;
                        bbox.z = track.bbox3d.center.position.z;
                        point_array.points.push_back(bbox);
                    }

                    processed_point = compute_human_position(point_array);
                    human_pos.confidence = track.score;
                    human_pos.detection_id = track.track_id;
                    human_pos.pose.pose.position.x = processed_point.x;
                    human_pos.pose.pose.position.y = processed_point.y;
                    human_pos.pose.pose.position.z = processed_point.z;

                    detected_persons.persons.push_back(human_pos);
                }
            }
        }
        detected_persons.header.frame_id = tracks.header.frame_id;
        detected_persons.number = tracks.number;
        detected_persons.header.stamp = this->get_clock()->now();
        person_pos_pub_->publish(detected_persons);
    }

    geometry_msgs::msg::Point compute_human_position(const yolo_msgs::msg::PointArray &point_array)
    {
        geometry_msgs::msg::Point human_pos;

        if (use_iqr_)
        {
            yolo_msgs::msg::PointArray removed_outlier_point_array;
            removed_outlier_point_array = this->remove_outlier(point_array, this->threshold_);
            if (use_mean_point_)
            {
                human_pos = this->compute_mean(removed_outlier_point_array);
            }
            else
            {
                human_pos = this->compute_median(removed_outlier_point_array);
            }
        }
        else
        {
            if (use_mean_point_)
            {
                human_pos = this->compute_mean(point_array);
            }
            else
            {
                human_pos = this->compute_median(point_array);
            }
        }

        return human_pos;
    }

    // Return the median point 
    geometry_msgs::msg::Point compute_median(const yolo_msgs::msg::PointArray &point_array)
    {
        geometry_msgs::msg::Point median_point;
        if (point_array.points.empty())
        {
            return median_point;
        }
        std::vector<geometry_msgs::msg::Point> sorted_points = point_array.points;

        // std::sort(sorted_points.begin(), sorted_points.end(), comparePoint);
        // Lambda func is more concisely
        std::sort(sorted_points.begin(), sorted_points.end(), [this](const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
                  { return compute_distance(p1) < compute_distance(p2); });

        size_t n = sorted_points.size();

        if (n % 2 == 0) // even number of points
        {
            median_point.x = (sorted_points[n / 2 - 1].x + sorted_points[n / 2].x) / 2.0;
            median_point.y = (sorted_points[n / 2 - 1].y + sorted_points[n / 2].y) / 2.0;
            median_point.z = (sorted_points[n / 2 - 1].z + sorted_points[n / 2].z) / 2.0;
        }
        else // odd number of points
        {
            median_point = sorted_points[n / 2];
        }

        return median_point;
    }

    // Return the mean point 
    geometry_msgs::msg::Point compute_mean(const yolo_msgs::msg::PointArray &point_array)
    {
        geometry_msgs::msg::Point mean_point;
        if (point_array.points.empty())
        {

            return mean_point;
        }
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (const auto &point : point_array.points)
        {
            sum_x += point.x;
            sum_y += point.y;
            sum_z += point.z;
        }

        size_t count = point_array.points.size();

        mean_point.x = sum_x / count;
        mean_point.y = sum_y / count;
        mean_point.z = sum_z / count;

        return mean_point;
    }

    // Use interquatile range to remove outlier (based on z value)
    yolo_msgs::msg::PointArray remove_outlier(const yolo_msgs::msg::PointArray &point_array, const float &threshold)
    {
        yolo_msgs::msg::PointArray removed_outlier_point_array;
        if (point_array.points.empty())
        {
            return removed_outlier_point_array;
        }

        std::vector<geometry_msgs::msg::Point> sorted_point_array = point_array.points;
        std::sort(sorted_point_array.begin(), sorted_point_array.end(), [this](const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
                  { return compute_distance(p1) < compute_distance(p2); });

        // Compute Q1 (25th percentile) and Q3 (75th percentile)
        size_t q1_idx = sorted_point_array.size() / 4;
        size_t q3_idx = 3 * sorted_point_array.size() / 4;
        float q1 = compute_distance(sorted_point_array[q1_idx]);
        float q3 = compute_distance(sorted_point_array[q3_idx]);

        // Compute IQR and boundary
        float iqr = q3 - q1;

        float lower_bound = q1 - 1.5 * iqr;
        float upper_bound = q3 + 1.5 * iqr;

        std::vector<geometry_msgs::msg::Point> non_outliers;
        std::vector<geometry_msgs::msg::Point> outliers;

        for (const auto &point : sorted_point_array)
        {
            double distance = compute_distance(point);
            if (distance < lower_bound || distance > upper_bound)
            {
                outliers.push_back(point);
            }
            else
            {
                non_outliers.push_back(point);
            }
        }

        // Compute the sum of z value of non-outliers
        float sum_distance = std::accumulate(non_outliers.begin(), non_outliers.end(), 0.0,
                                      [this](float sum, const geometry_msgs::msg::Point &point)
                                      { return sum + compute_distance(point); });

        // Compute mean z value of non-outliers
        float mean_distance = sum_distance / non_outliers.size();

        for (const auto &point : non_outliers)
        {
            removed_outlier_point_array.points.push_back(point);
        }

        for (const auto &point : outliers)
        {
            if (std::abs(compute_distance(point) - mean_distance) <= threshold)
            {
                removed_outlier_point_array.points.push_back(point);
            }
        }

        return removed_outlier_point_array;
    }

    double compute_distance(const geometry_msgs::msg::Point &point)
    {
        return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    }

    // Publisher
    rclcpp::Publisher<people_msgs::msg::DetectedPersonArray>::SharedPtr person_pos_pub_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Subcriber
    rclcpp::Subscription<yolo_msgs::msg::TrackArray>::SharedPtr track_array_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Variable
    bool use_iqr_;
    bool include_bbox_center_;
    bool use_mean_point_;
    std::vector<int64_t> keypoint_ids_;
    float threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonPosition>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}