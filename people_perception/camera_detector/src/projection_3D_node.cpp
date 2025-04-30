#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "yolo_msgs/msg/track.hpp"
#include "yolo_msgs/msg/track_array.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class ProjectionNode : public rclcpp::Node
{
public:
    ProjectionNode() : Node("projection_node")
    {
        RCLCPP_INFO(this->get_logger(), "Start projection from image to 3D node");

        // Params
        target_frame_ = this->declare_parameter<std::string>("target_frame", "camera_link");

        // Publisher
        tracks_3D_pub_ = this->create_publisher<yolo_msgs::msg::TrackArray>("/camera/track_3D_array", 50);

        // Subcriber
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/aligned_depth_to_color/camera_info", 10, std::bind(&ProjectionNode::camera_info_callback, this, _1));

        // Sync handle
        depth_img_sub_.subscribe(this, "/camera/camera/aligned_depth_to_color/image_raw", rmw_qos_profile_system_default);
        tracks_sub_.subscribe(this, "camera/track_2D_array", rmw_qos_profile_system_default);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), depth_img_sub_, tracks_sub_);
        sync_->registerCallback(std::bind(&ProjectionNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        // tf2 
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:

    // Get camera info
    void camera_info_callback(const sensor_msgs::msg::CameraInfo & msg) 
    {
        // RCLCPP_INFO(this->get_logger(), "Got camera info");
        // Extract focal lengths and principal point
        this->fx = msg.k[0];
        this->fy = msg.k[4];
        this->cx = msg.k[2];
        this->cy = msg.k[5];

    }

    // Process data
    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img,
        const yolo_msgs::msg::TrackArray::ConstSharedPtr & tracks)
    {
        auto tracks_3D = std::make_shared<yolo_msgs::msg::TrackArray>();

        tf2::Duration transform_duration = tf2::durationFromSec(0.5);
        
        for (const auto &track : tracks->tracks)
        {
            yolo_msgs::msg::Track aux_msg = track;


            aux_msg.bbox3d.frame_id = this->target_frame_;
            aux_msg.keypoints3d.frame_id = this->target_frame_;

            auto bbox = track.bbox;
            float center_x = bbox.center.position.x;
            float center_y = bbox.center.position.y;
            float size_x = bbox.size.x;
            float size_y = bbox.size.y;

            int u = static_cast<int>(center_x);
            int v = static_cast<int>(center_y);

            float z = get_depth(img, u, v); // divide by 1000 if using Intel Realsense D435i

            float x = z * ((center_x - this->cx) / this->fx); 
            float y = z * ((center_y - this->cy) / this->fy);

            geometry_msgs::msg::Point bbox_center;
            bbox_center.x = x;
            bbox_center.y = y;
            bbox_center.z = z;

            geometry_msgs::msg::Point transformed_center = transform_to_frame(bbox_center, tracks->header, target_frame_, transform_duration);

            aux_msg.bbox3d.center.position = transformed_center;
            
            aux_msg.bbox3d.size.x = size_x;
            aux_msg.bbox3d.size.y = size_y;
            
            for (const auto &keypoint : track.keypoints.keypoints)
            {
                yolo_msgs::msg::KeyPoint3D kp_3d;
                kp_3d.id = keypoint.id;
                kp_3d.score = keypoint.score;

                float kp_x = keypoint.point.x;
                float kp_y = keypoint.point.y;

                int kp_u = static_cast<int>(kp_x);
                int kp_v = static_cast<int>(kp_y);

                float kp_z = get_depth(img, kp_u, kp_v);

                kp_3d.point.x = kp_z * ((kp_x - this->cx) / this->fx);
                kp_3d.point.y = kp_z * ((kp_y - this->cy) / this->fy);
                kp_3d.point.z = kp_z;

                geometry_msgs::msg::Point kp_point;
                kp_point.x = kp_3d.point.x;
                kp_point.y = kp_3d.point.y;
                kp_point.z = kp_3d.point.z;

                geometry_msgs::msg::Point transformed_kp_point = transform_to_frame(kp_point, tracks->header, target_frame_, transform_duration);

                kp_3d.point = transformed_kp_point;

                aux_msg.keypoints3d.keypoints.push_back(kp_3d);
            }

            aux_msg.class_id = track.class_id;
            aux_msg.class_name = track.class_name;
            aux_msg.score = track.score;
            aux_msg.track_id = track.track_id;
            tracks_3D->tracks.push_back(aux_msg);
        }

        tracks_3D->header.frame_id = this->target_frame_;
        tracks_3D->header.stamp = this->get_clock()->now();
        tracks_3D->number = tracks->number;
        tracks_3D_pub_->publish(*tracks_3D);
    }

    // Get depth value given x,y coors in image
    float get_depth(const sensor_msgs::msg::Image::ConstSharedPtr &img, int x, int y)
    {
        if (x >= 0 && x < static_cast<int>(img->width) && y >= 0 && y < static_cast<int>(img->height))
        {
            const float *depth_row = reinterpret_cast<const float*>(&img->data[0]) + (y * img->width);
            return depth_row[x];
        }
        return 0.0;
    }

    // Perform tf2 transform
    geometry_msgs::msg::Point transform_to_frame(const geometry_msgs::msg::Point &point_in, const std_msgs::msg::Header &source_header, const std::string &target_frame, tf2::Duration duration)
    {
        geometry_msgs::msg::PointStamped point_stamped_in;
        point_stamped_in.header = source_header;
        point_stamped_in.point = point_in;

        geometry_msgs::msg::PointStamped point_stamped_out;
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(target_frame, source_header.frame_id, tf2::timeFromSec(source_header.stamp.sec) + tf2::durationFromSec(source_header.stamp.nanosec * 1e-9), duration);
            tf2::doTransform(point_stamped_in, point_stamped_out, transform_stamped);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", source_header.frame_id.c_str(), target_frame.c_str(), ex.what());
        }

        return point_stamped_out.point;
    }

    // Publisher
    rclcpp::Publisher<yolo_msgs::msg::TrackArray>::SharedPtr tracks_3D_pub_;

    // Subcriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    //Sync
    // defined the synchronization policy, in this case is the approximate time
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, yolo_msgs::msg::TrackArray> SyncPolicy;

    message_filters::Subscriber<sensor_msgs::msg::Image> depth_img_sub_;
    message_filters::Subscriber<yolo_msgs::msg::TrackArray> tracks_sub_;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // tf2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Variable
    double fx, fy, cx, cy;
    std::string target_frame_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProjectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}