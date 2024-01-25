#ifndef VISUAL_ODOMETRY_NODE_HPP_
#define VISUAL_ODOMETRY_NODE_HPP_

#include <vector>
#include <limits>
#include <string>

#include <opencv2/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include "visual_odometry/visual_odom_core.hpp"

using namespace std::placeholders;

namespace visual_odometry
{
class VisualOdometryNode : public rclcpp::Node
{
public:
    VisualOdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("visual_odometry_node", options)
    , static_tf_initialized(false)
    , map_frame("map")
    , robot_frame("base_link")
    , camera_frame("depth_camera_optical_link")
    , img_topic("depth_camera/image_raw")
    , pc_topic("depth_camera/points")
    , fx(168.6042205484022)
    , fy(168.6042205484022)
    , cx(160.5)
    , cy(160.5)
    , img_width(320)
    , img_height(320)
    , max_distance(10.0)
    , visualize(true)
    {
        // declare parameters
        this->declare_parameter("map_frame", map_frame);
        this->declare_parameter("robot_frame", robot_frame);
        this->declare_parameter("camera_frame", camera_frame);
        this->declare_parameter("img_topic", img_topic);
        this->declare_parameter("pc_topic", pc_topic);
        this->declare_parameter("fx", fx);
        this->declare_parameter("fy", fy);
        this->declare_parameter("cx", cx);
        this->declare_parameter("cy", cy);
        this->declare_parameter("img_width", img_width);
        this->declare_parameter("img_height", img_height);
        this->declare_parameter("max_distance", max_distance);
        this->declare_parameter("visualize", visualize);

        // get parameters
        map_frame = this->get_parameter("map_frame").as_string();
        robot_frame = this->get_parameter("robot_frame").as_string();
        camera_frame = this->get_parameter("camera_frame").as_string();
        img_topic = this->get_parameter("img_topic").as_string();
        pc_topic = this->get_parameter("pc_topic").as_string();
        fx = this->get_parameter("fx").as_double();
        fy = this->get_parameter("fy").as_double();
        cx = this->get_parameter("cx").as_double();
        cy = this->get_parameter("cy").as_double();
        img_width = this->get_parameter("img_width").as_int();
        img_height = this->get_parameter("img_height").as_int();
        max_distance = this->get_parameter("max_distance").as_double();
        visualize = this->get_parameter("visualize").as_bool();

        max_distance_sqr = max_distance * max_distance;
        map_to_base_link.header.frame_id = map_frame;
        map_to_base_link.child_frame_id = robot_frame;

        voc_ = VisualOdometryCore(fx, fy, cx, cy, img_width, img_height);
        
        // synchronize the image and point cloud messages
        img_sub_.subscribe(this, img_topic);
        pc_sub_.subscribe(this, pc_topic);

        sync_ = std::make_shared<message_filters::Synchronizer<exact_policy>>(exact_policy(10), img_sub_, pc_sub_);
        sync_->registerCallback(std::bind(&VisualOdometryNode::imgPcCallback, this, _1, _2));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // visualization stuff
        visualize = true;
        rviz_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "depth_camera/features", 10
        );
        rviz_pc_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "rviz/key_point_markers", 10
        );
    }

private:
    VisualOdometryCore voc_;
    message_filters::Subscriber<sensor_msgs::msg::Image> img_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> exact_policy;
    std::shared_ptr<message_filters::Synchronizer<exact_policy>> sync_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    Eigen::Affine3f depth_camera_to_base_link;
    geometry_msgs::msg::TransformStamped map_to_base_link;
    bool static_tf_initialized;
    std::string map_frame, robot_frame, camera_frame;
    std::string img_topic, pc_topic;
    float fx, fy, cx, cy;
    int img_width, img_height;
    float max_distance, max_distance_sqr;

    void imgPcCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc)
    {
        if (!static_tf_initialized)
        {
            getStaticTransform();
        }

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc, *pcl_ptr);
        
        // filter out points at max distance
        for (auto& p : *pcl_ptr)
        {
            if (p.x*p.x + p.y*p.y + p.z*p.z >= max_distance_sqr)
            {
                p.x = -std::numeric_limits<float>::infinity();
                p.y = -std::numeric_limits<float>::infinity();
                p.z = -std::numeric_limits<float>::infinity();
            }
        }

        // transform point cloud from camera frame to base_link frame
        pcl::transformPointCloud(*pcl_ptr, *pcl_ptr, depth_camera_to_base_link);

        voc_.processImgPc(cv_ptr->image, pcl_ptr, visualize);

        // publish map -> base_link transform
        map_to_base_link.header.stamp = this->now();
        Eigen::Affine3f T = voc_.curr_pose_;
        map_to_base_link.transform.translation.x = T.matrix()(0, 3);
        map_to_base_link.transform.translation.y = T.matrix()(1, 3);
        map_to_base_link.transform.translation.z = T.matrix()(2, 3);
        Eigen::Quaternionf quat(T.matrix().block<3, 3>(0, 0));
        map_to_base_link.transform.rotation.x = quat.x();
        map_to_base_link.transform.rotation.y = quat.y();
        map_to_base_link.transform.rotation.z = quat.z();
        map_to_base_link.transform.rotation.w = quat.w();
        tf_broadcaster_->sendTransform(map_to_base_link);

        if (visualize)
        {
            // visualizePc(voc_.frame_.pc_, 0.01, 0.0, 0.0, 1.0);
            visualizeKeyPoints(cv_ptr);

            visualizePc(voc_.filt_pc, 0.01, 1.0, 0.0, 0.0); // red
            visualizePc(voc_.prev_filt_pc, 0.01, 0.0, 1.0, 0.0); // green
            visualizeCorrespondence(voc_.prev_filt_pc, voc_.filt_pc, 0.01, 1.0, 1.0, 1.0); // white
            marker_id++;
        }
    }
    
    void getStaticTransform()
    {
        // get the static transformation matrix between camera frame and base_link
        geometry_msgs::msg::TransformStamped T;
        try 
        {
            T = tf_buffer_->lookupTransform(robot_frame, camera_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
            return;
        }
        // convert geometry_msgs::msg::TransformStamped to Eigen::Affine3f
        depth_camera_to_base_link = 
            Eigen::Translation3f(T.transform.translation.x, 
                T.transform.translation.y, 
                T.transform.translation.z
            )
            * 
            Eigen::Quaternionf(T.transform.rotation.x, 
                T.transform.rotation.y, 
                T.transform.rotation.z, 
                T.transform.rotation.w
            );
        static_tf_initialized = true;
    }

    // visualization stuff
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rviz_img_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_pc_pub_;
    bool visualize;
    int32_t marker_id{std::numeric_limits<int32_t>::min()};

    void visualizePc(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, float pt_size, float r, float g, float b) const
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame;
        m.header.stamp = this->now();
        m.lifetime = rclcpp::Duration::from_seconds(0);
        m.frame_locked = false;
        m.type = visualization_msgs::msg::Marker::POINTS;
        m.scale.x = pt_size;
        m.scale.y = pt_size;
        m.scale.z = pt_size;
        m.color.a = 1.0;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.id = marker_id;

        geometry_msgs::msg::Point point;
        for (auto& p : *pc)
        {
            if (pcl::isFinite(p))
            {
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;

                m.points.push_back(point);
            }
        }
        rviz_pc_pub_->publish(m);
    }

    void visualizeKeyPoints(const cv_bridge::CvImagePtr& cv_ptr)
    {
        sensor_msgs::msg::Image::SharedPtr processed_img = cv_bridge::CvImage(cv_ptr->header, cv_ptr->encoding, cv_ptr->image).toImageMsg();
        rviz_img_pub_->publish(*processed_img);
    }

    void visualizePc(const std::vector<Eigen::Vector3f>& pc, float pt_size, float r, float g, float b) const
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame;
        m.header.stamp = this->now();
        m.lifetime = rclcpp::Duration::from_seconds(0);
        m.frame_locked = false;
        m.type = visualization_msgs::msg::Marker::POINTS;
        m.scale.x = pt_size;
        m.scale.y = pt_size;
        m.scale.z = pt_size;
        m.color.a = 1.0;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point point;
        for (auto& p : pc)
        {
            point.x = p.x();
            point.y = p.y();
            point.z = p.z();

            m.points.push_back(point);
        }
        rviz_pc_pub_->publish(m);
    }

    void visualizeCorrespondence(const std::vector<Eigen::Vector3f>& prev_pc, std::vector<Eigen::Vector3f>& pc,
        float line_size, float r, float g, float b) const
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = map_frame;
        m.header.stamp = this->now();
        m.lifetime = rclcpp::Duration::from_seconds(0);
        m.frame_locked = false;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.scale.x = line_size;
        m.color.a = 1.0;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point point;
        for (int i=0; i<pc.size(); ++i)
        {
            point.x = prev_pc[i].x();
            point.y = prev_pc[i].y();
            point.z = prev_pc[i].z();
            m.points.push_back(point);

            point.x = pc[i].x();
            point.y = pc[i].y();
            point.z = pc[i].z();
            m.points.push_back(point);
        }
        rviz_pc_pub_->publish(m);
    }
};
}

#endif