#include <string>
#include <chrono>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "message_filters/cache.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class OdometryEvaluator : public rclcpp::Node
{
    
public:
    OdometryEvaluator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("odometry_evaluator_node", options),
    parent_frame_("map"),
    body_frame_("base_link"),
    true_pose_topic_name_("ground_truth/odom"),
    time_freq_(10.0),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    true_pose_search_interval_(rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(0.05 * 1e9))), // 50ms
    tf_timeout_(rclcpp::Duration::from_seconds(5)),
    first_iter_(false),
    id_(std::numeric_limits<int32_t>::min())
    {
        this->declare_parameter("parent_frame", parent_frame_);
        this->declare_parameter("body_frame", body_frame_);
        this->declare_parameter("true_pose_topic_name", true_pose_topic_name_);
        this->declare_parameter("odom_pub_rate", time_freq_);

        parent_frame_ = this->get_parameter("parent_frame").as_string();
        body_frame_ = this->get_parameter("body_frame").as_string();
        true_pose_topic_name_ = this->get_parameter("true_pose_topic_name").as_string();
        time_freq_ = this->get_parameter("odom_pub_rate").as_double();

        timer_ = this->create_wall_timer(
            std::chrono::nanoseconds(int(1.0 / time_freq_ * 1e9)),
            std::bind(&OdometryEvaluator::evaluate, this)
        );

        true_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            true_pose_topic_name_, rclcpp::SystemDefaultsQoS(),
            std::bind(&OdometryEvaluator::true_pose_callback, this, _1)
        );

        true_pose_cache_ = std::make_shared<message_filters::Cache<nav_msgs::msg::Odometry>>(10);

        est_true_odom_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "est_true_odom", rclcpp::SystemDefaultsQoS()
        );
    }

    void true_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        true_pose_cache_->add(msg); // add message to message filter
    }

    void evaluate()
    {
        // get the estimated pose
        geometry_msgs::msg::TransformStamped est_pose_tf;
        try
        {
            est_pose_tf = tf_buffer_->lookupTransform(
                parent_frame_, body_frame_,
                this->now(),
                tf_timeout_
            );
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from %s to %s at time %f",
                parent_frame_.c_str(), body_frame_.c_str(), this->now().seconds()
            );
            return;
        }
        
        // get the true pose
        // get true pose within +-true_pose_interval_ range of est_pose
        rclcpp::Time est_pose_time(est_pose_tf.header.stamp.sec, est_pose_tf.header.stamp.nanosec, this->get_clock()->get_clock_type());
        rclcpp::Time start = est_pose_time - true_pose_search_interval_;
        rclcpp::Time end = est_pose_time + true_pose_search_interval_;
        std::vector<std::shared_ptr<const nav_msgs::msg::Odometry>> true_poses = 
            true_pose_cache_->getInterval(start, end);

        // no ground truth data in within the time interval
        if (true_poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Could not get true pose at interval %f to %f",
                start.seconds(), end.seconds()
            );
            return;
        }
        
        // find the ground truth data that is closest to the tf_time
        std::shared_ptr<const nav_msgs::msg::Odometry> closest_true_pose = true_poses[0];
        int64_t least_time_diff = getAbsTimeDiff(rclcpp::Time(closest_true_pose->header.stamp.sec, closest_true_pose->header.stamp.nanosec, this->get_clock()->get_clock_type()), est_pose_time);
        for (auto& true_pose : true_poses)
        {
            int64_t time_diff = getAbsTimeDiff(rclcpp::Time(true_pose->header.stamp.sec, true_pose->header.stamp.nanosec, this->get_clock()->get_clock_type()), est_pose_time);
            if (least_time_diff > time_diff)
            {
                least_time_diff = time_diff;
                closest_true_pose = true_pose;
            }
        }
        
        // find the offset between the est_pose and true_pose
        nav_msgs::msg::Odometry est_pose;
        est_pose.pose.pose.position.x = est_pose_tf.transform.translation.x;
        est_pose.pose.pose.position.y = est_pose_tf.transform.translation.y;
        est_pose.pose.pose.position.z = est_pose_tf.transform.translation.z;
        est_pose.pose.pose.orientation.x = est_pose_tf.transform.rotation.x;
        est_pose.pose.pose.orientation.y = est_pose_tf.transform.rotation.y;
        est_pose.pose.pose.orientation.z = est_pose_tf.transform.rotation.z;
        est_pose.pose.pose.orientation.w = est_pose_tf.transform.rotation.w;

        if (!first_iter_)
        {
            offset_ = getOffset(est_pose.pose.pose, closest_true_pose->pose.pose);
            first_iter_ = true;
        }

        // apply offset
        nav_msgs::msg::Odometry true_pose;
        tf2::doTransform<geometry_msgs::msg::Pose>(closest_true_pose->pose.pose, true_pose.pose.pose, offset_);
        
        // send visualization markers to rviz
        printEstandTruePosesRviz(est_pose.pose.pose, true_pose.pose.pose);
    }

private:
    std::string parent_frame_;
    std::string body_frame_;
    std::string true_pose_topic_name_;
    double time_freq_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr true_pose_sub_;
    std::shared_ptr<message_filters::Cache<nav_msgs::msg::Odometry>> true_pose_cache_;
    rclcpp::Duration true_pose_search_interval_;
    rclcpp::Duration tf_timeout_;
    bool first_iter_;
    geometry_msgs::msg::TransformStamped offset_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr est_true_odom_pub_;
    int32_t id_;

    int64_t getAbsTimeDiff(const rclcpp::Time& t1, const rclcpp::Time& t2)
    {
        rclcpp::Duration diff = t1 - t2;
        return std::abs(diff.nanoseconds());
    }

    geometry_msgs::msg::TransformStamped getOffset(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2)
    {
        geometry_msgs::msg::TransformStamped ret;

        tf2::Quaternion q1(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
        tf2::Quaternion q2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);

        // get relative rotation
        // invert q2 then multiply by q1 to get relative rotation
        // store the result in q1
        q2 = q2.inverse();
        q1 = q1 * q2;

        ret.transform.rotation.x = q1.getX();
        ret.transform.rotation.y = q1.getY();
        ret.transform.rotation.z = q1.getZ();
        ret.transform.rotation.w = q1.getW();

        // need to rotate p2 first then get the translation
        // beccause the offset is applied using tf2::doTransform(), which applies rotation first then translation
        geometry_msgs::msg::Pose rotated_p2;
        tf2::doTransform<geometry_msgs::msg::Pose>(p2, rotated_p2, ret);

        // get relative translation
        ret.transform.translation.x = p1.position.x - rotated_p2.position.x;
        ret.transform.translation.y = p1.position.y - rotated_p2.position.y;
        ret.transform.translation.z = p1.position.z - rotated_p2.position.z;

        return ret;
    }

    void printEstandTruePosesRviz(const geometry_msgs::msg::Pose& est_pose, const geometry_msgs::msg::Pose& true_pose)
    {
        visualization_msgs::msg::Marker m;

        // send estimated pose
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.lifetime = rclcpp::Duration::from_seconds(0);
        m.frame_locked = false;
        m.id = id_;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.scale.x = 0.07;
        m.scale.y = 0.04;
        m.scale.z = 0.04;
        // make sure alpha and color is set or else the points will be invisible
        m.color.a = 1.0;
        m.color.r = 1.0; // estimated pose is red
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position = est_pose.position;
        m.pose.orientation = est_pose.orientation;
        est_true_odom_pub_->publish(m);

        id_++;

        // send true pose
        m.header.stamp = this->now();
        m.id = id_;
        // make sure alpha and color is set or else the points will be invisible
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0; // true pose is blue
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position = true_pose.position;
        m.pose.orientation = true_pose.orientation;
        est_true_odom_pub_->publish(m);

        id_++;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryEvaluator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}