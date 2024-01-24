#ifndef VISUAL_ODOMETRY_CORE_HPP_
#define VISUAL_ODOMETRY_CORE_HPP_

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace visual_odometry
{
// class for storing all the information related to an image frame
class Frame
{
public:
    Frame()
    {
    }

    void clear()
    {
        kps_.clear();
        desc_.release();
    }

    std::vector<cv::KeyPoint> kps_;
    cv::Mat desc_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;
};

class VisualOdometryCore
{
public:
    VisualOdometryCore();
    VisualOdometryCore(float fx, float fy, float cx, float cy, int img_width, int img_height);
    void processImgPc(cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, bool visualize=false);

    Frame frame_, prev_frame_;
    Eigen::Affine3f curr_pose_;

    // TODO delete
    std::vector<Eigen::Vector3f> filt_pc, prev_filt_pc;

private:
    void calcualteAllignment(const std::vector<Eigen::Vector3f>& tgt_pc, const std::vector<Eigen::Vector3f>& src_pc, Eigen::Affine3f& T);

    std::shared_ptr<cv::SIFT> sift_; // feature detector
    std::shared_ptr<cv::FlannBasedMatcher> fbm_; // feature matcher
    cv::Matx33f K_; // camera intrinsic matrix
    cv::Size2i img_size_;

    int min_kps = 100;
    int min_matched_kps = 50;
};
} // visual_odometry

#endif