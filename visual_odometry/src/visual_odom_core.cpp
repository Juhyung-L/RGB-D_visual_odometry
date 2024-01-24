#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <algorithm>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/common/transforms.h>

#include <Eigen/Core>

#include "visual_odometry/visual_odom_core.hpp"

namespace visual_odometry
{
VisualOdometryCore::VisualOdometryCore()
{ 
}

VisualOdometryCore::VisualOdometryCore(float fx, float fy, float cx, float cy, int img_width, int img_height)
{
    K_ = cv::Matx33f(fx,   0.0,   cx,
                     0.0,  fy,    cy,
                     0.0,  0.0,   1.0);

    sift_ = cv::SIFT::create(1000, 3, 0.04, 10, 1.6, false);
    fbm_ = cv::FlannBasedMatcher::create();
    curr_pose_ = Eigen::Affine3f::Identity(); // initial pose

    img_size_ = cv::Size2i(img_width, img_height);
}

void VisualOdometryCore::processImgPc(cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>::Ptr& pc, bool visualize)
{
    // TODO delete
    filt_pc.clear();
    prev_filt_pc.clear();

    prev_frame_ = frame_;
    frame_.clear();
    pcl::transformPointCloud(*pc, *pc, curr_pose_);
    frame_.pc_ = pc;

    // convert to gray scale
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

    // extract features
    sift_->detectAndCompute(gray_img, cv::noArray(), frame_.kps_, frame_.desc_);

    if (min_kps > frame_.kps_.size())
    {
        std::cout << "Not enough key points detected in current frame. Skipping.\n";
        return;
    }

    // match features
    std::vector<cv::Point2f> matched_kps, prev_matched_kps;
    std::vector<std::vector<cv::DMatch>> matches;
    std::vector<uchar> inlier_mask;
    std::vector<cv::Point2f> filt_kps, prev_filt_kps;

    if (prev_frame_.desc_.empty())
    {
        return;
    }

    // ensure descriptors are of type CV_32F because knnMatch only takes CV_32F
    if (frame_.desc_.type() != CV_32F) {
        frame_.desc_.convertTo(frame_.desc_, CV_32F);
    }
    if (prev_frame_.desc_.type() != CV_32F) {
        prev_frame_.desc_.convertTo(prev_frame_.desc_, CV_32F);
    }

    fbm_->knnMatch(frame_.desc_, prev_frame_.desc_, matches, 5);

    if (min_matched_kps > matches.size())
    {
        std::cout << "Not enough matched key points between current frame and previous frame. Skipping.\n";
        return;
    }
    
    // Lowe's ratio test to remove bad matches
    for (int i=0; i<matches.size(); ++i)
    {
        if (matches[i][0].distance < 0.8*matches[i][1].distance)
        {
            // distance is the measure of likelihood for correct match between a pair of key points in the search space
            // there are multiple matches per key point ordered in increasing distance (greater distance = less likely for match)
            // 1st closest match = match[i][0], 2nd closest match = match[i][1], ...
            // for a match to be considered "good" under Lowe's ratio test,
            // the distance to the first matched key point should be significantly less than the distance to the second matched key point
            matched_kps.emplace_back(frame_.kps_[matches[i][0].queryIdx].pt);
            prev_matched_kps.emplace_back(prev_frame_.kps_[matches[i][0].trainIdx].pt);
        }
    }
    // calculate the mask that filters out the bad matches
    cv::findEssentialMat(prev_matched_kps, matched_kps, K_, cv::FM_RANSAC, 0.999, 1.0, 1000, inlier_mask);

    // apply the mask
    for (int i=0; i<inlier_mask.size(); ++i)
    {
        if (inlier_mask[i])
        {
            filt_kps.emplace_back(matched_kps[i]);
            prev_filt_kps.emplace_back(prev_matched_kps[i]);
        }
    }

    // draw lines connecting current key points and previous key points
    if (visualize)
    {
        for (int i=0; i<filt_kps.size(); ++i)
        {
            cv::line(img, filt_kps[i], prev_filt_kps[i], cv::Scalar(0, 0, 255), 3);
        }
    }
    // at this point, filt_kps and prev_filt_kps are the same sized vectors
    // key points at the same index are matches

    // get the depth information for each key points
    for (int i=0; i<filt_kps.size(); ++i)
    {
        int index = static_cast<int>(filt_kps[i].y) * img_size_.width + static_cast<int>(filt_kps[i].x);
        int prev_index = static_cast<int>(prev_filt_kps[i].y) * img_size_.width + static_cast<int>(prev_filt_kps[i].x);
        if (pcl::isFinite(frame_.pc_->at(index)) && pcl::isFinite(prev_frame_.pc_->at(prev_index))) // both points need to be finite to be stored inside filtered point cloud
        {
            filt_pc.emplace_back(frame_.pc_->at(index).x, frame_.pc_->at(index).y, frame_.pc_->at(index).z);
            prev_filt_pc.emplace_back(prev_frame_.pc_->at(prev_index).x, prev_frame_.pc_->at(prev_index).y, prev_frame_.pc_->at(prev_index).z);
        }
    }
    // find the ridig body transformation that will align the two point clouds
    Eigen::Affine3f T;
    calcualteAllignment(prev_filt_pc, filt_pc, T);

    pcl::transformPointCloud(*(frame_.pc_), *(frame_.pc_), T);
    curr_pose_ = T * curr_pose_;
}

void VisualOdometryCore::calcualteAllignment(const std::vector<Eigen::Vector3f>& tgt_pc, const std::vector<Eigen::Vector3f>& src_pc, Eigen::Affine3f& T)
{
    // calcualte the centroid of both pc
    Eigen::Vector3f tgt_mean = Eigen::Vector3f::Zero();
    Eigen::Vector3f src_mean = Eigen::Vector3f::Zero();
    
    // tgt_pc and src_pc have the same number of points
    for (int i=0; i<tgt_pc.size(); ++i)
    {
        tgt_mean.x() += tgt_pc[i].x();
        tgt_mean.y() += tgt_pc[i].y();
        tgt_mean.z() += tgt_pc[i].z();

        src_mean.x() += src_pc[i].x();
        src_mean.y() += src_pc[i].y();
        src_mean.z() += src_pc[i].z();
    }
    tgt_mean /= tgt_pc.size();
    src_mean /= src_pc.size();
    
    // calculate W matrix
    Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
    Eigen::Vector3f X, P;
    for (int i=0; i<tgt_pc.size(); ++i)
    {
        X.x() = tgt_pc[i].x() - tgt_mean.x();
        X.y() = tgt_pc[i].y() - tgt_mean.y();
        X.z() = tgt_pc[i].z() - tgt_mean.z();

        P.x() = src_pc[i].x() - src_mean.x();
        P.y() = src_pc[i].y() - src_mean.y();
        P.z() = src_pc[i].z() - src_mean.z();

        W += X * P.transpose();
    }
    
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    T.linear() = svd.matrixU() * svd.matrixV().transpose(); // calculate rotation matrix
    T.translation() = tgt_mean - T.linear() * src_mean; // calcualte translation vector
    return;
}
} // visual_odometry