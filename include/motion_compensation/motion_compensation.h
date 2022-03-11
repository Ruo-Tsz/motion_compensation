#ifndef MOTION_COMPENSTAION
#define MOTION_COMPENSTAION

#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <point_container.h>

class MotionCompensation
{
    private:
        ros::NodeHandle mNodeHandler, mPrivateHandler;
        ros::Subscriber mSubScan;
        ros::Publisher mPubScan, mPubScanOrder, mPubGridSlice;
        tf::StampedTransform mLastTransform, mCurrentTransform;
        tf::TransformListener mListener;
        std::string mInputCloud, mOutputCloud;
        std_msgs::Header mLastHeader, mCurrentHeader;
        float mAngResol;
        float mStartAzi;
        bool mClockwise;

        void Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    public:
        MotionCompensation();
        void Run();
};







#endif