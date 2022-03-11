/*
 *  The package is used for ego-motion compesation of pointcloud data.
 *  Input:
 *    Global tf: /map for example
 *    Pointcloud with no individual pt timestamp
 */

#include "motion_compensation/motion_compensation.h"


MotionCompensation::MotionCompensation()
: mPrivateHandler("~")
, mLastTransform ()
, mCurrentTransform ()
, mLastHeader ()
, mCurrentHeader ()
, mStartAzi (0)
, mClockwise (false)
{
    mPrivateHandler.param<std::string>("input_cloud", mInputCloud, "velodyne_points");
    ROS_INFO("Input cloud: %s", mInputCloud.c_str());
    mPrivateHandler.param<float>("ang_resolusion", mAngResol, 5);
    ROS_INFO("Ang_resolusion: %lf [deg]", mAngResol);
    mOutputCloud = "Compensated/" + mInputCloud;

    mSubScan = mNodeHandler.subscribe(mInputCloud, 1, &MotionCompensation::Callback, this);
    mPubScan = mPrivateHandler.advertise<sensor_msgs::PointCloud2>(mOutputCloud, 1);
    mPubScanOrder = mPrivateHandler.advertise<sensor_msgs::PointCloud2>("scaning_order", 1);
    mPubGridSlice = mPrivateHandler.advertise<visualization_msgs::MarkerArray>("grid_slice_map", 1);
}

void MotionCompensation::Run()
{
    ros::Rate loopRate(100);
    while (ros::ok())
    {
        ros::spin();
        loopRate.sleep();
    }
}

void MotionCompensation::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mLastHeader = mCurrentHeader;
    mLastTransform = mCurrentTransform;
    mCurrentHeader = msg->header;
    std::cout << "Last cloud: " << mLastHeader.stamp << std::endl;
    std::cout << "Get cloud: " << mCurrentHeader.stamp << std::endl;

    try
    {
        mListener.waitForTransform(
            "map", mCurrentHeader.frame_id, mCurrentHeader.stamp, ros::Duration(1.0));
        mListener.lookupTransform(
            "map", mCurrentHeader.frame_id, mCurrentHeader.stamp, mCurrentTransform);
    }
    catch (tf::TransformException ex)
    {
        // Not get current tf
        ROS_WARN("%s",ex.what());
        mPubScan.publish(msg);
        return;
    }

    if(mLastHeader.stamp.toSec() == 0 || mLastTransform.stamp_.toSec() == 0)
    {
        // Not get last tf
        std::cout << "First frame, no compensation\n";
        mPubScan.publish(msg);
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr inCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *inCloud);

    if(inCloud->points.size() == 0)
    {
        ROS_WARN("No points in scan");
        mPubScan.publish(msg);
        return;
    }

}
