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

    getScanRotation(
        inCloud,
        mStartAzi,
        mClockwise);

    int col_num = static_cast<int> (360.0/mAngResol);
    auto gridMap = buildScanGrid(inCloud, col_num);

void MotionCompensation::getScanRotation(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inCloud,
    float& start_azi,
    bool& clockwise)
{
    /*
        Get starting scan azimuth in pointcloud coordinate and rotate orientation.
        This function is for non-stamped pointcloud.
        @Param
            INPUT:
                inCloud: original input cloud
            OUTPUT:
                start_azi: record first pt azimuth (degree) in poincloud sequence
                clockwise: scanning orientation
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZI>);
    int counter = 0;
    for(auto pt: inCloud->points)
    {
        pcl::PointXYZI pt_out;
        pt_out = pt;
        pt_out.intensity = counter;
        outCloud->points.push_back(pt_out);
        counter++;
    }
    sensor_msgs::PointCloud2 outCloud_msg;
    pcl::toROSMsg(*outCloud, outCloud_msg);
    outCloud_msg.header = mCurrentHeader;
    mPubScanOrder.publish(outCloud_msg);

    // check scanning direction by points' order in cloud
    float first_azimuth=0, second_azimuth=0, first_counter=0, second_counter=0;
    float window_size = 10;
    for(int i = 0; i < inCloud->points.size(); i++)
    {
        float azi = std::atan2(inCloud->points[i].y, inCloud->points[i].x) * 180 / M_PI;
        azi = (azi < 0) ? azi + 360 : azi;
        if(i == 0)
            start_azi = azi;

        if(i < inCloud->points.size()/window_size)
        {
            first_azimuth += azi;
            first_counter++;
        }
        else if (i < inCloud->points.size()/(window_size/2))
        {
            second_azimuth += azi; 
            second_counter++;
        }
        else
            break;
    }
    first_azimuth /= first_counter;
    second_azimuth /= second_counter;

    if(second_azimuth < first_azimuth)
        clockwise = true;
    else
        clockwise = false;

    std::cout << "start:" << start_azi << "; first: " << first_azimuth << "; second: " << second_azimuth << "; clockwise: " << clockwise << std::endl;
}

GridSlice MotionCompensation::buildScanGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inCloud,
    const int& col_num)
{
    /*
        Build 1-d grid map in angular direction.
        Base on coordinate of poincloud, NOT from SCANNING ORDER.
        Map store pt index in original inCloud.
        @Param
            INPUT:
                inCloud: pointcloud
                col_num: total num of grid
            OUTPUT:
                1-d map of grid slices
            
    */
    GridSlice polarGrid(col_num);
    
    int col_idx;
    float range, azimuth;
    int pt_idx = 0;
    for(const auto& pt: inCloud->points)
    {
        azimuth = std::atan2(pt.y, pt.x)*180/M_PI;
        if(azimuth < 0) azimuth+=360;

        col_idx = (floor)(azimuth/mAngResol);

        if(col_idx > col_num-1) 
        {
            // if(row_idx > row_num-1)
            //     std::cout << "Out of range: " << range << std::endl;
            // else
            //     std::cout << "\033[1;33mError azi\033[0m, col_idx: " << col_idx << ", azimuth: " << azimuth << std::endl;
            pt_idx++;
            continue;
        }

        polarGrid[col_idx].points().push_back(pt_idx);
        pt_idx++;
    }

    return polarGrid;
}
}
