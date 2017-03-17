#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

#include "geometry_msgs/Transform.h"
#include <tf/transform_broadcaster.h>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"
#include <math.h>

#include "Eigen/Eigen"
#include <eigen3/Eigen/Dense>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#define PI 3.14159265

using namespace PointMatcherSupport;
using namespace std;
using namespace grid_map;

using Eigen::MatrixXd;
using Eigen::MatrixXf;


int rows = 15, cols = 500;
MatrixXf buffer(rows, cols);

class LaserScanToPointCloud{

public:
    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher scan_pub_;

    // Create the default ICP algorithm
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    PM::ICP icp;

    DP cloud_current;
    DP cloud_previous;

    float x, y, theta;

    tf::TransformBroadcaster br;
    tf::Transform tf_transform;

    bool is_cloud_init;

    ros::Publisher publisher;

    grid_map::GridMap map;

    tf::TransformListener tfListener;
    PM::TransformationParameters tranform;

    sensor_msgs::LaserScan currentScan;
    sensor_msgs::LaserScan previousScan;

    LaserScanToPointCloud(ros::NodeHandle n) :
            n_(n),
            laser_sub_(n_, "/scan", 1),
            laser_notifier_(laser_sub_,listener_, "odom", 1)
    {
        // See the implementation of setDefault() to create a custom ICP algorithm
        icp.setDefault();

        is_cloud_init = false;
        tranform = PM::TransformationParameters::Identity(4, 4);

        buffer.setZero();
        map = GridMap({"buffer"});
        map.setFrameId("odom");
        map.setGeometry(Length(rows, cols), 1);

        tf_transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );

        laser_notifier_.registerCallback(
                boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.05));
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/my_cloud", 1);

        publisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        previousScan = sensor_msgs::LaserScan(currentScan);
        currentScan = *scan_in;

        sensor_msgs::PointCloud2 cloud_input;
        try
        {
            projector_.transformLaserScanToPointCloud(
                    "odom", *scan_in, cloud_input, listener_);
        }
        catch (tf::TransformException& e)
        {
            std::cout << e.what();
            return;
        }

        // Do something with cloud.

        // Container for original & filtered data
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2 cloud_filtered;

        // Convert to PCL data type
        pcl_conversions::toPCL(cloud_input, cloud);

        // Perform the actual filtering
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud (cloudPtr);
        sor.setLeafSize (0.1, 0.1, 0.1);
        sor.filter (cloud_filtered);

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output;
        pcl_conversions::moveFromPCL(cloud_filtered, output);


        scan_pub_.publish(output);

        //if(currentScan.ranges.size() )
        //    compute_icp();

        //publish_grid_map();
    }

    void compute_icp(){
        // good
        //DP data_current = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(currentScan, &tfListener, "odom");
        //DP data_previous = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(previousScan, &tfListener, "odom");

        std::unique_ptr<DP> cloud_current(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(currentScan, &tfListener, "odom")));
        std::unique_ptr<DP> cloud_previous(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(previousScan, &tfListener, "odom")));

        //std::cout << "Number of points: cloud (" << cloud->features.cols() << "), scanIn (" << scanMsgIn.ranges.size() << ")" << std::endl;
        //ROS_INFO("cloud_current: %d - cloud_previous: %d", cloud_current->features.cols(), cloud_previous->features.cols() );

        if(cloud_current->features.cols() > 0 && cloud_previous->features.cols() > 0){
            tranform = icp(*cloud_current, *cloud_previous);

            x = tranform(0,2);
            y = tranform(1,2);
            theta = acos(tranform(1, 1)); //(float)(acos(tranform(1, 1)) * 180.0 / PI);

            //ROS_INFO_STREAM("T: " << T << "\n");
            std::cout << "TF: [" << cloud_current->features.cols() << ":" << cloud_previous->features.cols() << "] "
                      << " x: (" << x << ") y: (" << y << ") t: (" << theta << ")" << std::endl;



            //geometry_msgs::Transform transformmsg;
            //tf::transformTFToMsg(PointMatcher_ros::eigenMatrixToTransform<float>(tranform), transformmsg);

            //tf::Quaternion q (transformmsg.rotation.x, transformmsg.rotation.y, transformmsg.rotation.z, transformmsg.rotation.w);
            //tf_transform.setRotation(q);
            //tf::Vector3 v(transformmsg.translation.x, transformmsg.translation.y, transformmsg.translation.z);
            //tf_transform.setOrigin(v);

            //br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "odom", "estimated"));
        }
    }

    void publish_grid_map(){
        map.get("buffer") = buffer;
        ros::Time time = ros::Time::now();

        map.add("buffer", buffer);

        // Publish grid map.
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map, message);
        publisher.publish(message);
    }

};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "my_scan_to_cloud");
    ros::NodeHandle n;
    LaserScanToPointCloud lstopc(n);

    ros::spin();

    return 0;
}