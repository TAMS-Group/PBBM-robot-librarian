#ifndef LIBRARIAN_PC_PROCESS_NODE_H
#define LIBRARIAN_PC_PROCESS_NODE_H

#include <ros/ros.h>
#include "librarian_vision_node/LibrarianPcProcess.h"
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <fstream>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

namespace librarian_vision
{
    /**
     * @brief Caches the point cloud to return the average position of given points.
     * 
     * The pc process node offers a service to return the average position of
     * given points specified by their coordinated. This only works with ordered
     * point clouds (though you have to manually do it for the azure kinect).
     * 
     * Points are specified using their x/y coordinate.
     */
    class LibrarianProcessNode
    {
    private:
        ros::Subscriber point_cloud_sub;
        const static int FIELD_BYTES[8];
        bool initialized = false;
        PointCloud pcl;
        ros::ServiceServer server;

        void receivePointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
        bool processAction(librarian_vision_node::LibrarianPcProcess::Request &req, librarian_vision_node::LibrarianPcProcess::Response &res);

    public:
        LibrarianProcessNode(/* args */);
        ~LibrarianProcessNode();
    };
}

#endif