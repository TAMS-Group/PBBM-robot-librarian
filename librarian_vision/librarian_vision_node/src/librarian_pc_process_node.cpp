#include <librarian_pc_process_node.h>

namespace librarian_vision
{
    const int LibrarianProcessNode::FIELD_BYTES[8] = {1, 1, 2, 2, 4, 4, 4, 8};

    LibrarianProcessNode::LibrarianProcessNode()
    {
        ros::NodeHandle nh, pnh("~");
        server = nh.advertiseService("librarian_pc_process", &LibrarianProcessNode::processAction, this);
        point_cloud_sub = nh.subscribe("/librarian/points2", 1, &LibrarianProcessNode::receivePointCloud, this);
    }

    LibrarianProcessNode::~LibrarianProcessNode() {}

    void LibrarianProcessNode::receivePointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Cache the point cloud
        pcl::fromROSMsg(*msg, pcl);
        initialized = true;
    }

    bool LibrarianProcessNode::processAction(librarian_vision_node::LibrarianPcProcess::Request &req, librarian_vision_node::LibrarianPcProcess::Response &res)
    {
        if (!initialized)
        {
            ROS_INFO_STREAM("No Point cloud available!");
            return false;
        }

        // ROS_INFO_STREAM("PC: " << pcl.width << "x" << pcl.height << " ordered:" << pcl.isOrganized());

        float x_sum = 0.0;
        float y_sum = 0.0;
        float z_sum = 0.0;
        int count = 0;

        for (int i = 0; i < req.xIndices.size(); i++)
        {
            // pcl::PointXYZRGB p = pcl.at(req.xIndices[i], req.yIndices[i]);
            pcl::PointXYZRGB p = pcl.points[req.xIndices[i] + req.yIndices[i] * req.width];

            float xx = p.x;
            float yy = p.y;
            float zz = p.z;

            if (isfinite(xx) && isfinite(yy) && isfinite(zz))
            {
                x_sum += xx;
                y_sum += yy;
                z_sum += zz;
                count++;
            }
        }

        res.pose.position.x = x_sum / (float)count;
        res.pose.position.y = y_sum / (float)count;
        res.pose.position.z = z_sum / (float)count;

        return true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "librarian_pc_process_node");
    librarian_vision::LibrarianProcessNode node;
    ros::spin();
    return 0;
}