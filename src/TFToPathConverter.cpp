//This program subscribes to the /tf topic and generates a Path message

// SHANKER KESHAVDAS , DFKI , FEB 2011
// Benoit Larochelle, January 2012

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Polygon.h"

const double MARGIN = 0.01; // If positions are as close as 1 cm, they are considered to be the same

inline bool are3DPositionsSimilar(geometry_msgs::Point& pos1, geometry_msgs::Point& pos2)
{
    //ROS_ERROR("Inputs: %f %f %f %f", pos1.x, pos2.x, pos1.y, pos2.y);
    //ROS_ERROR("Margins: %f %f", std::abs(pos1.x - pos2.x), std::abs(pos1.y - pos2.y));
    
    return (std::abs(pos1.x - pos2.x) < MARGIN
            && std::abs(pos1.y - pos2.y) < MARGIN
            && std::abs(pos1.z - pos2.z) < MARGIN);
}

int main(int argc, char **argv)
{

    // Initializes ROS
    ros::init(argc, argv, "TFToPathConverter");
    ros::NodeHandle node;

    // Listens to the TFs so that we can query them at any time
    tf::TransformListener listener;

    // Will add the pose to the path every second (no need to do it faster)
    ros::Rate rate(1);

    // Gets the robot's base frame of reference
    std::string paramName;
    paramName.append(ROS_PACKAGE_NAME).append("/robotFrameOfReference");
    std::string robotFrameOfReference;
    bool found = node.getParam(paramName, robotFrameOfReference);
    if(found == false)
    {
        std::cerr << "Parameter \"robotFrameOfReference\" missing. Normally base_link or uav_base_link." << std::endl;
        return -1;
    }
    
    // Publishes the path
    ros::Publisher pathPublisher = node.advertise<nav_msgs::Path > ("/traveledPath", 1);
    nav_msgs::Path pathMsg;
    int sequenceNumber = 0;

    // Creates a proper header for the first message
    pathMsg.header.frame_id = "/map";
    pathMsg.header.seq = sequenceNumber;
    pathMsg.header.stamp = ros::Time::now();

    // Container for error messages, if any
    std::string* errorMsg = new std::string();

    geometry_msgs::PoseStamped lastPoseMsg;

    // Waits for up to a second to ensure that the transform has been received at least once
    listener.waitForTransform("/map", robotFrameOfReference, ros::Time(0), ros::Duration(1));

    while (node.ok())
    {
        ros::spinOnce();

        if (!listener.waitForTransform("/map", robotFrameOfReference, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), errorMsg))
        {
            ROS_ERROR("%s", errorMsg->c_str());
            delete errorMsg;
            errorMsg = new std::string();

            // Waits a little before adding another pose to the path
            rate.sleep();
            continue;
        }

        tf::StampedTransform transformStamped;

        try
        {
            listener.lookupTransform("/map", robotFrameOfReference, ros::Time(0), transformStamped);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());

            // Waits a little before adding another pose to the path
            rate.sleep();
            continue;
        }

        geometry_msgs::TransformStamped transformStampedMsg;

        // Performs a conversion to make things easier
        tf::transformStampedTFToMsg(transformStamped, transformStampedMsg);

        // Stores the pose in a pose message (for the path)
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header = transformStampedMsg.header;
        poseMsg.header.seq = ++sequenceNumber;
        poseMsg.pose.orientation = transformStampedMsg.transform.rotation;
        poseMsg.pose.position.x = transformStampedMsg.transform.translation.x;
        poseMsg.pose.position.y = transformStampedMsg.transform.translation.y;
        poseMsg.pose.position.z = transformStampedMsg.transform.translation.z;

        // Does not add this position to the path if it's nearly the same as the previous one
        if (!are3DPositionsSimilar(lastPoseMsg.pose.position, poseMsg.pose.position))
        {
            // Prepares and publishes the path message
            pathMsg.header = poseMsg.header;
            pathMsg.poses.push_back(poseMsg);
        }

        //ROS_INFO("header frame id: %s %i %f", pathMsg.header.frame_id.c_str(), pathMsg.header.seq, pathMsg.header.stamp.toSec());

        pathPublisher.publish(pathMsg);

        lastPoseMsg = poseMsg;

        // Waits a little before adding another pose to the path
        rate.sleep();

    } // end loop

    delete errorMsg;

    return 0;

}


