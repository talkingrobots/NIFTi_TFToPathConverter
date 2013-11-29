//This program subscribes to the /tf topic and generates Marker_Array messages to create a nicer looking path

// SHANKER KESHAVDAS , DFKI , FEB 2011
// Benoit Larochelle, January 2012

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "AutonomyModeAnalyzer.h"

static const double MARGIN = 0.01; // If positions are as close as 1 cm, they are considered to be the same

static std_msgs::ColorRGBA GREEN, ORANGE, RED;

inline bool are2DPositionsSimilar(geometry_msgs::Point& pos1, geometry_msgs::Point& pos2)
{
    //ROS_ERROR("Inputs: %f %f %f %f", pos1.x, pos2.x, pos1.y, pos2.y);
    //ROS_ERROR("Margins: %f %f", std::abs(pos1.x - pos2.x), std::abs(pos1.y - pos2.y));

    return (std::abs(pos1.x - pos2.x) < MARGIN
            && std::abs(pos1.y - pos2.y) < MARGIN);
}

double get2DDistance(geometry_msgs::Point& vertex0, geometry_msgs::Point& vertex1)
{
    return std::sqrt(std::pow(std::abs(vertex1.x - vertex0.x), 2) + std::pow(std::abs(vertex1.y - vertex0.y), 2));
}

double getSegmentSize(geometry_msgs::Point& vertex0, geometry_msgs::Point& vertex1)
{
    const double MAX_SIZE = 0.5;
    double s;
    s = 0.01 / get2DDistance(vertex0, vertex1);
    s = std::min(s, MAX_SIZE);
    return s;
}

void addEdge(visualization_msgs::MarkerArray& markerArrayMsg, geometry_msgs::Point& vertex0, geometry_msgs::Point& vertex1, short autonomyMode)
{
    static int id = 0;

    visualization_msgs::Marker edgeMarker;

    edgeMarker.header.frame_id = "map";
    edgeMarker.header.stamp = ros::Time::now();
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.id = id++;
    edgeMarker.type = visualization_msgs::Marker::LINE_STRIP;
    edgeMarker.scale.x = 0.1;
    edgeMarker.scale.y = 0.1;
    edgeMarker.scale.z = 0.1;

    switch (autonomyMode)
    {
        case eu::nifti::mapping::AutonomyModeAnalyzer::MANUAL:
            edgeMarker.color = GREEN;
            break;
        case eu::nifti::mapping::AutonomyModeAnalyzer::SHORT_MOVES:
            edgeMarker.color = ORANGE;
            break;
        case eu::nifti::mapping::AutonomyModeAnalyzer::PATH_PLANNING:
            edgeMarker.color = RED;
            break;
    }

    edgeMarker.points.push_back(vertex0);
    edgeMarker.points.push_back(vertex1);

    markerArrayMsg.markers.push_back(visualization_msgs::Marker(edgeMarker));
}

bool safelyLookUpTransform(tf::TransformListener& listener, tf::StampedTransform& transformStamped)
{
    // Container for error messages, if any
    std::string* errorMsg = new std::string();

    ros::spinOnce();

    if (!listener.canTransform("/map", "/base_link", ros::Time(0), errorMsg))
    {
        ROS_ERROR("%s", errorMsg->c_str());
        delete errorMsg;

        return false;
    }
    else
    {
        delete errorMsg;
    }

    try
    {
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transformStamped);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());

        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    {
        GREEN.r = 0;
        GREEN.g = 1;
        GREEN.b = 0;
        GREEN.a = 1;
        ORANGE.r = 1;
        ORANGE.g = 0.7;
        ORANGE.b = 0;
        ORANGE.a = 1;
        RED.r = 1;
        RED.g = 0;
        RED.b = 0;
        RED.a = 1;
    }

    // Initializes ROS
    ros::init(argc, argv, "TFToPathConverter_LineStrips");
    ros::NodeHandle node;

    // Listens to the TFs so that we can query them at any time
    tf::TransformListener listener;

    // Will add the pose to the path every second (no need to do it faster)
    ros::Rate rate(1);

    // Publishes the path
    ros::Publisher pathPublisher = node.advertise<visualization_msgs::MarkerArray > ("traveledPath_Markers_array", 1);
    visualization_msgs::MarkerArray markerArrayMsg;
    int sequenceNumber = 0;

    tf::StampedTransform transformStamped;

    geometry_msgs::PoseStamped lastPoseMsg;

    // Listens to the planner to decide the color of the line
    eu::nifti::mapping::AutonomyModeAnalyzer a(node);

    // Waits for up to a second to ensure that the transform has been received at least once
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1));



    // This initial loop is just to set the first point (otherwise the first edge starts at 0,0)
    while (node.ok())
    {
        if (!safelyLookUpTransform(listener, transformStamped))
        {
            // Waits a little before adding another pose to the path
            rate.sleep();
            continue;
        }

        lastPoseMsg.pose.position.x = transformStamped.getOrigin().getX();
        lastPoseMsg.pose.position.y = transformStamped.getOrigin().getY();
        lastPoseMsg.pose.position.z = transformStamped.getOrigin().getZ();

        break;
    }


    while (node.ok())
    {
        if (!safelyLookUpTransform(listener, transformStamped))
        {
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
        poseMsg.header.seq = sequenceNumber++;
        poseMsg.pose.orientation = transformStampedMsg.transform.rotation;
        poseMsg.pose.position.x = transformStampedMsg.transform.translation.x;
        poseMsg.pose.position.y = transformStampedMsg.transform.translation.y;
        poseMsg.pose.position.z = transformStampedMsg.transform.translation.z;

        // Does not add this position to the path if it's nearly the same as the previous one
        if (!are2DPositionsSimilar(lastPoseMsg.pose.position, poseMsg.pose.position))
        {
            addEdge(markerArrayMsg, lastPoseMsg.pose.position, poseMsg.pose.position, a.getAutonomyMode());
        }

        pathPublisher.publish(markerArrayMsg);

        lastPoseMsg = poseMsg;

        // Waits a little before adding another pose to the path
        rate.sleep();

    } // end loop

    return 0;

}

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "slam_karto");
//
//  SlamKarto kn;
//
//  ros::spin();
//
//  return 0;
//}

