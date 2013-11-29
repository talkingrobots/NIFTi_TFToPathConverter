//This program subscribes to the /tf topic and generates Marker_Array messages to create a nicer looking path

// SHANKER KESHAVDAS , DFKI , FEB 2011
// Benoit Larochelle, January 2012

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

static const double IMMOBILITY_THRESHOLD = 0.1; // If positions are as close as this, they are considered to be the same
static const double MIN_SIZE_SPHERE = 0.2;
static const double MAX_SIZE_SPHERE = 0.6;
static const int MAX_SECONDS_AT_LOCATION = 15; // After this much time at a location, the spheres do not grow anymore

static std_msgs::ColorRGBA GREEN, BLACK;

inline bool arePositionsSimilar(tf::Transform& pos1, tf::Transform& pos2)
{
    //ROS_INFO("(%f, %f, %f) (%f, %f, %f) distance: %f", pos1.getOrigin().x(), pos1.getOrigin().y(), pos1.getOrigin().z(), pos2.getOrigin().x(), pos2.getOrigin().y(), pos2.getOrigin().z(), pos1.getOrigin().distance(pos2.getOrigin()));
    return pos1.getOrigin().distance(pos2.getOrigin()) < IMMOBILITY_THRESHOLD;
}

inline double getSphereSize(tf::StampedTransform& lastTransform, tf::StampedTransform& currentTransform)
{
    const double SIZE_INCREASE_RATIO = (MAX_SIZE_SPHERE - MIN_SIZE_SPHERE) / MAX_SECONDS_AT_LOCATION;

    //ROS_INFO("Delay: %i", (currentTransform.stamp_ - lastTransform.stamp_).sec);

    double s;
    s = SIZE_INCREASE_RATIO * (currentTransform.stamp_ - lastTransform.stamp_).sec + MIN_SIZE_SPHERE;

    s = std::min(s, MAX_SIZE_SPHERE); // Applies a maximum to the sphere size

    //    if(s >= MAX_SIZE_SPHERE)
    //        ROS_WARN("Hit MAX_SIZE %f", s);
    //    else if(s <= MIN_SIZE_SPHERE)
    //        ROS_WARN("Hit MIN_SIZE %f", s);
    //    else
    //        ROS_INFO("Size: %f", s);

    return s;
}

void addSphereAtLastPosition(visualization_msgs::MarkerArray& markerArrayMsg, tf::StampedTransform& lastTransform, tf::StampedTransform& currentTransform)
{
    static int id = 1000000;

    visualization_msgs::Marker sphereMarker;

    sphereMarker.header.frame_id = "map";
    sphereMarker.header.stamp = currentTransform.stamp_;
    sphereMarker.action = visualization_msgs::Marker::ADD;
    sphereMarker.id = id++;
    sphereMarker.type = visualization_msgs::Marker::SPHERE;

    sphereMarker.pose.orientation.w = lastTransform.getRotation().getW();
    sphereMarker.pose.orientation.x = lastTransform.getRotation().getX();
    sphereMarker.pose.orientation.y = lastTransform.getRotation().getY();
    sphereMarker.pose.orientation.z = lastTransform.getRotation().getZ();
    sphereMarker.pose.position.x = lastTransform.getOrigin().getX();
    sphereMarker.pose.position.y = lastTransform.getOrigin().getY();
    sphereMarker.pose.position.z = lastTransform.getOrigin().getZ();

    sphereMarker.scale.x = getSphereSize(lastTransform, currentTransform);
    sphereMarker.scale.y = sphereMarker.scale.x;
    sphereMarker.scale.z = sphereMarker.scale.x;

    sphereMarker.color = BLACK;
    //sphereMarker.color = GREEN;

    markerArrayMsg.markers.push_back(visualization_msgs::Marker(sphereMarker));
}

/**
 * Returns true if it succeeded
 */
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
        BLACK.r = 0;
        BLACK.g = 0;
        BLACK.b = 0;
        BLACK.a = 0.2;
    }

    // Initializes ROS
    ros::init(argc, argv, "TFToPathConverter_Spheres");
    ros::NodeHandle node;

    // Listens to the TFs so that we can query them at any time
    tf::TransformListener listener;

    // Will add the pose to the path every second (no need to do it faster)
    ros::Rate rate(1);

    // Publishes the path
    ros::Publisher pathPublisher = node.advertise<visualization_msgs::MarkerArray > ("traveledPath_Markers_array", 1);
    visualization_msgs::MarkerArray markerArrayMsg;

    tf::StampedTransform stampedTransform;


    // Waits for up to a second to ensure that the transform has been received at least once
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1));

    // This initial loop is just to set the first point (otherwise the first edge starts at 0,0)
    while (node.ok() && !safelyLookUpTransform(listener, stampedTransform))
    {
        // Waits a little before adding another pose to the path
        rate.sleep();
    }

    addSphereAtLastPosition(markerArrayMsg, stampedTransform, stampedTransform);

    tf::StampedTransform lastStampedTransform = stampedTransform;

    while (node.ok())
    {
        if (!safelyLookUpTransform(listener, stampedTransform))
        {
            // Waits a little before adding another pose to the path
            rate.sleep();
            continue;
        }

        // Does not add this position to the path if it's nearly the same as the previous one
        if (!arePositionsSimilar(lastStampedTransform, stampedTransform))
        {
            addSphereAtLastPosition(markerArrayMsg, lastStampedTransform, stampedTransform);
            lastStampedTransform = stampedTransform;
        }

        pathPublisher.publish(markerArrayMsg);

        // Waits a little before adding another pose to the path
        rate.sleep();

    } // end loop

    return 0;

}