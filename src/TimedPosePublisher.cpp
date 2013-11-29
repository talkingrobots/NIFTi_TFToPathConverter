//This program subscribes to the /tf topic and generates markers for the robot's pose every 2 minutes

// Benoit Larochelle, 2012-01-09

#include <vector>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;

static const ros::Duration COGNITIVE_LOAD_INTERVAL(120); // 120 seconds because TNO's PDA beeped every two minutes to the measure cognitive load

void fillMarkerMessage_Arrow(visualization_msgs::Marker& markerMsg, geometry_msgs::PoseStamped& poseMsg)
{
    markerMsg.header = poseMsg.header;
    markerMsg.id = markerMsg.header.seq; // id of the marker
    markerMsg.type = visualization_msgs::Marker::ARROW;
    markerMsg.action = visualization_msgs::Marker::ADD;
    markerMsg.pose = poseMsg.pose;
    markerMsg.pose.position.z += 0.5; // Lifts the arrow 0.5m higher in the air (for a better view)
    
    markerMsg.scale.x = 0.55; // Makes the arrow the length of the robot
    markerMsg.scale.y = 2;
    markerMsg.scale.z = 2;
    markerMsg.color.a = 1.0;
    markerMsg.color.r = 1.0;
    markerMsg.color.g = 0.0;
    markerMsg.color.b = 0.0;
}

void fillMarkerMessage_Number(visualization_msgs::Marker& markerMsg, geometry_msgs::PoseStamped& poseMsg)
{
    markerMsg.header = poseMsg.header;
    markerMsg.id = markerMsg.header.seq + 1000000; // id of the marker
    markerMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    markerMsg.action = visualization_msgs::Marker::ADD;
    markerMsg.pose = poseMsg.pose;
    markerMsg.pose.position.z += 1.0; // Lifts the number 1.0m higher in the air (for a better view)
    
    // Converts the id to text in order to display it
    static int minutesIntoTheScenario = 0;
    char buffer [5];
    sprintf (buffer, "%d", minutesIntoTheScenario);
    markerMsg.text = buffer;
    minutesIntoTheScenario += 2; // This gets called every two minutes;
    
    markerMsg.scale.x = 0.75;
    markerMsg.scale.y = 0.75;
    markerMsg.scale.z = 0.75;
    markerMsg.color.a = 1;
    markerMsg.color.r = 1;
    markerMsg.color.g = 0.7;
    markerMsg.color.b = 0;
}

int main(int argc, char **argv)
{

    // Initializes ROS
    ros::init(argc, argv, "TimedPosePublisher");
    ros::NodeHandle nodeHandle("~"); // Looks for the parameters in the private namespace


    ////////////////////////////////
    //
    // 1) Gets the two parameters
    //
    ////////////////////////////////

    if (!nodeHandle.hasParam("startTime"))
    {
        ROS_ERROR("Ensure that you provide the start time: rosrun TFToPathConverter TimedPosePublisher _startTime:=\"123456789\"");
        return -1;
    }

    int temp;
    nodeHandle.getParam("startTime", temp);
    ros::Time startTime(temp);

    ////////////////////////////////
    //
    // 2) Waits for the TFs and the right time
    //
    ////////////////////////////////


    // Listens to the TFs so that we can query them at any time
    tf::TransformListener listener;

    ros::Publisher posePublisher = nodeHandle.advertise<geometry_msgs::Pose > ("/timedPose", 0);
    int sequenceNumber = 0;

    ros::Publisher markerPublisher = nodeHandle.advertise<visualization_msgs::Marker > ("/timedPoseMarker", 0);

    // Container for error messages, if any
    std::string* errorMsg = new std::string();

    printf("TimedPosePublisher started.\n");
    printf("Ensure that you are playing the bag with the --clock option and that you called \"rosparam set use_sim_time true\".\n");

    printf("Waiting for a valid time stamp...");
    fflush(stdout); // Forces the display of the previous line, which does not have a \n
    ros::Time::waitForValid(); // This waits for a time stamp on the /clock topic
    printf("obtained.\n");

    printf("Waiting for a transform between /map and /base_link...");
    fflush(stdout); // Forces the display of the previous line, which does not have a \n
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(15)); // Waits for up to 15 seconds to ensure that the transform has been received at least once

    if (listener.canTransform("/map", "/base_link", ros::Time(0), errorMsg))
    {
        printf("obtained.\n");
    }
    else
    {
        ROS_ERROR("%s", errorMsg->c_str());
        delete errorMsg;
        errorMsg = new std::string();

        return -1;
    }

    if (ros::Time::now() >= startTime)
    {
        ROS_WARN("This program started after the requested start time of %i", startTime.sec);
    }
    else
    {
        ROS_INFO("Will sleep until the requested start time of %i (in %i seconds)...", startTime.sec, startTime.sec - ros::Time::now().sec);
        ros::Time::sleepUntil(startTime);
    }



    ////////////////////////////////
    //
    // 3) Publishes every two minutes
    //
    ////////////////////////////////

    vector<visualization_msgs::Marker> markersMsgs;
    
    ROS_INFO("Starting...");

    ros::Time nextPublicationTime(startTime);

    while (nodeHandle.ok())
    {
        ros::spinOnce();

        if (!listener.canTransform("/map", "/base_link", ros::Time(0), errorMsg))
        {
            ROS_ERROR("%s", errorMsg->c_str());
            delete errorMsg;
            errorMsg = new std::string();

            return -1;
        }

        tf::StampedTransform transformStamped;

        try
        {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transformStamped);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());

            return -1;
        }

        geometry_msgs::TransformStamped transformStampedMsg;

        // Performs a conversion to make things easier
        tf::transformStampedTFToMsg(transformStamped, transformStampedMsg);

        // Stores the pose in a pose message (for the path)
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header = transformStampedMsg.header;
        poseMsg.header.seq = sequenceNumber;
        poseMsg.pose.orientation = transformStampedMsg.transform.rotation;
        poseMsg.pose.position.x = transformStampedMsg.transform.translation.x;
        poseMsg.pose.position.y = transformStampedMsg.transform.translation.y;
        poseMsg.pose.position.z = transformStampedMsg.transform.translation.z;


        ////////////////////////////////
        //
        // 4) Converts to /base_link, moves the arrow back a bit, and converts back to /map
        //
        ////////////////////////////////

        geometry_msgs::PoseStamped poseMsgWRTBaseLink;
        poseMsgWRTBaseLink.header.seq = sequenceNumber;
        listener.transformPose("/base_link", poseMsg, poseMsgWRTBaseLink);

        poseMsgWRTBaseLink.pose.position.x -= 0.35; // Places it at the back of the robot (just so that it does not stick out)

        geometry_msgs::PoseStamped poseMsgWRTMap;
        poseMsgWRTMap.header.seq = sequenceNumber;
        listener.transformPose("/map", poseMsgWRTBaseLink, poseMsgWRTMap);

        
        ////////////////////////////////
        //
        // 5) Creates, stores, and publishes the marker messages
        //
        ////////////////////////////////
                
        visualization_msgs::Marker markerMsg_Arrow, markerMsg_Number;
        fillMarkerMessage_Arrow(markerMsg_Arrow, poseMsgWRTMap);
        fillMarkerMessage_Number(markerMsg_Number, poseMsgWRTMap); // This makes the number attached to the tail of the arrow (to see the tip more clearly

        markersMsgs.push_back(markerMsg_Arrow);
        markersMsgs.push_back(markerMsg_Number);
        
        // Republishes all markers every time (for redundancy)
        for (vector<visualization_msgs::Marker>::const_iterator it=markersMsgs.begin() ; it < markersMsgs.end(); it++ )
        {
            markerPublisher.publish(*it);
        }

        nextPublicationTime += COGNITIVE_LOAD_INTERVAL; // Will publish again in two minutes
        sequenceNumber++;

        posePublisher.publish(poseMsg);
        ROS_INFO("Published the pose. Will now sleep until ROS time %i", nextPublicationTime.sec);

        ros::Time::sleepUntil(nextPublicationTime);

    } // end loop

    delete errorMsg;

    return 0;

}


