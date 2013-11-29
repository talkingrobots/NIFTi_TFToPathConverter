//This program subscribes to the /tf topic and generates Marker_Array messages to create a nicer looking path

// SHANKER KESHAVDAS , DFKI , FEB 2011
// Benoit Larochelle, January 2012

#include <vector>
#include <queue>

#include <ros/ros.h>

//#include <tf/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>

#include <geometry_msgs/Twist.h>


//static const double IMMOBILITY_THRESHOLD = 0.1; // If positions are as close as this, they are considered to be the same
//static const double MIN_SIZE_SPHERE = 0.2;
//static const double MAX_SIZE_SPHERE = 0.6;
//static const int MAX_SECONDS_AT_LOCATION = 15; // After this much time at a location, the spheres do not grow anymore

bool movementIsAutonomous = false;
ros::Time timeLastCmdVel;
ros::Time timeLastVerbalCommand;

inline bool isMovementAutonomous()
{
    return movementIsAutonomous;

}

//inline bool arePositionsSimilar(tf::Transform& pos1, tf::Transform& pos2)
//{
//    //ROS_INFO("(%f, %f, %f) (%f, %f, %f) distance: %f", pos1.getOrigin().x(), pos1.getOrigin().y(), pos1.getOrigin().z(), pos2.getOrigin().x(), pos2.getOrigin().y(), pos2.getOrigin().z(), pos1.getOrigin().distance(pos2.getOrigin()));
//    return pos1.getOrigin().distance(pos2.getOrigin()) < IMMOBILITY_THRESHOLD;
//}
//
//double getSphereSize(tf::StampedTransform& lastTransform, tf::StampedTransform& currentTransform)
//{
//    const double SIZE_INCREASE_RATIO = (MAX_SIZE_SPHERE - MIN_SIZE_SPHERE) / MAX_SECONDS_AT_LOCATION;
//    
//    //ROS_INFO("Delay: %i", (currentTransform.stamp_ - lastTransform.stamp_).sec);
//    
//    double s;
//    s = SIZE_INCREASE_RATIO * (currentTransform.stamp_ - lastTransform.stamp_).sec + MIN_SIZE_SPHERE;
//    
//    s = std::min(s, MAX_SIZE_SPHERE); // Applies a maximum to the sphere size
//
////    if(s >= MAX_SIZE_SPHERE)
////        ROS_WARN("Hit MAX_SIZE %f", s);
////    else if(s <= MIN_SIZE_SPHERE)
////        ROS_WARN("Hit MIN_SIZE %f", s);
////    else
////        ROS_INFO("Size: %f", s);
//
//    return s;
//}
//
//void addSphereAtLastPosition(visualization_msgs::MarkerArray& markerArrayMsg, tf::StampedTransform& lastTransform, tf::StampedTransform& currentTransform)
//{
//    static int id = 0;
//
//    visualization_msgs::Marker sphereMarker;
//
//    sphereMarker.header.frame_id = "map";
//    sphereMarker.header.stamp = currentTransform.stamp_;
//    sphereMarker.action = visualization_msgs::Marker::ADD;
//    sphereMarker.id = id++;
//    sphereMarker.type = visualization_msgs::Marker::SPHERE;
//
//    sphereMarker.pose.orientation.w = lastTransform.getRotation().getW();
//    sphereMarker.pose.orientation.x = lastTransform.getRotation().getX();
//    sphereMarker.pose.orientation.y = lastTransform.getRotation().getY();
//    sphereMarker.pose.orientation.z = lastTransform.getRotation().getZ();
//    sphereMarker.pose.position.x = lastTransform.getOrigin().getX();
//    sphereMarker.pose.position.y = lastTransform.getOrigin().getY();
//    sphereMarker.pose.position.z = lastTransform.getOrigin().getZ();
//
//    sphereMarker.scale.x = getSphereSize(lastTransform, currentTransform);
//    sphereMarker.scale.y = sphereMarker.scale.x;
//    sphereMarker.scale.z = sphereMarker.scale.x;
//
//    sphereMarker.color.a = 1.0;
//    sphereMarker.color.r = 0.0;
//    sphereMarker.color.g = 1.0;
//    sphereMarker.color.b = 0.0;
//
//    markerArrayMsg.markers.push_back(visualization_msgs::Marker(sphereMarker));
//}
//
///**
// * Returns true if it succeeded
// */
//bool safelyLookUpTransform(tf::TransformListener& listener, tf::StampedTransform& transformStamped)
//{
//    // Container for error messages, if any
//    std::string* errorMsg = new std::string();
//
//    ros::spinOnce();
//
//    if (!listener.canTransform("/map", "/base_link", ros::Time(0), errorMsg))
//    {
//        ROS_ERROR("%s", errorMsg->c_str());
//        delete errorMsg;
//
//        return false;
//    }
//    else
//    {
//        delete errorMsg;
//    }
//
//    try
//    {
//        listener.lookupTransform("/map", "/base_link", ros::Time(0), transformStamped);
//    }
//    catch (tf::TransformException ex)
//    {
//        ROS_ERROR("%s", ex.what());
//
//        return false;
//    }
//
//    return true;
//}

void onCmdVelReceived(geometry_msgs::Twist msg)
{
    timeLastCmdVel = ros::Time::now();
}

int main(int argc, char **argv)
{

    // Initializes ROS
    ros::init(argc, argv, "TFToPathConverter_Spheres");
    ros::NodeHandle node;

    // Listens to the TFs so that we can query them at any time
    //tf::TransformListener listener;

    // Will add the pose to the path every second (no need to do it faster)
    ros::Rate rate(10);

    // Publishes the path
    //    ros::Publisher pathPublisher = node.advertise<visualization_msgs::MarkerArray > ("traveledPath_Markers_array", 1);
    //    visualization_msgs::MarkerArray markerArrayMsg;

    //tf::StampedTransform stampedTransform;



    std::queue<ros::Time> verbalCommands;

    




    //1324045949.94 start

    verbalCommands.push(ros::Time(1324046037894 / 1000.0));
    verbalCommands.push(ros::Time(1324046046352 / 1000.0));
    verbalCommands.push(ros::Time(1324046052970 / 1000.0));
    verbalCommands.push(ros::Time(1324046148059 / 1000.0));
    verbalCommands.push(ros::Time(1324046178078 / 1000.0));
    verbalCommands.push(ros::Time(1324046182703 / 1000.0));
    verbalCommands.push(ros::Time(1324046190210 / 1000.0));
    verbalCommands.push(ros::Time(1324046194035 / 1000.0));
    verbalCommands.push(ros::Time(1324046194286 / 1000.0));
    verbalCommands.push(ros::Time(1324046240075 / 1000.0));
    
    //verbalCommands.push(ros::Time(111 / 1000.0));

    

    // Waits for the first clock signal
    ros::Time::waitForValid();
    

    // Removes verbals commands that are in the past
    while (true)
    {
        ros::Duration durationUntilNextVerbalCommand = verbalCommands.front() - ros::Time::now();

        if (durationUntilNextVerbalCommand < ros::Duration(0))
        {
            ROS_WARN("Found a verbal command with a timestamp in the past. Removing it.");
            verbalCommands.pop();

            if (verbalCommands.size() == 0)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }


    // If at this point there are no more verbal commands, then just exit
    if (verbalCommands.size() == 0)
    {
        // There are no more verbal commands, so just leave it to non-autonomous
        ROS_INFO("There are no more verbal commands coming");
        return 0;
    }




    //ros::Time nextCommand = commands.front();

    //timeLastCmdVel = ros::Time::now();

    ros::Subscriber subscriberCmdVel = node.subscribe<geometry_msgs::Twist > ("/cmd_vel", 10, onCmdVelReceived);


    while (node.ok())
    {
        ros::spinOnce();

        //        if ((ros::Time::now() - timeLastCmdVel).sec >= 1)
        //        {
        //            movementIsAutonomous = false;
        //            ROS_INFO("The last cmd vel was long ago");
        //        }
        //        else
        //        {
        //            ROS_WARN("The last cmd vel was recently");
        //        }


        if (movementIsAutonomous)
        {
            if ( (ros::Time::now() - timeLastCmdVel).sec >= 1 && (ros::Time::now() - timeLastVerbalCommand).sec >= 1 )
            {
                movementIsAutonomous = false;
                ROS_INFO("movementIsAutonomous = false;");
            }
        }
        else
        {
            if (verbalCommands.size() == 0)
            {
                // There are no more verbal commands, so just leave it to non-autonomous
                ROS_INFO("There are no more verbal commands coming");
                return 0;
            }




            ROS_INFO("Size: %i", verbalCommands.size());
            ROS_INFO("Now: %i \t Next: %i", ros::Time::now().sec, verbalCommands.front().sec);
            ROS_INFO("Time until next command: %i", (verbalCommands.front() - ros::Time::now()).sec);



            if (ros::Time::now() >= verbalCommands.front())
            {
                movementIsAutonomous = true;
                ROS_INFO("movementIsAutonomous = true;");
                verbalCommands.pop();
                timeLastVerbalCommand = ros::Time::now();
            }
        }

        
        if(movementIsAutonomous)
        {
            ROS_INFO("AUTONOMOUS");
        }
        else
        {
            ROS_INFO("TELEOP");
        }
        
        rate.sleep();

    }



    //    // Waits for up to a second to ensure that the transform has been received at least once
    //    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1));
    //
    //    // This initial loop is just to set the first point (otherwise the first edge starts at 0,0)
    //    while (node.ok() && !safelyLookUpTransform(listener, stampedTransform))
    //    {
    //        // Waits a little before adding another pose to the path
    //        rate.sleep();
    //    }
    //
    //    addSphereAtLastPosition(markerArrayMsg, stampedTransform, stampedTransform);
    //
    //    tf::StampedTransform lastStampedTransform = stampedTransform;
    //
    //    while (node.ok())
    //    {
    //        if (!safelyLookUpTransform(listener, stampedTransform))
    //        {
    //            // Waits a little before adding another pose to the path
    //            rate.sleep();
    //            continue;
    //        }
    //
    //        // Does not add this position to the path if it's nearly the same as the previous one
    //        if (!arePositionsSimilar(lastStampedTransform, stampedTransform))
    //        {
    //            addSphereAtLastPosition(markerArrayMsg, lastStampedTransform, stampedTransform);
    //            lastStampedTransform = stampedTransform;
    //        }
    //
    //        pathPublisher.publish(markerArrayMsg);
    //
    //        // Waits a little before adding another pose to the path
    //        rate.sleep();
    //
    //    } // end loop

    return 0;

}