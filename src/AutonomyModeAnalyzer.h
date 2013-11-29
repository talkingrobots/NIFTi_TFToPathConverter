//This program subscribes to the planner and analyzes if the robot is moving autonomously or not

// Benoit Larochelle, January 2012

#include <ros/ros.h>

#include "eclipse_prolog_msgs/ActionScheduled.h"

namespace eu
{
    namespace nifti
    {
        namespace mapping
        {

            class AutonomyModeAnalyzer
            {
            public:

                static const short MANUAL = 0;
                static const short SHORT_MOVES = 1;
                static const short PATH_PLANNING = 2;

                short autonomyMode;

                AutonomyModeAnalyzer(ros::NodeHandle& node);
                
                short getAutonomyMode();
                
            protected:
                void onPlanningMsgReceived(const eclipse_prolog_msgs::ActionScheduledConstPtr& msg);

                // Just for debugging
                void onCmdVelReceived(geometry_msgs::Twist msg);
                
            protected:
                ros::Subscriber subscriberPlanner;
                
            }; // end class

        } // end namespace mapping
    } // end namespace nifti
} // end namespace eu
