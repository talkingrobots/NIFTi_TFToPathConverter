//This program subscribes to the planner and analyzes if the robot is moving autonomously or not

// Benoit Larochelle, January 2012

#include <queue>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

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

                //                bool movementIsAutonomous = false;
                ros::Time timeLastCmdVel;
                ros::Time timeLastVerbalCommand;

                AutonomyModeAnalyzer()
                : autonomyMode(MANUAL)
                {

                }

                //                inline bool isMovementAutonomous()
                //                {
                //                    return movementIsAutonomous;
                //
                //                }

                inline short getAutonomyMode()
                {
                    return autonomyMode;

                }

                void onCmdVelReceived(geometry_msgs::Twist msg)
                {
                    timeLastCmdVel = ros::Time::now();

                    //                    std::string s;
                    //
                    //                    if (movementIsAutonomous)
                    //                    {
                    //                        s = " autonomously";
                    //                    }
                    //                    else
                    //                    {
                    //                        s = " manually";
                    //                    }
                    //
                    //                    ROS_INFO("Moved %s %i", s.c_str(), autonomyMode);

                    ROS_INFO("Moved in mode %i", autonomyMode);
                }

                void onPlanningMsgReceived(const eclipse_prolog_msgs::ActionScheduledConstPtr& msg)
                {
                    //        cout << "TASK: " << msg->task_name << endl;
                    //        cout << "ACTION: " << msg->action << endl;
                    //        cout << "STATUS: " << msg->status << endl;

                    if (msg->status == "executed" || msg->status == "failed")
                    {
                        //                        movementIsAutonomous = false;
                        autonomyMode = MANUAL;
                    }
                    else if (msg->status == "pending")
                    {
                        std::string s;
                        if (s.find("goto") != std::string::npos)
                        {
                            autonomyMode = PATH_PLANNING;
                        }
                        else
                        {
                            autonomyMode = SHORT_MOVES;
                        }

                        //                        movementIsAutonomous = true;
                    }
                    else
                    {
                        ROS_ERROR("Unknown planner message status: %s", msg->status.c_str());
                        assert(false);
                    }


                } // end function

            }; // end class

        } // end namespace mapping
    } // end namespace nifti
} // end namespace eu

int main(int argc, char **argv)
{

    // Initializes ROS
    ros::init(argc, argv, "AutonomyModeAnalyzer");
    ros::NodeHandle node;


    eu::nifti::mapping::AutonomyModeAnalyzer a;

    ros::Subscriber subscriberCmdVel = node.subscribe<geometry_msgs::Twist > ("/cmd_vel", 10, &eu::nifti::mapping::AutonomyModeAnalyzer::onCmdVelReceived, &a);
    ros::Subscriber subscriberPlanner = node.subscribe<eclipse_prolog_msgs::ActionScheduled > ("/planner/task", 10, &eu::nifti::mapping::AutonomyModeAnalyzer::onPlanningMsgReceived, &a);


    while (node.ok())
    {
        ros::spinOnce();



    }



    return 0;

}