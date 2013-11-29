

#include <geometry_msgs/Twist.h>

#include "AutonomyModeAnalyzer.h"

namespace eu
{
    namespace nifti
    {
        namespace mapping
        {

            AutonomyModeAnalyzer::AutonomyModeAnalyzer(ros::NodeHandle& node)
            : autonomyMode(MANUAL)
            {
                subscriberPlanner = node.subscribe<eclipse_prolog_msgs::ActionScheduled > ("/planner/task", 10, &eu::nifti::mapping::AutonomyModeAnalyzer::onPlanningMsgReceived, this);
            }

            short AutonomyModeAnalyzer::getAutonomyMode()
            {
                return autonomyMode;
            }

            void AutonomyModeAnalyzer::onPlanningMsgReceived(const eclipse_prolog_msgs::ActionScheduledConstPtr& msg)
            {
                //        cout << "TASK: " << msg->task_name << endl;
                //        cout << "ACTION: " << msg->action << endl;
                //        cout << "STATUS: " << msg->status << endl;

                if (msg->status == "executed" || msg->status == "failed")
                {
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
                }
                else
                {
                    ROS_ERROR("Unknown planner message status: %s", msg->status.c_str());
                    assert(false);
                }


            } // end function

            
            void AutonomyModeAnalyzer::onCmdVelReceived(geometry_msgs::Twist msg)
            {
                ROS_INFO("Moved in mode %i", autonomyMode);
            }


        } // end namespace mapping
    } // end namespace nifti
} // end namespace eu


// THIS MAIN WORKS, BUT IT'S MAINLY FOR DEBUGGING
//int main(int argc, char **argv)
//{
//
//    // Initializes ROS
//    ros::init(argc, argv, "AutonomyModeAnalyzer");
//    ros::NodeHandle node;
//
//
//    eu::nifti::mapping::AutonomyModeAnalyzer a;
//
//    ros::Subscriber subscriberCmdVel = node.subscribe<geometry_msgs::Twist > ("/cmd_vel", 10, &eu::nifti::mapping::AutonomyModeAnalyzer::onCmdVelReceived, &a);
//    ros::Subscriber subscriberPlanner = node.subscribe<eclipse_prolog_msgs::ActionScheduled > ("/planner/task", 10, &eu::nifti::mapping::AutonomyModeAnalyzer::onPlanningMsgReceived, &a);
//
//    ros::spin();
//
//    return 0;
//}