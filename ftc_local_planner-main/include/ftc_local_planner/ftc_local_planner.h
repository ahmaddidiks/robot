/*

Copyright (c) 2021, Ghanis Nugraha
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef FTC_LOCAL_PLANNER_FTC_PLANNER_ROS_H_
#define FTC_LOCAL_PLANNER_FTC_PLANNER_ROS_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <angles/angles.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

namespace ftc_local_planner {

    class FTCPlannerROS : public nav_core::BaseLocalPlanner {
        public:
            FTCPlannerROS();

            ~FTCPlannerROS();

            void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

            bool isGoalReached();

            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

            bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

            double max_x_vel = 0.2;
            double max_rotation_vel = 1.5;
            double min_rotation_vel = 0.2;
            double distance_accuracy = 0.05;
            double rotation_accuracy = 0.02;
            double acceleration_x = 0.1;
            double acceleration_z = 1.0;
            double sim_time = 1.0;
            double local_planner_frequence = 10;

        private:
            int state_;
            int current_point_;
            int max_point_;
            bool goal_reached_;

            ros::Publisher local_plan_pub_;
            ros::Publisher goal_pose_publisher_;
            ros::Subscriber simple_goal_sub_;
            tf2_ros::Buffer* tf_;
            costmap_2d::Costmap2DROS* costmap_;
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            std::vector<geometry_msgs::PoseStamped> local_plan_;
            geometry_msgs::PoseStamped goal_pose_;
            geometry_msgs::PoseStamped current_pose_;
            geometry_msgs::Twist last_cmd_vel_;

            double distanceBetweenPose(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
            void createLocalPlan(geometry_msgs::PoseStamped current_pose);
            double calculatePlanAngle(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped goal_pose);
            double calculateAngleError(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped goal_pose);
            void calculateNextCurrentPoint(geometry_msgs::PoseStamped current_pose);
            bool rotateToOrientation(double angle, geometry_msgs::Twist& cmd_vel, double accuracy);
            int driveToward(geometry_msgs::PoseStamped current_pose, geometry_msgs::Twist& cmd_vel);
            bool getXPose(const tf2_ros::Buffer& tf,
                        const std::vector<geometry_msgs::PoseStamped>& global_plan,
                        const std::string& global_frame,
                        geometry_msgs::PoseStamped& goal_pose,
                        int plan_point);

            void simpleGoalCallback(const geometry_msgs::PoseStamped::Ptr& goal_pose);
    };

};
#endif