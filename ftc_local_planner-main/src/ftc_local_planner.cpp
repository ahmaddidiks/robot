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

#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include "ftc_local_planner/ftc_local_planner.h"

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlannerROS, nav_core::BaseLocalPlanner)

namespace ftc_local_planner {

    FTCPlannerROS::FTCPlannerROS() {}

    FTCPlannerROS::~FTCPlannerROS() {}

    void FTCPlannerROS::simpleGoalCallback(const geometry_msgs::PoseStamped::Ptr& finish_pose) {
        state_ = 0;
        ROS_INFO("[FTCPlannerROS] Get new goal");
        ROS_INFO("[FTCPlannerROS] State: 0");
    }

    void FTCPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        ros::NodeHandle private_nh("~/" + name);
        local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        goal_pose_publisher_ = private_nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);
        simple_goal_sub_ = private_nh.subscribe("/move_base_simple/goal", 1, &FTCPlannerROS::simpleGoalCallback, this);

        state_ = 0;
        costmap_ = costmap_ros;
        tf_ = tf;

        ROS_INFO("[FTCPlannerROS] Initialize");
    }

    bool FTCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
        ROS_INFO("[FTCPlannerROS] Get new plan");

        global_plan_ = plan;
        current_point_ = 0;
        max_point_ = 0;
        goal_reached_ = false;
        return true;
    }

    bool FTCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
        ros::Time begin = ros::Time::now();

        costmap_->getRobotPose(current_pose_);

        if (state_ == 0) {
            geometry_msgs::PoseStamped start_pose;
            getXPose(*tf_, global_plan_, costmap_->getGlobalFrameID(), start_pose, 0);
            double angle_error = calculateAngleError(current_pose_, start_pose);
            rotateToOrientation(angle_error, cmd_vel, rotation_accuracy);
            
            goal_pose_ = current_pose_;
            goal_pose_.pose.orientation = start_pose.pose.orientation;
            goal_pose_publisher_.publish(goal_pose_);

            if(std::abs(angle_error) <= rotation_accuracy) {
                state_ = 1;
                ROS_INFO("[FTCPlannerROS] State: 1");
            }
        }

        else if (state_ == 1) {
            calculateNextCurrentPoint(current_pose_);
            createLocalPlan(current_pose_);
            max_point_ = driveToward(current_pose_, cmd_vel);

            goal_pose_ = local_plan_[max_point_ - current_point_];
            goal_pose_publisher_.publish(goal_pose_);
            base_local_planner::publishPlan(local_plan_, local_plan_pub_);

            geometry_msgs::PoseStamped finish_pose;
            getXPose(*tf_, global_plan_, costmap_->getGlobalFrameID(), finish_pose, global_plan_.size()-1);
            if (abs(distanceBetweenPose(current_pose_, finish_pose)) <= distance_accuracy) {
                state_ = 2;
                ROS_INFO("[FTCPlannerROS] State: 2");
            }
        }

        else if (state_ == 2) {
            geometry_msgs::PoseStamped finish_pose;
            getXPose(*tf_, global_plan_, costmap_->getGlobalFrameID(), finish_pose, global_plan_.size()-1);
            double angle_error = calculateAngleError(current_pose_, finish_pose);
            rotateToOrientation(angle_error, cmd_vel, rotation_accuracy);

            goal_pose_ = current_pose_;
            goal_pose_.pose.orientation = finish_pose.pose.orientation;
            goal_pose_publisher_.publish(goal_pose_);

            if(std::abs(angle_error) <= rotation_accuracy) {
                state_ = 3;
                ROS_INFO("[FTCPlannerROS] State: 3");

                goal_reached_ = true;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
        }

        return true;
    }

    bool FTCPlannerROS::isGoalReached() {
        if (goal_reached_) {
            ROS_INFO("[FTCPlannerROS] Goal reached");
        }
        return goal_reached_;
    }

    double FTCPlannerROS::distanceBetweenPose(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2) {
        double x_square = pow((pose2.pose.position.x - pose1.pose.position.x), 2);
        double y_square = pow((pose2.pose.position.y - pose1.pose.position.y), 2);
        return sqrt(x_square + y_square);
    }

    void FTCPlannerROS::createLocalPlan(geometry_msgs::PoseStamped current_pose) {
        max_point_ = 0;
        local_plan_.clear();

        for (int i = current_point_; i < global_plan_.size(); i++) {
            geometry_msgs::PoseStamped x_pose;
            getXPose(*tf_, global_plan_, costmap_->getGlobalFrameID(), x_pose, i);
            double distance = distanceBetweenPose(current_pose_, x_pose);

            if(distance > (max_x_vel * sim_time)) break;

            geometry_msgs::PoseStamped pose = x_pose;
            local_plan_.push_back(pose);
            max_point_ = i;
        }

        for (int i = (max_point_ - current_point_); i >= 0; i--) {
            double angle = calculatePlanAngle(current_pose, local_plan_[i]);
            max_point_ = i + current_point_;

            if(fabs(angle) < (max_rotation_vel * sim_time)) break;
        }
    }

    double FTCPlannerROS::calculatePlanAngle(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped goal_pose) {
        double robot_angle = tf2::getYaw(current_pose.pose.orientation);
        double angle = atan2(goal_pose.pose.position.y - current_pose.pose.position.y, goal_pose.pose.position.x - current_pose.pose.position.x);
        return angles::shortest_angular_distance(robot_angle, angle);
    }

    double FTCPlannerROS::calculateAngleError(geometry_msgs::PoseStamped current_pose, geometry_msgs::PoseStamped goal_pose) {
        double robot_angle = tf2::getYaw(current_pose.pose.orientation);
        double goal_angle = tf2::getYaw(goal_pose.pose.orientation);
        return angles::shortest_angular_distance(robot_angle, goal_angle);
    }

    void FTCPlannerROS::calculateNextCurrentPoint(geometry_msgs::PoseStamped current_pose) {
        double last_distance = 999999;

        for (int i = current_point_; i < global_plan_.size(); i++) {
            geometry_msgs::PoseStamped x_pose;
            getXPose(*tf_, global_plan_, costmap_->getGlobalFrameID(), x_pose, i);
            double distance = fabs(distanceBetweenPose(current_pose, x_pose));
            
            if (last_distance < distance) break;

            current_point_ = i;
            last_distance = distance;
        }
    }

    bool FTCPlannerROS::rotateToOrientation(double angle_error, geometry_msgs::Twist& cmd_vel, double accuracy) {
        if((last_cmd_vel_.linear.x - 0.1)  >= 0) {
            last_cmd_vel_.linear.x = last_cmd_vel_.linear.x - 0.1;
            cmd_vel.linear.x = last_cmd_vel_.linear.x;
        }

        if(fabs(angle_error) > accuracy) {
            // Slow down
            if(fabs(angle_error) * acceleration_z <= max_rotation_vel) {
                if(angle_error < 0) {
                    if(last_cmd_vel_.angular.z >= -min_rotation_vel) {
                        last_cmd_vel_.angular.z = -min_rotation_vel;
                        cmd_vel.angular.z = last_cmd_vel_.angular.z;
                    }
                    else {
                        last_cmd_vel_.angular.z = last_cmd_vel_.angular.z + acceleration_z/local_planner_frequence;
                        cmd_vel.angular.z = last_cmd_vel_.angular.z;
                    }
                }
                if(angle_error > 0) {
                    if(last_cmd_vel_.angular.z  <= min_rotation_vel) {
                        last_cmd_vel_.angular.z = min_rotation_vel;
                        cmd_vel.angular.z = last_cmd_vel_.angular.z;
                    }
                    else {
                        last_cmd_vel_.angular.z = last_cmd_vel_.angular.z - acceleration_z/local_planner_frequence;
                        cmd_vel.angular.z = last_cmd_vel_.angular.z;
                    }
                }
            }

            else {
                // Speed up
                if(fabs(last_cmd_vel_.angular.z) < max_rotation_vel) {
                    if(angle_error < 0) {
                        last_cmd_vel_.angular.z = last_cmd_vel_.angular.z - acceleration_z/local_planner_frequence;

                        if(fabs(last_cmd_vel_.angular.z) > max_rotation_vel) {
                            last_cmd_vel_.angular.z = - max_rotation_vel;
                        }
                        cmd_vel.angular.z = last_cmd_vel_.angular.z;
                    }
                    if(angle_error > 0) {
                        last_cmd_vel_.angular.z = last_cmd_vel_.angular.z + acceleration_z/local_planner_frequence;

                        if(fabs(last_cmd_vel_.angular.z) > max_rotation_vel) {
                            last_cmd_vel_.angular.z = max_rotation_vel;
                        }

                        cmd_vel.angular.z = last_cmd_vel_.angular.z;
                    }
                }
                else {
                    cmd_vel.angular.z = last_cmd_vel_.angular.z;
                }
            }

            return true;
        }

        else {
            last_cmd_vel_.angular.z = 0;
            cmd_vel.angular.z = 0;
            return false;
        }
    }

    int FTCPlannerROS::driveToward(geometry_msgs::PoseStamped current_pose, geometry_msgs::Twist& cmd_vel) {
        double distance = 0;
        double angle = 0;
        int max_point = 0;

        double cmd_vel_linear_x_old = last_cmd_vel_.linear.x;
        double cmd_vel_angular_z_old = last_cmd_vel_.angular.z;

        geometry_msgs::PoseStamped x_pose;
        x_pose = local_plan_.at(max_point_ - current_point_);

        distance = distanceBetweenPose(current_pose_, x_pose);
        angle = calculatePlanAngle(current_pose, local_plan_[max_point_ - current_point_]);

        if ((distance/sim_time) > max_x_vel) {
            last_cmd_vel_.linear.x = max_x_vel;
        }
        else {
            last_cmd_vel_.linear.x = (distance/sim_time);
        }

        if(fabs(angle/sim_time) > max_rotation_vel) {
            last_cmd_vel_.angular.z = max_rotation_vel;
        }
        else {
            last_cmd_vel_.angular.z = (angle/sim_time);
        }

        if (last_cmd_vel_.linear.x > cmd_vel_linear_x_old + acceleration_x/local_planner_frequence) {
            last_cmd_vel_.linear.x = cmd_vel_linear_x_old + acceleration_x/local_planner_frequence;
        }
        else {
            if (last_cmd_vel_.linear.x < cmd_vel_linear_x_old - acceleration_x/local_planner_frequence) {
                last_cmd_vel_.linear.x = cmd_vel_linear_x_old - acceleration_x/local_planner_frequence;
            }
            else {
                last_cmd_vel_.linear.x = cmd_vel_linear_x_old;
            }
        }

        if(fabs(last_cmd_vel_.angular.z) > fabs(cmd_vel_angular_z_old)+fabs(acceleration_z/local_planner_frequence)) {
            if(last_cmd_vel_.angular.z < 0) {
                last_cmd_vel_.angular.z = cmd_vel_angular_z_old - acceleration_z/local_planner_frequence;
            }
            else {
                last_cmd_vel_.angular.z = cmd_vel_angular_z_old + acceleration_z/local_planner_frequence;
            }
        }

        if(last_cmd_vel_.angular.z < 0 && cmd_vel_angular_z_old > 0) {
            if( fabs(last_cmd_vel_.angular.z - cmd_vel_angular_z_old) > fabs(acceleration_z/local_planner_frequence)) {
                last_cmd_vel_.angular.z = cmd_vel_angular_z_old - acceleration_z/local_planner_frequence;
            }
        }

        if(last_cmd_vel_.angular.z > 0 && cmd_vel_angular_z_old < 0) {
            if( fabs(last_cmd_vel_.angular.z - cmd_vel_angular_z_old) > fabs(acceleration_z/local_planner_frequence)) {
                last_cmd_vel_.angular.z = cmd_vel_angular_z_old + acceleration_z/local_planner_frequence;
            }
        }

        if (last_cmd_vel_.angular.z > max_rotation_vel) {
            last_cmd_vel_.angular.z = max_rotation_vel;
        }
        if (last_cmd_vel_.angular.z < -max_rotation_vel) {
            last_cmd_vel_.angular.z = -max_rotation_vel;
        }
        if (last_cmd_vel_.linear.x >  max_x_vel) {
            last_cmd_vel_.linear.x = max_x_vel;
        }

        cmd_vel = last_cmd_vel_;

        return max_point_;
    }

    bool FTCPlannerROS::getXPose(const tf2_ros::Buffer& tf,
                  const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const std::string& global_frame,
                  geometry_msgs::PoseStamped& goal_pose,
                  int plan_point)
    {
        if (global_plan.empty())
        {
            ROS_ERROR("Received plan with zero length");
            return false;
        }
        if(plan_point >= (int)global_plan.size())
        {
            ROS_ERROR("Goal_functions: Plan_point %d to big. Plan size: %lu",plan_point, global_plan.size());
            return false;
        }

        const geometry_msgs::PoseStamped& plan_goal_pose = global_plan.at(plan_point);
        try
        {
            tf.canTransform(global_frame, ros::Time::now(),
                            plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                            plan_goal_pose.header.frame_id, ros::Duration(0.5));  

            geometry_msgs::TransformStamped transform = tf.lookupTransform(global_frame, ros::Time(),
                                                                    plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                                                                    plan_goal_pose.header.frame_id, ros::Duration(0.5));

            tf2::doTransform(plan_goal_pose, goal_pose, transform);
            goal_pose.header.frame_id = global_frame;
        }
        catch(tf2::LookupException& ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch(tf2::ConnectivityException& ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch(tf2::ExtrapolationException& ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }
        return true;
    }
}