#include <backoff_recovery/backoff_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/transform_listener.h>
#include <nav_core/parameter_magic.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Bool.h> // 추가

PLUGINLIB_EXPORT_CLASS(backoff_recovery::BackoffRecovery, nav_core::RecoveryBehavior)

namespace backoff_recovery
{
    BackoffRecovery::BackoffRecovery() : initialized_(false), should_stop_(false)
    {
        nh_ = new ros::NodeHandle();
    }

    void BackoffRecovery::initialize(std::string name, tf2_ros::Buffer *,
                                     costmap_2d::Costmap2DROS *, costmap_2d::Costmap2DROS *local_costmap)
    {
        if (!initialized_)
        {

            ros::NodeHandle private_nh("~/" + name);

            private_nh.param("backoff_distance", backoff_distance_, 0.2);
            private_nh.param("frequency", frequency_, 20.0);
            private_nh.param("vel", vel_, 0.1);

            goal_sub_ = nh_->subscribe("/move_base_simple/goal", 1,
                                       &BackoffRecovery::goalCallback, this);

            stop_sub_ = nh_->subscribe("/stop_recovery", 1,
                                       &BackoffRecovery::stopCallback, this);

            ROS_INFO("Initialized BackoffRecovery with velocity: %.2f, distance: %.2f", vel_, backoff_distance_);

            initialized_ = true;
        }
        else
        {
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }

    /////////////////////
    bool BackoffRecovery::hasGoalChanged(const geometry_msgs::PoseStamped::ConstPtr &new_goal)
    {
        double position_diff = std::hypot(
            new_goal->pose.position.x - current_goal_.pose.position.x,
            new_goal->pose.position.y - current_goal_.pose.position.y);

        if (position_diff > 0.1)
        { // 10cm 이상 차이
            current_goal_ = *new_goal;
            return true;
        }
        return false;
    }

    void BackoffRecovery::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // if (false)
        if (hasGoalChanged(msg))
        {
            should_stop_ = true;
        }
    }

    void BackoffRecovery::stopCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
        {
            should_stop_ = true;
        }
    }

    /////////////////////
    BackoffRecovery::~BackoffRecovery()
    {
        if (nh_)
        {
            delete nh_;
        }
    }

    void BackoffRecovery::runBehavior()
    {
        should_stop_ = false;
        euclidean_distance_ = 0.0;

        if (!initialized_)
        {
            ROS_ERROR("This object must be initialized before runBehavior is called.");
            return;
        }

        if (vel_ < 0)
        {
            ROS_ERROR("vel should be positive...");
            return;
        }

        ROS_WARN("Backoff recovery behavior started...");

        ros::Rate r(frequency_);
        ros::NodeHandle n;
        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        tf2_ros::Buffer tfBuffer;

        tf2_ros::TransformListener tfListener(tfBuffer);

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = -vel_;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        bool valid_tf = false;

        geometry_msgs::TransformStamped initPose;
        geometry_msgs::TransformStamped currentPose;
        tf2::Vector3 vec1, vec2;

        double diff_x;
        double diff_y;

        while (!valid_tf)
        {
            try
            {
                initPose = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0));
                valid_tf = true;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Transform lookup failed: %s", ex.what());
                ros::Duration(1.0).sleep();
            }
        }

        ROS_WARN("Backing off %.2fm", backoff_distance_);

        while (n.ok() && euclidean_distance_ < backoff_distance_)
        {
            // 새로운 목표점 수신 시 중단
            if (should_stop_)
            {
                // 정지 명령 발행
                geometry_msgs::Twist stop_cmd;
                stop_cmd.linear.x = 0;
                stop_cmd.linear.y = 0;
                stop_cmd.angular.z = 0;
                vel_pub.publish(stop_cmd);
                return;
            }
            try
            {
                currentPose = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0));

                diff_x = currentPose.transform.translation.x - initPose.transform.translation.x;
                diff_y = currentPose.transform.translation.y - initPose.transform.translation.y;

                euclidean_distance_ = std::sqrt(diff_x * diff_x + diff_y * diff_y);

                ROS_WARN_THROTTLE(0.5, "Backing off %.2fm", backoff_distance_ - euclidean_distance_);

                vel_pub.publish(cmd_vel);
            }

            catch (tf2::TransformException &ex)
            {
                ROS_WARN("Transform lookup failed: %s", ex.what());
                ros::Duration(1.0).sleep();
            }

            r.sleep();
        }
        should_stop_ = false;
    }
}