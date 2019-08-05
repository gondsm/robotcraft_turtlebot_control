#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"


class TurtleController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::ServiceServer pause_server;
    ros::Subscriber pose_sub;

    turtlesim::Pose latest_pose;

    float linear_vel = 1.0;
    float angular_vel = 1.0;

    float x_min = 2.0;
    float y_min = 2.0;
    float x_max = 9.0;
    float y_max = 9.0;

    bool is_moving;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        if (this->is_moving)
        {
            // Update the control values
            commandUpdate();

            // Control code goes here
            msg.linear.x = linear_vel;
            msg.angular.z = angular_vel;    
        }
        

        return msg;
    }

    bool flip(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        this->is_moving = !(this->is_moving);

        return true;
    }

    void poseUpdate(const turtlesim::Pose::ConstPtr& msg)
    {
        this->latest_pose.x = msg->x;
        this->latest_pose.y = msg->y;
        this->latest_pose.theta = msg->theta;
        this->latest_pose.linear_velocity = msg->linear_velocity;
        this->latest_pose.angular_velocity = msg->angular_velocity;
    }

    void commandUpdate()
    {
        // If the robot is outside the box,
        // it should stop
        if (latest_pose.x < x_min || latest_pose.x > x_max || latest_pose.y < y_min || latest_pose.y > y_max)
        {
            this->linear_vel *= 1.01;
            this->angular_vel = 2.0;
        }
        else
        {
            this->linear_vel = 1.0;
            this->angular_vel = 0;
        }
        
        
    }

public:
    TurtleController(){
        // Initialize ROS
        this->n = ros::NodeHandle();
        auto priv_nh = ros::NodeHandle("~");

        this->is_moving = true;

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Create a service server
        this->pause_server = n.advertiseService("pause_service", &TurtleController::flip, this);

        // Read params
        //priv_nh.getParam("linear_vel", this->linear_vel);
        //priv_nh.getParam("angular_vel", this->angular_vel);

        // 
        this->pose_sub = n.subscribe("pose", 10, &TurtleController::poseUpdate, this);

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "talker");

    // Create our controller object and run it
    auto controller = TurtleController();
    controller.run();

    // And make good on our promise
    return 0;
}