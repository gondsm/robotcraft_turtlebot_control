#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"


class TurtleController
{
private:
    // A publisher to publish messages
    ros::Publisher cmd_vel_pub;
    // A service server to stop and start the robot
    ros::ServiceServer stop_service;
    // The current linear and angular speeds
    float lin_speed;
    float ang_speed;
    // Tells us whether the turtle is moving
    bool is_running;

    geometry_msgs::Twist calculateCommand()
    {
        // Create a new message
        auto msg = geometry_msgs::Twist();

        // Fill it in
        if (this->is_running)
        {
            msg.linear.x = this->lin_speed;
            msg.angular.z = this->ang_speed;
        }

        // And give it back
        return msg;
    }

    bool flip_state(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {   
        // Flip them flags
        if (this->is_running)
            this->is_running = false;
        else
            this->is_running = true;

        // Services must always return a bool
        return true;
    }

public:
    TurtleController(){
        // Initialize ROS
        ros::NodeHandle n;
        ros::NodeHandle nh("~");

        // We're running by default
        this->is_running = true;

        // Read private params
        nh.param<float>("linear_speed", this->lin_speed, 1.0);
        nh.param<float>("angular_speed", this->ang_speed, 1.0);

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        // Create a service handler for stopping and re-starting the robot
        this->stop_service = n.advertiseService("stop_turtle", &TurtleController::flip_state, this);
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

            // Do a spinny thing
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