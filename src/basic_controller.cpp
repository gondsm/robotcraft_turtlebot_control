#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"


class TurtleController
{
private:
    ros::Publisher cmd_vel_pub;

    float lin_speed;
    float ang_speed;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        msg.linear.x = this->lin_speed;
        msg.angular.z = this->ang_speed;
        return msg;
    }

public:
    TurtleController(){
        // Initialize ROS
        ros::NodeHandle n;
        ros::NodeHandle nh("~");

        // Read private params
        nh.param<float>("linear_speed", this->lin_speed, 1.0);
        nh.param<float>("angular_speed", this->ang_speed, 1.0);

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
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