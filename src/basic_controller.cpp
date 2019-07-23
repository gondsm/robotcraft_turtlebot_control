#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Spawn.h"


class TurtleController
{
private:
    // The current nodehandle
    ros::NodeHandle n;
    // A publisher to publish messages
    ros::Publisher cmd_vel_pub;
    // A service server to stop and start the robot
    ros::ServiceServer stop_service;
    // The current linear and angular speeds
    float lin_speed;
    float ang_speed;
    // Tells us whether the turtle is moving
    bool is_running;
    // The name of the turtle we're controlling
    std::string turtle_name;

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

    void spawn_turtle()
    {
        // Create a service client real quick
        ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("spawn");

        // Call the service
        turtlesim::Spawn srv;
        srv.request.x = 2.0;
        srv.request.y = 2.0;
        srv.request.name = this->turtle_name;
        client.call(srv);
    }

public:
    TurtleController(){
        // Initialize ROS
        this->n = ros::NodeHandle();
        ros::NodeHandle nh("~");

        // We're running by default
        this->is_running = true;

        // Read private params
        nh.param<float>("linear_speed", this->lin_speed, 1.0);
        nh.param<float>("angular_speed", this->ang_speed, 1.0);
        nh.param<std::string>("spawn_turtle_name", this->turtle_name, "");

        if (this->turtle_name.length() > 1)
            // Give turtlesim a few seconds to wake up
            ros::Duration(5).sleep();
            this->spawn_turtle();

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