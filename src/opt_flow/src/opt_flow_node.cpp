// this is node for determination of velocity of quadrotor
// from opticalflow

// inclustions

#include<ros/ros.h>
#include<opencv_apps/FlowArrayStamped.h>
#include<opencv_apps/Flow.h>
#include<opencv_apps/Point2D.h>
#include<std_srvs/Empty.h>
#include<vector>
#include<numeric>

using namespace std;

// Global variables
int res_thresh = 30;

// a class for reading the opticalflow data

class listener
{
    public:
        vector<opencv_apps::Flow> flow_message;
        // callback function
        void callback(const opencv_apps::FlowArrayStamped::ConstPtr& msg)
        {
            flow_message = msg->flow;
        }

};

//main function

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opt_flow");
    ros::NodeHandle n;
    listener list;
    ros::Subscriber sub = n.subscribe("/lk_flow/flows", 1, &listener::callback, &list);
    ros::Publisher vel_pub = n.advertise<opencv_apps::Point2D>("/optical_velocity", 10);
    ros::Rate loop_rate(10);
    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/lk_flow/initialize_points");
    std_srvs::Empty emp;
    opencv_apps::Point2D vel_message;
    // a counter for the bug of no feature point at the 
    // start of this node.
    int count = 0;

    while(ros::ok())
    {
        count += 1;

        if(list.flow_message.size() < res_thresh && count > 4)
        {
            if(client.call(emp))
            {
                ROS_INFO("The points were reinitialized!");
            }
            else
            {
                ROS_INFO("can't call the service!!!");
                return 1;
            }
        }
        else
        {
            // determination of mean x and y velocity of quadrotor and 
            // publishing it to a topic for use in guidance code
            // vectors of x velocity of feature points
            vector<double> x_vel, y_vel;
 
                
            // determining the x_vel and y_vel from the flow_message
            // property
            for (size_t i=0; i < list.flow_message.size(); i++)
            {
                x_vel.push_back(list.flow_message[i].velocity.x);
                y_vel.push_back(list.flow_message[i].velocity.y);
            }
            vel_message.x = accumulate(x_vel.begin(), x_vel.end(), 0.0) / x_vel.size();
            vel_message.y = accumulate(y_vel.begin(), y_vel.end(), 0.0) / y_vel.size();
            vel_pub.publish(vel_message);
                
        }


        

        ros::spinOnce();
        loop_rate.sleep();

    }//end of ros while

return 0;
}
