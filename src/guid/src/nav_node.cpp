// The first try of Guidance code

// First of all include ros lib and header files of messages

#include <ros/ros.h>
#include <auto_msgs/auto_com.h>
#include <geometry_msgs/PoseStamped.h>

// let's define our variables to not forgot. later you can move them another places.
unsigned short int roll, pitch, throttle;
double x_r, y_r, z_r, x_c, y_c, z_c;
// p controller coefficients
float p_roll=5, p_pitch=5, p_throttle=5;
// hover values of roll pitch and throttle
unsigned short int hov_roll=1500, hov_pitch=1500, hov_throttle=1400, throttle_max=1900, throttle_min=900;

// definition of callback functions. Note that here callback functions
// just read some values and save them to variables. so let's go ....
// refCallback
void refCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_r = msg->pose.position.x;
    y_r = msg->pose.position.y;
    z_r = msg->pose.position.z;
}

// poseCallback
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x_c = msg->pose.position.x;
    y_c = msg->pose.position.y;
    z_c = msg->pose.position.z;
}

// now the main function 

int main(int argc, char **argv)
{

    //init the node
    ros::init(argc, argv, "Guider_Node");
    //create Node handler for working with this node and manipulate comms of publish and subscribe
    ros::NodeHandle n;
    //create Two subscriber, one for refrence point and one for current point
    ros::Subscriber ref_sub = n.subscribe("ref_pose", 1000, refCallback);
    ros::Subscriber pose_sub = n.subscribe("/aruco_single/pose", 1000, poseCallback);
    //create the publisher object to publish the roll pitch and throttle
    ros::Publisher auto_com_pub = n.advertise<auto_msgs::auto_com>("auto_command", 1000);
    ros::Rate loop_rate(10);

    // a while loop for continuously sending the roll pitch and throttle commands to the i2c_node
    while(ros::ok())
    {

        auto_msgs::auto_com command;
	/*
        // The simple P controller, this is the very first step and may it don't work good.
        // I will ok it in the test and further study and thinking...
        roll = p_roll * (x_r - x_c) + 1300;
        pitch = p_pitch * (z_r - z_c) + 1300;
        throttle = p_throttle * (y_r - y_c) + 1100;
	*/

	// now let's correct the auto commands for real quadrotor and test
	//roll
	roll = p_roll * (x_r - x_c);
	if (roll > 500){
		roll = 500;
	}
	else if (roll < -500){
		roll = -500;
	}
	else{
		// do nothing
	}
	//roll value to send mid-ware board
	roll = hov_roll + roll;
	//pitch
	pitch = p_pitch * (z_r - z_c);
	if (pitch > 500){
		pitch = 500;
	}
	else if (pitch < -500){
		pitch = -500;
	}
	else{
		//do nothing
	}
	//pitch value to send mid-ware board
	pitch = hov_pitch + pitch;
	//throttle
	throttle = p_throttle * (y_r - y_c);
	if (throttle > (throttle_max - hov_throttle)){
		throttle = throttle_max - hov_throttle;
	}
	else if (throttle < (throttle_min - hov_throttle)){
		throttle = throttle_min - hov_throttle;
	}
	else{
		// do nothing
	}
	// throttle value to send mid-ware board
	throttle = hov_throttle + throttle;

        // now assign these values to the message content to publish the message.
        //command.auto_com = {roll, pitch, throttle};
	command.auto_com.x = roll;
	command.auto_com.y = pitch;
	command.auto_com.z = throttle;
        //publish the message
        auto_com_pub.publish(command);
        ros::spinOnce(); // this is actually subscribing the refrence and currect positions via callback functions
        loop_rate.sleep();

    }

    return 0; // the end of the main function
}
