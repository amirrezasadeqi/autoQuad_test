// The first try of Guidance code

// First of all include ros lib and header files of messages

#include <ros/ros.h>
#include <auto_msgs/auto_com.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv_apps/Point2D.h>

// define global variables for detecting which the marker is lost
ros::Time ref_time, current_time;
// setting the loss time duration threshold to 0.3 second
ros::Duration loss_time, loss_thresh_time(0, 300000000);

// let's define our variables to not forgot. later you can move them another places.
unsigned short int roll, pitch, throttle;
double x_r, y_r, z_r, x_c, y_c, z_c, x_vel, y_vel;
// p controller coefficients of navigation controller
float p_roll=20, p_pitch=20, p_throttle=10;
//p controller coefficients of opticalflow stabilization controller
float p_vx = -20, p_vy = -20;
// hover values of roll pitch and throttle
unsigned short int hov_roll=1503, hov_pitch=1503, hov_throttle=1569, throttle_max=1580, throttle_min=1560;

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
    //update the time which last position data recieved
    ref_time = ros::Time::now();
    // reading the data
    x_c = msg->pose.position.x;
    y_c = msg->pose.position.y;
    z_c = msg->pose.position.z;
}

//opt_callback
void opt_callback(const opencv_apps::Point2D::ConstPtr& msg)
{
	// reading the velocity which comes from opticalflow
	x_vel = msg->x;
	y_vel = msg->y;
}

// now the main function 

int main(int argc, char **argv)
{

    //init the node
    ros::init(argc, argv, "Guider_Node");
    //create Node handler for working with this node and manipulate comms of publish and subscribe
    ros::NodeHandle n;
	//create a subscriber for velocity which comes from opticalflow
	ros::Subscriber optflow_sub = n.subscribe("/optical_velocity", 10, opt_callback);
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
	//see if the data is lost or not
	current_time = ros::Time::now();
	loss_time = current_time - ref_time;
	if(loss_time < loss_thresh_time)
	{
		// now let's correct the auto commands for real quadrotor and test
		//roll
		roll = p_roll * (x_r - x_c);
		if (roll > 10){
			roll = 10;
		}
		else if (roll < -10){
			roll = -10;
		}
		else{
			// do nothing
		}
		//roll value to send mid-ware board
		roll = hov_roll + roll;
		//pitch
		pitch = p_pitch * (z_r - z_c);
		if (pitch > 10){
			pitch = 10;
		}
		else if (pitch < -10){
			pitch = -10;
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
	}
	else
	{
		// this is for when the marker is lost and we must quadrotor stop at the point
		//roll = hov_roll;
		//pitch = hov_pitch;
		//throttle = hov_throttle;

		// let's test for using the opticalflow and holding quadrotor at a point
		//roll
		roll = p_vx * (-x_vel);
		if (roll > 10){
			roll = 10;
		}
		else if (roll < -10){
			roll = -10;
		}
		else{
			// do nothing
		}
		//roll value to send mid-ware board
		roll = hov_roll + roll;
		//pitch
		pitch = p_vy * (-y_vel);
		if (pitch > 10){
			pitch = 10;
		}
		else if (pitch < -10){
			pitch = -10;
		}
		else{
			//do nothing
		}
		//pitch value to send mid-ware board
		pitch = hov_pitch + pitch;	
		//because we use altitude hold mode of flight we use the hover
		//value of throttle to hode the alltitude. in future we use this 
		//with p controller and the data of ultrasonic sensor.
		throttle = hov_throttle;
	}
	
        // now assign these values to the message content to publish the message.
        //command.auto_com = {roll, pitch, throttle};
	command.auto_com.x = roll;
	command.auto_com.y = pitch;
	command.auto_com.z = throttle;
        //publish the message
        auto_com_pub.publish(command);
        //we use spinOnce because we are in while loop and every spinOnce wait just for one coming
        //messages.
        ros::spinOnce(); // this is actually subscribing the refrence and currect positions via callback functions
        loop_rate.sleep();

    }

    return 0; // the end of the main function
}
