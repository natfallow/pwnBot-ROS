#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>



ros::Publisher vel_pub;
ros::Subscriber joy_sub;

void joyCallback(const sensor_msgs::Joy& joy){
	double z_scale =0.9;
	double x_scale = 2.2;
	double y_scale = 2.2;
	geometry_msgs::Twist twist;
	
	if(joy.buttons[5] ==1){
  		twist.angular.z = z_scale*joy.axes[0];
  		twist.linear.x = x_scale*joy.axes[1];
		twist.linear.y = y_scale*joy.axes[3];
	}
	else{
		twist.angular.z = 0;
  		twist.linear.x = 0;
		twist.linear.y = 0;
	}

 	 vel_pub.publish(twist);
}






int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_hol_node");
  ros::NodeHandle nh;
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub = nh.subscribe("joy", 10, joyCallback);

  ros::spin();
}

