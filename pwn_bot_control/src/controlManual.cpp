#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <termios.h>

#include <sstream>

#define maxVelY 0.38
#define maxVelX  0.38
#define maxVelT 1.5
#define maxVelFactor 0.1

geometry_msgs::Twist controlManual;

/* function to set control according to keys depressed on keyboard */
void updateControl(char key){

    switch(key){
	case 97:
	   controlManual.linear.x = 0.0;	   
	   controlManual.linear.y = -maxVelFactor*maxVelY;
           controlManual.angular.z = 0.0;
	   break;
	case 119:
	   controlManual.linear.x = maxVelFactor*maxVelX;
           controlManual.linear.y = 0.0;
           controlManual.angular.z = 0.0;
	   break;
	case 100:
	   controlManual.linear.x = 0.0;
           controlManual.linear.y = maxVelFactor*maxVelY;
           controlManual.angular.z = 0.0;;
	   break;
	case 115:
	   controlManual.linear.x = -maxVelFactor*maxVelX;
           controlManual.linear.y = 0.0;
           controlManual.angular.z = 0.0;
	   break;
	case 113:
           controlManual.linear.x = 0.0;
           controlManual.linear.y = 0.0;
           controlManual.angular.z = -maxVelFactor*maxVelT;
           break;
	case 101:
	   controlManual.linear.x = 0.0;
           controlManual.linear.y = 0.0;
           controlManual.angular.z = maxVelFactor*maxVelT;
  	   break;
	case 32:
	   controlManual.linear.x = 0.0;
           controlManual.linear.y = 0.0;
           controlManual.angular.z = 0.0;
	   break;
    }

    return;
}

/* try to read character from keyboard (non-blocking) */
void readKeyboard()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	int len = 1;
	char key;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");
	
        if(rv == -1){
		ROS_ERROR("select");
	}else if(rv == 0){
		;
	}else{
		read(filedesc, &key, len );
                //ROS_INFO("%d",key);
	        updateControl(key);
	}

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard");

  ros::NodeHandle n;

  ros::Publisher keyboard_pub = n.advertise<geometry_msgs::Twist>("controlManual", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    // try keyboard
    readKeyboard();
	
    // publish controls
    keyboard_pub.publish(controlManual);

    loop_rate.sleep();
  }


  return 0;
}
