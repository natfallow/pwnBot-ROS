#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pwn_bot_hardware/motors.h>
#include <pwn_bot_hardware/hardware_diag_msg.h>
#include <ros/console.h>
 #include <algorithm> 


#define maxVelY 2.6
#define maxVelX  2.6
#define maxVelT 1.5


#define R1 30000
#define R2 1000
#define bottomThresh 11
#define upperThresh 12.5
#define badDooDoo 10.5

ros::Publisher pub;

bool voltageGood=true;



//Maps the velcoity to the pwm value for the motors
int mapX(float velocity){
	float speedFrac = velocity/maxVelX;
	return (int) (speedFrac*255.00);
}

int mapY(float velocity){
	float speedFrac = velocity/maxVelY;
	return (int) (speedFrac*255.00);
}

int mapT(float velocity){
	float speedFrac = velocity/maxVelT;
	return (int) (speedFrac*255.00);
}


int sign(int input){
	if(input<0){
		return 0;
	}
	else{
		return 1;
	}
}



/* The call back for the subscriber that gnerates and publishes the motor PWM, break and direction bits
*	Then publishes this data in a custom mesage form to the arduino over the topic "motorData"
*
*/
void cmdVel_cb(const geometry_msgs::Twist& vel){

	pwn_bot_hardware::motors msg; // The message to be sent to the motors on the arduino
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/world";
	
	if(voltageGood){
		msg.globalEN=1;
	}
	else{
		msg.globalEN=0;
		ROS_ERROR("Battery Voltage out of safe bounds: Motors Stopped");
	}


	if(vel.linear.x == 0 && vel.linear.y == 0 && vel.angular.z == 0){ //if no movment is needed
		msg.break1 = 1; //apply break to all motors
		msg.break2 = 1;
		msg.break3 = 1;
		msg.break4 = 1;
	}
	else{
		//release break to all motors
		msg.break1 = 0; 
		msg.break2 = 0;
		msg.break3 = 0;
		msg.break4 = 0;
	}

	float xvel=vel.linear.x;
	float yvel = vel.linear.y;
	float thetavel = vel.angular.z;

	


	int scaledX = mapX(xvel);
	int scaledY = mapY(-yvel);
	int scaledT = mapT(-thetavel);

	ROS_INFO("Scaled PWMs X: %d  Y: %d Z:%d", scaledX, scaledY, scaledT); 



	int speed1 = scaledX + scaledT + scaledY; //front left

	int speed2 = scaledX+scaledT-scaledY;//rear Left

	int speed3 = scaledX -scaledT-scaledY; //Front right

	int speed4 = scaledX-scaledT+scaledY; //rear right


	//front left motor
	msg.speed1 = std::min(abs(0.9*speed1),229);
	msg.dir1 = sign(speed1);

	//rear left motor
	msg.speed2 = std::min(abs(0.9*speed2),229);
	msg.dir2 = sign(speed2);

	//front right
	msg.speed3 = std::min(abs(speed3),255);
	msg.dir3 = sign(speed3);


	//rear right
	msg.speed4 = std::min(abs(speed4),255);
	msg.dir4 = sign(speed4);

	pub.publish(msg); // Publish the mesage to the arduino
}


/*
void hardwareDiag_cb(const pwn_bot_hardware::hardware_diag_msg& diagData){
	
	
	
	int adc = diagData.adcVal;
	float adcVolt = adc*(5.0/1024); //calulate the adc voltage from its reading
	
	float batVolt = R2/(adcVolt*(R1+R2)); //calulate the batery voltage from the ADC reading
	
	if(batVolt < bottomThresh || batVolt > upperThresh){
		batVolt = false;
	}
	else{
		batVolt = true;
	}

	if(batVolt<badDooDoo){
		ROS_ERROR("Batery Voltage Dangrously Low: Shutting down NUC");
	}
	
}
*/

int main(int argc,char **argv){

	ros::init(argc,argv, "pwn_bot_hardware");
	ros::NodeHandle nh;

	pub = nh.advertise<pwn_bot_hardware::motors>("motorData",10);
	//ros::Subscriber diag = nh.subscribe("hardware_diag",10,hardwareDiag_cb);
	
	ros::Subscriber sub = nh.subscribe("cmd_vel",10, cmdVel_cb);

	
	ros::spin();

}
