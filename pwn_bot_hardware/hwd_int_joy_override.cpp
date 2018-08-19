#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pwn_bot_hardware/motors.h>
#include <pwn_bot_hardware/hardware_diag_msg.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <algorithm> 



#define maxVelY 0.38
#define maxVelX  0.38
#define maxVelT 1.5


#define R1 30000
#define R2 1000
#define bottomThresh 11
#define upperThresh 12.5
#define badDooDoo 10.5

ros::Publisher pub;
ros::Subscriber keyboard_sub ;

bool voltageGood=true;

bool button=false;
float xVelJoy;
float yVelJoy;
float zVelJoy;

void streamCombiner();


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

float biggestAmp(float a, float b){

	if(std::abs(a)>std::abs(b)){
		return a;	
	}
	else{
		return b;
	}
}

/*void joyCallback(const sensor_msgs::Joy& joy){
	double z_scale =0.9;
	double x_scale = 0.38;
	double y_scale = 0.38;
	if(joy.buttons[5] ==1){
		button=true;
  		zVelJoy = z_scale*joy.axes[0];
  		xVelJoy = x_scale*biggestAmp(joy.axes[1],joy.axes[4]);
		yVelJoy = y_scale*joy.axes[3];
	}
	else{
		button=false;
	}
	streamCombiner();
}*/

float xVelCmd;
float yVelCmd;
float zVelCmd;

/* callback function for SLAM subscriber */
/*void cmdVel_cb(const geometry_msgs::Twist& vel){
		xVelCmd=  vel.linear.x;
		yVelCmd = vel.linear.y;
		zVelCmd = vel.angular.z;	
		streamCombiner();	
}*/


/* callback function for keyboard subscriber */
void controlManualCallback(const geometry_msgs::Twist& vel){
	xVelCmd = vel.linear.x;
	yVelCmd = vel.linear.y;
	zVelCmd = vel.angular.z;
	streamCombiner();
}


/* Takes the /cmd_vel velocitys and velocities from manual control and publishes the 'winning' msg to the motors as pwm and directions bits.*/
void streamCombiner(){
	pwn_bot_hardware::motors msg; // The message to be sent to the motors on the arduino
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/world";
	
	float xvel;
	float yvel;
	float thetavel;
	if(button){
		xvel=xVelJoy;
		yvel = yVelJoy;
		thetavel = zVelJoy;
	}
	else{
		xvel=xVelCmd;
		yvel=yVelCmd;
		thetavel=zVelCmd;	
	}

	int scaledX = mapX(xvel);
	int scaledY = mapY(-yvel);
	int scaledT = mapT(-thetavel);
	ROS_INFO("Scaled PWMs X: %d  Y: %d Z:%d", scaledX, scaledY, scaledT); 
	int speed1 = scaledX + scaledT + scaledY; //front left
	int speed2 = scaledX + scaledT - scaledY;//rear Left
	int speed3 = scaledX - scaledT - scaledY; //Front right
	int speed4 = scaledX - scaledT + scaledY; //rear right
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

	// publish motor data to arduino
	pub = nh.advertise<pwn_bot_hardware::motors>("motorData",10);
	
	// subscribe to autonomous control
	//ros::Subscriber sub = nh.subscribe("cmd_vel",10, cmdVel_cb);

	// subscribe to manual controls through keyboard
	keyboard_sub = nh.subscribe("controlManual", 10, controlManualCallback);
	
	ros::spin();

}
