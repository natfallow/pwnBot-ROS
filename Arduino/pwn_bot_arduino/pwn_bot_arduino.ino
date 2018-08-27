
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <pwn_bot_hardware/motors.h>



#define pwm1pin 2
#define dir1pin A2
#define break1pin A3

#define pwm2pin 3
#define dir2pin A5
#define break2pin A1

#define pwm3pin 8
#define dir3pin 9
#define break3pin 10

#define pwm4pin 11
#define dir4pin 12
#define break4pin 13

#define EN A4

#define startPin A0
#define ADCvoltage A1

#define US_ServoCtrl 7   //ultrasonic pivot servo 

Servo ultraServo;

ros::NodeHandle  nh;

void servo_cb( const pwn_bot_hardware::motors& cmd_msg){
  
  if(cmd_msg.globalEN==1){
    digitalWrite(EN,LOW);
  }
  else{
    digitalWrite(EN,HIGH);
  }
  
  int tempwm=(int) cmd_msg.speed1;
  int tempdir = (int) cmd_msg.dir1;
  int tempbre = (int) cmd_msg.break1;
  
  setMotor(1,tempwm,tempdir,tempbre);
  setMotor(2,cmd_msg.speed2,cmd_msg.dir2,cmd_msg.break2);
  setMotor(3,cmd_msg.speed3,cmd_msg.dir3,cmd_msg.break3);
  setMotor(4,cmd_msg.speed4,cmd_msg.dir4,cmd_msg.break4);
  
 
}


ros::Subscriber<pwn_bot_hardware::motors> sub("motorData", servo_cb);

void setup(){
  definePins();
  
  nh.initNode();
  nh.subscribe(sub);

  ultraServo.attach(US_ServoCtrl);
}

void loop(){
  
  //setMotor(1,127,1,0);
  //setMotor(2,127,1,0);
  //setMotor(3,127,0,0);
  //setMotor(4,127,0,0);
  
  
  //analogWrite(pwm4pin,255);
 // digitalWrite(EN,LOW);
  //digitalWrite(break2pin,LOW);
  
 
  
  
  nh.spinOnce();
  delay(1);
}



void definePins(){
  //Back left motor
  pinMode(pwm1pin, OUTPUT);
  pinMode(dir1pin,OUTPUT);
  pinMode(break1pin,OUTPUT);
  
  //Back Right motor
  pinMode(pwm2pin, OUTPUT);
  pinMode(dir2pin,OUTPUT);
  pinMode(break2pin,OUTPUT);
  
  
  //Front Left Motor
  pinMode(pwm3pin, OUTPUT);
  pinMode(dir3pin,OUTPUT);
  pinMode(break3pin,OUTPUT);
  
  //Front Right motor
  pinMode(pwm4pin, OUTPUT);
  pinMode(dir4pin,OUTPUT);
  pinMode(break4pin,OUTPUT);
  
  
  //inputs
  pinMode(startPin,INPUT);
  pinMode(ADCvoltage,INPUT);
  
}



void setMotor(int motor, int pwm, int dir, int bre){
  nh.loginfo("data rec");
  switch(motor){
    case 1:
      analogWrite(pwm1pin,pwm);
      digitalWrite(dir1pin,dir);
      digitalWrite(break1pin,bre);
    break;
    case 2:
      analogWrite(pwm2pin,pwm);
      digitalWrite(dir2pin,inv(dir));
      digitalWrite(break2pin,bre);
    break;
    case 3:
      analogWrite(pwm3pin,pwm);
      digitalWrite(dir3pin,inv(dir));
      digitalWrite(break3pin,bre);
    break;
    case 4:
      analogWrite(pwm4pin,pwm);
      digitalWrite(dir4pin,inv(dir));
      digitalWrite(break4pin,bre); 
    break;
  }   
  
}

int inv(int bitb){
 if(bitb==1) return 0;
 else return 1; 
}
