#include "motorClass.h"

#include <ros.h>
#include <std_msgs/Float32.h>

// Robotzone motors
float z_actuator_gear_ratio = 721;
float z_actuator_enc_cnts_per_rev = 48.0;
float yaw_actuator_gear_ratio = 721;
float yaw_actuator_enc_cnts_per_rev = 48.0;

// create motor objects
//motorClass::motorClass(int pwmPin,int dirPin, int encPin,float gearRatio, float encCntsRev)
motorClass z_actuator =  motorClass(3,4,8,z_actuator_gear_ratio,z_actuator_enc_cnts_per_rev); 
motorClass yaw_actuator =  motorClass(5,6,9,yaw_actuator_gear_ratio,yaw_actuator_enc_cnts_per_rev);

// ros stuff
ros::NodeHandle arduino2Motor;

// publisher for testing
std_msgs::Float32 testing;
ros::Publisher testingPub("testing", &testing);

// home and open door position for actuators
int z_actuator_home_pos = 0; //TODO
int z_actuator_open_door_pos = 0; //TODO
int yaw_actuator_home_pos = 0; //TODO
int yaw_actuator_open_door_pos = 0; //TODO

void z_actuator_callback(const std_msgs::Float32& command)
{
	// go home
	if (command.data == 0)
	{
		z_actuator.setMotorPos(z_actuator_home_pos);
	}
	
	// go to open door position
	if (command.data == 1)
	{
		z_actuator.setMotorPos(z_actuator_open_door_pos);
	}
}

void yaw_actuator_callback(const std_msgs::Float32& command)
{
	// go home
	if (command.data == 0)
	{
		yaw_actuator.setMotorPos(yaw_actuator_home_pos);
	}
	
	// go to open door position
	if (command.data == 1)
	{
		yaw_actuator.setMotorPos(yaw_actuator_open_door_pos);
	}
}

// robotzone subscribers
ros::Subscriber<std_msgs::Float32> z_actuator_command_sub("z_actuator_command", &z_actuator_callback);
ros::Subscriber<std_msgs::Float32> yaw_actuator_command_sub("yaw_actuator_command", &yaw_actuator_callback);

// gripper subscriber
//ros::Subscriber<std_msgs::Float32> measurementSub("measurement2Arduino", &measurementCallback );


void setup () 
{ 
  // initialize ros
  arduino2Motor.initNode();
  
  // subscriber
  arduino2Motor.subscribe(z_actuator_command_sub);
  arduino2Motor.subscribe(yaw_actuator_command_sub);
  arduino2Motor.subscribe(measurementSub);

  // advertise testing publisher
  arduino2Motor.advertise(testingPub);

//  Serial.begin(57600);  
//  forceMotor.setMotorForce(1);

  delay(1000);
}

int printing = 0;
void loop ()
{
  if (printing > 1000)
  {  
	  // send testing
	  testing.data = forceMotor.currentCommandfd;
	  testingPub.publish(&testing);

	  // Serial.println(forceMotor.MotorForce);

	  // reset
	  printing = 0; 
  }
  
  // control robotzone motors
  z_actuator_command_sub.pos_closedLoopController();
  yaw_actuator_command_sub.pos_closedLoopController();

  printing = printing + 1;

  arduino2Motor.spinOnce();
}