#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wheel_odom/Speed.h"
#include "wheel_odom/DiffDriveOdomConfig.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sstream>
#include <math.h>

class SubscribeAndPublishRadar
{
public:
  SubscribeAndPublishRadar()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/radar_twist", 1);
	twist.header.frame_id="odom_link";
	seq = 0;
    //Topic you want to subscribe
    sub_ = n_.subscribe("/radar", 1, &SubscribeAndPublishRadar::callback, this);
  }

  void callback(const wheel_odom::Speed& input)
  {
    
    
	twist.header.stamp = ros::Time::now();
	
    //covariances are three values depending on the result
    // only reads zero for -0.53 to 0.53(0.15m/s), so 0.15^2 = 0.0225
    if (input.speed==0){cov=0.0225;}
    // 3% for 3.2kph(0.88m/s) and above
    else if (input.speed>=0.88){var=(input.speed*0.035);cov=var*var;}
    // 5% for .53(0.15m/s) to 3.2kph
    else if (input.speed>=0.15){var=(input.speed*0.055);cov=var*var;}
    
    
    twist.header.seq = seq++;
    twist.twist.covariance[0]=cov;
    twist.twist.twist.linear.x = input.speed;
    
    pub_.publish(twist);
  }

private:
  geometry_msgs::TwistWithCovarianceStamped twist;
  ros::NodeHandle  n_; 
  ros::Publisher   pub_;
  ros::Subscriber  sub_;
  uint32_t seq;
  float cov;
  float var;
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "radar_twist");

  //Create an object of class SubscribeAndPublish that will take care
  //of everything
  SubscribeAndPublishRadar SAPObject;

  ros::spin();

  return 0;
}