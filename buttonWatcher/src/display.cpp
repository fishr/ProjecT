#include <ros/ros.h>

#include <string.h>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "shapeMover");

  ros::NodeHandle node;

  cv::Mat screen = cv::Mat::zeros(480, 640, CV_32F);
  cv::namedWindow("Showy"); 
  float x = 0.;
  float y = 0.;
  float z = 0.;

  //ros::service::waitForService("spawn");
  //ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");

  //ros::Publisher turtle_vel = node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

  tf::TransformListener listener;

  ros::Rate rate(30.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/openni_depth_frame", "/head_1",  ros::Time(0), transform);
      screen = cv::Mat::zeros(480, 640, CV_8UC3);
      x=transform.getOrigin().x();
      y=transform.getOrigin().y();
      z=transform.getOrigin().z();
      

      cv::circle(screen, cv::Point((int)(y*100+320),(int)(z*-100+240)), 30, (0,0,255), -2);
      ROS_INFO("heads at %f", transform.getOrigin().x());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    cv::imshow("Showy", screen);
    cv::waitKey(1);
    rate.sleep();
  }
  return 0;
};

