#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <eigen_conversions/eigen_msg.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>

#include <Eigen/Eigen>

#include <tek4030_visual_servoing_msgs/ImageFeaturePoints.h>
#include <iostream>

using namespace cv;
using namespace Eigen;

ros::Publisher* pub_twist_global;
ros::Publisher* pub_setpoint_global;
ros::Publisher* pub_error_global;

Matrix<long double,8,8> Ks = Matrix<long double,8,8>::Identity();

Point2f points_normalized[4];
// Point2f points_setpoint[4] { Point2f(0.15, 0.15), Point2f(-0.15, 0.15), Point2f(-0.15, -0.15), Point2f(0.15, -0.15)};
// bool setpoint1 = true;
Point2f points_setpoint[4] {Point2f(0.3, -0.3), Point2f(0.3, 0.3),    Point2f(-0.3, 0.3),    Point2f(-0.3, -0.3)};
bool setpoint1 = false;

tek4030_visual_servoing_msgs::ImageFeaturePoints msgError;
tek4030_visual_servoing_msgs::ImageFeaturePoints msgSetpoint;
geometry_msgs::Twist msgTwist;

void calculateAndPublishErrorsandTwist(void);
Eigen::Matrix<long double,8,6> getLs();

geometry_msgs::Point cvPoint2fToRosPoint(Point2f p)
{
  geometry_msgs::Point q;
  q.x = p.x;
  q.y = p.y;
  q.z = 1.0;
  return q;
}

geometry_msgs::Point cvPoint2fToRosPointZ0(Point2f p)
{
  geometry_msgs::Point q;
  q.x = p.x;
  q.y = p.y;
  q.z = 0.0;
  return q;
}


void pointsNormalizedCallback(const tek4030_visual_servoing_msgs::ImageFeaturePointsPtr& msg)
{
  for (int i = 0; i< std::min((int) msg->p.size(),4); i++)
  {
    Vector3d s;
    tf::pointMsgToEigen(msg->p[i], s);
    points_normalized[i].x = s(0);
    points_normalized[i].y = s(1);
  }
  calculateAndPublishErrorsandTwist();
}

Eigen::Matrix<long double,8,6> getLs()
{
  Eigen::Matrix<long double,8,6> Ls = Eigen::Matrix<long double,8,6>::Zero();
  for(int i = 0; i < 4; i++)
  {
    double x = points_normalized[i].x;
    double y = points_normalized[i].y;
    Ls.block(2*i,0,2,6) << -1.0, 0, x,        x*y, (1+pow(x,2))*(-1), y,
                            0,-1.0, y, 1+pow(y,2),          -x*y,-x;
  }
  
  return Ls;
}

void calculateAndPublishErrorsandTwist(void)
{
  Eigen::Matrix<long double,8,1> es;
  es << points_setpoint[0].x - points_normalized[0].x, 
        points_setpoint[0].y - points_normalized[0].y,
        points_setpoint[1].x - points_normalized[1].x,
        points_setpoint[1].y - points_normalized[1].y,
        points_setpoint[2].x - points_normalized[2].x,
        points_setpoint[2].y - points_normalized[2].y,
        points_setpoint[3].x - points_normalized[3].x,
        points_setpoint[3].y - points_normalized[3].y;


  if( msgError.p.empty() !=true)
    msgError.p.clear();
  for(int i = 0; i < 4; i++)
    msgError.p.push_back(cvPoint2fToRosPointZ0(Point2f(es(2*i),es(2*i+1))));
  pub_error_global->publish(msgError);
  pub_setpoint_global->publish(msgSetpoint);

  Eigen::Matrix<long double,8,6> Ls = getLs();

  es = (Ks*es).eval();
  Eigen::Matrix<long double,6,1> Vcr = Ls.fullPivHouseholderQr().solve(es);
  
 
  // // Vcr.row(0).swap(Vcr.row(1));
  // // Vcr.row(3).swap(Vcr.row(4));
  // Vcr = -Vcr;
  // long double Vcr0 =
  // Vcr(0) = -(Vcr(0));
  // Vcr(3) = -(Vcr(3)),

  if(setpoint1)
  {
    long double Vcr0 = Vcr(0);
    long double Vcr3 = Vcr(3);
    Vcr(0) = -Vcr0;
    Vcr(3) = -Vcr3;
    Vcr = -Vcr;
    
  }
  else
  {
    Vcr.row(0).swap(Vcr.row(1));
    Vcr.row(3).swap(Vcr.row(4));
    Vcr = -Vcr;

  }




  tf::twistEigenToMsg(Vcr.cast <double> (),msgTwist);
  pub_twist_global->publish(msgTwist);
  
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Controller2");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  Ks = 0.5*Ks.eval();

  for(int i=0; i < 4; i ++)
  {
    msgSetpoint.p.push_back(cvPoint2fToRosPoint(points_setpoint[i]));
  }

  /* Allocate publishers */
  ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("camera_twist", 2000);
  pub_twist_global = &pub_twist;

  ros::Publisher pub_setpoint = nh.advertise<tek4030_visual_servoing_msgs::ImageFeaturePoints>("/imgproc/points_setpoint", 2000);
  pub_setpoint_global = &pub_setpoint;

  ros::Publisher pub_error = nh.advertise<tek4030_visual_servoing_msgs::ImageFeaturePoints>("/imgproc/points_error", 2000);
  pub_error_global = &pub_error;

  /* Allcate subscibers */
  ros::Subscriber sub_points_normalized = nh.subscribe("/imgproc/points_normalized", 2000, pointsNormalizedCallback);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  

  
}