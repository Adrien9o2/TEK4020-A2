#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <eigen_conversions/eigen_msg.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>

#include <Eigen/Eigen>

#include <tek4030_visual_servoing_msgs/ImageFeaturePoints.h>

#include <mutex>

using namespace cv;
using namespace Eigen;

Matrix3d K;

ros::Publisher *pub_points;
ros::Publisher *pub_points_normalized;
ros::Publisher *pub_image;

Point2f setpoint[4];

geometry_msgs::Point cvPoint2fToRosPoint(Point2f p)
{
  geometry_msgs::Point q;
  q.x = p.x;
  q.y = p.y;
  q.z = 1.0;
  return q;
}

Point2f rosPointToCvPoint2f(geometry_msgs::Point p)
{
  Point2f q;
  q.x = p.x;
  q.y = p.y;
}

Point2f segmentPoint(Mat &hsv, double lowerHueDeg, double upperHueDeg, Mat &mask)
{
  inRange(hsv, Scalar(lowerHueDeg, 120, 70), Scalar(upperHueDeg, 255, 255), mask);
  Moments mu = moments(mask);
  Point2f mass = Point2f( static_cast<float>(mu.m10 / (mu.m00 + 1e-5)),
                   static_cast<float>(mu.m01 / (mu.m00 + 1e-5)) );
  return mass;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    
  try
  {
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    Mat mask1,mask2,mask3,mask4;
    Point2f p1, p2, p3, p4;
    // Creating masks to detect color circles
    p1 = segmentPoint(hsv, 140, 160, mask1); // magenta
    p2 = segmentPoint(hsv, 50, 70, mask2);   // green
    p3 = segmentPoint(hsv, 110, 130, mask3); // blue
    p4 = segmentPoint(hsv, 20, 40, mask4);   // yellow
    
    tek4030_visual_servoing_msgs::ImageFeaturePoints points;
    points.p.push_back(cvPoint2fToRosPoint(p1));
    points.p.push_back(cvPoint2fToRosPoint(p2));
    points.p.push_back(cvPoint2fToRosPoint(p3));
    points.p.push_back(cvPoint2fToRosPoint(p4));

    pub_points->publish(points);
    
    tek4030_visual_servoing_msgs::ImageFeaturePoints points_normalized;
    for (int i = 0; i<points.p.size(); i++)
    {
      Vector3d p;
      tf::pointMsgToEigen(points.p[i], p);
      geometry_msgs::Point p_normalized;
      tf::pointEigenToMsg(K.inverse()*p, p_normalized);
      points_normalized.p.push_back(p_normalized);
    }
    
    pub_points_normalized->publish(points_normalized);
    
    circle(frame, p1, 3, Scalar(0,0,0));
    circle(frame, p2, 3, Scalar(0,0,0));
    circle(frame, p3, 3, Scalar(0,0,0));
    circle(frame, p4, 3, Scalar(0,0,0));

    circle(frame, setpoint[0], 3, Scalar(255,128,255));
    circle(frame, setpoint[1], 3, Scalar(128,255,128));
    circle(frame, setpoint[2], 3, Scalar(255,128,128));
    circle(frame, setpoint[3], 3, Scalar(128,255,255));
    
    sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub_image->publish(ros_image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void cameraInfoCallback(const sensor_msgs::CameraInfoPtr& msg)
{
  K << msg->K[0], msg->K[1], msg->K[2], 
       msg->K[3], msg->K[4], msg->K[5], 
       msg->K[6], msg->K[7], msg->K[8];
}

void setpointCallback(const tek4030_visual_servoing_msgs::ImageFeaturePointsPtr& msg)
{
  for (int i = 0; i< std::min((int) msg->p.size(),4); i++)
  {
    Vector3d s;
    tf::pointMsgToEigen(msg->p[i], s);
    Vector3d s_img = K*s;
    setpoint[i].x = s_img(0);
    setpoint[i].y = s_img(1);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  
  /* Init globale variables */
  for ( int i=0; i<4; i++)
  {
    setpoint[i].x = 0;
    setpoint[i].y = 0;
  }
  
  ROS_INFO("Starting image processing node...");
  
  /* Init publishers */
  ros::Publisher pub_1 = nh.advertise<tek4030_visual_servoing_msgs::ImageFeaturePoints>("/imgproc/points", 1000);
  ros::Publisher pub_2 = nh.advertise<tek4030_visual_servoing_msgs::ImageFeaturePoints>("/imgproc/points_normalized", 1000);
  pub_points = &pub_1;
  pub_points_normalized = &pub_2;
  
  /* Init subcribers */
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_1 = it.subscribe("camera/image_raw", 1, imageCallback);
  ros::Subscriber sub_2 = nh.subscribe("/camera/camera_info", 1000, cameraInfoCallback);
  ros::Subscriber sub_3 = nh.subscribe("/imgproc/setpoint_normalized", 1000, setpointCallback);
  
  ros::Publisher pub_3 = nh.advertise<sensor_msgs::Image>("/imgproc/image", 1);
  pub_image = &pub_3;
  
  ros::spin();

}
