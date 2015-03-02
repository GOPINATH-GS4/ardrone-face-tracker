#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include "PID.h"
#include "control.h"
static const std::string OPENCV_WINDOW = "Image window";

extern struct COMMAND control(cv::Mat src);
extern int  get_frames_without_image();
extern void  reset_frames_without_image();
struct COMMAND cmd;
bool tracking = false;
int altitude = 0;
int skip_count = 0;
#define SKIP_COUNT 5
#define NO_FRAMES_LIMIT 10
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher takeoff, land, reset,  drone;
  ros::ServiceClient trim;
  std_msgs::Empty empty;
  std_srvs::Empty flattrim;
  geometry_msgs::Twist command;
  ros::Subscriber startTracking;
  ros::Subscriber navdata;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

    image_sub_ = it_.subscribe("/ardrone/image_raw", 1, 
      &ImageConverter::imageCb, this);

	startTracking = nh_.subscribe("/ardrone/track", 1, &ImageConverter::tracker, this);
	navdata = nh_.subscribe("/ardrone/navdata", 100, &ImageConverter::navdataCb, this);

	takeoff = nh_.advertise<std_msgs::Empty>("/ardrone/takeoff",1);
	land = nh_.advertise<std_msgs::Empty>("/ardrone/land",1);
	reset = nh_.advertise<std_msgs::Empty>("/ardrone/reset",1);
	drone = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	trim = nh_.serviceClient<std_srvs::Empty>(nh_.resolveName("ardrone/flattrim"),1);

    cv::namedWindow(OPENCV_WINDOW);

	printf("Flat Trim() ....\n");
	sleep(2);
	trim.call(flattrim);

	printf("initializing...  Doing reset....\n");
	sleep(2);
	//reset.publish(empty);

	printf("Taking off .....\n");
	sleep(2);
	takeoff.publish(empty);		

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void tracker(const std_msgs::Empty msg) {
	printf("Tracking true");
	tracking = true;
  }
  void navdataCb(const ardrone_autonomy::NavdataConstPtr msg) {
	altitude = msg->altd;
	if(msg->tags_count > 0) {
		printf("Tag detected ...\n");
	}
  }

  float within(float val, float min , float max) {
	if(val > max) return max;
	else if (val < min) return min;
	else return val;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	char c = cv::waitKey(10);
	if(c == 't') { tracking = true; }

	memset(&command, 0x00, sizeof(command));
	cmd = control(cv_ptr->image);

	if(tracking == true) {
		skip_count = 0;
		printf("%f %f %f \n" , cmd.commandz, cmd.commandx, cmd.commandy);
		command.linear.x = within(cmd.commandz, -1.0, 1.0);
		command.linear.y = within(cmd.commandx, -1.0, 1.0);
		command.linear.z = within(cmd.commandy, -1.0, 1.0);

		if(get_frames_without_image() > NO_FRAMES_LIMIT) {
			command.angular.z = .9;
			reset_frames_without_image();
		}
		else 
			command.angular.x = command.angular.y = command.angular.z = 0;

		std::cout << command << std::endl;
		usleep(200);

		drone.publish(command);
	
	}
	else {}

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

	if(c != -1) printf("waitKey %c\n", c);

	switch(c) {
		case 't':
			break;
		case 's':
			break;
		case 'u': 
			break;
		case 'd':
			break;
		case 'l':
			break;
		case 'r':
			break;
		case 'f':
			break;
		case 'b':
			break;
		default: 
			break;
	}
  	  
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  cv::Mat src;
  return 0;
}
