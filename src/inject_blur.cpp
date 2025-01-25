/*Standard C++ libraries*/
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "boost/thread.hpp"

class IMG_BLUR {

    public:
        IMG_BLUR();
        void image_left_cb( const sensor_msgs::ImageConstPtr& );
        void image_right_cb( const sensor_msgs::ImageConstPtr& );
        void keyboard_input();
        void run();

    private:
        ros::NodeHandle _nh;

        

        ros::Subscriber _image1_sub;
        ros::Subscriber _image2_sub;

        ros::Publisher _image1_pub;
        ros::Publisher _image2_pub;
        ros::Publisher _blur_state;

        bool blur;

};

IMG_BLUR::IMG_BLUR() {

    _image1_sub = _nh.subscribe("/t265/fisheye1/image_raw", 1, &IMG_BLUR::image_left_cb, this);
    _image2_sub = _nh.subscribe("/t265/fisheye2/image_raw", 1, &IMG_BLUR::image_right_cb, this);

    _image1_pub = _nh.advertise<sensor_msgs::Image>("/t265/fisheye1/image_raw_blurred", 1);
    _image2_pub = _nh.advertise<sensor_msgs::Image>("/t265/fisheye2/image_raw_blurred", 1);
    _blur_state = _nh.advertise<std_msgs::Bool>("/inject_blur/state", 1000);

    blur = false;
}

void IMG_BLUR::image_left_cb( const sensor_msgs::ImageConstPtr& left_msg ) {
// Converti l'immagine ROS in un formato OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Applica il motion blur (esempio con filtro gaussiano)
  if( blur )
    cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(15, 15), 0);

  // Pubblica l'immagine modificata
  _image1_pub.publish(cv_ptr->toImageMsg());  
}

void IMG_BLUR::image_right_cb( const sensor_msgs::ImageConstPtr& right_msg ) {
// Converti l'immagine ROS in un formato OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Applica il motion blur (esempio con filtro gaussiano)
  if( blur )
    cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(15, 15), 0);

  // Pubblica l'immagine modificata
  _image2_pub.publish(cv_ptr->toImageMsg());  
}

void IMG_BLUR::keyboard_input() {
    ros::Rate r( 100 );

    std::cout<<"Inserire 'y' per motion blur\n";
    char cmd;
    std_msgs::Bool blur_state;
    while( ros::ok() ) {

        std::cin>>cmd;
        if( cmd == 'y' )
            blur = true;
        else 
            blur = false;


        blur_state.data = blur;
        _blur_state.publish( blur_state );
        
        r.sleep();
    }
    
}

void IMG_BLUR::run() {
    boost::thread keyboard_input_t( &IMG_BLUR::keyboard_input, this );
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inject_blur");
    IMG_BLUR img_blur;
    img_blur.run();
    return 0;
}