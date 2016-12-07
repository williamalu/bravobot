//
// Created by isaac on 12/3/16.
// Edited by lydia on 12/4/16.
//
//Tracks red/orange obstacles around the track
//Includes all the headers necessary to use the most common public pieces of the ROS system.
//
//Edit: Finds distance to and x location of red/orange obstacles

#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

int LowerH = 0;
int LowerS = 110;
int LowerV = 125;
int UpperH = 22;
int UpperS = 256;
int UpperV = 256;

cv::Mat src; 
cv::Mat src_gray;
int thresh = 100;
int max_thresh = 255;
cv::RNG rng(12345);
cv::Mat img_mask;
cv::Mat img_grey;
cv::Mat hsv_cnt;

cv::Mat img_hsv,thresh_output,drawing;
std::vector<cv::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;


void colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);
    cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),::img_mask);

    //Display the image using OpenCV
    // cv::circle(::img_mask, cv::Point(0,0), 128, cv::Scalar( 255, 255, 255 ), -1, 8);

    cv::imshow(WINDOW, ::img_mask);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);

    // thresh_callback(0, 0 );

    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
}
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    //Invert Image
    //Go through all the rows
    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<cv_ptr->image.cols; j++)
        {
            //Go through all the channels (b, g, r)
            for(int k=0; k<cv_ptr->image.channels(); k++)
            {
                //Invert the image by subtracting image data from 255
                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
            }
        }
    }


    //Display the image using OpenCV
    cv::imshow(WINDOW, cv_ptr->image);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);

    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
}


// /** @function thresh_callback */
// void thresh_callback(int, void*)
// {

//   while (cv::countNonZero(::img_mask) < 1) 
//   {
//     std::cout << "No thresh" << std::endl;
//     ros::spinOnce();
//   }

//   while (cv::countNonZero(::img_mask) > 1 && ros::ok()){
//   // std::cout << "Boo" << std::endl;

//   // // else
//   // // {
//   // // Detect edges using canny
//   cv::cvtColor(::img_mask,grey_cnt,CV_GRAY2BGR);
//   cv::cvtColor(grey_cnt,hsv_cnt,CV_BGR2HSV);
//   cv::Canny(grey_cnt, canny_output, thresh, thresh*2, 3 );
  
//   // Find contours
//   cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
//   // cv::cvtColor(contours,bgr_cnt,CV_BGR2GRAY);

//   std::vector<cv::vector<cv::Point> > contours_poly( contours.size() );
//   std::vector<cv::Point2f>center( contours.size() );
//   std::vector<float>radius( contours.size() );

//   drawing = cv::Mat( canny_output.size(), CV_8UC3);

//   //UNCOMMENT AND FIX
//   for( size_t i = 0; i < contours.size(); i++ )
//   {
//     // std::cout << contours.type() << std::endl;
//     // cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
//     cv::minEnclosingCircle( contours[i], center[i], radius[i] );
//     // cv::circle( drawing, center[i], radius[i], cv::Scalar( 255, 255, 255 ), -1, 8, 0 );
//   }

//   /// Show in a window
//   // cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//   cv::imshow( "Contours", drawing );
//   cv::waitKey(3);

//   // if (key == ord("q")){
//   // cv::destroyWindow( "Contours");
//   // break
//   // }
//   // }
//   ros::spinOnce();
//   }
// }

/** @function thresh_callback */
void thresh_callback(const sensor_msgs::ImageConstPtr& original_image)
{
  //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

  cv::cvtColor(cv_ptr->image,img_grey,CV_BGR2GRAY);
  // cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask);

  // // Detect edges using canny
  // cv::cvtColor(img_mask,grey_cnt,CV_GRAY2BGR);
  // cv::cvtColor(grey_cnt,hsv_cnt,CV_BGR2HSV);
  // cv::Canny(img_hsv, canny_output, thresh, thresh*2, 3 );
  cv::threshold( img_grey, thresh_output, thresh, 255, CV_THRESH_BINARY );
  
  // Find contours
  cv::findContours( thresh_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  // cv::cvtColor(contours,bgr_cnt,CV_BGR2GRAY);

  std::vector<cv::vector<cv::Point> > contours_poly( contours.size() );
  std::vector<cv::Point2f>center( contours.size() );
  std::vector<float>radius( contours.size() );

  drawing = cv::Mat( thresh_output.size(), CV_8UC3);

  //UNCOMMENT AND FIX
  for( size_t i = 0; i < contours.size(); i++ )
  {
    // std::cout << contours.type() << std::endl;
    // cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
    cv::minEnclosingCircle( contours[i], center[i], radius[i] );
    // cv::circle( drawing, center[i], radius[i], cv::Scalar( 255, 255, 255 ), -1, 8, 0 );
  }
  cv::imshow( "Contours", thresh_output );
  cv::waitKey(3);
  
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_processor");

    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    cv::namedWindow("Ball");
    cv::createTrackbar("LowerH","Ball",&LowerH,180,NULL);
    cv::createTrackbar("UpperH","Ball",&UpperH,180,NULL);
    cv::createTrackbar("LowerS","Ball",&LowerS,256,NULL);
    cv::createTrackbar("UpperS","Ball",&UpperS,256,NULL);
    cv::createTrackbar("LowerV","Ball",&LowerV,256,NULL);
    cv::createTrackbar("UpperV","Ball",&UpperV,256,NULL);

    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );

    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, colorDetectionCallback);
    image_transport::Subscriber thresh = it.subscribe("/image_raw", 1, thresh_callback);
    // thresh_callback(0, 0 );
    // cv::createTrackbar("Canny thresh:","Ball", &thresh, 255,thresh_callback);

    // thresh_callback(0,0);

    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
    cv::destroyWindow( "Contours");

    pub = it.advertise("camera/image_processed", 1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call.
    * ros::spin() will not return until the node has been shutdown, either through a call
    * to ros::shutdown() or a Ctrl-C.
    */
    ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");

}

