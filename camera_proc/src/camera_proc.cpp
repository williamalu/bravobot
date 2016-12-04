#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

static const std::string Video_Feed = "Video_Feed";
static const std::string Control = "Control";
Mat videoptr;

class ImageConverter
{
public:
  private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(Video_Feed, CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(Video_Feed);
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

    videoptr = cv_ptr->image;

    // Update GUI Window
    cv::imshow(Video_Feed, videoptr);

    // ROS_INFO("ROWS : %d, COLS : %d", cv_ptr->image.rows, cv_ptr->image.cols);

    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  // void colorTrack()
  // {

  //   namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  //   int iLowH = 170;
  //   int iHighH = 179;

  //   int iLowS = 150; 
  //   int iHighS = 255;

  //   int iLowV = 60;
  //   int iHighV = 255;

  //   //Create trackbars in "Control" window
  //   createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  //   createTrackbar("HighH", "Control", &iHighH, 179);

  //   createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  //   createTrackbar("HighS", "Control", &iHighS, 255);

  //   createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
  //   createTrackbar("HighV", "Control", &iHighV, 255);

  //   int iLastX = -1; 
  //   int iLastY = -1;


  //   Mat imgOriginal;
  //   imgOriginal = videoptr;

  //   Mat imgHSV;
  //   imgHSV = videoptr;

  //   //Create a black image with the size as the camera output
  //   Mat imgLines = Mat::zeros( imgOriginal.size(), CV_8UC3 );


  //   cvtColor(imgOriginal, imgHSV, CV_RGB2HSV); //Convert the captured frame from BGR to HSV

   
    // Mat imgThresholded;

    // inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        
    // //morphological opening (removes small objects from the foreground)
    // erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    // dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    // //morphological closing (removes small holes from the foreground)
    // dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    // erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // //Calculate the moments of the thresholded image
    // Moments oMoments = moments(imgThresholded);

    // double dM01 = oMoments.m01;
    // double dM10 = oMoments.m10;
    // double dArea = oMoments.m00;

    //  // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
    // if (dArea > 10000)
    // {
    //  //calculate the position of the ball
    // int posX = dM10 / dArea;
    // int posY = dM01 / dArea;        
          
    //   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
    //   {
    //   //Draw a red line from the previous point to the current point
    //   line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
    //   }

    //   iLastX = posX;
    //   iLastY = posY;
    // }

    //   cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image

    //   imgOriginal = imgOriginal + imgLines;
    //   cv::imshow("Original", imgOriginal); //show the original image
    // }

  // }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  // ic.colorTrack();


  while(ros::ok()){
    ros::spinOnce();
  }
  return 0;
}