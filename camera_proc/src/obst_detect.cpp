//
// Created by isaac on 12/6/16.
//

#include <string>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int16MultiArray.h>

namespace enc = sensor_msgs::image_encodings;

ros::Publisher pub;

static const std::string raw_window = "Raw";
static const std::string filtered_window = "Filtered";
static const std::string contoured_window = "Contoured";

// Define initial filter parameters
int blur = 3;
int lowerH = 0;
int upperH = 23;
int lowerS = 165;
int upperS = 256;
int lowerV = 203;
int upperV = 256;

void callback(const sensor_msgs::ImageConstPtr& original_image)
{
    // Convert from ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    // Apply blur and HSV filter
    cv::Mat img_mask, img_hsv;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::medianBlur(cv_ptr->image, cv_ptr->image, blur*2+1);
    cv::cvtColor(cv_ptr->image, img_hsv, CV_BGR2HSV);
    cv::inRange(img_hsv, cv::Scalar(lowerH, lowerS, lowerV), \
		cv::Scalar(upperH, upperS, upperV), img_mask);
    cv::imshow(filtered_window, img_mask);

    // Find contours
    cv::findContours(img_mask, contours, hierarchy, cv::RETR_TREE, \
		     cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector<int16_t> obst_pos;
    std_msgs::Int16MultiArray obst_pos_msg;
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect(contours.size());
    cv::Scalar color = cv::Scalar(255, 0, 0);
    int size_threshold = 1;

    for(size_t i=0; i<contours.size(); i++)
    {
        // Define bounding rectangles for each contour
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
    }

    cv::Rect curr_obst;
    for(size_t i=0; i<boundRect.size(); i++)
    {
        curr_obst = boundRect[i];
        // Filter out small objects
        if(curr_obst.height > size_threshold && curr_obst.width > size_threshold)
        {
            // Draw rectangles on image around obstacle
            cv::rectangle(cv_ptr->image, curr_obst.tl(), curr_obst.br(), \
		    color, 1, 8, 0 );
            // Set obstacle attributes as a vector
            obst_pos.push_back(curr_obst.x);
            obst_pos.push_back(curr_obst.y);
            obst_pos.push_back(curr_obst.width);
            obst_pos.push_back(curr_obst.height);
            // Label obstacles with text
            cv::putText(cv_ptr->image, "Obstacle", curr_obst.tl(), \
		    cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 255), 1.0);
        }
    }

    cv::imshow(raw_window, cv_ptr->image);
    cv::waitKey(3);

    // Convert from OpenCV image to ROS image message and publish
    obst_pos_msg.data = obst_pos;
    pub.publish(obst_pos_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_subscriber");

    ros::NodeHandle nh;

    pub = nh.advertise<std_msgs::Int16MultiArray>("obstacle_positions", 1000);

    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, callback);

    const std::string control_window = "Filter Controls";

    // Create filter control window
    cv::namedWindow(control_window);
    cv::createTrackbar("blur", control_window, &blur, 20, NULL);
    cv::createTrackbar("lowerH", control_window, &lowerH, 180, NULL);
    cv::createTrackbar("upperH", control_window, &upperH, 180, NULL);
    cv::createTrackbar("lowerS", control_window, &lowerS, 256, NULL);
    cv::createTrackbar("upperS", control_window, &upperS, 256, NULL);
    cv::createTrackbar("lowerV", control_window, &lowerV, 256, NULL);
    cv::createTrackbar("upperV", control_window, &upperV, 256, NULL);

    ros::spin();
}