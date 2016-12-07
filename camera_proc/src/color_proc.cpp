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

namespace enc = sensor_msgs::image_encodings;

static const std::string raw_window = "Raw";
static const std::string filtered_window = "Filtered";
static const std::string contoured_window = "Contoured";

// Define initial filter parameters
int blur = 20;
int lowerH = 0;
int upperH = 84;
int lowerS = 165;
int upperS = 256;
int lowerV = 125;
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
    cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);
    cv::inRange(img_hsv,cv::Scalar(lowerH,lowerS,lowerV),cv::Scalar(upperH,upperS,upperV),\
		img_mask);
    cv::imshow(filtered_window, img_mask);

    // Find contours
    cv::findContours(img_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE,\
		     cv::Point(0, 0));

    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect(contours.size());
    cv::Scalar color = cv::Scalar(255, 0, 0);
    int height_threshold = 3;

    for(size_t i=0; i<contours.size(); i++)
    {
        // Define bounding rectangles for each contour
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
        // Remove really small rectangles
        if(boundRect[i].height < height_threshold) boundRect.erase(boundRect.begin()+i);
    }

    for(size_t i=0; i<boundRect.size(); i++)
    {
        // Draw rectangles
        cv::rectangle(cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), color, 1, 8, 0 );

        // Label tall things as cones
        if(boundRect[i].height >= 1.5*boundRect[i].width)
        {
            cv::putText(cv_ptr->image, "cone", boundRect[i].tl(), cv::FONT_HERSHEY_PLAIN, 1.0, \
		    CV_RGB(0, 0, 255), 1.0);
        }
            // Round things are floats
        else
        {
            cv::putText(cv_ptr->image, "float", boundRect[i].tl(), cv::FONT_HERSHEY_PLAIN, 1.0, \
		    CV_RGB(0, 0, 255), 1.0);
        }
    }

    cv::imshow(raw_window, cv_ptr->image);
    cv::waitKey(3);

    // Convert from OpenCV image to ROS image message and publish
    // pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    const std::string control_window = "Filter Controls";

    cv::namedWindow(control_window);
    cv::createTrackbar("blur", control_window, &blur, 30, NULL);
    cv::createTrackbar("lowerH", control_window, &lowerH, 180, NULL);
    cv::createTrackbar("upperH", control_window, &upperH, 180, NULL);
    cv::createTrackbar("lowerS", control_window, &lowerS, 256, NULL);
    cv::createTrackbar("upperS", control_window, &upperS, 256, NULL);
    cv::createTrackbar("lowerV", control_window, &lowerV, 256, NULL);
    cv::createTrackbar("upperV", control_window, &upperV, 256, NULL);

    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, callback);

    //    cv::destroyWindow(main_window);

    ros::spin();
}
