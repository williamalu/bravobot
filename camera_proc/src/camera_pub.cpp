// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

// // MAY REQUIRE ADDITIONAL DEPENDENCIES- SEE ROS IMAGE PUBLISHER TUTORIAL


// int main(int argc, char** argv)
// {
//   // Check if video source has been passed as a parameter
//   if(argv[1] == NULL) return 1;

//   ros::init(argc, argv, "image_publisher");
//   ros::NodeHandle nh;
//   image_transport::ImageTransport it(nh);
//   image_transport::Publisher pub = it.advertise("camera/image", 1);

//   cv::VideoCapture cap(cv::VideoCapture(0));
//   // Check if video device can be opened with the given index
//   if(!cap.isOpened()) return 1;
//   cv::Mat frame;
//   sensor_msgs::ImagePtr msg;

//   ros::Rate loop_rate(5);
//   while (nh.ok()) {
//     cap >> frame;
//     // Check if grabbed frame is actually full with some content
//     if(!frame.empty()) {
//       msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//       pub.publish(msg);
//       cv::waitKey(1);
//     }

//     ros::spinOnce();
//     loop_rate.sleep();
//   }
// }