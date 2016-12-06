// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/NavSatFix.h>

// QT
#include <QApplication>
#include <QObject>
#include <QThread>

// Custom Classes
#include "funrobo_gpsview/mainwindow.h"
#include "funrobo_gpsview/Worker.h"

MainWindow* w_ptr;

void plotLocation(const sensor_msgs::NavSatFix::ConstPtr& msg){
    emit w_ptr->updateGPS(msg->latitude,msg->longitude);
    std::cout << "Latitude : " << msg->latitude << ", Longitude : " << msg->longitude << std::endl;
}

int main(int argc, char *argv[])
{

    // initialize window
    QApplication a(argc, argv);
    QString image_path = (ros::package::getPath("funrobo_gpsview") + "/assets/olin.png").c_str();
    MainWindow w(nullptr,image_path);
    w_ptr = &w;

    //start ROS thread for pumping ROS messages

    ros::init(argc, argv, "mapper");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/fix", 1000, plotLocation);

    QThread* thread = new QThread;
    Worker* worker = new Worker();
    worker->moveToThread(thread);

    QObject::connect(thread, SIGNAL(started()), worker, SLOT(process()));
    QObject::connect(worker, SIGNAL(finished()), thread, SLOT(quit()));

    QObject::connect(worker, SIGNAL(finished()), worker, SLOT(deleteLater()));
    QObject::connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    thread->start();

    // begin interaction
    w.show();
    return a.exec();
}
