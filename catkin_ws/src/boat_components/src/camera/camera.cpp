#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <btmx/init_param.h>

int main(int argc, char **argv)
{
    // Node initialization
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    const std::string node_name = ros::this_node::getName();

    // Parameters
    int frame_rate;
    int video_source;
    btmx::initParam(frame_rate, "frame_rate", 30, node_name);
    if (!btmx::initParam(video_source, "video_source", node_name))
        return 1;

    // ImageTransport and OpenCV objects
    image_transport::ImageTransport it(nh);
    cv::VideoCapture cap(video_source);
    cv::Mat frame;

    // Check if video device can be opened with the given index
    if (!cap.isOpened())
        return 1;

    // ROS
    sensor_msgs::ImagePtr msg;
    image_transport::Publisher pub = it.advertise(node_name + "/image", 1);
    ros::Rate loop_rate(frame_rate);
    
    while (nh.ok())
    {
        cap >> frame;
        if (!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
