#include <cstdio>
#include <string>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

class ImagePub : public rclcpp::Node
{

public:
  ImagePub()
      : Node("image_pub")
  {
    loadParam();
    initNode();
    videoCapture();
  }
  ~ImagePub(){};

  void loadParam()
  {
    this->declare_parameter<string>("image_pub_topic_name", "/camera/image_raw");
    this->declare_parameter<string>("capture_format", "0");

    this->get_parameter("image_pub_topic_name", image_pub_topic_name);
    this->get_parameter("capture_format", capture_format);
  }

  void initNode()
  {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        image_pub_topic_name, 3);
    // cp_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    //     image_pub_topic_name, 3);
    timer_ = this->create_wall_timer(
        40ms, std::bind(&ImagePub::timerCallback, this));
  }

  void videoCapture()
  {
    if (isNum(capture_format))
    {
      int port = atoi(capture_format.c_str());
      capture = VideoCapture(port);
    }
    else
    {
      capture = VideoCapture(capture_format);
      frame_num = capture.get(cv::CAP_PROP_FRAME_COUNT);
      flag = 0;

      RCLCPP_INFO(this->get_logger(), "total frame number is: '%d'", frame_num);
    }

    // if (!capture.isOpened())
    // {
    //   RCLCPP_INFO(this->get_logger(), "Open videoCapture failed");
    //   return;
    // }
    // else
    //   RCLCPP_INFO(this->get_logger(), "Open videoCapture success");
  }

  void timerCallback()
  {
    if (!isNum(capture_format) && flag == frame_num)
      videoCapture();

    Mat frame;
    capture.read(frame);
    if (!frame.empty())   //避免帧差引起发布空图
    {
      sensor_msgs::msg::Image img_msg;
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg(img_msg);
      img_msg.header.frame_id = "camera";
      image_pub_->publish(img_msg);
    }

    // sensor_msgs::msg::CompressedImage cp_img_msg;
    // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toCompressedImageMsg(cp_img_msg);
    // cp_img_msg.header.frame_id = "camera";
    // cp_image_pub_->publish(cp_img_msg);
    if (!isNum(capture_format))
      flag++;
  }

  bool isNum(string str)
  {
    stringstream sin(str);
    double d;
    char c;
    if (!(sin >> d))
    {
      return false;
    }
    if (sin >> c)
    {
      return false;
    }
    return true;
  }

private:
  string image_pub_topic_name;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr cp_image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  VideoCapture capture;
  int frame_num;
  int flag;
  string capture_format;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePub>());
  rclcpp::shutdown();
  return 0;
}
