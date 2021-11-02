#ifndef YOLOV5_TENSORRT_H
#define YOLOV5_TENSORRT_H

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "target_bbox_msgs/msg/bounding_box.hpp"
#include "target_bbox_msgs/msg/bounding_boxes.hpp"
#include "yolov5.h"
#include "yoloparam.h"


using namespace std::chrono_literals;
using namespace std;
using namespace cv;

class DetectionNode : public rclcpp::Node
{

public:
    DetectionNode();
    ~DetectionNode(){};

private:
    void loadParam();
    void initNode();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr obj_image_pub;
    rclcpp::Publisher<target_bbox_msgs::msg::BoundingBoxes>::SharedPtr bboxs_pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    string image_sub_topic_name;
    Yolov5 *yolov5;

public:
    yoloparam YP;
    void* YParam;
};
#endif