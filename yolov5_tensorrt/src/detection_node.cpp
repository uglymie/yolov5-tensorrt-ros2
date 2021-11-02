#include "detection_node.h"

using std::placeholders::_1;

DetectionNode::DetectionNode()
    : Node("yolov5_tensorrt")
{
  loadParam();
  initNode();
  yolov5 = new Yolov5(YParam);
}

void DetectionNode::loadParam()
{
  this->declare_parameter<string>("image_sub_topic_name", "/camera/image_raw");
  this->declare_parameter<int>("device", 0);
  this->declare_parameter<double>("nms", 0.45);
  this->declare_parameter<double>("conf", 0.50);
  this->declare_parameter<int>("batch_size", 1);
  this->declare_parameter<int>("input_h", 640);
  this->declare_parameter<int>("input_w", 640);
  this->declare_parameter<int>("class_num", 80);
  this->declare_parameter<string>("engine_dir", "engine/yolov5x.engine");

  this->get_parameter("image_sub_topic_name", image_sub_topic_name);
  this->get_parameter("device", YP.DEVICE);
  this->get_parameter("nms", YP.NMS_THRESH);
  this->get_parameter("conf", YP.CONF_THRESH);
  this->get_parameter("batch_size", YP.BATCH_SIZE);
  this->get_parameter("input_h", YP.INPUT_H);
  this->get_parameter("input_w", YP.INPUT_W);
  this->get_parameter("class_num", YP.CLASS_NUM);
  this->get_parameter("engine_dir", YP.ENGINE_DIR);

  RCLCPP_INFO(this->get_logger(), "class_num: '%d'", YP.CLASS_NUM);
  YParam = &YP;
}

void DetectionNode::initNode()
{
  obj_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
      "/image/obj_detection", 3);
  bboxs_pub = this->create_publisher<target_bbox_msgs::msg::BoundingBoxes>(
      "/targets/bboxs", 1000);
  image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      image_sub_topic_name, 3, std::bind(&DetectionNode::imageCallback, this, _1));
}

void DetectionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{

  cv_bridge::CvImagePtr cv_ptr_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  // cvtColor(cv_ptr_img->image, cv_ptr_img->image, COLOR_BGR2RGB);
  Mat out_img;
  vector<_bbox> bbox;
  yolov5->objDetection(cv_ptr_img->image, out_img, bbox); //Box

  target_bbox_msgs::msg::BoundingBoxes boxes;
  boxes.header = cv_ptr_img->header;
  for (auto obj : bbox)
  {
    target_bbox_msgs::msg::BoundingBox box;
    box.probability = obj.probability;
    box.class_id = obj.class_id;
    box.xmin = obj.xmin;
    box.ymin = obj.ymin;
    box.xmax = obj.xmax;
    box.ymax = obj.ymax;
    box.img_width = obj.img_width;
    box.img_height = obj.img_height;
    // tracking id
    // box.id = 0;
    // depth
    // box.center_dist = 0;
    boxes.bounding_boxes.emplace_back(box);
  }
  bboxs_pub->publish(boxes);

  sensor_msgs::msg::Image img_msg;
  cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", out_img).toImageMsg(img_msg);
  img_msg.header.frame_id = "camera";
  obj_image_pub->publish(img_msg);
}