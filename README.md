# yolov5-tensorrt-ros2
 yolov5在不久前更新了v6.0，主要更新为引入更小的Nano模型（yolov5n），模型保持了yolov5s的深度，宽度则是从0.5降到0.25,模型的总参数减少了 75%，从 7.5M 缩小到了 1.9M，非常适合于移动端或者是 CPU 的环境。

# 新特性
>整合了 [Roboflow](https://github.com/ultralytics/yolov5/issues/4975)
- roboflow公开了很多非常有用的数据集，在 v6.0 上可以直接使用他们的数据集

>支持 tensorflow 和 keras模型的导出
 - 使用 python export.py --include saved_model pb tflite tfjs 就可以完成 tensorFlow、keras、tflite 和 tf.js 模型的导出

>同时支持 OpenCV DNN 和 ONNX Runtime
 - 导出的 onnx 同时支持 opencv dnn 和 onnx runtime
```
python export --weights yolov5s.pt --include onnx --opset 12 --dynamic
```
 - 在检测的时候也可以使用指定的 onnx
```
python detect.py --weights yolov5s.onnx --dnn
```

>模型结构
 - 用 Conv(k=6, s=2, p=2) 代替 Focus 层，主要是为了方便模型导出
 - 使用 SPPF 代替 SPP 层
 - 减少 P3 主干层 C3
 - 将 SPPF 放在主干的后面
 - 在最后一个 C3 主干层中重新引入快捷方式更新超参数

>增加了 Flask REST API
 - 提供了 web api 的支持，远端测试非常方便

# 转TensorRT

 参考[这个项目](https://github.com/wang-xinyu/tensorrtx/tree/master/yolov5)

 该项目提供了一大批常见模型的转TensorRT的方法，在yolov5 v6.0版本出来后不久就已经支持

 按照操作流程将yolov5训练得到的参数文件转化为tensorrt支持的文件（例如：yolov5n.pt->yolov5n.wts->yolov5n.engine）

# ROS2+YOLOV5+TensorRT功能包

- 上面提到的项目里面包含了加载.engine文件到输出检测信息的代码实现流程，但是只有部分是被需要的。本项目提炼了实现这一流程的代码，并将其模块化

- 检测流程需要设置一些超参数，本项目将通过launch加载.yaml文件来实现便捷更改超参数，主要包含的参数如下：
  ```
  device: 0  
  nms: 0.45
  conf: 0.50
  batch_size: 1
  input_h: 640
  input_w: 640
  class_num: 80
  ```

- 本项目建立了target_bbox_msgs消息功能包，用于发布或订阅目标检测结果，消息主要包含以下内容：

  BoundingBox.msg
  ```
  float32 probability     #置信度
  uint16 xmin             #包围框左上点x
  uint16 ymin             #包围框左上点y
  uint16 xmax             #包围框右上点x
  uint16 ymax             #包围框右上点y
  uint16 id               #跟踪ID
  uint16 img_width        #图宽
  uint16 img_height       #图长
  int32 center_dist       #中心点到相机距离

  string class_id         #目标类别
  ```

  BoundingBoxes.msg
  ```
  std_msgs/Header header          #图像头
  BoundingBox[] bounding_boxes    #目标的信息
  ```

- 最终还会发布框选、标注后的图像，本项目以coco数据集为例，通过coco_name.hpp加载类别名称和颜色信息
如果是自己训练的模型，只需要按照格式修改这个文件即可

## 提示
 - 该项目暂时不包括目标跟踪和测距的功能，相关参数为预留
 - 以下为转换好了的engine文件，下载后放至yolov5_tensorrt/weights目录

 链接: https://pan.baidu.com/s/1tgnOVQtG-jl-2CIf9E2-uA  密码: 72oe

# 编译运行
 - 相关版本
 ```
   ROS2-foxy
   OpenCV4
   TensorRT8.0.1.6
  ```
 - 将功能包下载到自己的ROS2工作空间，然后编译：
 ```
   colcon build
 ```
 - 使用以下命令运行：
 ```
   ros2 launch yolov5_tensorrt yolov5_tensorrt_launch.py
 ```


# 展示效果

## 发布检测结果图像

![Default - rqt_015](https://user-images.githubusercontent.com/47886076/139780685-eddc136f-804f-43b1-b731-1d2fd6c2c2b9.png)

## 发布目标检测信息

![选区_006](https://user-images.githubusercontent.com/47886076/139781613-3aa66d14-784a-415b-8690-1e9a8423a512.png)




