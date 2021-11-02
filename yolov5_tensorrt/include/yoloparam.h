#ifndef YOLOPARAM
#define YOLOPARAM
#include <iostream>
#include <string>

class yoloparam
{

public:
    yoloparam(){}
    ~yoloparam(){}

public:
    static constexpr int CHECK_COUNT = 3;
    static constexpr float IGNORE_THRESH = 0.1f;
    struct YoloKernel
    {
        int width;
        int height;
        float anchors[CHECK_COUNT * 2];
    };
    YoloKernel mYoloKernel;
    static constexpr int MAX_OUTPUT_BBOX_COUNT = 1000;
    int CLASS_NUM; 
    int INPUT_H = 640; // yolov5's input height and width must be divisible by 32.
    int INPUT_W = 640;

    static constexpr int LOCATIONS = 4;
    struct alignas(float) Detection
    {
        //center_x center_y w h
        float bbox[LOCATIONS];
        float conf; // bbox_conf * cls_conf
        float class_id;
    };
    Detection mDetection;
    int DEVICE = 0;
    float NMS_THRESH = 0.4;
    float CONF_THRESH = 0.5;
    int BATCH_SIZE = 1;
    std::string ENGINE_DIR;
    static const int OUTPUT_SIZE = MAX_OUTPUT_BBOX_COUNT * sizeof(Detection) / sizeof(float) + 1;

};

#endif

