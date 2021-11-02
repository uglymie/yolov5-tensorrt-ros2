#include "yolov5.h"

Yolov5::Yolov5(void *YParam)
{
    com = new Common(YParam);
    YP = *(yoloparam *)YParam;
    std::cerr << "class_num in yolov5: " << YP.ENGINE_DIR << std::endl;
    initEngine();
}

Yolov5::~Yolov5()
{
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));

    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
}

void Yolov5::initEngine()
{
    cudaSetDevice(YP.DEVICE);
    // deserialize the .engine and run inference
    std::ifstream file(YP.ENGINE_DIR, std::ios::binary);
    if (!file.good())
    {
        std::cerr << "read " << YP.ENGINE_DIR << " error!" << std::endl;
        return;
    }
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();

    // prepare input data ---------------------------
    // static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
    // //for (int i = 0; i < 3 * INPUT_H * INPUT_W; i++)
    // //    data[i] = 1.0;
    // static float prob[BATCH_SIZE * OUTPUT_SIZE];
    runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    // void *buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], YP.BATCH_SIZE * 3 * YP.INPUT_H * YP.INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], YP.BATCH_SIZE * YP.OUTPUT_SIZE * sizeof(float)));
    // Create stream
    // cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
}

void Yolov5::doInference(IExecutionContext &context, cudaStream_t &stream,
                         void **buffers, float *input, float *output, int batchSize)
{
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * YP.INPUT_H * YP.INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * YP.OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void Yolov5::objDetection(Mat img, Mat &dst, vector<_bbox> &bbox) //
{
    float data[YP.BATCH_SIZE * 3 * YP.INPUT_H * YP.INPUT_W];
    float prob[YP.BATCH_SIZE * YP.OUTPUT_SIZE];

    dst = img.clone();
    Mat pr_img = preprocessImg(img, YP.INPUT_W, YP.INPUT_H); // letterbox BGR to RGB
    int i = 0;
    for (int row = 0; row < YP.INPUT_H; ++row)
    {
        uchar *uc_pixel = pr_img.data + row * pr_img.step;
        for (int col = 0; col < YP.INPUT_W; ++col)
        {
            data[0 * 3 * YP.INPUT_H * YP.INPUT_W + i] = (float)uc_pixel[2] / 255.0;
            data[0 * 3 * YP.INPUT_H * YP.INPUT_W + i + YP.INPUT_H * YP.INPUT_W] = (float)uc_pixel[1] / 255.0;
            data[0 * 3 * YP.INPUT_H * YP.INPUT_W + i + 2 * YP.INPUT_H * YP.INPUT_W] = (float)uc_pixel[0] / 255.0;
            uc_pixel += 3;
            ++i;
        }
    }

    // Run inference
    // auto start = std::chrono::system_clock::now();
    doInference(*context, stream, buffers, data, prob, YP.BATCH_SIZE);
    // auto end = std::chrono::system_clock::now();
    // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    std::vector<std::vector<Yolo::Detection>> batch_res(1);

    auto &res = batch_res[0];
    com->nms(res, &prob[0 * YP.OUTPUT_SIZE], YP.CONF_THRESH, YP.NMS_THRESH);

    // auto &res = batch_res[b];
    //std::cout << res.size() << std::endl;

    for (size_t j = 0; j < res.size(); j++)
    {

        Rect r = com->get_rect(img, res[j].bbox);
        int class_id = (int)res[j].class_id;
        cv::Scalar color = cv::Scalar(coco::color_list[class_id][0],
                                      coco::color_list[class_id][1],
                                      coco::color_list[class_id][2]);

        rectangle(dst, r, color * 255, 2);
        putText(dst, coco::COCO_CLASSES[class_id],
                Point(r.x, r.y - 1), FONT_HERSHEY_PLAIN, 1.2,
                Scalar(0xFF, 0xFF, 0xFF), 2);
        _bbox mbox;
        mbox.img_width = img.cols;
        mbox.img_height = img.rows;
        mbox.class_id = coco::COCO_CLASSES[(int)res[j].class_id];
        mbox.probability = res[j].conf;
        mbox.xmin = r.x;
        mbox.ymin = r.y;
        mbox.xmax = r.x + r.width;
        mbox.ymax = r.y + r.height;

        bbox.push_back(mbox);
    }
    // imshow("detect", dst);
    // waitKey(1);
}

Mat Yolov5::preprocessImg(Mat &img, int input_w, int input_h)
{
    int w, h, x, y;
    float r_w = input_w / (img.cols * 1.0);
    float r_h = input_h / (img.rows * 1.0);
    if (r_h > r_w)
    {
        w = input_w;
        h = r_w * img.rows;
        x = 0;
        y = (input_h - h) / 2;
    }
    else
    {
        w = r_h * img.cols;
        h = input_h;
        x = (input_w - w) / 2;
        y = 0;
    }
    Mat re(h, w, CV_8UC3);
    resize(img, re, re.size(), 0, 0, INTER_LINEAR);
    Mat out(input_h, input_w, CV_8UC3, Scalar(128, 128, 128));
    re.copyTo(out(Rect(x, y, re.cols, re.rows)));
    return out;
}
