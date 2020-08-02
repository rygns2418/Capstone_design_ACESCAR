#ifndef __ZED_DEPTH_H__
#define __ZED_DEPTH_H__
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#ifndef __CV_STD_NAMESPACE__
#define __CV_STD_NAMESPACE__
using namespace std;
using namespace cv;
#endif

void zed_init(sl::InitParameters &init_params, sl::Camera &zed, sl::RuntimeParameters &runtime_parameters);
void getDepth(sl::Camera &zed, int y, int x, double &d);
cv::Mat slMat2cvMat(sl::Mat& input);
void getCVImage(sl::Camera &zed, sl::RuntimeParameters &runtime_parameters, sl::Mat &image_zed, cv::Mat &image_origin);

#endif
