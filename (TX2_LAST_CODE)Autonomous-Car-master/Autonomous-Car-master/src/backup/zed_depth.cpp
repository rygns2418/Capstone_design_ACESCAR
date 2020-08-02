#include "zed_depth.h"
#define __DEBUG__

//main call this function to initialize zed camera
void zed_init(sl::InitParameters &init_params, sl::Camera &zed, sl::RuntimeParameters &runtime_parameters)
{
    init_params.camera_resolution = sl::RESOLUTION_HD1080;
    init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = sl::UNIT_METER;

    sl::ERROR_CODE zerr = zed.open(init_params);
    if(zerr != sl::SUCCESS)
    {
		std::cout<<zerr<<std::endl;
		zed.close();
		return;// Quit if an error occurred
	}
    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;
}


//main call this function to get zed depth image
void getDepth(sl::Camera &zed, int y, int x, double &d)
{
	sl::Mat zed_point_cloud, image;
	sl::float4 point_cloud_value[5];
		
	int dx[5] = { 0, 5, 5, -5, -5};
	int dy[5] = { 0, 5, -5, 5, -5};
	vector<double> distance(5,0);

	zed.retrieveMeasure(zed_point_cloud,sl::MEASURE_XYZRGBA); 
    int image_width = image.getWidth(), image_height = image.getHeight();
	
	for(int i=0;i<5;i++)
	{
		//stop distance
		zed_point_cloud.getValue(x+dx[i], y+dy[i], &point_cloud_value[i]);
		distance[i] =  sqrt(point_cloud_value[i].x * point_cloud_value[i].x + point_cloud_value[i].y * point_cloud_value[i].y + point_cloud_value[i].z * point_cloud_value[i].z);
	}
	//get 5 point distance for steering
    int k=0;
	double ave=0;

	for(int i=0;i<5;i++)
	{
		if(distance[i] > 0)
		{
			k++;
			ave += distance[i];
		}
	}
	d = ave / k;
	#ifdef __DEBUG__
	printf("d= %.3lf\n",d);
	#endif
	return;
}


//zed image convert to opencv image
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
	    case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	    case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	    case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	    case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	    case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	    case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	    case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	    case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	    default: break;
	}
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}
//get opencv image
void getCVImage(sl::Camera &zed, sl::RuntimeParameters &runtime_parameters, sl::Mat &image_zed, cv::Mat &image_origin)
{
	int delay=1;
	if(zed.grab(runtime_parameters) == sl::SUCCESS)
	{
	    //Retrieve left image
	    zed.retrieveImage(image_zed, sl::VIEW_LEFT, sl::MEM_CPU);
	    image_origin = slMat2cvMat(image_zed);
	}
	int ckey = waitKey(delay);
	return;
}
