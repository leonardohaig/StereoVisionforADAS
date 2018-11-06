/**
@file StereoMatcing.h
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com).
@brief create disparity map
*/

#pragma once
 
#include "opencv2/opencv.hpp"
#include <iostream>

#if CV_MAJOR_VERSION==3
//#include "opencv2\ximgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#endif

#include "./include/DefStruct.h"

using namespace std;
using namespace cv;


#if CV_MAJOR_VERSION==3
using namespace cv::ximgproc;
#endif

typedef enum error { NO_PROB, IMGSIZE_ERR, IMGSCALE_ERR } MATCHING_ERROR;
/**
	@class CStereoMatching
	@brief disparity map
*/
class CStereoMatching{
private:
	//----------------input-------------------
	Mat m_imgLeftInput;		///< rectified image
	Mat m_imgRightInput;	///< rectified image

	//----------------param-------------------
#if CV_MAJOR_VERSION==3
	Ptr<StereoBM> bm = StereoBM::create(0, 0); ///< default construct
	Ptr<DisparityWLSFilter> wls_filter;
	Rect m_rectFilterROI;
#endif
#if CV_MAJOR_VERSION == 2
	StereoBM bm;
#endif
	StereoCamParam_t m_objStereoParam;

	//-------------member image---------------

	//---------------function------------------
	void SetParamOCVStereo(); ///< Stereo parameter setting
	MATCHING_ERROR MakeDisparity();//计算视差,首先计算m_matDisp16，然后转换为m_matDisp8
	MATCHING_ERROR ImproveDisparity(Mat& imgDisp8);

public:

	//----------------output-------------------
	Mat m_matDisp16;
	Mat m_imgDisp8;

	//---------------function------------------
	CStereoMatching(StereoCamParam_t& objStereoParam);
	CStereoMatching(Mat& imgLeftInput, Mat& imgRightInput, StereoCamParam_t& objStereoParam);
	
	//Set param
	void SetParamOCVStereo(StereoCamParam_t& objStereoParam);

	//make disparity
	MATCHING_ERROR SetImage(Mat& imgLeft, Mat& imgRight);
	MATCHING_ERROR MakeDisparity(Mat& imgLeft, Mat& imgRight, bool flgUseWLSFilter=true);//计算视差，输入：已矫正的灰度图
	MATCHING_ERROR MakeDisparity(Mat& imgLeft, Mat& imgRight, Mat& matDisp16);//调用computer函数计算视差，函数内部根据OpenCV版本的不同，computer函数的调用方式也不同
	MATCHING_ERROR ImproveDisparity_Naive(Mat& imgDisp8);
	MATCHING_ERROR ImproveDisparity_WLSFilter(Mat& imgDisp8); ///< OCV310 new disparity postprocess.对视差图进行滤波操作，以使其平滑

};