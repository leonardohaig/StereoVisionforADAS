/**
@file StereoVisionForADAS.h
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief matching, stixel creation, objectness wrapper
*/
#pragma once 

#include "opencv2/opencv.hpp"
#include <iostream>

#include "StereoMatching.h"
#include "StixelEstimation.h"
#include "StixelSegmenation.h"
#include "./include/DefStruct.h"

using namespace std;
using namespace cv;

/**
	@class CStereoVisionForADAS
	@brief stixel wrapping class
*/
class CStereoVisionForADAS{
private:
	//----------------input--------------------
	Mat m_imgLeftInput;		///< rectified image,CV_8UC1
	Mat m_imgRightInput;	///< rectified image,CV_8UC1

	//----------------param--------------------
	StereoCamParam_t m_objStereoParam;//双目相机参数
	CStereoMatching m_objStereoMatching;
	CStixelEstimation m_objStixelEstimation;
	CStixelSegmentation m_objStixelSegmentation;

	MATCHING_ERROR match_err;
	STIXEL_ERROR stixel_err;
	SEG_ERROR seg_err;

	//LUT//伪色彩图像的LUT表
	unsigned char m_pseudoColorLUT[256][3]; ///< RGB pseudo color

	//-------------member image----------------
	Mat m_imgColorDisp; ///< 8bit 3ch disparity image
	Mat m_imgStixelGray;//CV_8UC1类型
	Mat m_imgTopView;

	//---------------function------------------
	//Pseudo color Disparity
	void MakePseudoColorLUT(); ///< pseudo color LUT//伪色彩图像的LUT
	void cvtPseudoColorImage(Mat& srcGray, Mat& dstColor); ///< input : gray image, output : color image
	
	void TopViewStixel(vector<stixel_t>& objStixelInROI);
	void TopViewLane(Mat& imgTopView, float fLaneInterval, int nCenterPointX);

public:
	//----------------output-------------------
	/// Matching output
	Mat m_matDisp16;
	Mat m_imgDisp8;

	/// Stixel output
	vector<stixel_t> m_vecobjStixels; ///< stixel output
	vector<stixel_t> m_vecobjStixelInROI; ///< stixel output in 3D ROI
	Mat m_imgGround;//CV_8UC1类型

	/// segmentation output
	vector<Object_t> m_vecobjBB;

	//---------------function------------------
	CStereoVisionForADAS(StereoCamParam_t& objStereoParam);//构造函数
	int Objectness(Mat& imgLeft, Mat& imgRight);
	int Objectness(Mat& imgLeft, Mat& imgRight, Mat& imgDisp8);

	// depth
	int RectToDisp(Rect& rectBox, Mat& matRect);
	int Disp16ToDepth(const uchar fDisparity, float& fDistanceMeter);

	/// display
	void Display();
	void Display(Mat& imgDisplay);
	void Display(Mat& imgDisplay, Mat& imgStixelResult);

	// display, draw
	void DrawLane(Mat& imgResult, StereoCamParam_t& objStereoParam);
	void DrawStixel(Mat& imgResult, vector<stixel_t>& vecobjStixels);
	void DrawGround(Mat& imgResult, Mat& imgGround);

	//根据选择的数据集初始化相关参数
	static StereoCamParam_t InitStereoParam(int nDatasetName);
	//根据pitch角/单位:°和图像大小计算天地线位置
	static int PitchDegToVanishingLine(StereoCamParam_t& objStereoParam);
	
};
