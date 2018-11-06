/**
@file StixelEstimation.cpp
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief stixel creation
*/
#pragma once 

#include <iostream>

#include "./include/DefStruct.h"

using namespace std;
using namespace cv;

typedef enum stixel_error { OK, SIZE_ERR, GND_ERR, VDISP_ERR, GNDRANSAC_ERR } STIXEL_ERROR;

/**
	@class CStixelEstimation
	@brief stixel ������ ���� class
*/
class CStixelEstimation{
private:
	//----------------input-------------------
	Mat m_matDisp16;
	Mat m_imgDisp8;

	//----------------param-------------------
	StereoCamParam_t m_objStereoParam;
	int m_nStixelWidth;

	double m_dGroundVdispSlope; ///< The ground line slope in v-disparity//斜率
	double m_dGroundVdispOrig; ///< The ground line origin in v-disparity
	int m_nVdispGroundThreshold; ///< Ground plane threshold value in Vdisparity map.V-视差图滤波阈值，下限,上限为255
	int m_nStixelGroundMargin;	///< Marginal compensation value for ground plane//地面的边际补偿阈值

	vector<Point2f> m_vecLinePoint; ///< v-disparity point : ground point//地面点的像素坐标
	Vec4f m_vec4fLine; ///< ground line//对地面点的像素坐标进行RANSAC拟合得到的直线参数

	//-------------member image---------------
	Mat m_imgVdisp;//V-视差图
	Mat m_imgTopView;

	//---------------function------------------

public:
	//----------------output-------------------
	vector<stixel_t> m_vecobjStixels; ///< stixel output
	vector<stixel_t> m_vecobjStixelInROI; ///< stixel output in 3D ROI
	Mat m_imgGround; ///< ground

	//---------------function------------------
	CStixelEstimation(StereoCamParam_t& objStereoParam);
	STIXEL_ERROR SetDispImage(Mat& matDisp16);
	STIXEL_ERROR SetDispImage(Mat& matDisp16, Mat& imgDisp8);//设置函数所处理的视差图对象
	STIXEL_ERROR SetDisp8Image(Mat& imgDisp8);

	//============processing part==============
	//wrapping function
	STIXEL_ERROR EstimateStixels(Mat& matDisp16);
	STIXEL_ERROR EstimateStixels(Mat& matDisp16, Mat& imgDisp8, bool flgUseMultiLayer=true);
	STIXEL_ERROR EstimateStixels_only8bitDisp(Mat& imgDisp8, bool flgUseMultiLayer=true);

	//Ground Estimation
	STIXEL_ERROR GroundEstimation(Mat& imgDisp8);

	STIXEL_ERROR ComputeVDisparity(Mat& imgDisp8);//计算V-视差图.只计算天地线以下的
	STIXEL_ERROR RmVDisparityNoise(Mat& imgVdisp);//对V视差图滤波，去除噪声（二值化操作）
	STIXEL_ERROR ExtractGroundPoint(Mat& imgVdisp, vector<Point2f>& vecLinePoint);//从V-视差图中提取地面点。vecLinePoint保存的为地面点的像素坐标,来源与V-视差
	STIXEL_ERROR FitLineRansac(vector<Point2f>& vecLinePoint, Vec4f& vec4fLine);//对V-视差图中提取地面点进行RANSAC直线拟合
	STIXEL_ERROR RmGround(Vec4f vec4fLine, Mat& imgDisp8);

	//Sky Estimation
	STIXEL_ERROR RmSky(Mat& imgDisp8);

	//Stixel distance estimation
	STIXEL_ERROR StixelDistanceEstimation(Mat& imgDisp8, vector<stixel_t>& vecStixels, bool flgUseMultiLayer=true);
	STIXEL_ERROR StixelDisparityEstimation_col_SL(Mat& imgDisp8, int col, stixel_t& objStixel); ///< column single layer
	STIXEL_ERROR StixelDisparityEstimation_col_ML(Mat& imgDisp8, int col, vector<stixel_t>& vecStixels); ///< column multi layer

	STIXEL_ERROR StixelDisparityToDistance(vector<stixel_t>& vecStixels);

	//Stixel Optimization
	//STIXEL_ERROR StixelROIConstraint(vector<stixel_t>& vecStixelsInput, vector<stixel_t>& vecStixelsOutput);
	STIXEL_ERROR StixelROIConstraint_Lane(vector<stixel_t>& vecStixelsInput, vector<stixel_t>& vecStixelsOutput, float fLaneInterval, int nCenterPointX);

};