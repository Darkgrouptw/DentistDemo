#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <caffe/caffe.hpp>
#include <string>

using namespace caffe;
using namespace cv;

class SegnetModel
{
public:
	SegnetModel();

	void Load(string, string);

private:
	//////////////////////////////////////////////////////////////////////////
	// ���������ܼ�
	//////////////////////////////////////////////////////////////////////////
	caffe::Net<float> 			*SegNet;
	int							NumChannels;
	//size						InputSize;
};

