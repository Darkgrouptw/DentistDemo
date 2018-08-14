#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <caffe/caffe.hpp>
#include <string>

using namespace caffe;
using namespace std;

ref class SegnetModel
{
public:
	SegnetModel();

	void Load(string, string);

private:
	Net<float> *SegNet;
};

