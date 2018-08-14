#include "SegnetModel.h"

SegnetModel::SegnetModel()
{

}

void SegnetModel::Load(string ModelDefPath, string ModelPath)
{
	Caffe::set_mode(Caffe::GPU);
	//TEST
	//SegNet = new Net<float>(ModelDefPath, TEST);
}
