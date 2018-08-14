#include "SegnetModel.h"

SegnetModel::SegnetModel()
{
	Caffe::set_mode(Caffe::GPU);
	
}

void SegnetModel::Load(string ModelDefPath, string ModelPath)
{
	//TEST
	SegNet = new Net<float>(ModelDefPath, TEST);
	SegNet->CopyTrainedLayersFrom(ModelPath);


	/*Blob<float>* input_layer = SegNet->input_blobs()[0];
	NumChannels = input_layer->channels();
	InputSize = Size(input_layer->width(), input_layer->height());
	std::cout << "Network\t=> " << NumChannels << std::endl;
	std::cout << "Width\t=>" << input_layer->width() << std::endl;
	std::cout << "Height\t=>" << input_layer->height() << std::endl;*/
}
