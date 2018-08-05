#pragma managed(push,on)  
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

//using namespace std;

struct RawDataProperty
{
	int size_X;
	int size_Y;
	int size_Z;
	int sample;
};

class DataManager
{
public:
	DataManager(void);
	~DataManager(void);

	
	void ReadCalibrationData();
	void Read_test_file();

	float dataQuaternion[4];
	RawDataProperty rawDP;
	int Mapping_X;
	int Mapping_Y;
	float* MappingMatrix;
	float zRatio;
};

