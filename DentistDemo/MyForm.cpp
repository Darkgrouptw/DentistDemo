﻿#include "MyForm.h"

using namespace System::Threading::Tasks;

System::Void DentistDemo::MyForm::MyForm_Load(System::Object^ sender, System::EventArgs^ e)
{
	mResources = (gcnew System::ComponentModel::ComponentResourceManager(MyForm::typeid));
	sysManager = new SystemManager();

	mState_Connection = State_Connection::Start;
	mState_Processing = State_Processing::Idle;

	// Initialize BLE devices
	bleDeviceL = gcnew BluetoothLeDevice();
	bleDeviceR = gcnew BluetoothLeDevice();

	/*preQuaternL = nullptr;
	preQuaternR = nullptr;*/

	// Register for callbacks
	bleDeviceL->RegisterCallback_DeviceInitDone(gcnew BLECallbacks::BLECallback_DeviceInitDone(this, &MyForm::Callback_DeviceInitDone));
	//bleDeviceR->RegisterCallback_DeviceInitDone(gcnew BLECallbacks::BLECallback_DeviceInitDone(this, &MyForm::Callback_DeviceInitDone));
	std::cout << "DeviceInitDone" << std::endl;
	bleDeviceL->RegisterCallback_DeviceCloseDone(gcnew BLECallbacks::BLECallback_DeviceCloseDone(this, &MyForm::Callback_DeviceCloseDone));
	//bleDeviceR->RegisterCallback_DeviceCloseDone(gcnew BLECallbacks::BLECallback_DeviceCloseDone(this, &MyForm::Callback_DeviceCloseDone));
	std::cout << "DeviceCloseDone" << std::endl;
	bleDeviceL->RegisterCallback_DeviceDiscovered(gcnew BLECallbacks::BLECallback_DeviceDiscovered(this, &MyForm::Callback_DeviceDiscovered));
	//bleDeviceR->RegisterCallback_DeviceDiscovered(gcnew BLECallbacks::BLECallback_DeviceDiscovered(this, &MyForm::Callback_DeviceDiscovered));
	std::cout << "DeviceDiscovered" << std::endl;
	bleDeviceL->RegisterCallback_EstablishLinkDone(gcnew BLECallbacks::BLECallback_EstablishLinkDone(this, &MyForm::Callback_EstablishLinkDone));
	//bleDeviceR->RegisterCallback_EstablishLinkDone(gcnew BLECallbacks::BLECallback_EstablishLinkDone(this, &MyForm::Callback_EstablishLinkDone));
	std::cout << "EstablishLinkDone" << std::endl;
	bleDeviceL->RegisterCallback_TerminateLinkDone(gcnew BLECallbacks::BLECallback_TerminateLinkDone(this, &MyForm::Callback_TerminateLinkDone));
	//bleDeviceR->RegisterCallback_TerminateLinkDone(gcnew BLECallbacks::BLECallback_TerminateLinkDone(this, &MyForm::Callback_TerminateLinkDone));
	std::cout << "TerminateLinkDone" << std::endl;
	bleDeviceL->RegisterCallback_HandleValueNotification(gcnew BLECallbacks::BLECallback_HandleValueNotification(this, &MyForm::Callback_HandleValueNotification));
	//bleDeviceR->RegisterCallback_HandleValueNotification(gcnew BLECallbacks::BLECallback_HandleValueNotification(this, &MyForm::Callback_HandleValueNotification));
	std::cout << "HandleValueNotification" << std::endl;
	bleDeviceL->RegisterCallback_HandleCheckNotifyStatus(gcnew BLECallbacks::BLECallback_HandleCheckNotifyStatus(this, &MyForm::Callback_HandleCheckNotifyStatus));
	//bleDeviceR->RegisterCallback_HandleCheckNotifyStatus(gcnew BLECallbacks::BLECallback_HandleCheckNotifyStatus(this, &MyForm::Callback_HandleCheckNotifyStatus));
	std::cout << "HandleCheckNotifyStatus" << std::endl;
	bleDeviceL->RegisterCallback_HandleChangeNotifyStatus(gcnew BLECallbacks::BLECallback_HandleChangeNotifyStatus(this, &MyForm::Callback_HandleChangeNotifyStatus));
	//bleDeviceR->RegisterCallback_HandleChangeNotifyStatus(gcnew BLECallbacks::BLECallback_HandleChangeNotifyStatus(this, &MyForm::Callback_HandleChangeNotifyStatus));
	std::cout << "HandleChangeNotifyStatus" << std::endl;
	UpdateConnectionStatus(State_Connection::DeviceReady);



	
	//pointCloudSet = new std::vector<std::vector<GlobalRegistration::Point3D>>;
	//finalPC = new std::vector<GlobalRegistration::Point3D>;
	//aligned = new std::vector<GlobalRegistration::Point3D>;
	//tmp_alignedPC1 = new std::vector<GlobalRegistration::Point3D>;
	//tmp_alignedPC2 = new std::vector<GlobalRegistration::Point3D>;
	//overlap_cloud1 = new std::vector<std::vector<GlobalRegistration::Point3D>>;
	//overlap_cloud2 = new std::vector<std::vector<GlobalRegistration::Point3D>>;
	//pointCloud_aligned = new std::vector<std::vector<GlobalRegistration::Point3D>>;
	translate_mat = new std::vector<GlobalRegistration::Match4PCSBase::MatrixType>;
	Combine_cloud_PC = new std::vector<PointData>;
	//finalPCid = new std::vector<int>;
	//nearest_idx = new std::vector<int>;
	quat1 = new Quaternion;
	quat2 = new Quaternion;
	quat3 = new Quaternion;
	quat_tmp = new Quaternion;
	all_quat = new Quaternion;
	camera_quat = new Quaternion;
	theta_diff = new Radian;
	output_diff = new Radian;
	rotationAxix_diff = new Vector3;
	output_rotVec = new Vector3;
	
	LocalTransform_M = new glm::mat4;
	float_rotate_M = new GLfloat;
	tmp_center = new Vector3;
	Initial_OK = false;
	show_first_point = true;
	show_second_point = true;
	show_third_point = true;
	show_tmp_align1 = true;
	show_tmp_align2 = true;
	show_aligned_cloud = true;
	theTRcuda = new TRcuda;
	//acc_vector1 = new Vector3;
	//acc_vector2 = new Vector3;
	//acc_vector3 = new Vector3;
	//velocity_offset = new Vector3;
	//movement_vector = new Vector3;
	//result_movement = new Vector3;
	//final_max = new Vector3;
	//final_min = new Vector3;
	//pre_velocity = new Vector3;
	//now_velocity = new Vector3;
	pre_acc = new Vector3;
	now_acc = new Vector3;
	draw_plane1 = new Vector3;
	draw_plane2 = new Vector3;
	draw_plane3 = new Vector3;
	draw_plane4 = new Vector3;
	combine_max = new Vector3(0,0,0);
	combine_min = new Vector3(99999, 99999, 99999);
	//outside_center = new std::vector<Vector3>;
	//Point_cloud_Max = new std::vector<Vector3>;
	//Point_cloud_Min = new std::vector<Vector3>;
	//Point_cloud_center = new std::vector<Vector3>;
	Plane_vector = new std::vector<Vector3>;
	overlap_idx1 = new std::vector<std::vector<int>>;
	overlap_idx2 = new std::vector<std::vector<int>>;
	filename_array = new std::vector<file_name>;
	//tmp_input_quat = new std::vector<Quaternion>;
	//pointCloud_radian = new std::vector<Radian>;
	//pointCloud_quat_vector = new std::vector<Vector3>;
	tmp_MM = new glm::mat4;
	mani_MM = new glm::mat4;
	reset_M = new glm::mat4;
	is_camera_move = true;
	input_rotVec = new std::vector<Vector3>;
	input_dgree = new std::vector<float>;
	//cloud_vec_arr = new std::vector<Vector3>;
	all_quat1 = new Quaternion;
	//near_idx2 = new std::vector<int>;
	PointCloudArr = new std::vector<PointCloudArray>;
	//show_third_point = false;
	obj1 = new objData;
	obj2 = new objData;
	obj3 = new objData;
	obj4 = new objData;
	obj5 = new objData;

	tmp_fileName = new std::string;
	out_fileName = new std::string;
	is_before_camera = false;
	show_noGyro_PC = false;
	ErrorString = new char[500];
	octstatus = new char[10];
	ErrorString_len_in = 500;
	ErrorString_len_out = 0;
	fftnumber = 2048;

	scan_count = 0;
	volumeZ_now = 1;
	super_theshold = 0.35;
	super_max_time = 1;
	super_max_angle = 20;
	super_sample_point = 500;
	super_overlap = 0.4;
	super_delta = 0.1;
	super_iterTre = 0.4;
	super_iterCount = 1;

	full_scan_time = 0;
	show_Cloud_1ID = 0;

	color_idt = 2.2;
	//pic_num = 125;
	slice_p1 = new Vector3;
	slice_p2 = new Vector3;
	slice_p3 = new Vector3;
	slice_p4 = new Vector3;
	teeth_1 = new std::vector<Teeth_PC>;
	teeth_2 = new std::vector<Teeth_PC>;
	teeth_3 = new std::vector<Teeth_PC>;
	teeth_4 = new std::vector<Teeth_PC>;
	teeth_5 = new std::vector<Teeth_PC>;
	tmp_pic_teeth = new std::vector<int>;
	tmp_pic_meat = new std::vector<int>;
	tmp_pic_disease = new std::vector<int>;
	tmp_display_PC = new Teeth_PC;
	tmp_Mask_Model = new cv::Mat/*(250, 141, CV_8UC3, cv::Scalar(0, 0, 0))*/;
	tmp_tooth_M = new cv::Mat;
	(*tmp_tooth_M) = cv::imread("./image1.jpg");
	
	
	teeth_slice_idx = false;
	meat_slice_idx = false;
	disease_slice_idx = false;
	alpha_dis_teeth = 0;
	alpha_dis_meat = 0;
	alpha_dis_disease = 0;

	glManager->Rotate_90();
	all_time = 0;
	Push_back_file();
}

System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
	

}
void DentistDemo::MyForm::Gyro_test_Click(System::Object^  sender, System::EventArgs^  e) {
	gyro_test = !gyro_test;
}
void DentistDemo::MyForm::UpdateConnectionStatus(State_Connection state_to)
{
	if (this->InvokeRequired)
	{
		this->Invoke(gcnew UpdateConnectionStatusDelegate(this, &MyForm::UpdateConnectionStatus)
			, gcnew cli::array<Object^>{ state_to });
	}
	else
	{
		switch (mState_Connection)
		{
#pragma region State_Connection::Start
		case State_Connection::Start:
			// Start -> NewDevice -> DeviceReady
			if (state_to == State_Connection::DeviceReady)
				mState_Connection = State_Connection::DeviceReady;
			break;
#pragma endregion
#pragma region State_Connection::DeviceReady
		case State_Connection::DeviceReady:
			// DeviceReady -> ComOpen -> ComOpened
			if (state_to == State_Connection::ComOpened)
			{
				if (bleDeviceL->IsInitialized/* && bleDeviceR->IsInitialized*/)
				{
					// Initialized/Connected
					//btnComOpen->Image = (cli::safe_cast<System::Drawing::Image^>(mResources->GetObject(L"btnComClose.Image")));

					// Update status
					mState_Connection = State_Connection::ComOpened;
				}
				else
				{

				}
			}
			else
			{

			}
			break;
#pragma endregion State_Connection::DeviceReady
#pragma region State_Connection::ComOpened
		case State_Connection::ComOpened:
			// ComOpened -> Establish -> BleEstablished
			if (state_to == State_Connection::BleEstablished)
			{
				if (bleDeviceL->IsNotifying /*&& bleDeviceR->IsNotifying*/)
				{
					// Initialized/Connected
					//btnBleEstablish->Image = (cli::safe_cast<System::Drawing::Image^>(mResources->GetObject(L"btnBleTerminate.Image")));

					// Update status
					mState_Connection = State_Connection::BleEstablished;
				}
				else
				{

				}
			}
			// ComOpened -> ComClose -> DeviceReady
			else if (state_to == State_Connection::DeviceReady)
			{
				if (!bleDeviceL->IsInitialized/* && !bleDeviceR->IsInitialized*/)
				{
					// Initialized/Connected
					//btnComOpen->Image = (cli::safe_cast<System::Drawing::Image^>(mResources->GetObject(L"btnComOpen.Image")));

					// Update status
					mState_Connection = State_Connection::DeviceReady;
				}
				else
				{

				}
			}
			break;
#pragma endregion State_Connection::DeviceInitialized
#pragma region State_Connection::BleEstablished
		case State_Connection::BleEstablished:
			// BleEstablished -> Terminate -> ComOpened
			if (state_to == State_Connection::ComOpened)
			{
				if (!bleDeviceL->IsEstablished /*&& !bleDeviceR->IsEstablished*/)
				{
					//btnBleEstablish->Image = (cli::safe_cast<System::Drawing::Image^>(mResources->GetObject(L"btnBleEstablish.Image")));

					mState_Connection = State_Connection::ComOpened;
				}
				else
				{

				}
			}
			else
			{

			}
			break;
#pragma endregion State_Connection::BleEstablished
		default:
			break;
		}
	}
}
void DentistDemo::MyForm::Initial_scan_Click(System::Object^  sender, System::EventArgs^  e) {

	//final_max->x = 0;
	//final_max->y = 0;
	//final_max->z = 0;
	//final_min->x = 999999;
	//final_min->y = 999999;
	//final_min->z = 999999;

	
	
	PointCloud_idx_now = 1;

	pin_ptr<int32_t> tmp_deviceID = &deviceID;

	
	//if (Initial_OK) {
	//	pointCloudSet->clear();
	//	Initial_OK = false;
	//	PointCloud_idx_now = 1;
	//}
	file_idx = 0;
	//delete result_movement;

	//result_movement = new Vector3;
	//std::cout << "Initial OK is: " << Initial_OK << std::endl;
	
	std::cout << "Initial OK" << std::endl;

}
void DentistDemo::MyForm::btnComUpdate_Click(System::Object^  sender, System::EventArgs^  e) {
	// Update COM port combobox
	cbComPortL->Items->Clear();
	//cbComPortR->Items->Clear();

	cli::array<Object^>^ comports = SerialPort::GetPortNames();

	cbComPortL->Items->AddRange(comports);
	//cbComPortR->Items->AddRange(comports);
	//std::cout <<  << std::endl;
}
void DentistDemo::MyForm::btnComOpen_Click(System::Object^  sender, System::EventArgs^  e) {
	if (mState_Connection == State_Connection::DeviceReady)
	{
		// Open COM Port Phase
		// Check item selection
		if (!(cbComPortL->SelectedIndex > -1) /*|| !(cbComPortR->SelectedIndex > -1)*/)
		{
			MessageBox::Show("No COM port selected");
			return;
		}

		// Get selected COM port
		System::String ^comL = cbComPortL->SelectedItem->ToString(); // left COM port
															 //String ^comR = cbComPortR->SelectedItem->ToString(); // right COM port

															 // Check different COM port selected and not null
															 //if (String::Compare(comL, comR, false) == 0)
															 //{
															 //	MessageBox::Show("Same COM port selected");
															 //	return;
															 //}

															 // Open the BLE device to read the COM port
															 // and start the communication between PC and dongles
		if (!bleDeviceL->Initialize(comL))
		{
			MessageBox::Show("Failed COM port open: " + comL);
		}

		//if (!bleDeviceR->Initialize(comR))
		//{
		//	MessageBox::Show("Failed COM port open: " + comR);
		//}

		std::cout << "Initialize:" << std::endl;
	}
	else if (mState_Connection == State_Connection::ComOpened)
	{
		// Close COM Port Phase
		bleDeviceL->Close();
		//bleDeviceR->Close();
	}
	else
	{

	}
}
void DentistDemo::MyForm::btnBleScan_Click(System::Object^  sender, System::EventArgs^  e) {
	// Cancel any BLE device discovery process
	bleDeviceL->ScanCancel();


	// Clear Bluetooth LE device combobox and list
	bleDeviceList = gcnew Dictionary<System::String^, DeviceInfo^>();
	cbBleDeviceL->DataSource = nullptr;
	//cbBleDeviceR->DataSource = nullptr;

	// Scan Bluetooth LE device through one of the dongle
	bleDeviceL->Scan();

}
void DentistDemo::MyForm::btnBleEstablish_Click(System::Object^  sender, System::EventArgs^  e) {
	if (mState_Connection == State_Connection::ComOpened)
	{
		// Establish BLE Phase

		// Cancel any BLE device discovery process
		bleDeviceL->ScanCancel();

		// Check item selection
		if (!(cbBleDeviceL->SelectedIndex > -1) /*|| !(cbBleDeviceR->SelectedIndex > -1)*/)
		{
			MessageBox::Show("No Bluetooth LE device selected");
			return;
		}

		// Get selected Bluetooth LE device address
		KeyValuePair<System::String^, DeviceInfo^> deviceL = (KeyValuePair<System::String^, DeviceInfo^>)(cbBleDeviceL->SelectedItem); // left hand BLE device
																													   //KeyValuePair<String^, DeviceInfo^> deviceR = (KeyValuePair<String^, DeviceInfo^>)(cbBleDeviceR->SelectedItem); // right hand BLE device

																													   // Check different Bluetooth LE devices are selected
																													   //if (String::Compare(deviceL.Key, deviceR.Key, false) == 0)
																													   //{
																													   //	MessageBox::Show("Same Bluetooth LE device selected");
																													   //	return;
																													   //}

																													   // Establish the Bluetooth LE device for communication
		bleDeviceL->Establish(deviceL.Value);
		//bleDeviceR->Establish(deviceR.Value);
	}
	else if (mState_Connection == State_Connection::BleEstablished)
	{
		// Terminate BLE Phase
		bleDeviceL->StopNotification();
		//bleDeviceR->StopNotification();
	}

}
void DentistDemo::MyForm::Single_scan_Click(System::Object^  sender, System::EventArgs^  e) {

	//AboutADC(deviceID);
	/////////////////////////////////////////////////////////H_StartCap///////////////////////////////////////////////
	float LV_65 = 65;
	char SaveName[] = "V:\OCT0107";
	unsigned int SampRec = 2048;
	//Savedata = false;
	//ErrorBoolean = false;
	ByteLen = 1;

	pin_ptr<int32_t> tmp_ErrorString_len_out = &ErrorString_len_out;
	pin_ptr<uint32_t> tmp_Handle = &HandleOut;
	pin_ptr<uint32_t> tmp_ByteLen = &ByteLen;
	//pin_ptr<LVBoolean> tmp_ErrorBoolean = &ErrorBoolean;


	//StartCap(deviceID, tmp_Handle, LV_65, SampRec, tmp_ByteLen, Savedata, SaveName,tmp_ErrorBoolean, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
	//test_start_cap(deviceID, tmp_Handle, LV_65, SampRec, tmp_ByteLen, Savedata, SaveName, tmp_ErrorBoolean, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);


	/////////////////////////////////////////////////////////H_StartCap///////////////////////////////////////////////
	/////////////////////////////////////////////////////////H_ScanADC///////////////////////////////////////////////

	AllDatabyte = ByteLen * 2;
	ArrSize = new unsigned short[PIC_SIZE];
	outarr = new unsigned short[PIC_SIZE];
	short_arr = new unsigned short[PIC_SIZE];
	char_arr = new char[PIC_SIZE * 2];

	OutArrLenIn = 2048 * 500 * 2;
	OutArrLenOut = 0;
	//button_stop_enable = true;
	pin_ptr<int32_t> tmp_OutArrLenOut = &OutArrLenOut;

	//ScanADC(HandleOut, AllDatabyte, ArrSize, ByteLen, outarr, OutArrLenIn, tmp_OutArrLenOut, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
	test_scanADC(HandleOut, AllDatabyte, ArrSize, ByteLen, outarr, OutArrLenIn, tmp_OutArrLenOut, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);

	for (int i = 0; i < 100; i++) {
		std::cout << "outarr[" << i << "]: " << outarr[i] << std::endl;
	}
	std::cout << "tmp_OutArrLenOut:" << OutArrLenOut << std::endl;

	unsigned_short_to_char(outarr, PIC_SIZE, char_arr);


	theTRcuda->RawToPointCloud(char_arr, PIC_SIZE, 250, 2048, 1024);

}
void DentistDemo::MyForm::hkoglPanelControl1_Load(System::Object^  sender, System::EventArgs^  e) {
	// Initialize GLEW
	GLenum result = glewInit();
	if (result != GLEW_OK)
	{
		std::cout << "Error: " << glewGetErrorString(result) << std::endl;
		return;
	}

	// Initialize OpenGL
	glManager = new GLManager();
	theTRcuda = new TRcuda();
	DManager = new DataManager;

	thePointSize = 3.0f;
	volumeDataIdx = 125;
	meat_alpha = 1.0f;

	showVolumeData = true;// true false
	showPointType = true;
	showBoardTemp = true;
	showPointCloud = false;
	can_rotate = false;
	//timer1->Start();
	load_obj();
	result_input = new std::vector<cv::Mat>;
	decay_imgPC = new std::vector<Point_3D>;
	teeth_imgPC = new std::vector<Point_3D>;
	meat_imgPC = new std::vector<Point_3D>;
	disease_imgPC = new std::vector<Point_3D>;
	RGB_image = new std::vector<cv::Mat>;
	Result_Image = new std::vector<cv::Mat>;
	raw_input = new std::vector<cv::Mat>;

	DManager->ReadCalibrationData();
	//load_result();
	//decay_img2space();

	std::cout << "GL initial OK " << std::endl;
}
void DentistDemo::MyForm::clear_PC_buffer() {
	if ((*teeth_imgPC).size() != 0);
		(*teeth_imgPC).clear();
	if ((*disease_imgPC).size() != 0);
		(*disease_imgPC).clear();
	if ((*meat_imgPC).size() != 0);
		(*meat_imgPC).clear();
	if ((*raw_input).size() != 0)
		(*raw_input).clear();
	if ((*Result_Image).size() != 0)
		(*Result_Image).clear();
	if ((*RGB_image).size() != 0)
		(*RGB_image).clear();
	if ((*tmp_pic_teeth).size() != 0);
		(*tmp_pic_teeth).clear();
	if ((*tmp_pic_meat).size() != 0);
		(*tmp_pic_meat).clear();
	if ((*tmp_pic_disease).size() != 0);
		(*tmp_pic_disease).clear();
	if ((*tmp_Mask_Model).empty() == 0);
		(*tmp_Mask_Model).release();
}
void DentistDemo::MyForm::Full_scan_Click(System::Object^  sender, System::EventArgs^  e) {
	pin_ptr<int32_t> tmp_deviceID = &deviceID;
	InitADC(4, tmp_deviceID);
	clock_t scan_t1 = clock();

	//AboutADC(deviceID);
	/////////////////////////////////////////////////////////H_StartCap///////////////////////////////////////////////
	float LV_65 = 65;
	char SaveName[] = "V:\\OCT20170928";
	unsigned int SampRec = 2048;

	char SaveName2[1024];
	std::string name_title = "V:\\";

	(*out_fileName) = name_title + (*out_fileName);

	strcpy(SaveName2, out_fileName->c_str());

	#ifndef TEST_NO_OCT
	SerialPort port("COM6", 9600);
	port.Open();

	if (!port.IsOpen) {
		std::cout << "COM6 fail to open!" << std::endl;
	}
	#endif // !TEST_NO_OCT
	Sleep(100);

	Savedata = true;
	ErrorBoolean = false;
	ByteLen = 1;


	pin_ptr<int32_t> tmp_ErrorString_len_out = &ErrorString_len_out;
	pin_ptr<uint32_t> tmp_Handle = &HandleOut;
	pin_ptr<uint32_t> tmp_ByteLen = &ByteLen;
	pin_ptr<LVBoolean> tmp_ErrorBoolean = &ErrorBoolean;


	#ifndef TEST_NO_OCT
	StartCap(deviceID, tmp_Handle, LV_65, SampRec, tmp_ByteLen, Savedata, SaveName, tmp_ErrorBoolean, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
	port.RtsEnable = true;
	#else
	test_start_cap(deviceID, tmp_Handle, LV_65, SampRec, tmp_ByteLen, Savedata, SaveName2, tmp_ErrorBoolean, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
	#endif // !TEST_NO_OCT
	


	/////////////////////////////////////////////////////////H_StartCap///////////////////////////////////////////////


	/////////////////////////////////////////////////////////H_ScanADC///////////////////////////////////////////////

	AllDatabyte = ByteLen * 2;
	ArrSize = new unsigned short[PIC_SIZE];
	outarr = new unsigned short[PIC_SIZE];
	final_oct_arr = new unsigned short[PIC_SIZE * 125];
	final_oct_char = new char[PIC_SIZE * 250];

	OutArrLenIn = 2048 * 500 * 2;
	OutArrLenOut = 0;
	//button_stop_enable = true;
	pin_ptr<int32_t> tmp_OutArrLenOut = &OutArrLenOut;
	int pic_count = 0;
	clock_t t3, t4, t5;
	bool change_point_cloud = false;

	//Sleep(100);

	t3 = clock();

	unsigned short *interator;
	interator = new unsigned short;
	interator = final_oct_arr;
	//final_oct_arr = final_oct_arr + PIC_SIZE * 62;


	while (pic_count < 125) {
		#ifdef TEST_NO_OCT
		test_scanADC(HandleOut, AllDatabyte, ArrSize, ByteLen, outarr, OutArrLenIn, tmp_OutArrLenOut, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
		#else
		ScanADC(HandleOut, AllDatabyte, ArrSize, ByteLen, outarr, OutArrLenIn, tmp_OutArrLenOut, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
		#endif // TEST_NO_OCT



		//for (int i = 0; i < PIC_SIZE; i++) {
		//	final_oct_arr[i + pic_count * PIC_SIZE] = outarr[i];
		//}
		//*interator = *outarr;
		//std::cout << "pic_count" << pic_count << std::endl;


		memcpy(final_oct_arr, outarr, sizeof(unsigned short) * PIC_SIZE);

		pic_count++;

		//std::cout << "pic_count" << pic_count << std::endl;
		//if(pic_count!= 249)

		final_oct_arr = final_oct_arr + PIC_SIZE;

		//if (pic_count == 63&& !change_point_cloud) {
		//	final_oct_arr = interator;
		//	change_point_cloud = true;
		//}


	}
	#ifndef TEST_NO_OCT
	AboutADC(deviceID);
	port.RtsEnable = false;
	port.Close();
	//std::cout << "tmp_char_finish " << std::endl;
	#endif // !TEST_NO_OCT

	//for (int i = PIC_SIZE * 125 - 100; i < PIC_SIZE * 125 ; i++) {
	//	std::cout << "interator: " << interator[i] << std::endl;
	//}

	t4 = clock();

	unsigned_short_to_char(interator, PIC_SIZE * 125, final_oct_char);
	t5 = clock();

	std::vector<char> tmp_char(final_oct_char, final_oct_char + PIC_SIZE * 250);
	//memccpy(tmp_char.data(), final_oct_char,std::strlen(final_oct_char), sizeof(char));

	std::cout << "to array time:" << (t4 - t3) / (double)(CLOCKS_PER_SEC) << "s" << std::endl;
	std::cout << "short to char time:" << (t5 - t4) / (double)(CLOCKS_PER_SEC) << "s" << std::endl;

	//int scanline = (2048 * 2) * 250 * 2;
	//for (int i = tmp_char.size() - scanline - 1; i > 0; i -= scanline * 2)
	//{
	//	tmp_char.erase(tmp_char.begin() + i, tmp_char.begin() + i + scanline);
	//}
	//if (theTRcuda != NULL) {
	//	delete theTRcuda;
	//	theTRcuda = new TRcuda;
	//}

	theTRcuda->RawToPointCloud(tmp_char.data(), tmp_char.size(), 250, 2048);
	//std::cout << "Scan3D!" << std::endl;
	scan_count++;

	int PCidx, Mapidx;
	float ratio = 1;
	float zRatio = DManager->zRatio;
	PointCloudArray tmpPC_T;
	//std::vector<PointData> tmpPC;
	//std::vector<int> tmp_VolumeData;

	for (int x = 2; x < theTRcuda->VolumeSize_X - 2; x++)
	{
		for (int y = 0; y < theTRcuda->VolumeSize_Y; y++)
		{
			if (x > DManager->Mapping_X || y > DManager->Mapping_Y)
				continue;
			//Mapidx = ((y)* theTRcuda->VolumeSize_X) + x;
			Mapidx = ((y * theTRcuda->sample_Y * theTRcuda->VolumeSize_X) + x) * theTRcuda->sample_X;
			PCidx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
			if (theTRcuda->PointType[PCidx + 3] == 1 && theTRcuda->PointType[PCidx + 1] != 0)
			{
				GlobalRegistration::Point3D tmp;
				PointData tmp_Data;
				//int tmpVolumeData;
				//tmp.x() = DManager->MappingMatrix[Mapidx * 2] * ratio + 0.2;
				//tmp.y() = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio;
				//tmp.z() = theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio;
				tmp_Data.Position.x() = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio + 0.2;
				tmp_Data.Position.y() = DManager->MappingMatrix[Mapidx * 2] * ratio;
				tmp_Data.Position.z() = theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio;
				//tmp_VolumeData.push_back(theTRcuda->VolumeData[PCidx]);
				//tmpPC_T.mPC.
				tmpPC_T.mPC.push_back(tmp_Data);
				if (x == 124) {
					point_z_124 = point_z_124 + tmp_Data.Position.z();
				}
				if (x == 125) {
					point_z_125 = point_z_125 + tmp_Data.Position.z();
				}
				//tmpPC.push_back(tmp);
			}
		}
	}

	//int tmpIdx;
	//float idt;
	//cv::Mat tmp_cali = cv::imread("calibration.png");
	//cv::Mat tmp_result = cv::Mat(theTRcuda->VolumeSize_Y, theTRcuda->VolumeSize_Z, CV_32F, cv::Scalar(0, 0, 0));
	//for (int row = 0; row < tmp_result.rows; row++) {
	//	for (int col = 0; col < tmp_result.cols; col++) {
	//		tmpIdx = ((125 * theTRcuda->VolumeSize_Y) + row) * theTRcuda->VolumeSize_Z + col;
	//		idt = ((float)theTRcuda->VolumeDataAvg[tmpIdx] - 3) / 2;// theTRcuda->VolumeData[tmpIdx]/6
	//		idt = (float)(idt - 0.2) / 1.4f;
	//		tmp_result.at<float>(row, col) = idt;
	//	}
	//}
	//cv::Mat tmp_result2 = cv::Mat(360, 480, CV_8UC3, cv::Scalar(0, 0, 0));
	//cv::resize(tmp_result, tmp_result, cv::Size(480, 360), 0, 0, CV_INTER_LINEAR);
	//tmp_result.convertTo(tmp_result2, CV_8UC3, 255);
	//
	//cv::imshow("tmp_result", tmp_result2);
	//cv::imshow("tmp_cali", tmp_cali);
	////cv::imwrite("tmp_result.png", tmp_result);
	//cv::Mat tmp_compare = cv::Mat(720, 480, CV_8UC3, cv::Scalar(0, 0, 0));
	//for (int row = 0; row < tmp_compare.rows; row++) {
	//	if (row < 360) {
	//		for (int col = 0; col < tmp_result2.cols; col++) {
	//			tmp_compare.at<cv::Vec3b>(row, col) = tmp_cali.at<cv::Vec3b>(row, col);
	//		}
	//	}
	//	else {
	//		for (int col = 0; col < tmp_compare.cols; col++) {
	//			tmp_compare.at<cv::Vec3b>(row, col) = tmp_result2.at<cv::Vec3b>(row - 360, col);
	//		}
	//	}
	//}
	//cv::imshow("tmp_compare", tmp_compare);
	//cv::imwrite(std::to_string(125) + ".png", tmp_compare);
	//cv::imwrite("tmp_result.png", tmp_result);
	//color_img();
	//ToRGB();

	if (tmpPC_T.mPC.size() > 10000) {
		(*PointCloudArr).push_back(tmpPC_T);
		//(*pointCloudSet).push_back(tmpPC);
		PointCloud_idx_show = (*PointCloudArr).size() - 1;

		(*PointCloudArr)[PointCloud_idx_show].mPC_noGyro = tmpPC_T.mPC;
		std::cout << "tmpPC_T.mPC.size(): " << tmpPC_T.mPC.size() << std::endl;
		//std::cout << "(*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size(): " << (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size() << std::endl;
		///////////////////////////////
		//if (pointCloudSet->size() == 1) {
		//	CombinePC(0);
		//}

		std::cout << "point_z_124 Total: " << point_z_124 << std::endl;
		std::cout << "point_z_125 Total: " << point_z_125 << std::endl;

		Find_max_min();
		draw_before_mapping();
		//numericUpDown4->Maximum = (*PointCloudArr).size() - 1;
		std::cout << "now point cloud size:" << (*PointCloudArr).size() << std::endl;
		std::cout << "Scan count:" << scan_count << std::endl;
	}

	//std::cout << "PointCloud_idx_show: " << PointCloud_idx_show << std::endl;

	is_camera_move = true;

	if ((*PointCloudArr).size() == 1) {
		//*quat1 = sysManager->GetGloveDataLPtr()->GetQuaternion();
		//*quat1 = quat1->Inverse();
		//std::cout << "quat1" << (*quat1) << "\n";


		//*acc_vector1 = sysManager->GetGloveDataLPtr()->GetAcceleration();
		first_t = clock();

		//Radian *tmp_diff;
		//Vector3 *tmp_rotation;
		//std::vector<GlobalRegistration::Point3D> *tmp_PC;
		//tmp_PC = new std::vector<GlobalRegistration::Point3D>;
		int tmp_nearest = 0;
		//tmp_diff = new Radian;
		//tmp_rotation = new Vector3;
		//(*pointCloud_radian).push_back(*tmp_diff);
		//(*pointCloud_quat_vector).push_back(*tmp_rotation);
		//(*nearest_idx).push_back(tmp_nearest);
		//(*nearest_idx).push_back(tmp_nearest);
		//(*pointCloud_aligned).push_back(*tmp_PC);

		//(*cloud_vec_arr).push_back(*tmp_rotation);
		//(*near_idx2).push_back(tmp_nearest);
		//(*near_idx2).push_back(tmp_nearest);
	}
	else if ((*PointCloudArr).size() == 2) {
		//Rotate2();
		Rotate_cloud();
		//rotate_quat2();
		//RotatePC();
		//Find_Plane();
		//MovePC();
		std::cout << "now idx:" << PointCloud_idx_show << "     now size is:" << (*PointCloudArr).size() << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr;
		//std::cout << "now idx:" << PointCloud_idx_show << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr << std::endl;
	}
	else if ((*PointCloudArr).size()>2) {
		Rotate_cloud();
		Find_min_quat3();
		std::cout << "now idx:" << PointCloud_idx_show << "     now size is:" << (*PointCloudArr).size() << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr;
		//std::cout << "now idx:" << PointCloud_idx_show << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << std::endl;
		//std::cout << "now idx:" << PointCloud_idx_show << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr << std::endl;
	}
	//if ((*PointCloudArr).size() > 1) {
	//	Point_cloud_idx_sec = (*pointCloudSet).size() - 2;
	//}
	clear_PC_buffer();
	color_img();

	delete[] interator;
	//delete[] final_oct_arr;
	delete[] final_oct_char;
	delete[] ArrSize;
	delete[] outarr;

	clock_t scan_t2 = clock();
	full_scan_time = (scan_t2 - scan_t1) / (double)(CLOCKS_PER_SEC);



	all_time = all_time + full_scan_time;

	std::cout << "full_scan_time:" << full_scan_time << "s" << std::endl;
	std::cout << "all_time:" << all_time << "s" << std::endl;


	//PointCloud_idx_now++;
	//PointCloud_idx_show = (*pointCloudSet).size() - 1;
	//std::cout << "PointCloud_idx_show: " << PointCloud_idx_show << std::endl;
}

void DentistDemo::MyForm::color_img() {
	//mkdir("origin_v2");
	int tmpIdx;
	int midIdx;
	float idt, idtmax = 0, idtmin = 9999;
	float ori_idt, total_idt = 0, avg_idt = 0;
	//fstream fp;
	//fp.open("vol_data.txt", ios::out);


	for (int x = 0; x < theTRcuda->VolumeSize_X; x++) {
		cv::Mat tmp_floating = cv::Mat(theTRcuda->VolumeSize_Y, theTRcuda->VolumeSize_Z, CV_32F, cv::Scalar(0, 0, 0));
		//cv::Mat tmp_test = cv::Mat(theTRcuda->VolumeSize_Y, theTRcuda->VolumeSize_Z, CV_32F, cv::Scalar(0, 0, 0));
		for (int row = 0; row < theTRcuda->VolumeSize_Y; row++) {
			for (int col = 0; col < theTRcuda->VolumeSize_Z; col++) {
				tmpIdx = ((x * theTRcuda->VolumeSize_Y) + row) * theTRcuda->VolumeSize_Z + col;
				midIdx = ((125 * theTRcuda->VolumeSize_Y) + row) * theTRcuda->VolumeSize_Z + col;

				//idt = ((float)theTRcuda->VolumeData[tmpIdx] - 3) / 2;// theTRcuda->VolumeData[tmpIdx]/6
				ori_idt = (float)theTRcuda->VolumeData[tmpIdx];

				//idt = ((float)theTRcuda->VolumeData[tmpIdx] / (float)3.36f) - (float)(3.51f/3.36f);
				//idt = ((float)theTRcuda->VolumeData[tmpIdx] / (float)3.71933f) - (float)(3.29559f / 3.71933f);
				idt = ((float)theTRcuda->VolumeDataAvg[tmpIdx] / (float)3.509173f) - (float)(3.39f / 3.509173f);// 調整後能量區間
				//idt = ((float)theTRcuda->VolumeData[tmpIdx] / (float)7.298081f) + (float)(0.05186f / 7.298081f);// 原始能量區間

				//idt = (float)(idt - 0.2) / 1.4f;

				tmp_floating.at<float>(row, col) = idt;
				//tmp_test.at<float>(row, col) = (float)(ori_idt - 1) / 8;
				if (ori_idt > idtmax)
					idtmax = ori_idt;
				if (ori_idt < idtmin)
					idtmin = ori_idt;

				total_idt += ori_idt;
				//orig_pic_1024_3ch.at<cv::Vec3b>(row, col) = cv::Vec3b(idt, idt, idt);
			}
		}

		cv::resize(tmp_floating, tmp_floating, cv::Size(480, 360), 0, 0, CV_INTER_LINEAR);
		//cv::imshow("eee", tmp_test);
		tmp_floating.convertTo(tmp_floating, CV_8UC3, 255);
		//cv::imshow("ddd", tmp_floating);

		cvtColor(tmp_floating.clone(), tmp_floating, CV_GRAY2BGR);

		RGB_image->push_back(tmp_floating);
		//cv::imwrite("origin_v2\\" + std::to_string(x) + ".png", tmp_floating);

	}
	for (int i = 60; i < 201; i++) {

		(*raw_input).push_back((*RGB_image)[i]);
	}

	//fp.close();
	//avg_idt = total_idt / (theTRcuda->VolumeSize_X*theTRcuda->VolumeSize_Y*theTRcuda->VolumeSize_Z);
	std::cout << "avg_idt = " << avg_idt << "\n";
	std::cout << "idtmax = " << idtmax << "\n";
	std::cout << "idtmin = " << idtmin << "\n";

}
void DentistDemo::MyForm::load_result() {
	
	for (int i = 60; i < 201; i++) {
		cv::Mat tmp_input;
		tmp_input = cv::imread("./fileT01_02M/" + std::to_string(i) + ".png");
		result_input->push_back(tmp_input);
	}
	//std::cout << "read OK\n";
}
void DentistDemo::MyForm::load_rawImg() {
	for (int i = 60; i < 201; i++) {
		cv::Mat tmp_input;
		tmp_input = cv::imread("./fileT01_13M/" + std::to_string(i) + ".png");
		raw_input->push_back(tmp_input);
		std::cout << "read " << i << "\n";
	}
	
}
void DentistDemo::MyForm::slice_quad(int index) {
	int tmp_num = index + 60;//x
	float ratio = 1;
	float zRatio = DManager->zRatio;
	int Mapidx_0, Mapidx_250, col_0, col_1024, row_0, row_250;
	float tmp_x_0, tmp_y_0, tmp_z_0, tmp_x_250, tmp_y_250, tmp_z_1024;



	Mapidx_0 = ((tmp_num * 250) + 0);////x, y:0
	Mapidx_250 = ((tmp_num * 250) + 249);////x, y:249
	tmp_x_0 = DManager->MappingMatrix[Mapidx_0 * 2 + 1] * ratio + 0.2;
	tmp_y_0 = DManager->MappingMatrix[Mapidx_0 * 2] * ratio;
	tmp_z_0 = 0 * zRatio / 1024 * ratio;

	tmp_x_250 = DManager->MappingMatrix[Mapidx_250 * 2 + 1] * ratio + 0.2;
	tmp_y_250 = DManager->MappingMatrix[Mapidx_250 * 2] * ratio;
	tmp_z_1024 = 480 * zRatio / 1024 * ratio;


	(*slice_p1) = Vector3(tmp_x_0, tmp_y_0, tmp_z_0);
	(*slice_p2) = Vector3(tmp_x_0, tmp_y_0, tmp_z_1024);
	(*slice_p3) = Vector3(tmp_x_250, tmp_y_250, tmp_z_1024);
	(*slice_p4) = Vector3(tmp_x_250, tmp_y_250, tmp_z_0);
}
void DentistDemo::MyForm::classify_result() {
	std::vector<cv::Mat> tmp_result_img;
	int PCidx, Mapidx;
	float ratio = 1;
	float zRatio = DManager->zRatio;
	int tmpIdx;
	float idt;
	int control_pic_idx = this->control_pic->Value;
	//cv::Mat tmp_Mask_Model = cv::Mat(250,141 , CV_8UC3, cv::Scalar(0, 0, 0));
	Teeth_PC tmp_teeth_PC;
	(*tmp_Mask_Model) = cv::Mat(250, 141, CV_8UC4, cv::Scalar(0, 0, 0,0));
	for (int i = 0; i < (*Result_Image).size(); i++) {
		cv::Mat tmp_img;
		//cv::resize((*Result_Image)[i], tmp_img, cv::Size(1024, 250), 0, 0, CV_INTER_LINEAR);
		//tmp_result_img.push_back(tmp_img);

		tmp_result_img.push_back((*Result_Image)[i]);
	}

	std::cout << "row:" << tmp_result_img[0].rows << "\n";
	std::cout << "col:" << tmp_result_img[0].cols << "\n";
	std::cout << "tmp_result_img.size() = " << tmp_result_img.size() << "\n";

	for (int i = 0; i < tmp_result_img.size(); i++) {
		std::cout << "i = " << i << "\n";
		//std::cout << "tmp_result_img.size() = " << tmp_result_img.size() << "\n";
		

		for (int row = 0; row < tmp_result_img[i].rows; row++) {
			bool disease_class = false;
			bool meat_class = false;


			for (int col = 50; col < tmp_result_img[i].cols; col++) {

				
				//std::cout << "col:" << col << "\n";
				if (tmp_result_img[i].at<uchar>(row, col) == uchar(2) ) {
					
					Point_3D tmp_point_3D;

					int tmp_num = i + 60;//x
					int tmp_row = (row * 250) / 360;//y
					int tmp_col = (col * 1024) / 480;//z

					//////////////////////////////////////////////////////////////////////////////////////////
					//tmpIdx = ((tmp_num * theTRcuda->VolumeSize_Y) + row) * theTRcuda->VolumeSize_Z + col;
					//idt = ((float)theTRcuda->VolumeDataAvg[tmpIdx] / (float)3.509173f) - (float)(3.39f / 3.509173f);// 調整後能量區間
					//Mapidx = ((tmp_num * 250) + row);
					//////////////////////////////////////////////////////////////////////////////////////////

					//////////////////////////////////////////////////////////////////////////////////////////
					tmpIdx = ((tmp_num * 250) + tmp_row) * 1024 + tmp_col;
					idt = (*raw_input)[i].at<cv::Vec3b>(row,col)[0];// 調整後能量區間
					idt = (float)idt / 255;
					//std::cout << "idt:" << idt << "\n";
					Mapidx = ((tmp_num * 250) + tmp_row);
					//////////////////////////////////////////////////////////////////////////////////////////

					float tmp_x, tmp_y, tmp_z;


					tmp_x = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio + 0.2;
					tmp_y = DManager->MappingMatrix[Mapidx * 2] * ratio;
					tmp_z = col*zRatio / 1024 * ratio;

					Vector3 tmp_imgPC = Vector3(tmp_x, tmp_y, tmp_z);
					Vector3 tmp_idt = Vector3(idt, idt, idt);
					tmp_point_3D.position = tmp_imgPC;
					tmp_point_3D.idt = tmp_idt;


					//std::cout << "(" << tmp_x << "," << tmp_y << "," << tmp_z << "\n";
					teeth_imgPC->push_back(tmp_point_3D);
				}

				if (tmp_result_img[i].at<uchar>(row, col) == uchar(3)) {
					Point_3D tmp_point_3D;

					meat_class = true;
					int tmp_num = i + 60;//x
					int tmp_row = (row * 250) / 360;//y
					int tmp_col = (col * 1024) / 480;//z

					//////////////////////////////////////////////////////////////////////////////////////////
					//tmpIdx = ((tmp_num * theTRcuda->VolumeSize_Y) + row) * theTRcuda->VolumeSize_Z + col;
					//idt = ((float)theTRcuda->VolumeDataAvg[tmpIdx] / (float)3.509173f) - (float)(3.39f / 3.509173f);// 調整後能量區間
					//Mapidx = ((tmp_num * 250) + row);
					//////////////////////////////////////////////////////////////////////////////////////////

					//////////////////////////////////////////////////////////////////////////////////////////
					tmpIdx = ((tmp_num * 250) + tmp_row) * 1024 + tmp_col;
					idt = (*raw_input)[i].at<cv::Vec3b>(row, col)[0];// 調整後能量區間
					idt = (float)idt / 255;
					//std::cout << "idt:" << idt << "\n";
					Mapidx = ((tmp_num * 250) + tmp_row);
					//////////////////////////////////////////////////////////////////////////////////////////

					float tmp_x, tmp_y, tmp_z;


					tmp_x = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio + 0.2;
					tmp_y = DManager->MappingMatrix[Mapidx * 2] * ratio;
					tmp_z = col*zRatio / 1024 * ratio;

					Vector3 tmp_imgPC = Vector3(tmp_x, tmp_y, tmp_z);
					Vector3 tmp_idt = Vector3(idt, idt, idt);
					tmp_point_3D.position = tmp_imgPC;
					tmp_point_3D.idt = tmp_idt;

					//std::cout << "(" << tmp_x << "," << tmp_y << "," << tmp_z << "\n";
					meat_imgPC->push_back(tmp_point_3D);
				}
				if (tmp_result_img[i].at<uchar>(row, col) == uchar(4)) {
					Point_3D tmp_point_3D;

					disease_class = true;
					int tmp_num = i + 60;//x
					int tmp_row = (row * 250) / 360;//y
					int tmp_col = (col * 1024) / 480;//z

					//////////////////////////////////////////////////////////////////////////////////////////
					//tmpIdx = ((tmp_num * theTRcuda->VolumeSize_Y) + row) * theTRcuda->VolumeSize_Z + col;
					//idt = ((float)theTRcuda->VolumeDataAvg[tmpIdx] / (float)3.509173f) - (float)(3.39f / 3.509173f);// 調整後能量區間
					//Mapidx = ((tmp_num * 250) + row);
					//////////////////////////////////////////////////////////////////////////////////////////

					//////////////////////////////////////////////////////////////////////////////////////////
					tmpIdx = ((tmp_num * 250) + tmp_row) * 1024 + tmp_col;
					idt = (*raw_input)[i].at<cv::Vec3b>(row, col)[0];// 調整後能量區間
					idt = (float)idt / 255;
					//std::cout << "idt:" << idt << "\n";
					Mapidx = ((tmp_num * 250) + tmp_row);
					//////////////////////////////////////////////////////////////////////////////////////////

					float tmp_x, tmp_y, tmp_z;


					tmp_x = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio + 0.2;
					tmp_y = DManager->MappingMatrix[Mapidx * 2] * ratio;
					tmp_z = col*zRatio / 1024 * ratio;

					Vector3 tmp_imgPC = Vector3(tmp_x, tmp_y, tmp_z);
					Vector3 tmp_idt = Vector3(idt, idt, idt);
					tmp_point_3D.position = tmp_imgPC;
					tmp_point_3D.idt = tmp_idt;

					//std::cout << "(" << tmp_x << "," << tmp_y << "," << tmp_z << "\n";
					disease_imgPC->push_back(tmp_point_3D);
				}
			}
			if (meat_class == true) {
				//std::cout << " meat_class = true\n";
				int tmp_row = (row * 250) / 360;
				//std::cout << "row:" << row << "\n";
				//std::cout << "i:" << i << "\n";
				
				(*tmp_Mask_Model).at<cv::Vec4b>(tmp_row, i)[0] = 126;
				(*tmp_Mask_Model).at<cv::Vec4b>(tmp_row, i)[1] = 126;
				(*tmp_Mask_Model).at<cv::Vec4b>(tmp_row, i)[2] = 255;
				(*tmp_Mask_Model).at<cv::Vec4b>(tmp_row, i)[3] = 127;
				if (disease_class == true) {
					//std::cout << " disease_class = true\n";
					(*tmp_Mask_Model).at<cv::Vec4b>(tmp_row, i)[0] = 255;
					(*tmp_Mask_Model).at<cv::Vec4b>(tmp_row, i)[1] = 0;
					(*tmp_Mask_Model).at<cv::Vec4b>(tmp_row, i)[2] = 0;
					(*tmp_Mask_Model).at<cv::Vec4b>(tmp_row, i)[3] = 127;
				}
			}


		}

		(*tmp_pic_teeth).push_back(teeth_imgPC->size());
		(*tmp_pic_meat).push_back(meat_imgPC->size());
		(*tmp_pic_disease).push_back(disease_imgPC->size());
	}
	

	//cvtColor((*tmp_Mask_Model).clone(), (*tmp_Mask_Model), CV_BGR2BGRA);
	cv::resize((*tmp_Mask_Model).clone(), (*tmp_Mask_Model), cv::Size(42, 75), 0, 0, CV_INTER_LINEAR);

	cv::imwrite("mask.png", (*tmp_Mask_Model));

	//tmp_teeth_PC.Model_mask = tmp_Mask_Model;
	//for (int i = 0; i < (*teeth_imgPC).size(); i++) {
	//	//std::cout << "teeth_imgPC i = " << i << "\n";
	//	glManager->read_point(-(double)(*teeth_imgPC)[i].position.x, (double)(*teeth_imgPC)[i].position.y, (double)(*teeth_imgPC)[i].position.z);
	//	glManager->read_color(1*(double)(*teeth_imgPC)[i].idt.x*color_idt, 1*(double)(*teeth_imgPC)[i].idt.y*color_idt, 0.2*(double)(*teeth_imgPC)[i].idt.z*color_idt);
	//	glManager->initial_alpha();
	//}
	//for (int i = 0; i < (*disease_imgPC).size(); i++) {
	//	//std::cout << "teeth_imgPC i = " << i << "\n";
	//	glManager->read_point(-(double)(*disease_imgPC)[i].position.x, (double)(*disease_imgPC)[i].position.y, (double)(*disease_imgPC)[i].position.z);
	//	glManager->read_color(0.2*(double)(*disease_imgPC)[i].idt.x*color_idt, 0.2*(double)(*disease_imgPC)[i].idt.y*color_idt, 1*(double)(*disease_imgPC)[i].idt.z*color_idt);
	//	glManager->initial_alpha();
	//}
	//for (int i = 0; i < (*meat_imgPC).size(); i++) {
	//	//std::cout << "teeth_imgPC i = " << i << "\n";
	//	glManager->read_point(-(double)(*meat_imgPC)[i].position.x, (double)(*meat_imgPC)[i].position.y, (double)(*meat_imgPC)[i].position.z);
	//	glManager->read_color(1*(double)(*meat_imgPC)[i].idt.x*color_idt, 0.2*(double)(*meat_imgPC)[i].idt.y*color_idt, 0.2*(double)(*meat_imgPC)[i].idt.z*color_idt);
	//	glManager->initial_alpha();
	//}
	//
	////std::cout << "glManager->read_point OK\n";
	//glManager->Initial_vert();
	//std::cout << "glManager->Initial_vert() OK\n";
}
void DentistDemo::MyForm::show_Teeth_PC(Teeth_PC tmp_Teeth) {
	glManager->Clear_all();

	for (int i = 0; i < tmp_Teeth.teeth_imgPC.size(); i++) {
		//std::cout << "teeth_imgPC i = " << i << "\n";
		glManager->read_point(-(double)tmp_Teeth.teeth_imgPC[i].position.x, (double)tmp_Teeth.teeth_imgPC[i].position.y, (double)tmp_Teeth.teeth_imgPC[i].position.z);
		glManager->read_color(1 * (double)tmp_Teeth.teeth_imgPC[i].idt.x*color_idt, 1 * (double)tmp_Teeth.teeth_imgPC[i].idt.y*color_idt, 1*(double)tmp_Teeth.teeth_imgPC[i].idt.z*color_idt);
		glManager->initial_alpha();
	}
	for (int i = 0; i < tmp_Teeth.disease_imgPC.size(); i++) {
		//std::cout << "teeth_imgPC i = " << i << "\n";
		glManager->read_point(-(double)tmp_Teeth.disease_imgPC[i].position.x, (double)tmp_Teeth.disease_imgPC[i].position.y, (double)tmp_Teeth.disease_imgPC[i].position.z);
		glManager->read_color(0.2*(double)tmp_Teeth.disease_imgPC[i].idt.x*color_idt, 0.2*(double)tmp_Teeth.disease_imgPC[i].idt.y*color_idt, 1 * (double)tmp_Teeth.disease_imgPC[i].idt.z*color_idt);
		glManager->initial_alpha();
	}
	for (int i = 0; i < tmp_Teeth.meat_imgPC.size(); i++) {
		//std::cout << "teeth_imgPC i = " << i << "\n";
		glManager->read_point(-(double)tmp_Teeth.meat_imgPC[i].position.x, (double)tmp_Teeth.meat_imgPC[i].position.y, (double)tmp_Teeth.meat_imgPC[i].position.z);
		glManager->read_color(1 * (double)tmp_Teeth.meat_imgPC[i].idt.x*color_idt, 0.5*(double)tmp_Teeth.meat_imgPC[i].idt.y*color_idt, 0.5*(double)tmp_Teeth.meat_imgPC[i].idt.z*color_idt);
		glManager->initial_alpha();
	}

	//std::cout << "glManager->read_point OK\n";
	glManager->Initial_vert();

}
void DentistDemo::MyForm::teeth_vert() {


}
void DentistDemo::MyForm::decay_img2space() {
	int PCidx, Mapidx;
	float ratio = 1;
	float zRatio = DManager->zRatio;

	
	std::ofstream  output_deacyPC;
	output_deacyPC.open("decayPC.asc");

	std::cout << "row:" << (*Result_Image)[0].rows << "\n";
	std::cout << "col:" << (*Result_Image)[0].cols << "\n";

	

	for (int i = 0; i < Result_Image->size(); i++) {
		//std::cout << i << "\n";
		for (int row = 0; row < (*Result_Image)[i].rows; row++) {
			for (int col = 0; col < (*Result_Image)[i].cols; col++) {



				if ((*Result_Image)[i].at<cv::Vec3b>(row, col)[0] == 222) {
					Point_3D tmp_point;
					//Mapidx = ((y * theTRcuda->sample_Y * theTRcuda->VolumeSize_X) + x) * theTRcuda->sample_X;
					//PCidx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;

					int tmp_num = i + 60;//x
					int tmp_row = (row * 250) / 360;//y
					int tmp_col = (col * 1024) / 480;//z


					//output_deacyPC << tmp_num << " " << tmp_row << " " << tmp_col << "\n";
					//std::cout << "Mapidx, PCidx:" << Mapidx << ", " << PCidx << "\n";

					float tmp_x, tmp_y, tmp_z;

					Mapidx = ((tmp_num * 250) + tmp_row);
					//PCidx = ((tmp_num * 250) + tmp_row) * 1024;

					//tmp_Data.Position.x() = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio + 0.2;
					//tmp_Data.Position.y() = DManager->MappingMatrix[Mapidx * 2] * ratio;
					//tmp_Data.Position.z() = theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio;

					tmp_x = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio + 0.2;
					tmp_y = DManager->MappingMatrix[Mapidx * 2] * ratio;
					tmp_z = tmp_col*zRatio / 1024 * ratio;
					//std::cout << "tmp_x, tmp_y, tmp_z:" << tmp_x << ", " << tmp_y <<","<< tmp_z <<"\n";


					output_deacyPC << tmp_x << " " << tmp_y << " " << tmp_z << "\n";



					//Vector3 tmp_imgPC = Vector3(i+60, tmp_row, tmp_col);
					Vector3 tmp_imgPC = Vector3(tmp_x, tmp_y, tmp_z);

					tmp_point.position = tmp_imgPC;
					//std::cout << row << ", " << col << "\n";
					decay_imgPC->push_back(tmp_point);
				}

			}
		}
	}
	output_deacyPC.close();

}
void DentistDemo::MyForm::Test_decay_img2space() {

	int PCidx, Mapidx;
	float ratio = 1;
	float zRatio = DManager->zRatio;
	std::ofstream  output_deacyPC;
	output_deacyPC.open("decayPC.asc");

	std::cout << "row:"<<(*result_input)[0].rows<<"\n";
	std::cout << "col:" << (*result_input)[0].cols << "\n";
	for (int i = 0; i < result_input->size(); i++) {
		//std::cout << i << "\n";
		for (int row = 0; row < (*result_input)[i].rows; row++) {
			for (int col = 0; col < (*result_input)[i].cols; col++) {
				


				if ((*result_input)[i].at<cv::Vec3b>(row, col)[0] == 255) {
					//Mapidx = ((y * theTRcuda->sample_Y * theTRcuda->VolumeSize_X) + x) * theTRcuda->sample_X;
					//PCidx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
					Point_3D tmp_point;
					int tmp_num = i + 60;//x
					int tmp_row = (row * 250) / 360;//y
					int tmp_col = (col * 1024) / 480;//z

					
					//output_deacyPC << tmp_num << " " << tmp_row << " " << tmp_col << "\n";

					//std::cout << "Mapidx, PCidx:" << Mapidx << ", " << PCidx << "\n";

					float tmp_x, tmp_y, tmp_z;

					Mapidx = ((tmp_num * 250) + tmp_row);
					//PCidx = ((tmp_num * 250) + tmp_row) * 1024;

					//tmp_Data.Position.x() = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio + 0.2;
					//tmp_Data.Position.y() = DManager->MappingMatrix[Mapidx * 2] * ratio;
					//tmp_Data.Position.z() = theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio;

					tmp_x = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio+0.2;
					tmp_y = DManager->MappingMatrix[Mapidx * 2] * ratio;
					tmp_z = tmp_col*zRatio / 1024 * ratio;
					//std::cout << "tmp_x, tmp_y, tmp_z:" << tmp_x << ", " << tmp_y <<","<< tmp_z <<"\n";


					output_deacyPC << tmp_x << " " << tmp_y << " " << tmp_z << "\n";

					//Vector3 tmp_imgPC = Vector3(i+60, tmp_row, tmp_col);
					Vector3 tmp_imgPC = Vector3(tmp_x, tmp_y, tmp_z);
					tmp_point.position = tmp_imgPC;
					//std::cout << row << ", " << col << "\n";
					decay_imgPC->push_back(tmp_point);
				}			
				
			}
		}
	}
	output_deacyPC.close();
}


void DentistDemo::MyForm::Rotate_cloud() {
	Quaternion cloud_quat;
	Radian *cloud_radian;
	Vector3 *cloud_vec;
	cloud_radian = new Radian;
	cloud_vec = new Vector3;
	cloud_quat = sysManager->GetGloveDataLPtr()->GetQuaternion();
	//cloud_quat = Quaternion(-cloud_quat.x, -cloud_quat.y, cloud_quat.z, cloud_quat.w);
	cloud_quat.ToAngleAxis(*cloud_radian, *cloud_vec);
	//std::cout << "Output  valueDegrees():" << (*cloud_radian).valueDegrees() << "--- rotation:(" << (*cloud_vec).x << "---," << (*cloud_vec).y << "---," << (*cloud_vec).z << "---)\n";
	Rotate_quat((*cloud_radian).valueDegrees(), -(*cloud_vec).z, (*cloud_vec).y, (*cloud_vec).x);
	Vector3 tmp_vec = Vector3((*cloud_vec).x, (*cloud_vec).y, (*cloud_vec).z);
	//(*cloud_vec_arr).push_back(tmp_vec);
	
	(*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr = tmp_vec;
	(*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr = (*cloud_radian).valueDegrees();
	rotate_quat2();
}
void DentistDemo::MyForm::rotate_quat2() {
	//for (int j = 0; j < (*pointCloudSet)[PointCloud_idx_show].size(); j++) {
	//	(*pointCloudSet)[PointCloud_idx_show][j].x() = (*pointCloudSet)[PointCloud_idx_show][j].x() - (*Point_cloud_center)[0].x;
	//	(*pointCloudSet)[PointCloud_idx_show][j].y() = (*pointCloudSet)[PointCloud_idx_show][j].y() - (*Point_cloud_center)[0].y;
	//	(*pointCloudSet)[PointCloud_idx_show][j].z() = (*pointCloudSet)[PointCloud_idx_show][j].z() - (*Point_cloud_center)[0].z;
	//	glm::vec4 *tmp_vv;
	//	tmp_vv = new glm::vec4((*pointCloudSet)[PointCloud_idx_show][j].x(), (*pointCloudSet)[PointCloud_idx_show][j].y(), (*pointCloudSet)[PointCloud_idx_show][j].z(), 1);
	//	//*tmp_v = (*tmp_M)* (*tmp_v);
	//	*tmp_vv = (*tmp_MM)*(*tmp_vv);
	//	(*pointCloudSet)[PointCloud_idx_show][j].x() = (*tmp_vv).x + (*Point_cloud_center)[0].x;
	//	(*pointCloudSet)[PointCloud_idx_show][j].y() = (*tmp_vv).y + (*Point_cloud_center)[0].y;
	//	(*pointCloudSet)[PointCloud_idx_show][j].z() = (*tmp_vv).z + (*Point_cloud_center)[0].z;
	//}

	for (int j = 0; j < (*PointCloudArr)[PointCloud_idx_show].mPC.size(); j++) {
		(*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.x() = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.x() - (*PointCloudArr)[0].Point_cloud_center.x;
		(*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.y() = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.y() - (*PointCloudArr)[0].Point_cloud_center.y;
		(*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.z() = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.z() - (*PointCloudArr)[0].Point_cloud_center.z;
		glm::vec4 *tmp_vv;
		tmp_vv = new glm::vec4((*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.x(), (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.y(), (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.z(), 1);
		//*tmp_v = (*tmp_M)* (*tmp_v);
		*tmp_vv = (*tmp_MM)*(*tmp_vv);
		(*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.x() = (*tmp_vv).x + (*PointCloudArr)[0].Point_cloud_center.x;
		(*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.y() = (*tmp_vv).y + (*PointCloudArr)[0].Point_cloud_center.y;
		(*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.z() = (*tmp_vv).z + (*PointCloudArr)[0].Point_cloud_center.z;
	}
}
void DentistDemo::MyForm::mani_rotate_cloud(int rot_idx) {
	//std::cout << "start mani rotate\n";
	for (int j = 0; j < (*PointCloudArr)[rot_idx].mPC.size(); j++) {
		(*PointCloudArr)[rot_idx].mPC[j].Position.x() = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.x() - (*PointCloudArr)[0].Point_cloud_center.x;
		(*PointCloudArr)[rot_idx].mPC[j].Position.y() = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.y() - (*PointCloudArr)[0].Point_cloud_center.y;
		(*PointCloudArr)[rot_idx].mPC[j].Position.z() = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.z() - (*PointCloudArr)[0].Point_cloud_center.z;
		glm::vec4 *tmp_vv;
		tmp_vv = new glm::vec4((*PointCloudArr)[rot_idx].mPC[j].Position.x(), (*PointCloudArr)[rot_idx].mPC[j].Position.y(), (*PointCloudArr)[rot_idx].mPC[j].Position.z(), 1);
		//*tmp_v = (*tmp_M)* (*tmp_v);
		*tmp_vv = (*mani_MM)*(*tmp_vv);
		(*PointCloudArr)[rot_idx].mPC[j].Position.x() = (*tmp_vv).x + (*PointCloudArr)[0].Point_cloud_center.x;
		(*PointCloudArr)[rot_idx].mPC[j].Position.y() = (*tmp_vv).y + (*PointCloudArr)[0].Point_cloud_center.y;
		(*PointCloudArr)[rot_idx].mPC[j].Position.z() = (*tmp_vv).z + (*PointCloudArr)[0].Point_cloud_center.z;
	}
}
void DentistDemo::MyForm::hkoglPanelControl1_Paint(System::Object ^ sender, System::Windows::Forms::PaintEventArgs ^ e)
{
	//glEnable(GL_COLOR_MATERIAL); //允許使用glColor

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Radian thetaR;
	//Vector3 rotationAxixR;
	//sysManager->GetGloveDataLPtr()->GetQuaternion().ToAngleAxis(thetaR, rotationAxixR);
	//float radius_R = thetaR.valueDegrees() * glm::pi<float>() / 180.0f;
	//float x_R = rotationAxixR.x;
	//float y_R = rotationAxixR.y;
	//float z_R = rotationAxixR.z;
	//glm::mat4 M_R = glm::mat4(1);
	//// Calculate the rotate matrix
	//glm::vec4 c0_R = glm::vec4(cos(radius_R) + (1 - cos(radius_R))*x_R*x_R, (1 - cos(radius_R))*y_R*x_R + sin(radius_R)*z_R, (1 - cos(radius_R))*z_R*x_R - sin(radius_R)*y_R, 0);
	//glm::vec4 c1_R = glm::vec4((1 - cos(radius_R))*y_R*x_R - sin(radius_R)*z_R, cos(radius_R) + (1 - cos(radius_R))*y_R*y_R, (1 - cos(radius_R))*z_R*y_R + sin(radius_R)*x_R, 0);
	//glm::vec4 c2_R = glm::vec4((1 - cos(radius_R))*z_R*x_R + sin(radius_R)*y_R, (1 - cos(radius_R))*z_R*y_R - sin(radius_R)*x_R, cos(radius_R) + (1 - cos(radius_R))*z_R*z_R, 0);
	//glm::vec4 c3_R = glm::vec4(0, 0, 0, 1);
	//M_R = glm::mat4(c0_R, c1_R, c2_R, c3_R);
	//glManager->rotateAxis_R = M_R*glm::vec4(-1.5f, 0.0f, 0.0f, 1.0f);


	

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixf(glManager->GetCamera()->GetProjectionMatrixF());

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(glManager->GetCamera()->GetViewMatrixF());
	glDisable(GL_LIGHTING); //關燈
	glEnable(GL_DEPTH_TEST);

	/*glBegin(GL_QUADS);
	glVertex3f(0,0,0);
	glVertex3f(1,0,0);
	glVertex3f(1,1,0);
	glVertex3f(0,1,0);
	glEnd();*/

	//int length = 10;
	//glBegin(GL_LINES);
	//glLineWidth(10.0);
	//glColor4f(1.0, 0.0, 0.0, 1.0);
	//glVertex3f(0, 0, 0);
	//glVertex3f(length, 0, 0);
	//
	//glColor4f(0.0, 1.0, 0.0, 1.0);
	//glVertex3f(0, 0, 0);
	//glVertex3f(0, length, 0);
	//
	//glColor4f(0.0, 0.0, 1.0, 1.0);
	//glVertex3f(0, 0, 0);
	//glVertex3f(0, 0, length);
	//glEnd();
	//
	//glPushMatrix();
	//glBegin(GL_QUADS);
	//	glColor3f(1, 1, 0);
	//	glVertex3f(1.5, -1.5, 0);
	//	glVertex3f(1.5, 1.5, 0);
	//	glVertex3f(-1.5, 1.5, 0);
	//	glVertex3f(-1.5, -1.5, 0);
	//glEnd();
	//glBegin(GL_QUADS);
	//	glColor3f(1, 1, 0);
	//	glVertex3f(1.5, -1.5, 0);
	//	glVertex3f(-1.5, -1.5, 0);
	//	glVertex3f(-1.5, 1.5, 0);
	//	glVertex3f(1.5, 1.5, 0);
	//glEnd();
	//glBegin(GL_QUADS);
	//	glColor3f(0, 1, 1);
	//	glVertex3f(0, -1.5, 1.5);
	//	glVertex3f(0, 1.5, 1.5);
	//	glVertex3f(0, 1.5, -1.5);
	//	glVertex3f(0, -1.5, -1.5);
	//glEnd();
	//glBegin(GL_QUADS);
	//	glColor3f(0, 1, 1);
	//	glVertex3f(0, -1.5, 1.5);
	//	glVertex3f(0, -1.5, -1.5);
	//	glVertex3f(0, 1.5, -1.5);
	//	glVertex3f(0, 1.5, 1.5);
	//glEnd();
	//glBegin(GL_QUADS);
	//	glColor3f(1, 0, 1);
	//	glVertex3f(1.5, 0, -1.5);
	//	glVertex3f(1.5, 0, 1.5);
	//	glVertex3f(-1.5, 0, 1.5);
	//	glVertex3f(-1.5, 0, -1.5);
	//glEnd();
	//glBegin(GL_QUADS);
	//	glColor3f(1, 0, 1);
	//	glVertex3f(1.5, 0, -1.5);
	//	glVertex3f(-1.5, 0, -1.5);
	//	glVertex3f(-1.5, 0, 1.5);
	//	glVertex3f(1.5, 0, 1.5);
	//glEnd();
	//glPopMatrix();

	if (showVolumeData) draw_volumeData(theTRcuda);
	if (showBoardTemp) drawPointType(theTRcuda);
	if (showPointType) drawBoardTemp(theTRcuda);
	if (showPointCloud) drawPointTypeCloud(theTRcuda);
	if (show_rawData) drawZAxisValue(theTRcuda);
	drawPCSet3();
	

	if (is_before_camera)draw_before_camera();


	glPushMatrix();
	//glTranslatef(-5,-5,-5);
	//glRotatef(90,0,0,1);
	draw_decay();
	glManager->Draw();
	//glTranslatef(5, 5, 5);
	glPopMatrix();


	//glPushMatrix();
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glRotatef(90, 0, 0, 1);
	//glBegin(GL_QUADS);
	//glColor4f(1, 0, 1, 0.3f);
	//glVertex3f(-slice_p1->x, slice_p1->y, slice_p1->z);
	//glVertex3f(-slice_p2->x, slice_p2->y, slice_p2->z);
	//glVertex3f(-slice_p3->x, slice_p3->y, slice_p3->z);
	//glVertex3f(-slice_p4->x, slice_p4->y, slice_p4->z);
	//glEnd();
	//
	//glBegin(GL_QUADS);
	//glColor4f(1, 0, 1, 0.3f);
	//glVertex3f(-slice_p4->x, slice_p4->y, slice_p4->z);
	//glVertex3f(-slice_p3->x, slice_p3->y, slice_p3->z);
	//glVertex3f(-slice_p2->x, slice_p2->y, slice_p2->z);
	//glVertex3f(-slice_p1->x, slice_p1->y, slice_p1->z);
	//glEnd();
	//glPopMatrix();

	glDisable(GL_BLEND);
	
	hkoglPanelControl1->Invalidate();
}
void DentistDemo::MyForm::draw_before_camera() {


	glPushMatrix();
	glRotatef((*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr, -(*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr.z, (*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr.y, (*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr.x);

	glTranslatef(0, 0, -5);
	//glTranslatef((*Point_cloud_center)[0].x, (*Point_cloud_center)[0].y, (*Point_cloud_center)[0].z);
	//camera_cube(1.0f, 1.0f, 2.0f);
	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(0, 0, -3);
	glVertex3f(3, 0, -3);
	glEnd();
	glBegin(GL_LINES);
	glColor3f(0, 1, 0);
	glVertex3f(0, 0, -3);
	glVertex3f(0, 3, -3);
	glEnd();
	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	glVertex3f(0, 0, -3);
	glVertex3f(0, 0, 0);
	glEnd();
	glPopMatrix();
	glPopMatrix();

	glPushMatrix();
	glRotated((*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr, -(*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr.z, (*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr.y, (*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr.x);
	glBegin(GL_LINES);
	glColor3f(1, 1, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, -5);
	glEnd();
	glPopMatrix();

}
void DentistDemo::MyForm::hkoglPanelControl1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
{
	if (e->Button == System::Windows::Forms::MouseButtons::Left)
	{
		//std::cout << "X:" << e->X << ", Y:" << e->Y << std::endl;
		glManager->GetCamera()->StartRotate(e->X, e->Y);
	}
	else if (e->Button == System::Windows::Forms::MouseButtons::Middle)
	{
		// Turn off translate
		mouse_move = true;
		glManager->GetCamera()->StartTranslate(e->X, e->Y);
	}
}

void DentistDemo::MyForm::hkoglPanelControl1_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
{
	glManager->GetCamera()->mouse_move = true;
	if (glManager->GetCamera()->IsMoving())
	{
		//std::cout << "X:" << e->X << ", Y:" << e->Y << std::endl;
		glManager->GetCamera()->Move(e->X, e->Y);
	}
}

void DentistDemo::MyForm::hkoglPanelControl1_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
{
	if (e->Button == System::Windows::Forms::MouseButtons::Left)
	{
		glManager->GetCamera()->StopRotate(e->X, e->Y);
	}
	else if (e->Button == System::Windows::Forms::MouseButtons::Middle)
	{
		glManager->GetCamera()->StopTranslate(e->X, e->Y);
	}
}

void DentistDemo::MyForm::hkoglPanelControl1_MouseWheel(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
{
	glManager->GetCamera()->ZoomIn(e->Delta);
}

void DentistDemo::MyForm::hkoglPanelControl1_KeyPress(System::Object^  sender, System::Windows::Forms::KeyPressEventArgs^  e) {


}
void DentistDemo::MyForm::hkoglPanelControl1_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
	if (e->KeyCode == System::Windows::Forms::Keys::F2) {
		//std::cout << "F2" << std::endl;
		showPointCloud = !showPointCloud;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::F1) {
		//std::cout << "F1" << std::endl;
		showVolumeData = !showVolumeData;
		showPointType = !showPointType;
		showBoardTemp = !showBoardTemp;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::F3) {
		//std::cout << "F3" << std::endl;
		show_first_point = !show_first_point;
		//if (show_first_point)
		//	std::cout << "show_second_point = true" << std::endl;
		//else
		//	std::cout << "show_second_point = false" << std::endl;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::F4) {
		//std::cout << "F4" << std::endl;


		show_second_point = !show_second_point;
		//if (show_second_point)
		//	std::cout << "show_second_point = true" << std::endl;
		//else
		//	std::cout << "show_second_point = false" << std::endl;
	}

	else if (e->KeyCode == System::Windows::Forms::Keys::F5) {
		//std::cout << "F5" << std::endl;


		//showoverlap1 = !showoverlap1;
		//if (showoverlap1)
		//	std::cout << "show_second_point = true" << std::endl;
		//else
		//	std::cout << "show_second_point = false" << std::endl;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::F6) {
		//std::cout << "F6" << std::endl;


		//showoverlap2 = !showoverlap2;
		//if (showoverlap2)
		//	std::cout << "show_second_point = true" << std::endl;
		//else
		//	std::cout << "show_second_point = false" << std::endl;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::F7) {
		//std::cout << "F6" << std::endl;


		show_combine_cloud = !show_combine_cloud;
		//if (showoverlap2)
		//	std::cout << "show_second_point = true" << std::endl;
		//else
		//	std::cout << "show_second_point = false" << std::endl;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::F8) {
		//std::cout << "F6" << std::endl;

		is_before_camera = !is_before_camera;
		//show_tmp_align1 = !show_tmp_align1;
		//if (showoverlap2)
		//	std::cout << "show_second_point = true" << std::endl;
		//else
		//	std::cout << "show_second_point = false" << std::endl;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::F9) {
		//std::cout << "F6" << std::endl;

		show_noGyro_PC = !show_noGyro_PC;
		//show_tmp_align2 = !show_tmp_align2;
		if (show_noGyro_PC)
			std::cout << "show_noGyro_PC = true" << std::endl;
		else
			std::cout << "show_noGyro_PC = false" << std::endl;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::F10) {
		show_rawData = !show_rawData;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::E) {
		//std::cout << "Up" << std::endl;
		if (volumeDataIdx < theTRcuda->VolumeSize_X - 1) {
			volumeDataIdx++;
			std::cout << "\rvolumeDataIdx: " << volumeDataIdx << "---";
		}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::Q) {
		//std::cout << "Down" << std::endl;
		if (volumeDataIdx > 0) {
			volumeDataIdx--;
			std::cout << "\rvolumeDataIdx: " << volumeDataIdx << "---";
		}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::A) {
		//std::cout << "Point cloud id:"<< PointCloud_idx_show << std::endl;
		enable_review = true;
		if (PointCloud_idx_show > 1) {
			PointCloud_idx_show--;
			std::cout << "\rnow idx:" << PointCloud_idx_show <<"     now size is:"<< (*PointCloudArr) .size()<< "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr;
			//std::cout << "now idx:" << PointCloud_idx_show << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr << std::endl;
		}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::D) {
		//std::cout << "Point cloud id:"<< PointCloud_idx_show << std::endl;
		enable_review = true;
		if (PointCloud_idx_show < (*PointCloudArr).size() - 1) {
			PointCloud_idx_show++;
			std::cout << "\rnow idx:" << PointCloud_idx_show << "     now size is:" << (*PointCloudArr).size() << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr;
			//std::cout << "now idx:" << PointCloud_idx_show << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr << std::endl;
		}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::Z) {
		////std::cout << "Point cloud id:"<< PointCloud_idx_show << std::endl;
		//enable_review = true;
		//if (Point_cloud_idx_sec > 1) {
		//	Point_cloud_idx_sec--;
		//	std::cout << "Point_cloud_idx_sec: " << Point_cloud_idx_sec << std::endl;
		//}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::C) {
		is_camera_move = !is_camera_move;
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::Delete) {
		//std::cout << "Point cloud id:"<< PointCloud_idx_show << std::endl;
		if ((*PointCloudArr).size() > 1) {
			if (PointCloud_idx_show == (*PointCloudArr).size() - 1) {
				PointCloud_idx_show--;
				//(*pointCloudSet).erase((*pointCloudSet).begin() + PointCloud_idx_show + 1);
				//(*Point_cloud_center).erase((*Point_cloud_center).begin() + PointCloud_idx_show + 1);
				//(*outside_center).erase((*outside_center).begin() + PointCloud_idx_show + 1);
				//(*Point_cloud_Max).erase((*Point_cloud_Max).begin() + PointCloud_idx_show + 1);
				//(*Point_cloud_Min).erase((*Point_cloud_Min).begin() + PointCloud_idx_show + 1);
				(*PointCloudArr).erase((*PointCloudArr).begin() + PointCloud_idx_show + 1);
				//(*near_idx2).erase((*near_idx2).begin() + PointCloud_idx_show + 1);
				//(*cloud_vec_arr).erase((*cloud_vec_arr).begin() + PointCloud_idx_show + 1);
				//(*overlap_cloud1).erase((*overlap_cloud1).begin() + PointCloud_idx_show);
				//(*overlap_cloud2).erase((*overlap_cloud2).begin() + PointCloud_idx_show);
				//numericUpDown4->Maximum = (*PointCloudArr).size() - 1;
				std::cout << "\rnow idx:" << PointCloud_idx_show << "     now size is:" << (*PointCloudArr).size() << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr;
				//std::cout << "PointCloudArr size is:" << (*PointCloudArr).size() << std::endl;
			}
			else {
				//(*pointCloudSet).erase((*pointCloudSet).begin() + PointCloud_idx_show);
				//(*Point_cloud_center).erase((*Point_cloud_center).begin() + PointCloud_idx_show);
				//(*outside_center).erase((*outside_center).begin() + PointCloud_idx_show);
				//(*Point_cloud_Max).erase((*Point_cloud_Max).begin() + PointCloud_idx_show);
				//(*Point_cloud_Min).erase((*Point_cloud_Min).begin() + PointCloud_idx_show);
				(*PointCloudArr).erase((*PointCloudArr).begin() + PointCloud_idx_show);
				//(*near_idx2).erase((*near_idx2).begin() + PointCloud_idx_show);
				//(*cloud_vec_arr).erase((*cloud_vec_arr).begin() + PointCloud_idx_show);
				//(*overlap_cloud1).erase((*overlap_cloud1).begin() + PointCloud_idx_show);
				//(*overlap_cloud2).erase((*overlap_cloud2).begin() + PointCloud_idx_show);
				//std::cout << "PointCloud_idx_show: " << PointCloud_idx_show << std::endl;
				//std::cout << "PointCloudArr size is:" << (*PointCloudArr).size() << std::endl;
				//numericUpDown4->Maximum = (*PointCloudArr).size() - 1;
				std::cout << "\rnow idx:" << PointCloud_idx_show << "     now size is:" << (*PointCloudArr).size() << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr;
			}
		}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::I) {
		//std::cout << "Down" << std::endl;
		if (volumeZ_now > 0) {
			volumeZ_now--;
			std::cout << "\rvolumeZ_now: " << volumeZ_now << "---";
		}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::O) {
		//std::cout << "Down" << std::endl;
		if (volumeZ_now < theTRcuda->VolumeSize_Y - 1) {
			volumeZ_now++;
			std::cout << "\rvolumeZ_now: " << volumeZ_now << "---";
		}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::M) {
		if (meat_alpha > 0)
			meat_alpha = meat_alpha - 0.05f;
		else
			meat_alpha = meat_alpha;
		//std::cout << "(*teeth_imgPC).size():" << (*teeth_imgPC).size() << "\n";
		//std::cout << "meat_alpha:" << meat_alpha << "\n";
		glManager->read_alpha(meat_alpha, (*teeth_imgPC).size()+ (*disease_imgPC).size(), (*tmp_display_PC).teeth_imgPC.size()+(*tmp_display_PC).disease_imgPC.size()+(*tmp_display_PC).meat_pic_index[control_pic->Value]);

		
		//color_idt = color_idt + 0.1f;
		//glManager->change_color(color_idt);
		//std::cout << "color_idt:" << color_idt << "\n";
		
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::N) {
		if (meat_alpha < 1)
			meat_alpha = meat_alpha + 0.05f;
		else
			meat_alpha = meat_alpha;
		//std::cout << "(*teeth_imgPC).size():" << (*teeth_imgPC).size() << "\n";
		//std::cout << "meat_alpha:" << meat_alpha << "\n";
		glManager->read_alpha(meat_alpha, (*teeth_imgPC).size() + (*disease_imgPC).size(), (*tmp_display_PC).teeth_imgPC.size() + (*tmp_display_PC).disease_imgPC.size() + (*tmp_display_PC).meat_pic_index[control_pic->Value]);



		//if (color_idt > 1) {
		//	color_idt = color_idt - 0.1f;
		//	glManager->change_color(color_idt);
		//	std::cout << "color_idt:" << color_idt << "\n";
		//}
	}
	else if (e->KeyCode == System::Windows::Forms::Keys::G) {

		delete theTRcuda;
		theTRcuda = new TRcuda;
	
	}
}

void DentistDemo::MyForm::Test_file_Click(System::Object^  sender, System::EventArgs^  e) {

	char file_name1[10] = "1C_01_p";
	char file_name2[10] = "C3_01_p";
	char file_name[10];

	std::vector<char> test_char;
	//std::vector<char> test_char2;

	std::ifstream myfile;
	//std::ifstream myfile2;

	if (readRaw_count == 0) {
		memcpy(file_name, file_name1, 10);
	}
	else {
		memcpy(file_name, file_name2, 10);
	}
	readRaw_count++;

	myfile.open(file_name, std::ios::in);
	//myfile.open("C_02", std::ios::in);

	int length;
	//int length2;

	if (!myfile) {
		std::cout << "open file fail" << std::endl;
	}
	else {
		myfile.seekg(0, myfile.end);
		length = myfile.tellg();
		myfile.seekg(0, myfile.beg);

		final_oct_char = new char[length];

		std::cout << " file length" << length << std::endl;
		test_char.resize(length);


		myfile.read(test_char.data(), sizeof(char) * length);
		myfile.close();
	}


	//unsigned short *interator;
	//interator = new unsigned short[PIC_SIZE];
	//std::cout << " file length" << length << std::endl;

	//for (int i = 0; i < 10; i++) {
	//	std::bitset<sizeof(final_oct_char[i]) * 8> tmp_char(final_oct_char[i]);
	//	std::cout << "tmp_char[" << i << "]: " << tmp_char << std::endl;
	//	std::cout << "char[" << i << "]: " << final_oct_char[i] << std::endl;;
	//
	//}
	//*
	//int scanline = (2048 * 2) * 250 * 2;
	//for (int i = length -  scanline - 1; i > 0; i -= scanline * 2)
	//{
	//	test_char.erase(test_char.begin() + i, test_char.begin() + i + scanline);
	//}//*/

	std::cout << "theTRcuda->peakGap:" << theTRcuda->peakGap << std::endl;
	std::cout << "theTRcuda->energyGap:" << theTRcuda->energyGap << std::endl;

	theTRcuda->RawToPointCloud(test_char.data(), test_char.size(), 250, 2048);
	std::cout << "FFT OK" << std::endl;

	int PCidx, Mapidx;
	float ratio = 1;
	float zRatio = DManager->zRatio;

	std::vector<GlobalRegistration::Point3D> tmpPC;
	PointCloudArray tmpPC_T;
	//std::vector<int> tmp_VolumeData;

	for (int x = 0; x < theTRcuda->VolumeSize_X; x++)
	{
		for (int y = 0; y < theTRcuda->VolumeSize_Y; y++)
		{
			if (x > DManager->Mapping_X || y > DManager->Mapping_Y)
				continue;
			//Mapidx = (x * theTRcuda->VolumeSize_Y) + y;
			//Mapidx = (y * theTRcuda->VolumeSize_X) + x;
			//Mapidx = ((y)* theTRcuda->VolumeSize_X) + x;
			//PCidx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
			Mapidx = ((y * theTRcuda->sample_Y * theTRcuda->VolumeSize_X) + x) * theTRcuda->sample_X;
			PCidx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
			if (theTRcuda->PointType[PCidx + 3] == 1 && theTRcuda->PointType[PCidx + 1] != 0)
			{
				GlobalRegistration::Point3D tmp;
				PointData tmp_Data;
				//int tmpVolumeData;
				//tmp.x() = DManager->MappingMatrix[Mapidx * 2] * ratio + 0.2;
				//tmp.y() = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio;
				//tmp.z() = theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio;
				tmp_Data.Position.x() = DManager->MappingMatrix[Mapidx * 2] * ratio + 0.2;
				tmp_Data.Position.y() = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio;
				tmp_Data.Position.z() = theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio;
				//tmp_VolumeData.push_back(theTRcuda->VolumeData[PCidx]);
				tmpPC.push_back(tmp);
				tmpPC_T.mPC.push_back(tmp_Data);
			}
		}
	}
	if (tmpPC_T.mPC.size() > 0) {
		(*PointCloudArr).push_back(tmpPC_T);
		PointCloud_idx_show = (*PointCloudArr).size() - 1;
		std::cout << "(*PointCloudArr)[PointCloud_idx_show].mPC.size(): "<< (*PointCloudArr)[PointCloud_idx_show].mPC.size() << std::endl;

		///////////////////////////////
		//if (PointCloudArr->size() == 1) {
		//	CombinePC(0);
		//}
		Find_max_min();
		draw_before_mapping();
	}



	//label3->Text = max_z.ToString();


	delete[] final_oct_char;
	
}
void DentistDemo::MyForm::Get_Quat_Click(System::Object^  sender, System::EventArgs^  e) {





	std::cout << "Get_Quat_Click OK" << std::endl;
	std::cout << "quat1:" << sysManager->GetGloveDataLPtr()->GetQuaternion() << std::endl;
	//std::cout << "theta:" << theta_1->valueDegrees() << std::endl;
	//std::cout << "rotationAxix: (" << rotationAxix_1->x << "," << rotationAxix_1->y << "," << rotationAxix_1->z << ")" << std::endl;
}
void DentistDemo::MyForm::Clear_cloud_Click(System::Object^  sender, System::EventArgs^  e) {
	//pointCloudSet->clear();
	//finalPC->clear();

	scan_count = 0;
	delete quat1;
	delete quat2;
	delete quat3;
	delete theta_diff;
	delete rotationAxix_diff;
	delete tmp_center;
	delete theTRcuda;

	//delete pointCloudSet;
	//delete finalPC;
	delete overlap_idx1;
	delete overlap_idx2;
	//delete Point_cloud_Max;
	//delete Point_cloud_Min;
	//delete Point_cloud_center;
	delete Plane_vector;
	//delete outside_center;
	delete draw_plane1;
	delete draw_plane2;
	delete draw_plane3;
	delete draw_plane4;
	//delete nearest_idx;
	delete PointCloudArr;
	//delete pointCloud_aligned;
	//delete[] final_oct_arr;

	//pointCloudSet = new std::vector<std::vector<GlobalRegistration::Point3D>>;
	//finalPC = new std::vector<GlobalRegistration::Point3D>;
	//pointCloud_aligned = new std::vector<std::vector<GlobalRegistration::Point3D>>;
	quat1 = new Quaternion;
	quat2 = new Quaternion;
	quat3 = new Quaternion;
	theta_diff = new Radian;
	rotationAxix_diff = new Vector3;
	tmp_center = new Vector3;
	theTRcuda = new TRcuda;
	readRaw_count = 0;
	//delete result_movement;

	//result_movement = new Vector3;
	overlap_idx1 = new std::vector<std::vector<int>>;
	overlap_idx2 = new std::vector<std::vector<int>>;
	//nearest_idx = new std::vector<int>;
	//Point_cloud_Max = new std::vector<Vector3>;
	//Point_cloud_Min = new std::vector<Vector3>;
	//Point_cloud_center = new std::vector<Vector3>;
	Plane_vector = new std::vector<Vector3>;
	//outside_center = new std::vector<Vector3>;
	draw_plane1 = new Vector3;
	draw_plane2 = new Vector3;
	draw_plane3 = new Vector3;
	draw_plane4 = new Vector3;
	PointCloudArr = new std::vector<PointCloudArray>;

	std::cout << "pointCloudSet.size: " << PointCloudArr->size() << std::endl;
}
//void DentistDemo::MyForm::Alignment_Click(System::Object^  sender, System::EventArgs^  e) {
//
//}
void DentistDemo::MyForm::Save_PointCloud_Click(System::Object^  sender, System::EventArgs^  e) {
	//saveFileDialog1->Filter = "All files (*.*)|*.*";
	clock_t tmp_file_name;
	std::string filename;
	tmp_file_name = clock();

	//std::cout << "tmp_file_name: " << tmp_file_name << std::endl;
	MarshalString(tmp_file_name.ToString(), filename);

	std::ofstream  output_file;
	output_file.open(filename + "_0.txt");

	//MarshalString((*out_fileName), filename);
	//output_file.open((*out_fileName) + ".txt");

	//output_file << "1234" << std::endl;

	//for (int i = 0; i < (*finalPC).size(); i++)
	//{
	//	//swap x and y
	//	output_file << (*finalPC)[i].y() << " ";
	//	output_file << (*finalPC)[i].x() << " ";
	//	output_file << (*finalPC)[i].z() << " ";
	//	output_file << "0 " << (*finalPCid)[i];
	//	output_file << "\n";
	//}
	int tmpIdx;
	int PCidx, Mapidx;
	float ratio = 1;
	float zRatio = DManager->zRatio;
	//output_file << "xmm,ymm,zmm,x,y,z\n";
	for (int x = 25; x < theTRcuda->VolumeSize_X-25; x++) {
		for (int y = 25; y < theTRcuda->VolumeSize_Y-25; y++)
		{
			//for (int z = 0; z < theTRcuda->VolumeSize_Z; z++)
			{
				//tmpIdx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z + z;
				//if (theTRcuda->PointType[tmpIdx] == 1)
				{
					if (x > DManager->Mapping_X || y > DManager->Mapping_Y)
						continue;
					//Mapidx = (( x) * theTRcuda->VolumeSize_Y) +  y;
					Mapidx = ((y) * theTRcuda->VolumeSize_X) + x;
					PCidx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
					if (theTRcuda->PointType[PCidx + 3] == 1 && theTRcuda->PointType[PCidx + 1] != 0)
					{
						//int tmpVolumeData;
						 output_file << DManager->MappingMatrix[Mapidx * 2] * ratio << " ";
						 output_file << DManager->MappingMatrix[Mapidx * 2 + 1] * ratio << " ";
						 output_file << theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio << " ";
						 //output_file << "0 0 ";
						//tmp_VolumeData.push_back(theTRcuda->VolumeData[PCidx]);
						 //output_file << "\n";
						 //output_file << x+1 << ",";
						 //output_file << y+1 << ",";
						 //output_file << 1024 - theTRcuda->PointType[PCidx + 1] << ",";
						 output_file << "\n";
						 /*
						 //int tmpVolumeData;
						 output_file << DManager->MappingMatrix[Mapidx * 2] * ratio << " ";
						 output_file << DManager->MappingMatrix[Mapidx * 2 + 1] * ratio << " ";
						 output_file << theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio << " ";
						 //tmp_VolumeData.push_back(theTRcuda->VolumeData[PCidx]);
						 //output_file << "\n";
						 output_file << x << " ";
						 output_file << y << " ";
						 output_file << theTRcuda->PointType[PCidx + 1] << " ";
						 output_file << "\n";
						 //*/
					}					
				}
			}
		}
	}
	output_file.close();
	//saveFileDialog1->ShowDialog();
	std::cout << "Save_OK" << std::endl;
}
void DentistDemo::MyForm::saveFileDialog1_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e) {

	//std::string filename;
	//MarshalString(saveFileDialog1->FileName, filename);
	//std::cout << "File name: " << filename << std::endl;
	//
	//std::ofstream  output_file;
	//output_file.open(filename);
	////output_file << "1234" << std::endl;
	//
	//for (int i = 0; i < (*finalPC).size(); i++)
	//{
	//	//swap x and y
	//	output_file << (*finalPC)[i].y() << " ";
	//	output_file << (*finalPC)[i].x() << " ";
	//	output_file << (*finalPC)[i].z() << " ";
	//	output_file << "0 " << (*finalPCid)[i];
	//	output_file << "\n";
	//}
	//output_file.close();
}
void DentistDemo::MyForm::Save_point_s_Click(System::Object^  sender, System::EventArgs^  e) {
	clock_t tmp_file_name;
	std::string filename;
	std::string fileidx;
	tmp_file_name = clock();
	int tnp_idx = PointCloud_idx_show;

	Quaternion test_quat;
	Radian *test_diff, *quat1_diff;
	Vector3 *test_rotation, *quat1_rotation;
	//test_diff = new Radian((*pointCloud_radian)[PointCloud_idx_show]);
	//test_rotation = new Vector3((*pointCloud_quat_vector)[PointCloud_idx_show].x,
	//							(*pointCloud_quat_vector)[PointCloud_idx_show].y,
	//							(*pointCloud_quat_vector)[PointCloud_idx_show].z);
	quat1_diff = new Radian;
	quat1_rotation = new Vector3;
	//test_quat = sysManager->GetGloveDataLPtr()->GetQuaternion();
	//test_quat = test_quat * (*quat1);
	//test_quat = test_quat.Inverse();

	//test_quat.ToAngleAxis(*test_diff, *test_rotation);
	(*quat1).ToAngleAxis(*quat1_diff, *quat1_rotation);
	


	////////////////////////////////////////////////////////////////////////////////////////////////////
	//std::vector<GlobalRegistration::Point3D> tmpPC;
	//for (int j = 0; j < (*pointCloudSet)[tnp_idx].size(); j++) {
	//	GlobalRegistration::Point3D tmp;
	//
	//	tmp.x() = (*pointCloudSet)[tnp_idx][j].x();
	//	tmp.y() = (*pointCloudSet)[tnp_idx][j].y();
	//	tmp.z() = (*pointCloudSet)[tnp_idx][j].z();
	//	tmpPC.push_back(tmp);
	//}
	//
	//for (int j = 0; j < tmpPC.size(); j++) {
	//
	//	tmpPC[j].x() = tmpPC[j].x() - (*Point_cloud_center)[0].x;
	//	tmpPC[j].y() = tmpPC[j].y() - (*Point_cloud_center)[0].y;
	//	tmpPC[j].z() = tmpPC[j].z() - (*Point_cloud_center)[0].z;
	//
	//	glm::vec4 *tmp_vv;
	//	tmp_vv = new glm::vec4(tmpPC[j].x(), tmpPC[j].y(), tmpPC[j].z(), 1);
	//	//*tmp_v = (*tmp_M)* (*tmp_v);
	//	*tmp_vv = (*tmp_MM)*(*tmp_vv);
	//
	//	tmpPC[j].x() = (*tmp_vv).x + (*Point_cloud_center)[0].x;
	//	tmpPC[j].y() = (*tmp_vv).y + (*Point_cloud_center)[0].y;
	//	tmpPC[j].z() = (*tmp_vv).z + (*Point_cloud_center)[0].z;
	//}
	//
	////std::cout << "test_diff" << (*test_diff).valueDegrees() << std::endl;
	//std::cout << "test_rotation:(" << test_rotation->x << "," << test_rotation->y << "," << test_rotation->z << ")" << std::endl;
	//std::cout << "(*Point_cloud_center)[0]:(" << (*Point_cloud_center)[0].x << "," << (*Point_cloud_center)[0].y << "," << (*Point_cloud_center)[0].z << ")" << std::endl;
	//glTranslatef(-(*Point_cloud_center)[0].x,-(*Point_cloud_center)[0].y, -(*Point_cloud_center)[0].z);
	//std::cout << "(*test_diff).valueDegrees():(" << (*output_diff).valueDegrees() << std::endl;
	//std::cout << "(*test_rotation):(" << (*output_rotVec).x << "," << (*output_rotVec).y << "," << (*output_rotVec).z << ")" << std::endl;
	///////////////////////////////////////////////////////////////////////////


	//std::cout << "tmp_file_name: " << tmp_file_name << ", PointCloud_idx_show:" << tnp_idx << std::endl;
	MarshalString(tmp_file_name.ToString(), filename);
	MarshalString(tnp_idx.ToString(), fileidx);

	std::ofstream  output_file;
	output_file.open(filename + "__" + fileidx + ".txt");


	//MarshalString(out_fileName, filename);
	//output_file.open((*out_fileName) + ".txt");

	//output_file << "1234" << std::endl;

	//output_file << (*quat1_diff).valueDegrees() << " ";
	//output_file << (*quat1_rotation).x << " ";
	//output_file << (*quat1_rotation).y << " ";
	//output_file << (*quat1_rotation).z << " ";
	//output_file << "\n";
	//
	//output_file <<  (*output_diff).valueDegrees()<< " ";
	//output_file << (*output_rotVec).x << " ";
	//output_file << (*output_rotVec).y << " ";
	//output_file << (*output_rotVec).z << " ";
	//
	//output_file << "\n";
	//
	/*output_file << (*quat3).x << " ";
	output_file << (*quat3).y << " ";
	output_file << (*quat3).z << " ";
	output_file << (*quat3).w << " ";
	output_file << "\n";*/
	output_file << (*PointCloudArr)[tnp_idx].cloud_degree_arr << " ";
	output_file << (*PointCloudArr)[tnp_idx].cloud_vec_arr.x << " ";
	output_file << (*PointCloudArr)[tnp_idx].cloud_vec_arr.y << " ";
	output_file << (*PointCloudArr)[tnp_idx].cloud_vec_arr.z << " \n";

	for (int i = 0; i < (*PointCloudArr)[tnp_idx].mPC.size(); i++)
	{
		//swap x and y
		output_file << (*PointCloudArr)[tnp_idx].mPC[i].Position.x() << " ";
		output_file << (*PointCloudArr)[tnp_idx].mPC[i].Position.y() << " ";
		output_file << (*PointCloudArr)[tnp_idx].mPC[i].Position.z() << " ";
		//output_file << "0 0";
		output_file << "\n";
	}
	output_file.close();
	//saveFileDialog1->ShowDialog();
	std::cout << "Save_OK1" << std::endl;

	std::ofstream  output_file2;
	output_file2.open(filename + "__" + fileidx + "_2.txt");
	output_file2 << (*PointCloudArr)[tnp_idx].cloud_degree_arr << " ";
	output_file2 << (*PointCloudArr)[tnp_idx].cloud_vec_arr.x << " ";
	output_file2 << (*PointCloudArr)[tnp_idx].cloud_vec_arr.y << " ";
	output_file2 << (*PointCloudArr)[tnp_idx].cloud_vec_arr.z << " \n";
	std::cout << "vec OK " << std::endl;
	for (int i = 0; i < (*PointCloudArr)[tnp_idx].mPC.size(); i++)
	{
		//swap x and y
		output_file2 << (*PointCloudArr)[tnp_idx].mPC_noGyro[i].Position.x() << " ";
		output_file2 << (*PointCloudArr)[tnp_idx].mPC_noGyro[i].Position.y() << " ";
		output_file2 << (*PointCloudArr)[tnp_idx].mPC_noGyro[i].Position.z() << " ";
		output_file2 << "0 0";
		output_file2 << "\n";
	}
	output_file2.close();
	//saveFileDialog1->ShowDialog();
	std::cout << "Save_OK2" << std::endl;
}
//void DentistDemo::MyForm::label1_Click(System::Object^  sender, System::EventArgs^  e) {
//}

void DentistDemo::MyForm::load_obj() {
	std::vector<glm::vec3> tmp_out_vertices;
	std::vector<glm::vec3> tmp_out_normals;
	std::vector<unsigned int> tmp_out_materialIndices;
	std::vector<std::string> tmp_out_mtls;

	//loadOBJ("test.obj", *obj1->out_vertices, *obj1->out_normals, *obj1->out_materialIndices, *obj1->out_mtls);
	

	//obj1->out_vertices = std::vector<glm::vec3>(tmp_out_vertices.begin(), tmp_out_vertices.end());

	//obj1->out_materialIndices = tmp_out_materialIndices;
	//obj1->out_mtls = tmp_out_mtls;
}


void DentistDemo::MyForm::name_output_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	System::String^ test_in = name_output->Text;
	if (test_in->Length > 0) {
		MarshalString(test_in, *tmp_fileName);
		//std::cout << "tmp_fileName:" << (*tmp_fileName) << std::endl;
	}
}
void DentistDemo::MyForm::name_output_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e) {
	if (e->KeyCode == System::Windows::Forms::Keys::Enter) {
		out_fileName = tmp_fileName;
		std::cout << "tmp_fileName:" << (*tmp_fileName) << std::endl;
		std::cout << "out_fileName:" << (*out_fileName) << std::endl;
	}
}

void DentistDemo::MyForm::read_point_Click(System::Object^  sender, System::EventArgs^  e) {
	char file_name1[14] = "122305__1.txt";
	char file_name2[14] = "1C15_01.txt";
	char file_name[14];
	bool is_quat = false;
	bool nega_x = false, nega_y = false, nega_z = false, nega_w = false;

	double tmp_quat_x, tmp_quat_y, tmp_quat_z, tmp_quat_w;
	//char file_name2[5] = "";
	//char file_name[5];
	if (readRaw_count == 0) {
		memcpy(file_name, file_name1, 14);
		std::cout << "read No1 " << std::endl;
	}
	else {
		memcpy(file_name, file_name2, 14);
		std::cout << "read No2 " << std::endl;
	}
	readRaw_count++;
	std::vector<int> tmp_int;
	std::vector<int> tmp_float_num;
	std::ifstream myfile;
	char *test_char;
	//std::vector<char> test_char;
	myfile.open(file_name, std::ios::in);
	//myfile.open("C_02", std::ios::in);

	int length;
	//int length2;

	if (!myfile) {
		std::cout << "open file fail" << std::endl;
	}
	else {
		myfile.seekg(0, myfile.end);
		length = myfile.tellg();
		myfile.seekg(0, myfile.beg);

		//final_oct_char = new char[length];

		std::cout << " file length" << length << std::endl;

		test_char = new char[length];

		//test_char.resize(length);


		myfile.read(test_char, sizeof(char) * length);
		myfile.close();
	}
	//for (int i = 0; i < 100; i++) {
	//	std::cout << test_char[i] << std::endl;
	//}

	
	char *end_str;
	char *token = strtok_s(test_char, "\n", &end_str);
	//char *tmp_char = strtok_s(test_char, "\n", &end_str);
	int point_count = 0;
	std::vector<GlobalRegistration::Point3D> tmpPC;
	PointCloudArray tmpPC_T;
	//tmp_char = strtok(test_char, "\n");
	while (token != NULL) {
		int dir_type = 0;
		char *end_token;
		//printf("a = %s\n", token);
		char *token2 = strtok_s(token, " ", &end_token);
		GlobalRegistration::Point3D tmp;
		
		while (token2 != NULL)
		{
			//printf("b = %s\n", token2);

			char *end_num;
			char *token3 = strtok_s(token2, ".", &end_num);
			//printf("c = %s\n", token3);
			
			if (!is_quat) {
				if (token3[0] == '-') {
					if (dir_type == 0) {
						nega_x = true;
					}
					else if (dir_type == 1) {						
						nega_y = true;
					}
					else if (dir_type == 2) {
						nega_z = true;
					}
					else if (dir_type == 3) {
						nega_w = true;
					}
				}
			}			
			double tmp_int = atof(token3);
			token3 = strtok_s(NULL, ".", &end_num);

			if (token3 != NULL) {
				double tmp_double = atof(token3);
				for (int i = 0; i < strlen(token3); i++) {
					tmp_double /= 10;
				}

				tmp_int += tmp_double;
				//std::cout << "double = " << tmp_int << std::endl;
				//std::cout << "point cloud = " << (*pointCloudSet)[PointCloud_idx_show][point_count].x() << std::endl;
				//(*pointCloudSet)[PointCloud_idx_show][point_count].x() = tmp_int;
				if (dir_type == 0) {
					if (!is_quat) {
						if (nega_x)
							tmp_quat_x = tmp_int*(-1);
						else
							tmp_quat_x = tmp_int;
					}
						
					else {
						tmp.x() = tmp_int;
					}
				}
				else if (dir_type == 1) {
					if (!is_quat)
					{
						if (nega_y)
							tmp_quat_y = tmp_int*(-1);
						else
							tmp_quat_y = tmp_int;
					}
					else
					tmp.y() = tmp_int;
				}
				else if (dir_type == 2) {
					if (!is_quat)
					{
						if (nega_z)
							tmp_quat_z = tmp_int*(-1);
						else
							tmp_quat_z = tmp_int;
					}
					else {
						tmp.z() = tmp_int;
						//std::cout << "tmp = " << tmp.x() <<","<< tmp.y()<<","<< tmp.z()<< std::endl;
						if (tmp.x() > 0&&tmp.y()>0)
							tmpPC.push_back(tmp);
					}
				}
				else if (dir_type == 3) {
					if (!is_quat) {
						//tmp_quat_w = tmp_int;
						//is_quat = true;
						//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
						if (nega_w) {
							tmp_quat_w = tmp_int*(-1);
							is_quat = true;
							//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
						}
						else {
							tmp_quat_w = tmp_int;
							is_quat = true;
							//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
						}
					}
				}
			}
			dir_type++;
			token2 = strtok_s(NULL, " ", &end_token);
		}

		//dir_type = 0;
		
		point_count++;
		token = strtok_s(NULL, "\n", &end_str);
	}

	std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w <<")"<< std::endl;

	if (tmpPC.size() > 0) {
		//(*pointCloudSet).push_back(tmpPC);
		(*PointCloudArr).push_back(tmpPC_T);

		PointCloud_idx_show = (*PointCloudArr).size() - 1;
		//std::cout << "(*pointCloudSet).size: " << (*pointCloudSet).size() << std::endl;
		std::cout << "(*PointCloudArr).size: " << (*PointCloudArr).size() << std::endl;
		std::cout << "(*tmpPC).size: " << (*PointCloudArr)[0].mPC.size() << std::endl;
		///////////////////////////////
		//if (pointCloudSet->size() == 1) {
		//	CombinePC(0);
		//}

		(*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr = tmp_quat_x;
		Vector3 tmp_vec = Vector3(tmp_quat_y, tmp_quat_z, tmp_quat_w);
		(*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr = tmp_vec;

		Find_max_min();
	}
	delete test_char;
	//PointCloud_idx_show = (*pointCloudSet).size() - 1;
	std::cout << "PointCloud_idx_show: " << PointCloud_idx_show << std::endl;

}
void DentistDemo::MyForm::First_cloud_Click(System::Object^  sender, System::EventArgs^  e) {
	char file_name1[14] = "C13_01.txt";
	char file_name[14];
	bool is_quat = false;

	//char file_name2[5] = "";
	//char file_name[5];

	memcpy(file_name, file_name1, 14);
	std::cout << "read No1 " << std::endl;

	std::vector<int> tmp_int;
	std::vector<int> tmp_float_num;
	std::ifstream myfile;
	char *test_char;
	//std::vector<char> test_char;
	myfile.open(file_name, std::ios::in);
	//myfile.open("C_02", std::ios::in);

	int length;
	//int length2;

	if (!myfile) {
		std::cout << "open file fail" << std::endl;
	}
	else {
		myfile.seekg(0, myfile.end);
		length = myfile.tellg();
		myfile.seekg(0, myfile.beg);

		//final_oct_char = new char[length];

		std::cout << " file length" << length << std::endl;

		test_char = new char[length];

		//test_char.resize(length);


		myfile.read(test_char, sizeof(char) * length);
		myfile.close();
	}
	//for (int i = 0; i < 100; i++) {
	//	std::cout << test_char[i] << std::endl;
	//}


	char *end_str;
	char *token = strtok_s(test_char, "\n", &end_str);
	//char *tmp_char = strtok_s(test_char, "\n", &end_str);
	int point_count = 0;
	std::vector<PointData> tmpPC;
	PointCloudArray tmpPC_T;
	//tmp_char = strtok(test_char, "\n");
	while (token != NULL) {
		int dir_type = 0;
		char *end_token;
		//printf("a = %s\n", token);
		char *token2 = strtok_s(token, " ", &end_token);
		PointData tmp;

		while (token2 != NULL)
		{
			//printf("b = %s\n", token2);

			char *end_num;
			char *token3 = strtok_s(token2, ".", &end_num);
			//printf("c = %s\n", token3);

			double tmp_int = atof(token3);
			token3 = strtok_s(NULL, ".", &end_num);

			if (token3 != NULL) {
				double tmp_double = atof(token3);
				for (int i = 0; i < strlen(token3); i++) {
					tmp_double /= 10;
				}

				tmp_int += tmp_double;
				//std::cout << "double = " << tmp_int << std::endl;
				//std::cout << "point cloud = " << (*pointCloudSet)[PointCloud_idx_show][point_count].x() << std::endl;
				//(*pointCloudSet)[PointCloud_idx_show][point_count].x() = tmp_int;
				if (dir_type == 0) {

					tmp.Position.x() = tmp_int;

				}
				else if (dir_type == 1) {
					tmp.Position.y() = tmp_int;
				}
				else if (dir_type == 2) {
					tmp.Position.z() = tmp_int;
					//std::cout << "tmp = " << tmp.x() <<","<< tmp.y()<<","<< tmp.z()<< std::endl;
					if (tmp.Position.x() > 0 && tmp.Position.y() > 0) {
						tmpPC.push_back(tmp);
						tmpPC_T.mPC.push_back(tmp);
					}
				}				
			}
			dir_type++;
			token2 = strtok_s(NULL, " ", &end_token);
		}

		//dir_type = 0;

		point_count++;
		token = strtok_s(NULL, "\n", &end_str);
	}


	if (tmpPC.size() > 0) {
		//(*pointCloudSet).push_back(tmpPC);
		(*PointCloudArr).push_back(tmpPC_T);
		PointCloud_idx_show = (*PointCloudArr).size() - 1;
		//std::cout << "(*pointCloudSet).size: " << (*pointCloudSet).size() << std::endl;
		std::cout << "(*pointCloudSet).size: " << (*PointCloudArr).size() << std::endl;
		std::cout << "(*pointCloudSet).size: " << (*PointCloudArr)[0].mPC.size() << std::endl;
		///////////////////////////////
		//if (pointCloudSet->size() == 1) {
		//	CombinePC(0);
		//}
		Vector3 null_vector;
		float null_degree;
		int null_int;
		(*input_rotVec).push_back(null_vector);
		(*input_dgree).push_back(null_degree);
		//nearest_idx->push_back(null_int);
		Find_max_min();
	}
	delete test_char;
	//PointCloud_idx_show = (*pointCloudSet).size() - 1;
	std::cout << "PointCloud_idx_show: " << PointCloud_idx_show << std::endl;

}
void DentistDemo::MyForm::Second_cloud_Click(System::Object^  sender, System::EventArgs^  e) {
	char file_name1[14] = "C1_01_r.txt";
	char file_name2[14] = "C3_01_r.txt";
	char file_name3[14] = "R3_01.txt";
	char file_name4[14] = "R13_01.txt";
	char file_name[14];
	bool is_quat = false;
	bool nega_x = false, nega_y = false, nega_z = false, nega_w = false;

	double tmp_quat_x, tmp_quat_y, tmp_quat_z, tmp_quat_w;
	//char file_name2[5] = "";
	//char file_name[5];
	if (readRaw_count == 0) {
		memcpy(file_name, file_name1, 14);
		std::cout << "read No1 " << std::endl;
	}
	else if(readRaw_count == 1){
		memcpy(file_name, file_name2, 14);
		std::cout << "read No2 " << std::endl;
	}
	else if (readRaw_count == 2) {
		memcpy(file_name, file_name3, 14);
		std::cout << "read No3 " << std::endl;
	}
	else if (readRaw_count == 3) {
		memcpy(file_name, file_name4, 14);
		std::cout << "read No4 " << std::endl;
	}
	readRaw_count++;
	std::vector<int> tmp_int;
	std::vector<int> tmp_float_num;
	std::ifstream myfile;
	char *test_char;
	//std::vector<char> test_char;
	myfile.open(file_name, std::ios::in);
	//myfile.open("C_02", std::ios::in);

	int length;
	//int length2;

	if (!myfile) {
		std::cout << "open file fail" << std::endl;
	}
	else {
		myfile.seekg(0, myfile.end);
		length = myfile.tellg();
		myfile.seekg(0, myfile.beg);

		//final_oct_char = new char[length];

		std::cout << " file length" << length << std::endl;

		test_char = new char[length];

		//test_char.resize(length);


		myfile.read(test_char, sizeof(char) * length);
		myfile.close();
	}
	//for (int i = 0; i < 100; i++) {
	//	std::cout << test_char[i] << std::endl;
	//}


	char *end_str;
	char *token = strtok_s(test_char, "\n", &end_str);
	//char *tmp_char = strtok_s(test_char, "\n", &end_str);
	int point_count = 0;
	std::vector<GlobalRegistration::Point3D> tmpPC;
	PointCloudArray tmpPC_T;
	//tmp_char = strtok(test_char, "\n");
	while (token != NULL) {
		int dir_type = 0;
		char *end_token;
		//printf("a = %s\n", token);
		char *token2 = strtok_s(token, " ", &end_token);
		PointData tmp;

		while (token2 != NULL)
		{
			//printf("b = %s\n", token2);

			char *end_num;
			char *token3 = strtok_s(token2, ".", &end_num);
			//printf("c = %s\n", token3);

			if (!is_quat) {
				if (token3[0] == '-') {
					if (dir_type == 0) {
						nega_x = true;
					}
					else if (dir_type == 1) {
						nega_y = true;
					}
					else if (dir_type == 2) {
						nega_z = true;
					}
					else if (dir_type == 3) {
						nega_w = true;
					}
				}
			}
			double tmp_int = atof(token3);
			token3 = strtok_s(NULL, ".", &end_num);

			if (token3 != NULL) {
				double tmp_double = atof(token3);
				for (int i = 0; i < strlen(token3); i++) {
					tmp_double /= 10;
				}

				tmp_int += tmp_double;
				//std::cout << "double = " << tmp_int << std::endl;
				//std::cout << "point cloud = " << (*pointCloudSet)[PointCloud_idx_show][point_count].x() << std::endl;
				//(*pointCloudSet)[PointCloud_idx_show][point_count].x() = tmp_int;
				if (dir_type == 0) {
					if (!is_quat) {
						if (nega_x)
							tmp_quat_x = tmp_int*(-1);
						else
							tmp_quat_x = tmp_int;
					}

					else {
						tmp.Position.x() = tmp_int;
					}
				}
				else if (dir_type == 1) {
					if (!is_quat)
					{
						if (nega_y)
							tmp_quat_y = tmp_int*(-1);
						else
							tmp_quat_y = tmp_int;
					}
					else
						tmp.Position.y() = tmp_int;
				}
				else if (dir_type == 2) {
					if (!is_quat)
					{
						if (nega_z)
							tmp_quat_z = tmp_int*(-1);
						else
							tmp_quat_z = tmp_int;
					}
					else {
						tmp.Position.z() = tmp_int;
						//std::cout << "tmp = " << tmp.x() <<","<< tmp.y()<<","<< tmp.z()<< std::endl;
						if (tmp.Position.x() > 0 && tmp.Position.y() > 0) {
							//tmpPC.push_back(tmp);
							tmpPC_T.mPC.push_back(tmp);
						}
					}
				}
				else if (dir_type == 3) {
					if (!is_quat) {
						//tmp_quat_w = tmp_int;
						//is_quat = true;
						//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
						if (nega_w) {
							tmp_quat_w = tmp_int*(-1);
							is_quat = true;
							//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
						}
						else {
							tmp_quat_w = tmp_int;
							is_quat = true;
							//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
						}
					}
				}
			}
			dir_type++;
			token2 = strtok_s(NULL, " ", &end_token);
		}

		//dir_type = 0;

		point_count++;
		token = strtok_s(NULL, "\n", &end_str);
	}

	std::cout << "degree:(" << tmp_quat_x << "\n";
	std::cout<<"vector3 rot:(" << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;

	if (tmpPC_T.mPC.size() > 0) {
		//(*pointCloudSet).push_back(tmpPC);
		(*PointCloudArr).push_back(tmpPC_T);
		PointCloud_idx_show = (*PointCloudArr).size() - 1;
		//std::cout << "(*pointCloudSet).size: " << (*pointCloudSet).size() << std::endl;
		std::cout << "(*PointCloudArr).size: " << (*PointCloudArr).size() << std::endl;
		std::cout << "(*mPC).size: " << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << std::endl;
		(*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr = tmp_quat_x;
		Vector3 tmp_vec = Vector3(tmp_quat_y, tmp_quat_z, tmp_quat_w);
		(*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr = tmp_vec;
		///////////////////////////////
		//if (pointCloudSet->size() == 1) {
		//	CombinePC(0);
		//}
		Vector3 *now_vector;
		now_vector = new Vector3(tmp_quat_y, tmp_quat_z, tmp_quat_w);
		float now_degree = tmp_quat_x;
		(*input_rotVec).push_back(*now_vector);
		(*input_dgree).push_back(now_degree);
		//Find_min_quat2();
		Find_min_quat3();
		Find_max_min();
	}
	delete test_char;
	//PointCloud_idx_show = (*pointCloudSet).size() - 1;
	std::cout << "PointCloud_idx_show: " << PointCloud_idx_show << std::endl;

}
void DentistDemo::MyForm::next_cloud_Click(System::Object^  sender, System::EventArgs^  e) {
}
void DentistDemo::MyForm::Combine_cloud_Click(System::Object^  sender, System::EventArgs^  e) {
	for (int i = 0; i < (*PointCloudArr).size(); i++) {
		for (int j = 0; j < (*PointCloudArr)[i].choose_PC.size(); j++) {
			PointData tmp = (*PointCloudArr)[i].choose_PC[j];
			(*Combine_cloud_PC).push_back(tmp);
		}
	}
	std::cout << "combinePC size:" << (*Combine_cloud_PC).size();
	Find_combine_maxmin();
	show_first_point = !show_first_point;
	show_second_point = !show_second_point;
	show_combine_cloud = !show_combine_cloud;
}
void DentistDemo::MyForm::Add2Combine_Click(System::Object^  sender, System::EventArgs^  e) {
	if (!show_noGyro_PC) {
		(*PointCloudArr)[PointCloud_idx_show].choose_PC = (*PointCloudArr)[PointCloud_idx_show].mPC;
	}
	else
		(*PointCloudArr)[PointCloud_idx_show].choose_PC = (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro;

}
//void DentistDemo::MyForm::Combine_Click(System::Object^  sender, System::EventArgs^  e) {
//}
void DentistDemo::MyForm::Save_combine_Click(System::Object^  sender, System::EventArgs^  e) {
	clock_t tmp_file_name;
	std::string filename;
	std::string fileidx;
	tmp_file_name = clock();
	int tnp_idx = PointCloud_idx_show;

	Quaternion test_quat;
	Radian *test_diff, *quat1_diff;
	Vector3 *test_rotation, *quat1_rotation;
	//test_diff = new Radian((*pointCloud_radian)[PointCloud_idx_show]);
	//test_rotation = new Vector3((*pointCloud_quat_vector)[PointCloud_idx_show].x,
	//							(*pointCloud_quat_vector)[PointCloud_idx_show].y,
	//							(*pointCloud_quat_vector)[PointCloud_idx_show].z);
	quat1_diff = new Radian;
	quat1_rotation = new Vector3;
	//test_quat = sysManager->GetGloveDataLPtr()->GetQuaternion();
	//test_quat = test_quat * (*quat1);
	//test_quat = test_quat.Inverse();

	//test_quat.ToAngleAxis(*test_diff, *test_rotation);
	(*quat1).ToAngleAxis(*quat1_diff, *quat1_rotation);



	////////////////////////////////////////////////////////////////////////////////////////////////////
	//std::vector<GlobalRegistration::Point3D> tmpPC;
	//for (int j = 0; j < (*pointCloudSet)[tnp_idx].size(); j++) {
	//	GlobalRegistration::Point3D tmp;
	//
	//	tmp.x() = (*pointCloudSet)[tnp_idx][j].x();
	//	tmp.y() = (*pointCloudSet)[tnp_idx][j].y();
	//	tmp.z() = (*pointCloudSet)[tnp_idx][j].z();
	//	tmpPC.push_back(tmp);
	//}
	//
	//for (int j = 0; j < tmpPC.size(); j++) {
	//
	//	tmpPC[j].x() = tmpPC[j].x() - (*Point_cloud_center)[0].x;
	//	tmpPC[j].y() = tmpPC[j].y() - (*Point_cloud_center)[0].y;
	//	tmpPC[j].z() = tmpPC[j].z() - (*Point_cloud_center)[0].z;
	//
	//	glm::vec4 *tmp_vv;
	//	tmp_vv = new glm::vec4(tmpPC[j].x(), tmpPC[j].y(), tmpPC[j].z(), 1);
	//	//*tmp_v = (*tmp_M)* (*tmp_v);
	//	*tmp_vv = (*tmp_MM)*(*tmp_vv);
	//
	//	tmpPC[j].x() = (*tmp_vv).x + (*Point_cloud_center)[0].x;
	//	tmpPC[j].y() = (*tmp_vv).y + (*Point_cloud_center)[0].y;
	//	tmpPC[j].z() = (*tmp_vv).z + (*Point_cloud_center)[0].z;
	//}
	//
	////std::cout << "test_diff" << (*test_diff).valueDegrees() << std::endl;
	//std::cout << "test_rotation:(" << test_rotation->x << "," << test_rotation->y << "," << test_rotation->z << ")" << std::endl;
	//std::cout << "(*Point_cloud_center)[0]:(" << (*Point_cloud_center)[0].x << "," << (*Point_cloud_center)[0].y << "," << (*Point_cloud_center)[0].z << ")" << std::endl;
	//glTranslatef(-(*Point_cloud_center)[0].x,-(*Point_cloud_center)[0].y, -(*Point_cloud_center)[0].z);
	//std::cout << "(*test_diff).valueDegrees():(" << (*output_diff).valueDegrees() << std::endl;
	//std::cout << "(*test_rotation):(" << (*output_rotVec).x << "," << (*output_rotVec).y << "," << (*output_rotVec).z << ")" << std::endl;
	///////////////////////////////////////////////////////////////////////////


	//std::cout << "tmp_file_name: " << tmp_file_name << ", PointCloud_idx_show:" << tnp_idx << std::endl;
	MarshalString(tmp_file_name.ToString(), filename);
	//MarshalString(tnp_idx.ToString(), fileidx);

	std::ofstream  output_file;
	output_file.open(filename + "__combine.txt");
	//output_file << "1234" << std::endl;

	//output_file << (*quat1_diff).valueDegrees() << " ";
	//output_file << (*quat1_rotation).x << " ";
	//output_file << (*quat1_rotation).y << " ";
	//output_file << (*quat1_rotation).z << " ";
	//output_file << "\n";
	//
	//output_file <<  (*output_diff).valueDegrees()<< " ";
	//output_file << (*output_rotVec).x << " ";
	//output_file << (*output_rotVec).y << " ";
	//output_file << (*output_rotVec).z << " ";
	//
	//output_file << "\n";
	//
	/*output_file << (*quat3).x << " ";
	output_file << (*quat3).y << " ";
	output_file << (*quat3).z << " ";
	output_file << (*quat3).w << " ";
	output_file << "\n";*/
	

	for (int i = 0; i < (*Combine_cloud_PC).size(); i++)
	{
		//swap x and y
		output_file << (*Combine_cloud_PC)[i].Position.x() << " ";
		output_file << (*Combine_cloud_PC)[i].Position.y() << " ";
		output_file << (*Combine_cloud_PC)[i].Position.z() << " ";
		output_file << "0 0";
		output_file << "\n";
	}
	output_file.close();
	//saveFileDialog1->ShowDialog();
	std::cout << "Save_OK" << std::endl;

}
//void DentistDemo::MyForm::Part_Align_Click(System::Object^  sender, System::EventArgs^  e) {
//
//}
//void DentistDemo::MyForm::Chose_Align_Click(System::Object^  sender, System::EventArgs^  e) {
//
//}
//void DentistDemo::MyForm::overlap_constant1_Scroll(System::Object^  sender, System::EventArgs^  e) {
//
//}
//void DentistDemo::MyForm::overlap_constant2_Scroll(System::Object^  sender, System::EventArgs^  e) {
//
//}
void DentistDemo::MyForm::Read_point_v2_Click(System::Object^  sender, System::EventArgs^  e) {
	std::cout << "filename_array->size():" << filename_array->size() << std::endl;
	for (int i = 0; i < filename_array->size(); i++) {
		read_point_v2(i);
	}

}
void DentistDemo::MyForm::Rot_test_Click(System::Object^  sender, System::EventArgs^  e) {


}
void DentistDemo::MyForm::Aligned_rot_Click(System::Object^  sender, System::EventArgs^  e) {
	
}
void DentistDemo::MyForm::Aligned12_Click(System::Object^  sender, System::EventArgs^  e) {
	if ((*PointCloudArr).size() > 1) {
		clock_t tmp_t1 = clock();
		std::vector<GlobalRegistration::Point3D> tmpPC_0,tmpPC_now,tmpPC_noGyro;
		for (int i = 0; i < (*PointCloudArr)[0].mPC.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[0].mPC[i].Position;
			tmpPC_0.push_back(tmp);
		}
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].mPC.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show].mPC[i].Position;
			tmpPC_now.push_back(tmp);
		}
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[i].Position;
			tmpPC_noGyro.push_back(tmp);
		}
		int try_count = 0;
		while (final_score < super_iterTre&&try_count<super_iterCount) {
			if (!show_noGyro_PC) {
				super4PCS_Align(tmpPC_0, &tmpPC_now, super_max_time);
			}
			else {
				super4PCS_Align(tmpPC_0, &tmpPC_noGyro, super_max_time);
			}
			//std::cout << "final_score:" << final_score << std::endl;
			try_count++;
		}
		//for (int i = 0; i < tmpPC_0.size(); i++) {
		//	(*PointCloudArr)[0].mPC[i].Position = tmpPC_0[i];
		//}
		for (int i = 0; i < tmpPC_now.size(); i++) {
			(*PointCloudArr)[PointCloud_idx_show].mPC[i].Position = tmpPC_now[i];
		}
		for (int i = 0; i < tmpPC_noGyro.size(); i++) {
			(*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[i].Position = tmpPC_noGyro[i];
		}

		final_score = 0;
		try_count = 0;
		//CombinePC(PointCloud_idx_show);
		//show_Cloud_1ID = 0;
		clock_t tmp_t2 = clock();
		float tmp_align_t = ((tmp_t2 - tmp_t1) / (double)(CLOCKS_PER_SEC));
		all_time = all_time + tmp_align_t;
		std::cout << "tmp_align_t:" << tmp_align_t << "s" << std::endl;
		std::cout << "all_time:" << all_time << "s" << std::endl;
	}

}
void DentistDemo::MyForm::Aligned_near_Click(System::Object^  sender, System::EventArgs^  e) {
	if (PointCloudArr->size() > 1) {
		clock_t tmp_t1 = clock();
		int try_count = 0;
		std::vector<GlobalRegistration::Point3D> tmpPC_Near, tmpPC_now, tmpPC_noGyro;
		int tmp_near_id = (*PointCloudArr)[PointCloud_idx_show].near_idx2;
		for (int i = 0; i < (*PointCloudArr)[tmp_near_id].mPC.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[tmp_near_id].mPC[i].Position;
			tmpPC_Near.push_back(tmp);
		}
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].mPC.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show].mPC[i].Position;
			tmpPC_now.push_back(tmp);
		}
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[i].Position;
			tmpPC_noGyro.push_back(tmp);
		}
		while (final_score < super_iterTre&&try_count<super_iterCount) {
			if (!show_noGyro_PC) {
				super4PCS_Align(tmpPC_Near, &tmpPC_now, super_max_time);
				//std::cout << "final_score:" << final_score << std::endl;
			}
			else {
				super4PCS_Align(tmpPC_Near, &tmpPC_noGyro, super_max_time);
			}
			try_count++;
		}
		//for (int i = 0; i < tmpPC_Near.size(); i++) {
		//	(*PointCloudArr)[tmp_near_id].mPC[i].Position = tmpPC_Near[i];
		//}
		for (int i = 0; i < tmpPC_now.size(); i++) {
			(*PointCloudArr)[PointCloud_idx_show].mPC[i].Position = tmpPC_now[i];
		}
		for (int i = 0; i < tmpPC_noGyro.size(); i++) {
			(*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[i].Position = tmpPC_noGyro[i];
		}
		final_score = 0;
		try_count = 0;
		//CombinePC(PointCloud_idx_show);
		//show_Cloud_1ID = tmp_near_id;
		clock_t tmp_t2 = clock();
		float tmp_align_t = ((tmp_t2 - tmp_t1) / (double)(CLOCKS_PER_SEC));
		all_time = all_time + tmp_align_t;
		std::cout << "Near cloud idx:" << tmp_near_id << "\n";
		std::cout << "tmp_align_t:" << tmp_align_t << "s" << std::endl;
		std::cout << "all_time:" << all_time << "s" << std::endl;
	}
}
void DentistDemo::MyForm::Aligned_Pre_Click(System::Object^  sender, System::EventArgs^  e) {
	if (PointCloudArr->size() > 1) {
		clock_t tmp_t1 = clock();
		int try_count = 0;
		std::vector<GlobalRegistration::Point3D> tmpPC_Pre, tmpPC_now, tmpPC_noGyro;
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show - 1].mPC.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show - 1].mPC[i].Position;
			tmpPC_Pre.push_back(tmp);
		}
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].mPC.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show].mPC[i].Position;
			tmpPC_now.push_back(tmp);
		}
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[i].Position;
			tmpPC_noGyro.push_back(tmp);
		}
		while (final_score < super_iterTre&&try_count<super_iterCount) {
			if (!show_noGyro_PC) {
				super4PCS_Align(tmpPC_Pre, &tmpPC_now, super_max_time);
			}
			else {
				super4PCS_Align(tmpPC_Pre, &tmpPC_noGyro, super_max_time);
			}
			//std::cout << "final_score:" << final_score << std::endl;
			try_count++;
		}
		//for (int i = 0; i < tmpPC_Pre.size(); i++) {
		//	(*PointCloudArr)[PointCloud_idx_show - 1].mPC[i].Position = tmpPC_Pre[i];
		//}
		for (int i = 0; i < tmpPC_now.size(); i++) {
			(*PointCloudArr)[PointCloud_idx_show].mPC[i].Position = tmpPC_now[i];
		}
		for (int i = 0; i < tmpPC_noGyro.size(); i++) {
			(*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[i].Position = tmpPC_noGyro[i];
		}
		final_score = 0;
		try_count = 0;
		//CombinePC(PointCloud_idx_show);
		//show_Cloud_1ID = PointCloud_idx_show - 1;
		clock_t tmp_t2 = clock();
		float tmp_align_t = ((tmp_t2 - tmp_t1) / (double)(CLOCKS_PER_SEC));
		all_time = all_time + tmp_align_t;
		std::cout << "tmp_align_t:" << tmp_align_t << "s" << std::endl;
		std::cout << "all_time:" << all_time << "s" << std::endl;
	}

}
void DentistDemo::MyForm::Aligned_target_Click(System::Object^  sender, System::EventArgs^  e) {
	if (PointCloudArr->size() > 1) {
		clock_t tmp_t1 = clock();
		int try_count = 0;
		std::vector<GlobalRegistration::Point3D> tmpPC_target, tmpPC_now, tmpPC_noGyro;
		for (int i = 0; i < (*PointCloudArr)[Aligned_target_id].mPC.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[Aligned_target_id].mPC[i].Position;
			tmpPC_target.push_back(tmp);
		}
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].mPC.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show].mPC[i].Position;
			tmpPC_now.push_back(tmp);
		}
		for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size(); i++) {
			GlobalRegistration::Point3D tmp;
			tmp = (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[i].Position;
			tmpPC_noGyro.push_back(tmp);
		}
		while (final_score < super_iterTre&&try_count<super_iterCount) {
			if (!show_noGyro_PC) {
				super4PCS_Align(tmpPC_target, &tmpPC_now, super_max_time);
			}
			else {
				super4PCS_Align(tmpPC_target, &tmpPC_noGyro, super_max_time);
			}
			//std::cout << "final_score:" << final_score << std::endl;
			try_count++;
		}
		//for (int i = 0; i < tmpPC_target.size(); i++) {
		//	(*PointCloudArr)[Aligned_target_id].mPC[i].Position = tmpPC_target[i];
		//}
		for (int i = 0; i < tmpPC_now.size(); i++) {
			(*PointCloudArr)[PointCloud_idx_show].mPC[i].Position = tmpPC_now[i];
		}
		for (int i = 0; i < tmpPC_noGyro.size(); i++) {
			(*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[i].Position = tmpPC_noGyro[i];
		}
		final_score = 0;
		try_count = 0;
		//CombinePC(PointCloud_idx_show);
		//show_Cloud_1ID = Aligned_target_id;
		clock_t tmp_t2 = clock();
		float tmp_align_t = ((tmp_t2 - tmp_t1) / (double)(CLOCKS_PER_SEC));
		all_time = all_time + tmp_align_t;
		std::cout << "show_Cloud_1ID:" << show_Cloud_1ID << "\n";
		std::cout << "tmp_align_t:" << tmp_align_t << "s" << std::endl;
		std::cout << "all_time:" << all_time << "s" << std::endl;
	}
}


void DentistDemo::MyForm::RotatePC() {
	
}
void DentistDemo::MyForm::unsigned_short_to_char(unsigned short *input, int inputlen, char *output) {

	int outputlen_tmp = 0;

	for (int i = 0; i < inputlen; i++) {
		unsigned short tmp_short = input[i];
		tmp_short = tmp_short >> 8;

		output[outputlen_tmp] = input[i];
		outputlen_tmp++;

		output[outputlen_tmp] = tmp_short;
		outputlen_tmp++;


	}

}
void DentistDemo::MyForm::test_start_cap(int32_t DevIn, uint32_t *Handles, float Lv_65,
	uint32_t SampRec, uint32_t *ByteLen, LVBoolean SaveDat, char SaveName[],
	LVBoolean *ErrBool, char ADCStat[], int32_t len, int32_t *len2) {

	*Handles = 8872;
	*ByteLen = PIC_SIZE;
	char test_ADCStat[] = "ATS : ApiSuccess <512>";
	strcpy(ADCStat, test_ADCStat);
	*len2 = 22;
}
void DentistDemo::MyForm::test_scanADC(uint32_t Handles, uint32_t ByteBuf, uint16_t ArrSize[],
	int32_t lenArr, uint16_t OutArr[], int32_t lenOut, int32_t *lenOut2,
	char ADCStat[], int32_t lenStat, int32_t *lenStatOut) {

	*lenOut2 = PIC_SIZE;
	*lenStatOut = 22;
	//unsigned short *interator;
	//interator = new unsigned short;
	//interator = OutArr;

	srand((unsigned)time(NULL));
	for (int i = 0; i < PIC_SIZE; i++) {
		OutArr[i] = rand() % 65536;
	}
	//*interator = *outarr;
	//interator += pic_count * PIC_SIZE;

}
void DentistDemo::MyForm::draw_volumeData(TRcuda *theTRcuda) {
	if (theTRcuda->VolumeData == NULL)
		return;

	int tmpIdx;
	float idt, posX, posY, posZ;
	glPointSize(thePointSize);
	glBegin(GL_POINTS);
	for (int y = 0; y < theTRcuda->VolumeSize_Y; y++)
	{
		for (int z = 0; z < theTRcuda->VolumeSize_Z; z++)
		{
			tmpIdx = ((volumeDataIdx * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z + z;
			idt = ((float)theTRcuda->VolumeData[tmpIdx] - 3)/2;
			glColor3f(idt, idt, idt);
			posX = volumeDataIdx;
			posY = y;
			posZ = z;
			glVertex3f(posX / 100, posY / 100, posZ / 200);
		}
	}
	int idy = volumeZ_now;
	glColor3f(0.2f, 0.8f, 0.2f);
	posX = volumeDataIdx;
	posY = volumeZ_now + 300;
	posZ = -1;
	glVertex3f(posX / 100, posY / 100, posZ / 200);
	for (int z = 0; z < theTRcuda->VolumeSize_Z; z++)
	{
		tmpIdx = ((volumeDataIdx * theTRcuda->VolumeSize_Y) + idy) * theTRcuda->VolumeSize_Z + z;
		idt = theTRcuda->VolumeDataAvg[tmpIdx];
		if (idt < 0)
			idt = 0;
		idt = idt / 5 * 100;
		if (theTRcuda->PointType[tmpIdx] == 1)
		{
			glColor3f(0.2f, 0.8f, 0.2f);
		}//*
		else if (theTRcuda->PointType[tmpIdx] == 3)
		{
			glColor3f(0.2f, 0.2f, 0.8f);
		}
		else if(theTRcuda->VolumeDataAvg[tmpIdx] > theTRcuda->energyGap)
		{
			glColor3f(1.0f, 1.0f, 1.0f);
		}
		else
		{
			glColor3f(0.0f, 0.0f, 0.0f);
		}//*
		posX = volumeDataIdx;
		posY = 550 + idt;
		posZ = z;
		glVertex3f(posX / 100, posY / 100, posZ / 200);
	}

	glEnd();

}
void DentistDemo::MyForm::drawPointType(TRcuda *theTRcuda) {
	if (theTRcuda->PointType == NULL)
		return;

	int tmpIdx;
	float idt, posX, posY, posZ;
	glPointSize(thePointSize);
	glBegin(GL_POINTS);
	for (int y = 0; y < theTRcuda->VolumeSize_Y; y++)
	{
		for (int z = 0; z < theTRcuda->VolumeSize_Z; z++)
		{
			tmpIdx = ((volumeDataIdx * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z + z;

			idt = (theTRcuda->VolumeData[tmpIdx] - 3)/2;
			glColor3f(idt, idt, idt);

			if ( z == 0)
			{
				// z axis 0 = peak count
				if (theTRcuda->PointType[tmpIdx] == 0)
					glColor3f(0.0f, 0.0f, 0.0f);
				if (theTRcuda->PointType[tmpIdx] == 1)
					glColor3f(0.8f, 0.2f, 0.2f);
				if (theTRcuda->PointType[tmpIdx] == -1)
					glColor3f(0.2f, 0.2f, 0.8f);
			}
			else if (z == 2)
			{
				// z axis 2 = init board : -1 = t, 0 = f
				if (theTRcuda->PointType[tmpIdx] == -1)
					glColor3f(0.8f, 0.2f, 0.2f);
			}
			else if (z == 3)
			{
				// z axis 3 = found board : -1 = no board, 0 = need find, 1 = found
				if (theTRcuda->PointType[tmpIdx] == -1)
					glColor3f(0.0f, 0.0f, 0.0f);
				if (theTRcuda->PointType[tmpIdx] == 0)
					glColor3f(0.2f, 0.2f, 0.8f);
				if (theTRcuda->PointType[tmpIdx] == 1)
					glColor3f(0.8f, 0.2f, 0.2f);
			}
			else if (z == 5)
			{
				// z axis 5 = neighbor board check state : -1 = find end, 0 = finding neighbor, 1 = finding nearist board
				if (theTRcuda->PointType[tmpIdx] == -1)
					glColor3f(0.0f, 0.0f, 0.0f);
				if (theTRcuda->PointType[tmpIdx] == 0)
					glColor3f(0.2f, 0.2f, 0.8f);
				if (theTRcuda->PointType[tmpIdx] == 1)
					glColor3f(0.8f, 0.2f, 0.2f);
			}
			//*
			else if (z > 5 && z < 10)
			{
				if (theTRcuda->PointType[tmpIdx] != 0)
					glColor3f(0.8f, 0.2f, 0.2f);
			}//*/
			 // 2 = up peak, 3 = peak >= gap, 1 = peak Continuous
			else if (theTRcuda->PointType[tmpIdx] == 1)
			{
				glColor3f(0.2f, 0.2f, 0.8f);
			}//*
			else if (theTRcuda->PointType[tmpIdx] == 3)
			{
				glColor3f(0.2f, 0.8f, 0.2f);
			}//*
			posX = volumeDataIdx;
			posY = y + 300;
			posZ = z;
			glVertex3f(posX / 100, posY / 100, posZ / 200);
		}
	}
	glEnd();

}
void DentistDemo::MyForm::draw_decay() {
	//std::cout << "draw_decay\n";
	if ((*teeth_imgPC).empty())
		return;
	//glPointSize(thePointSize);
	//glBegin(GL_POINTS);
	//for (int i = 0; i < teeth_imgPC->size(); i++) {
	//	//glColor3f((*teeth_imgPC)[i].idt.x, (*teeth_imgPC)[i].idt.y, (*teeth_imgPC)[i].idt.z);
	//	glColor3f(0.2*(*teeth_imgPC)[i].idt.x, 0.2*(*teeth_imgPC)[i].idt.y, 0.8*(*teeth_imgPC)[i].idt.z);
	//	glVertex3f((*teeth_imgPC)[i].position.x , (*teeth_imgPC)[i].position.y, (*teeth_imgPC)[i].position.z);
	//}
	//
	//glEnd();

	glManager->is_PC = true;
}

void DentistDemo::MyForm::Rotate_quat(float angle, float x, float y, float z)
{
	float r = angle * glm::pi<float>() / 180.0f;
	glm::mat4 M = glm::mat4(1);

	glm::vec4 c0 = glm::vec4(cos(r) + (1 - cos(r))*x*x, (1 - cos(r))*y*x + sin(r)*z, (1 - cos(r))*z*x - sin(r)*y, 0);
	glm::vec4 c1 = glm::vec4((1 - cos(r))*y*x - sin(r)*z, cos(r) + (1 - cos(r))*y*y, (1 - cos(r))*z*y + sin(r)*x, 0);
	glm::vec4 c2 = glm::vec4((1 - cos(r))*z*x + sin(r)*y, (1 - cos(r))*z*y - sin(r)*x, cos(r) + (1 - cos(r))*z*z, 0);
	glm::vec4 c3 = glm::vec4(0, 0, 0, 1);
	M = glm::mat4(c0, c1, c2, c3);


	*tmp_MM = M;
}
void DentistDemo::MyForm::mani_rotate(float angle, float x, float y, float z) {
	float r = angle * glm::pi<float>() / 180.0f;
	glm::mat4 M = glm::mat4(1);

	glm::vec4 c0 = glm::vec4(cos(r) + (1 - cos(r))*x*x, (1 - cos(r))*y*x + sin(r)*z, (1 - cos(r))*z*x - sin(r)*y, 0);
	glm::vec4 c1 = glm::vec4((1 - cos(r))*y*x - sin(r)*z, cos(r) + (1 - cos(r))*y*y, (1 - cos(r))*z*y + sin(r)*x, 0);
	glm::vec4 c2 = glm::vec4((1 - cos(r))*z*x + sin(r)*y, (1 - cos(r))*z*y - sin(r)*x, cos(r) + (1 - cos(r))*z*z, 0);
	glm::vec4 c3 = glm::vec4(0, 0, 0, 1);
	M = glm::mat4(c0, c1, c2, c3);


	*mani_MM = M;
}
void DentistDemo::MyForm::reset_rotation(float angle, float x, float y, float z)
{
	float r = angle * glm::pi<float>() / 180.0f;
	glm::mat4 M = glm::mat4(1);

	glm::vec4 c0 = glm::vec4(cos(r) + (1 - cos(r))*x*x, (1 - cos(r))*y*x + sin(r)*z, (1 - cos(r))*z*x - sin(r)*y, 0);
	glm::vec4 c1 = glm::vec4((1 - cos(r))*y*x - sin(r)*z, cos(r) + (1 - cos(r))*y*y, (1 - cos(r))*z*y + sin(r)*x, 0);
	glm::vec4 c2 = glm::vec4((1 - cos(r))*z*x + sin(r)*y, (1 - cos(r))*z*y - sin(r)*x, cos(r) + (1 - cos(r))*z*z, 0);
	glm::vec4 c3 = glm::vec4(0, 0, 0, 1);
	M = glm::mat4(c0, c1, c2, c3);


	*reset_M *= M;
}
void DentistDemo::MyForm::drawBoardTemp(TRcuda *theTRcuda) {
	if (theTRcuda->PointType == NULL)
		return;

	int tmpIdx;
	float idt, posX, posY, posZ;
	glPointSize(thePointSize);
	glBegin(GL_POINTS);
	glColor3f(0.8f, 0.2f, 0.2f);
	for (int y = 0; y < theTRcuda->VolumeSize_Y; y++)
	{
		tmpIdx = ((volumeDataIdx * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
		if (theTRcuda->PointType[tmpIdx + 3] == 1 && theTRcuda->PointType[tmpIdx + 1] != 0)
		{
			posX = volumeDataIdx;
			posY = y + 300;
			posZ = theTRcuda->PointType[tmpIdx + 1];
			glVertex3f(posX / 100, posY / 100, posZ / 200);
		}
	}
	glEnd();

}
void DentistDemo::MyForm::drawPointTypeCloud(TRcuda *theTRcuda) {
	if (theTRcuda->PointType == NULL)
		return;
	if ((*PointCloudArr)[PointCloud_idx_show].origin_volumePC.size() == 0)
		return;

	int tmpIdx;
	float idt, posX, posY, posZ;
	glPointSize(thePointSize);
	glBegin(GL_POINTS);
	glColor3f(0.8f, 0.2f, 0.2f);
	//for (int x = 0; x < theTRcuda->VolumeSize_X; x++)
	//{
	//	for (int y = 0; y < theTRcuda->VolumeSize_Y; y++)
	//	{
	//		tmpIdx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
	//		if (theTRcuda->PointType[tmpIdx + 3] == 1 && theTRcuda->PointType[tmpIdx + 1] != 0)
	//		{
	//			posX = x;
	//			posY = y + 300;
	//			posZ = theTRcuda->PointType[tmpIdx + 1];
	//			idt = 1 - (posZ / theTRcuda->VolumeSize_Z);
	//			if (idt < 0)
	//				idt = 0;
	//			glColor3f(idt, 0.2f, 0.2f);
	//			if (x == volumeDataIdx&&y == volumeZ_now)
	//			{
	//				glColor3f(0.2, 0.8f, 0.2f);
	//			}
	//			glVertex3f(posX / 100, posY / 100, posZ / 200);
	//		}
	//	}
	//}

	for (int i = 0; i < (*PointCloudArr)[PointCloud_idx_show].origin_volumePC.size(); i++) {
		idt = 1 - ((*PointCloudArr)[PointCloud_idx_show].origin_volumePC[i].Position.z() / theTRcuda->VolumeSize_Z);
		glColor3f(idt, 0.2f, 0.2f);
		glVertex3f((*PointCloudArr)[PointCloud_idx_show].origin_volumePC[i].Position.x() / 100, (*PointCloudArr)[PointCloud_idx_show].origin_volumePC[i].Position.y() / 100, (*PointCloudArr)[PointCloud_idx_show].origin_volumePC[i].Position.z() / 200);
	}
	glEnd();

}
void DentistDemo::MyForm::drawZAxisValue(TRcuda *theTRcuda) {
	if (theTRcuda->RawDataScanP == NULL)
		return;

	int tmpIdx, idy = 125;
	float idt, posX, posY, posZ, offsetY = 900, ratioY = 100;

	glPointSize(5.0f);
	glBegin(GL_POINTS);
	for (int z = 0; z < theTRcuda->VolumeSize_Z * 2; z++)
	{
		tmpIdx = idy * theTRcuda->VolumeSize_Z * 2 + z;
		idt = theTRcuda->RawDataScanP[tmpIdx];

		posX = volumeDataIdx;
		posY = offsetY + (idt) / ratioY;
		posZ = z;
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3f(posX / 100, posY / 100, posZ / 200);

		//glColor3f(0.0f, 0.0f, 0.0f);
		//glVertex3f(posX / 100, offsetY / 100, posZ / 200);
	}

	glEnd();

	glLineWidth(2.5f);
	glBegin(GL_LINES);

	glColor3f(0.0f, 0.0f, 0.0f);

	glVertex3f((float)volumeDataIdx / 100, offsetY / 100, 0 / 200);
	glVertex3f((float)volumeDataIdx / 100, offsetY / 100, (theTRcuda->VolumeSize_Z * 2 - 1) / 200);

	glColor3f(0.0f, 0.0f, 1.0f);

	for (int z = 1; z < theTRcuda->VolumeSize_Z * 2; z++)
	{
		tmpIdx = idy * theTRcuda->VolumeSize_Z * 2 + z - 1;
		idt = theTRcuda->RawDataScanP[tmpIdx];

		posX = volumeDataIdx;
		posY = offsetY + (idt) / ratioY;
		posZ = z - 1;
		glVertex3f(posX / 100, posY / 100, posZ / 200);

		tmpIdx = idy * theTRcuda->VolumeSize_Z * 2 + z;
		idt = theTRcuda->RawDataScanP[tmpIdx];

		posX = volumeDataIdx;
		posY = offsetY + (idt) / ratioY;
		posZ = z;
		glVertex3f(posX / 100, posY / 100, posZ / 200);
	}

	glEnd();

}
void DentistDemo::MyForm::draw_before_mapping() {
	int tmpIdx;
	float idt, posX, posY, posZ;
	for (int x = 0; x < theTRcuda->VolumeSize_X; x++)
	{
		for (int y = 0; y < theTRcuda->VolumeSize_Y; y++)
		{
			tmpIdx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
			if (theTRcuda->PointType[tmpIdx + 3] == 1 && theTRcuda->PointType[tmpIdx + 1] != 0)
			{

				PointData tmpPC;
				posX = x;
				posY = y + 300;
				posZ = theTRcuda->PointType[tmpIdx + 1];
				tmpPC.Position.x() = posX;
				tmpPC.Position.y() = posY;
				tmpPC.Position.z() = posZ;

				(*PointCloudArr)[PointCloud_idx_show].origin_volumePC.push_back(tmpPC);
				//idt = 1 - (posZ / theTRcuda->VolumeSize_Z);
				//if (idt < 0)
				//	idt = 0;
				//glColor3f(idt, 0.2f, 0.2f);
				//if (x == volumeDataIdx&&y == volumeZ_now)
				//{
				//	glColor3f(0.2, 0.8f, 0.2f);
				//}
				////glVertex3f(posX / 100, posY / 100, posZ / 200);
			}
		}
	}
}
void DentistDemo::MyForm::choose_slice(int pic_num) {
	Vector3 p1, p2, p3, p4;
	p1 = Vector3 (pic_num, 0, 0);
	p2 = Vector3(pic_num, 250, 0);
	p3 = Vector3(pic_num, 250, 250);
	p3 = Vector3(pic_num, 0, 250);

	glBegin(GL_QUADS);
	glColor3f(0.8f, 0.2f, 0.2f);


	glEnd();

}
//void DentistDemo::MyForm::drawPCSet() {
//
//}
//void DentistDemo::MyForm::drawPCSet2() {
//
//}
void DentistDemo::MyForm::drawPCSet3() {
	if (is_camera_move) {
		Radian *camera_diff;
		Vector3 *camera_rotation;
		Quaternion *tmp_quat2;
		tmp_quat2 = new Quaternion;
		camera_diff = new Radian;
		camera_rotation = new Vector3;



		(*camera_quat) = sysManager->GetGloveDataLPtr()->GetQuaternion();

		if (is_reset) {
			preQuaternL = nullptr;
			//std::cout << "reset\n";
			is_reset = false;
		}
		//camera_quat = new Quaternion(-(*camera_quat).x, -(*camera_quat).y, (*camera_quat).z, (*camera_quat).w);
		//(*camera_quat) = (*camera_quat) * Quaternion(0.7f, 0.0f, -0.7f, 0.0f);
		//(*camera_quat).ToAngleAxis(*camera_diff, *camera_rotation);
		//std::cout << "\rvalueDegrees():" << (*camera_diff).valueDegrees() << "--- rotation:(" << (*camera_rotation).x << "---," << (*camera_rotation).y << "---," << (*camera_rotation).z << "---)";
		//std::cout << "rotation:(" << (*camera_rotation).x << "," << (*camera_rotation).y << "," << (*camera_rotation).z << ")---";

		//glPushMatrix();
		//glRotatef((*camera_diff).valueDegrees(), -(*camera_rotation).z, (*camera_rotation).y, (*camera_rotation).x);

		//glTranslatef(0, 0, -5);
		//glTranslatef((*Point_cloud_center)[0].x, (*Point_cloud_center)[0].y, (*Point_cloud_center)[0].z);
		//camera_cube(1.0f, 1.0f, 2.0f);
		//glBegin(GL_LINES);
		//glColor3f(1, 0, 0);
		//glVertex3f(0, 0, -3);
		//glVertex3f(3, 0, -3);
		//glEnd();
		//glBegin(GL_LINES);
		//glColor3f(0, 1, 0);
		//glVertex3f(0, 0, -3);
		//glVertex3f(0, 3, -3);
		//glEnd();
		//glBegin(GL_LINES);
		//glColor3f(0, 0, 1);
		//glVertex3f(0, 0, -3);
		//glVertex3f(0, 0, 0);
		//glEnd();
		//glPopMatrix();
		//
		//glPushMatrix();
		//glRotated((*camera_diff).valueDegrees(), -(*camera_rotation).z, (*camera_rotation).y, (*camera_rotation).x);
		//glBegin(GL_LINES);
		//glColor3f(1, 1, 1);
		//glVertex3f(0, 0, 0);
		//glVertex3f(0, 0, -5);
		//glEnd();
		//glPopMatrix();

		//(*quat1) = (*camera_quat);
		//(*quat1) = (*quat1).Inverse();

		delete tmp_quat2;
		delete camera_diff;
		delete camera_rotation;
	}
	if (/*(*pointCloudSet).size()*/(*PointCloudArr).size() < 1)
	{
		return;
	}
	glPointSize(thePointSize);
	double idt, tmpx, tmpy, tmpz;
	if ((*PointCloudArr)[0].mPC.size() > 0) {
		if (show_first_point) {

			glPushMatrix();
			//glTranslatef((*PointCloudArr)[0].Point_cloud_center.x, (*PointCloudArr)[0].Point_cloud_center.y, (*PointCloudArr)[0].Point_cloud_center.z);
			//glRotatef(tmp_rotate_x, 1.0, 0, 0);
			//glRotatef(tmp_rotate_y, 0, 1.0, 0);
			//glRotatef(tmp_rotate_z, 0, 0, 1.0);
			//glTranslatef(-(*PointCloudArr)[0].Point_cloud_center.x, -(*PointCloudArr)[0].Point_cloud_center.y, -(*PointCloudArr)[0].Point_cloud_center.z);
			glBegin(GL_POINTS);
			for (int j = 0; j < (*PointCloudArr)[0].mPC.size(); j++)
			{
				tmpx = (*PointCloudArr)[0].mPC[j].Position.x();
				tmpy = (*PointCloudArr)[0].mPC[j].Position.y();
				tmpz = (*PointCloudArr)[0].mPC[j].Position.z();
				glColor3f(0.2f, 0.2f, 1 - (float)((*PointCloudArr)[0].mPC[j].Position.z() - (*PointCloudArr)[0].Point_cloud_Min.z) / ((*PointCloudArr)[0].Point_cloud_Max.z - (*PointCloudArr)[0].Point_cloud_Min.z));

				glVertex3f(tmpx, tmpy, tmpz);
			}
			glEnd();
			glPopMatrix();
		}
		if ((*PointCloudArr).size() > 1)
		{
			if (show_second_point) {
				if (!show_noGyro_PC) {
					glPushMatrix();
					glTranslatef((*PointCloudArr)[0].Point_cloud_center.x, (*PointCloudArr)[0].Point_cloud_center.y, (*PointCloudArr)[0].Point_cloud_center.z);
					glRotatef(tmp_rotate_x, 1.0, 0, 0);
					glRotatef(tmp_rotate_y, 0, 1.0, 0);
					glRotatef(tmp_rotate_z, 0, 0, 1.0);
					glTranslatef(-(*PointCloudArr)[0].Point_cloud_center.x, -(*PointCloudArr)[0].Point_cloud_center.y, -(*PointCloudArr)[0].Point_cloud_center.z);
					glBegin(GL_POINTS);
					for (int j = 0; j < (*PointCloudArr)[PointCloud_idx_show].mPC.size(); j++)
					{
						//std::cout << "PointCloud_idx_show : " << PointCloud_idx_show << std::endl;
						//idt = ((*pointCloudSet)[PointCloud_idx_show][j].z() - 1) / 2;
						tmpx = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.x();
						tmpy = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.y();
						tmpz = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.z();

						glColor3f(0.2f, 1 - (float)(tmpz - (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.z) / ((*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.z - (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.z), 0.2f);
						//glTranslatef((*result_movement).x, (*result_movement).y, (*result_movement).z);
						glVertex3f(tmpx, tmpy, tmpz);

					}
					glEnd();
					glPopMatrix();
				}
				else {
					glPushMatrix();
					glBegin(GL_POINTS);
					for (int j = 0; j < (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size(); j++)
					{
						//std::cout << "PointCloud_idx_show : " << PointCloud_idx_show << std::endl;
						//idt = ((*pointCloudSet)[PointCloud_idx_show][j].z() - 1) / 2;
						tmpx = (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[j].Position.x();
						tmpy = (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[j].Position.y();
						tmpz = (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro[j].Position.z();

						glColor3f(0.2f, 1 - (float)(tmpz - (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.z) / ((*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.z - (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.z), 0.2f);
						//glTranslatef((*result_movement).x, (*result_movement).y, (*result_movement).z);
						glVertex3f(tmpx, tmpy, tmpz);

					}
					glEnd();
					glPopMatrix();
				
				}
			}

		}
		if ((*Combine_cloud_PC).size() > 0) {
			if (show_combine_cloud) {
				glPushMatrix();
				glBegin(GL_POINTS);
				for (int j = 0; j < (*Combine_cloud_PC).size(); j++)
				{
					//std::cout << "PointCloud_idx_show : " << PointCloud_idx_show << std::endl;
					//idt = ((*pointCloudSet)[PointCloud_idx_show][j].z() - 1) / 2;
					tmpx = (*Combine_cloud_PC)[j].Position.x();
					tmpy = (*Combine_cloud_PC)[j].Position.y();
					tmpz = (*Combine_cloud_PC)[j].Position.z();

					glColor3f(0.2f, 0.2f, 1 - (float)(tmpz - (*combine_min).z) / ((*combine_max).z - (*combine_min).z));
					//glTranslatef((*result_movement).x, (*result_movement).y, (*result_movement).z);
					glVertex3f(tmpx, tmpy, tmpz);

				}
				glEnd();
				glPopMatrix();
			}

		}
	}

}
void DentistDemo::MyForm::Find_combine_maxmin() {
	float idt, tmpx, tmpy, tmpz;
	for (int j = 0; j < (*Combine_cloud_PC).size(); j++) {
		tmpx = (*Combine_cloud_PC)[j].Position.x();
		tmpy = (*Combine_cloud_PC)[j].Position.y();
		tmpz = (*Combine_cloud_PC)[j].Position.z();
		if (tmpx >(*combine_max).x) {
			(*combine_max).x = tmpx;
		}
		if (tmpx < (*combine_min).x) {
			(*combine_min).x = tmpx;
		}

		if (tmpy >(*combine_max).y) {
			(*combine_max).y = tmpy;
		}
		if (tmpy < (*combine_min).y) {
			(*combine_min).y = tmpy;
		}
		if (tmpz >(*combine_max).z) {
			(*combine_max).z = tmpz;
		}
		if (tmpz < (*combine_min).z) {
			(*combine_min).z = tmpz;
		}
	}
}
//void DentistDemo::MyForm::MovePC() {
//}
void DentistDemo::MyForm::super4PCS_Align(std::vector<GlobalRegistration::Point3D> PC1, std::vector<GlobalRegistration::Point3D> *PC2, int max_time_seconds) {
	if (PointCloudArr->size() > 1) {
		clock_t t1, t2;
		t1 = clock();

		// Delta (see the paper).
		double delta = 0.1;

		// Estimated overlap (see the paper).
		double overlap = 0.40;

		// Threshold of the computed overlap for termination. 1.0 means don't terminate
		// before the end.
		double thr = 0.35;

		// Maximum norm of RGB values between corresponded points. 1e9 means don't use.
		double max_color = 150;

		// Number of sampled points in both files. The 4PCS allows a very aggressive
		// sampling.
		int n_points = 500;

		// Maximum angle (degrees) between corresponded normals.
		double norm_diff = 20;

		// Maximum allowed computation time.
		//max_time_seconds = 1;//500

		bool use_super4pcs = true;

		// maximum per-dimension angle, check return value to detect invalid cases
		double max_angle = 20;

		//==========//

		//vector<Point3D> set1, set2;
		std::vector<cv::Point2f> tex_coords1, tex_coords2;
		std::vector<cv::Point3f> normals1, normals2;
		std::vector<tripple> tris1, tris2;
		std::vector<std::string> mtls1, mtls2;

		//input1 = std::string("Data\\hippo1.obj");
		//input2 = std::string("Data\\hippo2.obj");
		// super4pcs IOManager
		IOManager iomananger;


		// super4pcs matcher
		GlobalRegistration::Match4PCSOptions options;


		//cv::Mat mat_rot = cv::Mat::eye(3, 3, CV_64F);
		GlobalRegistration::Match4PCSBase::MatrixType *mat;
		mat = new GlobalRegistration::Match4PCSBase::MatrixType;
		

		bool overlapOk = options.configureOverlap(overlap, super_theshold);
		if (!overlapOk) {
			std::cerr << "Invalid overlap configuration. ABORT" << std::endl;
			/// TODO Add proper error codes

		}

		//overlap = options.getOverlapEstimation();
		options.sample_size = n_points;
		options.max_normal_difference = norm_diff;
		options.max_color_distance = max_color;
		options.max_time_seconds = max_time_seconds;
		options.delta = delta;
		options.max_angle = max_angle;

		GlobalRegistration::Match4PCSOptions::Scalar Estimation;

		Estimation = options.getOverlapEstimation();
		std::cout << "getOverlapEstimation:" << Estimation << std::endl;
		//// Match and return the score (estimated overlap or the LCP).  
		typename Point3D::Scalar score = 0;
		int v;

		constexpr GlobalRegistration::Utils::LogLevel loglvl = GlobalRegistration::Utils::Verbose;
		//cv::Mat mat_rot(*mat);

		//template <typename Visitor>
		using TrVisitorType = typename std::conditional <loglvl == Utils::NoLog,
			Match4PCSBase::DummyTransformVisitor,
			TransformVisitor>::type;

		GlobalRegistration::Utils::Logger logger(loglvl);
		try {

			if (use_super4pcs) {
				GlobalRegistration::MatchSuper4PCS *matcher;
				matcher = new GlobalRegistration::MatchSuper4PCS(options, logger);

				std::cout << "Use Super4PCS" << std::endl;
				score = matcher->ComputeTransformation(PC1, PC2, *mat);
				//mat_rot(*mat);
				//(*mat).an
				translate_mat->push_back(*mat);
				
				std::cout << std::endl;
				std::cout << "score_in:" << score << std::endl;
			}
			else {
				GlobalRegistration::Match4PCS *matcher;
				matcher = new GlobalRegistration::Match4PCS(options, logger);
				std::cout << "Use old 4PCS" << std::endl;
				score = matcher->ComputeTransformation(PC1, PC2, *mat);
				std::cout << "score_in:" << score << std::endl;
			}
		}
		catch (const std::exception& e) {
			std::cout << "[Error]: " << e.what() << '\n';
			std::cout << "Aborting with code -2 ..." << std::endl;
			return;
		}
		catch (...) {
			std::cout << "[Unknown Error]: Aborting with code -3 ..." << std::endl;
			return;
		}
		t2 = clock();
		//qDebug() << "Match done t: " << (t2 - t1) / (double)(CLOCKS_PER_SEC) << " s  ";
		final_score = score;
		std::cout << "Score: " << score << std::endl;
		//cerr << score << endl;
	}
}
//void DentistDemo::MyForm::CombinePC(int idx) {
//
//}
void DentistDemo::MyForm::Find_max_min() {
	//Vector3 *tmp_max, *tmp_min, *tmp_out;
	//tmp_out = new Vector3(0, 0, -10);
	//tmp_max = new Vector3(0, 0, 0);
	//tmp_min = new Vector3(999999, 999999, 999999);
	//(*Point_cloud_Max).push_back(*tmp_max);
	//(*Point_cloud_Min).push_back(*tmp_min);
	//(*Point_cloud_center).push_back(*tmp_max);
	//(*outside_center).push_back(*tmp_max);

	float idt, tmpx, tmpy, tmpz;
	if (/*(*pointCloudSet).size()*/(*PointCloudArr).size() == 1) {
		for (int j = 0; j < /*(*pointCloudSet)[0].size()*/(*PointCloudArr)[0].mPC.size(); j++)
		{
			//tmpx = (*pointCloudSet)[0][j].x();
			//tmpy = (*pointCloudSet)[0][j].y();
			//tmpz = (*pointCloudSet)[0][j].z();
			tmpx = (*PointCloudArr)[0].mPC[j].Position.x();
			tmpy = (*PointCloudArr)[0].mPC[j].Position.y();
			tmpz = (*PointCloudArr)[0].mPC[j].Position.z();

			//if (tmpx >(*Point_cloud_Max)[0].x) {
			//	(*Point_cloud_Max)[0].x = tmpx;
			//}
			//if (tmpx < (*Point_cloud_Min)[0].x) {
			//	(*Point_cloud_Min)[0].x = tmpx;
			//}
			if (tmpx >(*PointCloudArr)[0].Point_cloud_Max.x) {
				(*PointCloudArr)[0].Point_cloud_Max.x = tmpx;
			}
			if (tmpx < (*PointCloudArr)[0].Point_cloud_Min.x) {
				(*PointCloudArr)[0].Point_cloud_Min.x = tmpx;
			}

			//if (tmpy >(*Point_cloud_Max)[0].y) {
			//	(*Point_cloud_Max)[0].y = tmpy;
			//}
			//if (tmpy < (*Point_cloud_Min)[0].y) {
			//	(*Point_cloud_Min)[0].y = tmpy;
			//}

			if (tmpy >(*PointCloudArr)[0].Point_cloud_Max.y) {
				(*PointCloudArr)[0].Point_cloud_Max.y = tmpy;
			}
			if (tmpy < (*PointCloudArr)[0].Point_cloud_Min.y) {
				(*PointCloudArr)[0].Point_cloud_Min.y = tmpy;
			}

			//if (tmpz >(*Point_cloud_Max)[0].z) {
			//	(*Point_cloud_Max)[0].z = tmpz;
			//}
			//if (tmpz < (*Point_cloud_Min)[0].z) {
			//	(*Point_cloud_Min)[0].z = tmpz;
			//}

			if (tmpz >(*PointCloudArr)[0].Point_cloud_Max.z) {
				(*PointCloudArr)[0].Point_cloud_Max.z = tmpz;
			}
			if (tmpz < (*PointCloudArr)[0].Point_cloud_Min.z) {
				(*PointCloudArr)[0].Point_cloud_Min.z = tmpz;
			}
		}
		//std::cout << "max:" << (*Point_cloud_Max)[0].x<<","<< (*Point_cloud_Max)[0].y<<","<< (*Point_cloud_Max)[0].z << std::endl;
		//std::cout << "min:" << (*Point_cloud_Min)[0].x << "," << (*Point_cloud_Min)[0].y << "," << (*Point_cloud_Min)[0].z << std::endl;

		//std::cout << "max:" << (*PointCloudArr)[0].Point_cloud_Max.x<<","<< (*PointCloudArr)[0].Point_cloud_Max.y<<","<< (*PointCloudArr)[0].Point_cloud_Max.z << std::endl;
		//std::cout << "min:" << (*PointCloudArr)[0].Point_cloud_Min.x << "," << (*PointCloudArr)[0].Point_cloud_Min.y << "," << (*PointCloudArr)[0].Point_cloud_Min.z << std::endl;

		//(*Point_cloud_center)[0] = ((*Point_cloud_Max)[0] + (*Point_cloud_Min)[0]) / 2;

		(*PointCloudArr)[0].Point_cloud_center = ((*PointCloudArr)[0].Point_cloud_Max + (*PointCloudArr)[0].Point_cloud_Min) / 2;
		//(*Point_cloud_center)[0].x = (float)((*Point_cloud_Max)[0].x + (*Point_cloud_Min)[0].x) / 2;
		//(*Point_cloud_center)[0].y = (float)((*Point_cloud_Max)[0].y + (*Point_cloud_Min)[0].y) / 2;
		//(*Point_cloud_center)[0].z = (float)((*Point_cloud_Max)[0].z + (*Point_cloud_Min)[0].z) / 2;

		//std::cout << "center:" << (*Point_cloud_center)[0].x << "," << (*Point_cloud_center)[0].y << "," << (*Point_cloud_center)[0].z << std::endl;
		//std::cout << "center:" << (*PointCloudArr)[0].Point_cloud_center.x << "," << (*PointCloudArr)[0].Point_cloud_center.y << "," << (*PointCloudArr)[0].Point_cloud_center.z << std::endl;
		//(*outside_center)[0] = (*Point_cloud_center)[0] + (*tmp_out);
		//std::cout << "outside:" << (*outside_center)[0].x << "," << (*outside_center)[0].y << "," << (*outside_center)[0].z << std::endl;

	}
	if (/*(*pointCloudSet).size()*/(*PointCloudArr).size() > 1) {
		for (int j = 0; j < /*(*pointCloudSet)[PointCloud_idx_show].size()*/(*PointCloudArr)[PointCloud_idx_show].mPC.size(); j++)
		{
			//tmpx = (*pointCloudSet)[PointCloud_idx_show][j].x();
			//tmpy = (*pointCloudSet)[PointCloud_idx_show][j].y();
			//tmpz = (*pointCloudSet)[PointCloud_idx_show][j].z();

			tmpx = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.x();
			tmpy = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.y();
			tmpz = (*PointCloudArr)[PointCloud_idx_show].mPC[j].Position.z();
			
			/*if (tmpx >(*Point_cloud_Max)[PointCloud_idx_show].x) {
				(*Point_cloud_Max)[PointCloud_idx_show].x = tmpx;
			}
			if (tmpx < (*Point_cloud_Min)[PointCloud_idx_show].x) {
				(*Point_cloud_Min)[PointCloud_idx_show].x = tmpx;
			}
			if (tmpy > (*Point_cloud_Max)[PointCloud_idx_show].y) {
				(*Point_cloud_Max)[PointCloud_idx_show].y = tmpy;
			}
			if (tmpy < (*Point_cloud_Min)[PointCloud_idx_show].y) {
				(*Point_cloud_Min)[PointCloud_idx_show].y = tmpy;
			}
			if (tmpz > (*Point_cloud_Max)[PointCloud_idx_show].z) {
				(*Point_cloud_Max)[PointCloud_idx_show].z = tmpz;
			}
			if (tmpz < (*Point_cloud_Min)[PointCloud_idx_show].z) {
				(*Point_cloud_Min)[PointCloud_idx_show].z = tmpz;
			}*/

			if (tmpx >(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.x) {
				(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.x = tmpx;
			}
			if (tmpx < (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.x) {
				(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.x = tmpx;
			}
			if (tmpy >(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.y) {
				(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.y = tmpy;
			}
			if (tmpy < (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.y) {
				(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.y = tmpy;
			}
			if (tmpz >(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.z) {
				(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.z = tmpz;
			}
			if (tmpz < (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.z) {
				(*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.z = tmpz;
			}
		}
		//std::cout << "idx now :" << PointCloud_idx_show << std::endl;
		//std::cout << "max:" << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.x << "," << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.y << "," << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max.z << std::endl;
		//std::cout << "min:" << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.x << "," << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.y << "," << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min.z << std::endl;
		
		//(*Point_cloud_center)[PointCloud_idx_show].x = (float)((*Point_cloud_Max)[PointCloud_idx_show].x + (*Point_cloud_Min)[PointCloud_idx_show].x) / 2;
		//(*Point_cloud_center)[PointCloud_idx_show].y = (float)((*Point_cloud_Max)[PointCloud_idx_show].y + (*Point_cloud_Min)[PointCloud_idx_show].y) / 2;
		//(*Point_cloud_center)[PointCloud_idx_show].z = (float)((*Point_cloud_Max)[PointCloud_idx_show].z + (*Point_cloud_Min)[PointCloud_idx_show].z) / 2;

		(*PointCloudArr)[PointCloud_idx_show].Point_cloud_center = ((*PointCloudArr)[PointCloud_idx_show].Point_cloud_Max + (*PointCloudArr)[PointCloud_idx_show].Point_cloud_Min) / 2;

		//std::cout << "center:" << (*Point_cloud_center)[PointCloud_idx_show].x << "," << (*Point_cloud_center)[PointCloud_idx_show].y << "," << (*Point_cloud_center)[PointCloud_idx_show].z << std::endl;
		
		//std::cout << "center:" << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_center.x << "," << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_center.y << "," << (*PointCloudArr)[PointCloud_idx_show].Point_cloud_center.z << std::endl;

		//(*outside_center)[PointCloud_idx_show] = (*Point_cloud_center)[PointCloud_idx_show] + (*tmp_out);
		//std::cout << "outside:" << (*outside_center)[PointCloud_idx_show].x << "," << (*outside_center)[PointCloud_idx_show].y << "," << (*outside_center)[PointCloud_idx_show].z << std::endl;
	}
	//delete tmp_max;
	//delete tmp_min;
	//delete tmp_out;
}
//void DentistDemo::MyForm::Find_Plane() {
//}
//void DentistDemo::MyForm::Find_over_maxmin() {
//}
//void DentistDemo::MyForm::Rotate2() {
//}
void DentistDemo::MyForm::Push_back_file() {
	file_name tmp1;
	strcpy(tmp1.name, "C_01.txt");
	
	//char tmp_name1[14] = "C_01.txt";

	filename_array->push_back(tmp1);
	strcpy(tmp1.name, "C_02.txt");
	filename_array->push_back(tmp1);
	strcpy(tmp1.name, "C_02.txt");
	filename_array->push_back(tmp1);
	strcpy(tmp1.name, "C_03.txt");
	filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C_01.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C5.txt");
	//filename_array->push_back(tmp1); 
	//strcpy(tmp1.name, "C_02.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C7.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C_02.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C13.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C_01.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C15.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C_03.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C37.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C7.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C57.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "C1.txt");
	//filename_array->push_back(tmp1);
	//strcpy(tmp1.name, "R1.txt");
	//filename_array->push_back(tmp1);
	//std::cout << "print name:" << filename_array[0][0] << std::endl;

}
void DentistDemo::MyForm::read_point_v2(int file_id) {
	
	//char file_name[14];
	//memcpy(file_name, &filename_array[file_id], 14);
	bool is_quat = false;
	bool nega_x = false, nega_y = false, nega_z = false, nega_w = false;

	double tmp_quat_x, tmp_quat_y, tmp_quat_z, tmp_quat_w;
	//std::cout << "print name:" << *file_name << std::endl;
	std::vector<int> tmp_int;
	std::vector<int> tmp_float_num;
	std::ifstream myfile;
	char *test_char;
	//std::vector<char> test_char;


	myfile.open((*filename_array)[file_id].name, std::ios::in);
	//myfile.open("C_02", std::ios::in);

	int length;
	//int length2;

	if (!myfile) {
		std::cout << "open file fail" << std::endl;
	}
	else {
		myfile.seekg(0, myfile.end);
		length = myfile.tellg();
		myfile.seekg(0, myfile.beg);

		//final_oct_char = new char[length];

		std::cout << " file length" << length << std::endl;

		test_char = new char[length];

		//test_char.resize(length);


		myfile.read(test_char, sizeof(char) * length);
		myfile.close();
	}
	char *end_str;
	char *token = strtok_s(test_char, "\n", &end_str);
	//char *tmp_char = strtok_s(test_char, "\n", &end_str);
	int point_count = 0;
	std::vector<GlobalRegistration::Point3D> tmpPC;
	PointCloudArray tmpPC_T;
	//tmp_char = strtok(test_char, "\n");
	while (token != NULL) {
		int dir_type = 0;
		char *end_token;
		//printf("a = %s\n", token);
		char *token2 = strtok_s(token, " ", &end_token);
		GlobalRegistration::Point3D tmp;
		PointData tmp_Data;

		while (token2 != NULL)
		{
			//printf("b = %s\n", token2);

			char *end_num;
			char *token3 = strtok_s(token2, ".", &end_num);
			//printf("c = %s\n", token3);

			if (!is_quat) {
				if (token3[0] == '-') {
					if (dir_type == 0) {
						nega_x = true;
					}
					else if (dir_type == 1) {
						nega_y = true;
					}
					else if (dir_type == 2) {
						nega_z = true;
					}
					else if (dir_type == 3) {
						nega_w = true;
					}
				}
			}
			double tmp_int = atof(token3);
			token3 = strtok_s(NULL, ".", &end_num);

			if (token3 != NULL) {
				double tmp_double = atof(token3);
				for (int i = 0; i < strlen(token3); i++) {
					tmp_double /= 10;
				}

				tmp_int += tmp_double;
				//std::cout << "double = " << tmp_int << std::endl;
				//std::cout << "point cloud = " << (*pointCloudSet)[PointCloud_idx_show][point_count].x() << std::endl;
				//(*pointCloudSet)[PointCloud_idx_show][point_count].x() = tmp_int;
				if (dir_type == 0) {
					//if (!is_quat) {
					//	if (nega_x)
					//		tmp_quat_x = tmp_int*(-1);
					//	else
					//		tmp_quat_x = tmp_int;
					//}
					//
					//else
						tmp_Data.Position.y() = tmp_int;
						tmp.y() = tmp_int;
				}
				else if (dir_type == 1) {
					//if (!is_quat)
					//{
					//	if (nega_y)
					//		tmp_quat_y = tmp_int*(-1);
					//	else
					//		tmp_quat_y = tmp_int;
					//}
					//else
						tmp_Data.Position.x() = tmp_int;
						tmp.x() = tmp_int;
				}
				else if (dir_type == 2) {
					//if (!is_quat)
					//{
					//	if (nega_z)
					//		tmp_quat_z = tmp_int*(-1);
					//	else
					//		tmp_quat_z = tmp_int;
					//}
					//else {
						tmp_Data.Position.z() = tmp_int;
						tmp.z() = tmp_int;
						//std::cout << "tmp = " << tmp.x() <<","<< tmp.y()<<","<< tmp.z()<< std::endl;
						if (tmp_Data.Position.y() > 0&& tmp_Data.Position.x()&& tmp_Data.Position.z()) {
							tmpPC.push_back(tmp);
							tmpPC_T.mPC.push_back(tmp_Data);
						}
					//}
				}
				//else if (dir_type == 3) {
				//	if (!is_quat) {
				//		//tmp_quat_w = tmp_int;
				//		//is_quat = true;
				//		//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
				//		if (nega_w) {
				//			tmp_quat_w = tmp_int*(-1);
				//			is_quat = true;
				//			//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
				//		}
				//		else {
				//			tmp_quat_w = tmp_int;
				//			is_quat = true;
				//			//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;
				//		}
				//	}
				//}
			}
			dir_type++;
			token2 = strtok_s(NULL, " ", &end_token);
		}

		//dir_type = 0;

		point_count++;
		token = strtok_s(NULL, "\n", &end_str);
	}
	//std::cout << "quat:(" << tmp_quat_x << "," << tmp_quat_y << "," << tmp_quat_z << "," << tmp_quat_w << ")" << std::endl;

	if (/*tmpPC.size() > 0*/tmpPC_T.mPC.size()>0) {
		//(*pointCloudSet).push_back(tmpPC);
		(*PointCloudArr).push_back(tmpPC_T);
		PointCloud_idx_show = (*PointCloudArr).size() - 1;
		std::cout << "(*pointCloudSet).size: " << (*PointCloudArr).size() << std::endl;
		///////////////////////////////
		//if (PointCloudArr->size() == 1) {
		//	CombinePC(0);
		//}
		Find_max_min();
	}
	delete test_char;
	//PointCloud_idx_show = (*pointCloudSet).size() - 1;
	std::cout << "PointCloud_idx_show: " << PointCloud_idx_show << std::endl;

}
void DentistDemo::MyForm::camera_cube(float width, float height, float length) {
	glBegin(GL_QUADS);

	glColor3f(1, 1, 1);
	glVertex3f(width/2,-height/2,-length/2);
	glColor3f(0, 1, 1);
	glVertex3f(-width / 2,-height / 2,-length / 2);
	glColor3f(1, 0, 1);
	glVertex3f(-width / 2, height / 2,-length / 2);
	glColor3f(1, 1, 0);
	glVertex3f(width / 2, height / 2,-length / 2);

	glColor3f(1, 1, 1);
	glVertex3f(-width / 2, height / 2, length / 2);
	glColor3f(0, 1, 1);
	glVertex3f(-width / 2, -height / 2, length / 2);
	glColor3f(1, 0, 1);
	glVertex3f(width / 2, -height / 2, length / 2);
	glColor3f(1, 1, 0);
	glVertex3f(width / 2, height / 2, length / 2);

	glColor3f(1, 1, 1);
	glVertex3f(width / 2, height / 2, length / 2);
	glColor3f(0, 1, 1);
	glVertex3f(width / 2, -height / 2, length / 2);
	glColor3f(1, 0, 1);
	glVertex3f(width / 2, -height / 2, -length / 2);
	glColor3f(1, 1, 0);
	glVertex3f(width / 2, height / 2, -length / 2);

	glColor3f(1, 1, 1);
	glVertex3f(-width / 2, height / 2, length / 2);
	glColor3f(0, 1, 1);
	glVertex3f(-width / 2, height / 2, -length / 2);
	glColor3f(1, 0, 1);
	glVertex3f(-width / 2, -height / 2, -length / 2);
	glColor3f(1, 1, 0);
	glVertex3f(-width / 2, -height / 2, length / 2);

	glColor3f(1, 1, 1);
	glVertex3f(-width / 2, height / 2, length / 2);
	glColor3f(0, 1, 1);
	glVertex3f(width / 2, height / 2, length / 2);
	glColor3f(1, 0, 1);
	glVertex3f(width / 2, height / 2, -length / 2);
	glColor3f(1, 1, 0);
	glVertex3f(-width / 2, height / 2, -length / 2);

	glColor3f(1, 1, 1);
	glVertex3f(-width / 2, -height / 2, length / 2);
	glColor3f(0, 1, 1);
	glVertex3f(-width / 2, -height / 2, -length / 2);
	glColor3f(1, 0, 1);
	glVertex3f(width / 2, -height / 2, -length / 2);
	glColor3f(1, 1, 0);
	glVertex3f(width / 2, -height / 2, length / 2);


	glEnd();
}
//void DentistDemo::MyForm::Find_min_quat() {
//}
void DentistDemo::MyForm::Find_min_quat2() {
	//int tmp_min_idx;
	//for (int i = 1; i < (*input_rotVec).size() - 1; i++) {
	//	Vector3 tmp_vector1 = (*input_rotVec)[i];
	//	Vector3 tmp_vector2 = (*input_rotVec)[(*input_rotVec).size() - 1];
	//
	//	Radian tmp_diff = tmp_vector1.angleBetween(tmp_vector2);
	//	std::cout << "Compare with idx:" << i << " ,the degree is:" << tmp_diff.valueDegrees() << "\n";
	//	double tmp_diff_degree = 999999;
	//	if (tmp_diff.valueDegrees() < tmp_diff_degree) {
	//		tmp_diff_degree = tmp_diff.valueDegrees();
	//		//std::cout << "now min idx:" << i << "\n";
	//		tmp_min_idx = i;
	//	}
	//}
	//
	//nearest_idx->push_back(tmp_min_idx);
	//std::cout << "min degree idx is:" << (*nearest_idx)[PointCloud_idx_show] << "\n";
}
void DentistDemo::MyForm::Find_min_quat3() {

	//int tmp_min_idx;
	//double tmp_diff_degree = 999999;
	//for (int i = 1; i < (*cloud_vec_arr).size() - 1; i++) {
	//	Vector3 tmp_vector1 = (*cloud_vec_arr)[i];
	//	Vector3 tmp_vector2 = (*cloud_vec_arr)[(*cloud_vec_arr).size() - 1];
	//
	//	Radian tmp_diff = tmp_vector1.angleBetween(tmp_vector2);
	//	std::cout << "Compare with idx:" << i << " ,the degree is:" << tmp_diff.valueDegrees() << "\n";
	//	
	//	if (tmp_diff.valueDegrees() < tmp_diff_degree) {
	//		tmp_diff_degree = tmp_diff.valueDegrees();
	//		std::cout << "now min idx:" << i << "\n";
	//		tmp_min_idx = i;
	//	}
	//}
	//
	//near_idx2->push_back(tmp_min_idx);
	//std::cout << "min degree idx is:" << (*near_idx2)[PointCloud_idx_show] << "\n";

	int tmp_min_idx;
	double tmp_diff_degree = 999999;
	for (int i = 1; i < (*PointCloudArr).size() - 1; i++) {
		Vector3 tmp_vector1 = (*PointCloudArr)[i].cloud_vec_arr;
		Vector3 tmp_vector2 = (*PointCloudArr)[PointCloud_idx_show].cloud_vec_arr;
	
		Radian tmp_diff = tmp_vector1.angleBetween(tmp_vector2);
		//std::cout << "Compare with idx:" << i << " ,the degree is:" << tmp_diff.valueDegrees() << "\n";
		
		if (tmp_diff.valueDegrees() < tmp_diff_degree) {
			tmp_diff_degree = tmp_diff.valueDegrees();
			//std::cout << "now min idx:" << i << "\n";
			tmp_min_idx = i;
		}
	}
	
	(*PointCloudArr)[PointCloud_idx_show].near_idx2 = tmp_min_idx;
	//near_idx2->push_back(tmp_min_idx);
	//std::cout << "min degree idx is:" << (*PointCloudArr)[PointCloud_idx_show].near_idx2 << "\n";
}
void DentistDemo::MyForm::pictureBox1_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
	Graphics^ g = e->Graphics;
	Pen^ RedPen = gcnew Pen(Color::Red, 1.5f);
	Pen^ BluePen = gcnew Pen(Color::Blue, 1.5f);


	int index = this->control_pic->Value;
	// Draw a line in the PictureBox.
	g->DrawLine(RedPen, mouse_loc_x + block_size_x, mouse_loc_y + block_size_y, mouse_loc_x - block_size_x, mouse_loc_y + block_size_y);
	g->DrawLine(RedPen, mouse_loc_x - block_size_x, mouse_loc_y + block_size_y, mouse_loc_x - block_size_x, mouse_loc_y - block_size_y);
	g->DrawLine(RedPen, mouse_loc_x - block_size_x, mouse_loc_y - block_size_y, mouse_loc_x + block_size_x, mouse_loc_y - block_size_y);
	g->DrawLine(RedPen, mouse_loc_x + block_size_x, mouse_loc_y - block_size_y, mouse_loc_x + block_size_x, mouse_loc_y + block_size_y);

	float tmp_origin_x = mouse_loc_x - block_size_x;
	float tmp_origin_y = mouse_loc_y - block_size_y;

	if ((*tmp_Mask_Model).empty() == 0) {
		for (int row = 0; row < (*tmp_Mask_Model).rows ; row++) {
			for (int col = 0; col < (*tmp_Mask_Model).cols; col++) {
				SolidBrush^ MeatBrush = gcnew SolidBrush(Color::FromArgb((*tmp_Mask_Model).at<cv::Vec4b>(row, col)[3], (*tmp_Mask_Model).at<cv::Vec4b>(row, col)[2], (*tmp_Mask_Model).at<cv::Vec4b>(row, col)[1], (*tmp_Mask_Model).at<cv::Vec4b>(row, col)[0]));
				g->FillRectangle((Brush^)MeatBrush, (float)(tmp_origin_x+col), (float)(tmp_origin_y+row), 1.0f, 1.0f);
			}
		}
	}

	g->DrawLine(BluePen, (mouse_loc_x - block_size_x) + block_size_x * 2 * ((float)index / 141), mouse_loc_y - block_size_y, (mouse_loc_x - block_size_x) + block_size_x * 2 * ((float)index / 141), mouse_loc_y + block_size_y);

}
void DentistDemo::MyForm::pictureBox1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	//Point mouseDownLocation = Point(e->X, e->Y);
	pic_mouse_move = true;
	//std::cout << "down\n";
}
void DentistDemo::MyForm::pictureBox1_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	if (pic_mouse_move == true) {
		mouse_loc_x = e->X;
		mouse_loc_y = e->Y;
		//std::cout << "mouse_loc_x:" << mouse_loc_x << ", mouse_loc_y:" << mouse_loc_y << "\n";
		//std::cout << "down & move\n";
		teeth_Model->Invalidate();
	}
}
void DentistDemo::MyForm::pictureBox1_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
	pic_mouse_move = false;
	//std::cout << "up\n";
}
void DentistDemo::MyForm::Choose_teeth_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
	ComboBox^ comboBox = (ComboBox^)(sender);

	int count = 0;
	int resultIndex = -1;

	// Call the FindStringExact method to find the first 
	// occurrence in the list.
	int index = this->control_pic->Value;

	if (Choose_teeth->SelectedIndex == 0) {
		std::cout << " " << Choose_teeth->SelectedIndex << "\n";
		if ((*teeth_1).size() != 0) {
			std::cout << "(*teeth_1).size() != 0";
			show_Teeth_PC((*teeth_1)[0]);
			(*tmp_display_PC) = (*teeth_1)[0];
			(*tmp_Mask_Model) = (*teeth_1)[0].Model_mask;

			glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
				(*tmp_display_PC).disease_pic_index[index],
				(*tmp_display_PC).meat_pic_index[index],
				(*tmp_display_PC).teeth_imgPC.size(),
				(*tmp_display_PC).disease_imgPC.size(), meat_alpha);
		}
		else {
			//std::cout << "(*teeth_1).size() == 0";
			clear_PC_buffer();
			glManager->Clear_all();
		}
		teeth_Model->Image = Image::FromFile("./image1.jpg");
		teeth_Model->Invalidate();
		tooth_table->Image = Image::FromFile("./teeth_table1.jpg");
		tooth_table->Invalidate();
	}
	else if (Choose_teeth->SelectedIndex == 1) {
		std::cout << " " << Choose_teeth->SelectedIndex << "\n";
		if ((*teeth_2).size() != 0) {
			show_Teeth_PC((*teeth_2)[0]);
			(*tmp_display_PC) = (*teeth_2)[0];
			(*tmp_Mask_Model) = (*teeth_2)[0].Model_mask;

			glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
				(*tmp_display_PC).disease_pic_index[index],
				(*tmp_display_PC).meat_pic_index[index],
				(*tmp_display_PC).teeth_imgPC.size(),
				(*tmp_display_PC).disease_imgPC.size(), meat_alpha);
		}
		else {
			clear_PC_buffer();
			glManager->Clear_all();
		}
		teeth_Model->Image = Image::FromFile("./image2.jpg");
		teeth_Model->Invalidate();
		tooth_table->Image = Image::FromFile("./teeth_table2.jpg");
		tooth_table->Invalidate();
	}
	else if (Choose_teeth->SelectedIndex == 2) {
		std::cout << " " << Choose_teeth->SelectedIndex << "\n";
		if ((*teeth_3).size() != 0) {
			show_Teeth_PC((*teeth_3)[0]);
			(*tmp_display_PC) = (*teeth_3)[0];
			(*tmp_Mask_Model) = (*teeth_3)[0].Model_mask;

			glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
				(*tmp_display_PC).disease_pic_index[index],
				(*tmp_display_PC).meat_pic_index[index],
				(*tmp_display_PC).teeth_imgPC.size(),
				(*tmp_display_PC).disease_imgPC.size(), meat_alpha);
		}
		else {
			clear_PC_buffer();
			glManager->Clear_all();
		}
		teeth_Model->Image = Image::FromFile("./image3.jpg");
		teeth_Model->Invalidate();
		tooth_table->Image = Image::FromFile("./teeth_table3.jpg");
		tooth_table->Invalidate();
	}
	else if (Choose_teeth->SelectedIndex == 3) {
		std::cout << " " << Choose_teeth->SelectedIndex << "\n";
		if ((*teeth_4).size() != 0) {
			show_Teeth_PC((*teeth_4)[0]);
			(*tmp_display_PC) = (*teeth_4)[0];
			(*tmp_Mask_Model) = (*teeth_4)[0].Model_mask;

			glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
				(*tmp_display_PC).disease_pic_index[index],
				(*tmp_display_PC).meat_pic_index[index],
				(*tmp_display_PC).teeth_imgPC.size(),
				(*tmp_display_PC).disease_imgPC.size(), meat_alpha);
		}
		else {
			clear_PC_buffer();
			glManager->Clear_all();
		}
		teeth_Model->Image = Image::FromFile("./image4.jpg");
		teeth_Model->Invalidate();
		tooth_table->Image = Image::FromFile("./teeth_table4.jpg");
		tooth_table->Invalidate();
	}
	else if (Choose_teeth->SelectedIndex == 4) {
		std::cout << " " << Choose_teeth->SelectedIndex << "\n";
		if ((*teeth_5).size() != 0) {
			show_Teeth_PC((*teeth_5)[0]);
			(*tmp_display_PC) = (*teeth_5)[0];
			(*tmp_Mask_Model) = (*teeth_5)[0].Model_mask;

			glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
				(*tmp_display_PC).disease_pic_index[index],
				(*tmp_display_PC).meat_pic_index[index],
				(*tmp_display_PC).teeth_imgPC.size(),
				(*tmp_display_PC).disease_imgPC.size(), meat_alpha);
		}
		else {
			clear_PC_buffer();
			glManager->Clear_all();
		}
		teeth_Model->Image = Image::FromFile("./image5.jpg");
		teeth_Model->Invalidate();
		tooth_table->Image = Image::FromFile("./teeth_table5.jpg");
		tooth_table->Invalidate();
	}
	// Remove the name as it is found, and increment the found count. 
	// Then call the FindStringExact method again, passing in the 
	// index of the current found item so the search starts there 
	// instead of at the beginning of the list.
	//while (resultIndex != -1)
	//{
	//	Choose_teeth->Items->RemoveAt(resultIndex);
	//	count += 1;
	//	resultIndex = Choose_teeth->FindStringExact(selectedEmployee, resultIndex);
	//}
}
void DentistDemo::MyForm::reset_rot_Click(System::Object^  sender, System::EventArgs^  e) {
	is_reset = true;
}
void DentistDemo::MyForm::timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
	timer_rot = true;

	//std::cout << "timer:" << timer_rot << "\n";
	hkoglPanelControl1->Invalidate();
}
void DentistDemo::MyForm::Test_Model_Click(System::Object^  sender, System::EventArgs^  e)
{
	Mat img;
	if (indexA == 0)
		img = imread("./TestImage/95.png");
	else if (indexA == 1)
		img = imread("./TestImage/76_reverse.png");
	indexA = (indexA + 1) % 2;

	img = NetworkModel->Predict(img);
	img = NetworkModel->Visualization(img);

	cv::imshow("Display window", img);
	cv::waitKey(0);
}
void DentistDemo::MyForm::Test_detect_Click(System::Object^  sender, System::EventArgs^  e)
{
	/////////////////////////////////////// SCAN
	//progressBar->Value = 0;
	//color_img();
	//
	//for (int i = 0; i < (*RGB_image).size(); i++) {
	//	cv::Mat tmp_image = NetworkModel->Predict((*RGB_image)[i]);
	//	
	//	(*Result_Image).push_back(tmp_image);
	//	progressBar->Value++;
	//	std::cout << "num = " << i << " is OK\n";
	//}
	/////////////////////////////////////// SCAN

	/////////////////////////////////////// TEST
	progressBar->Value = 0;

	//if ((*teeth_imgPC).size() != 0);
	//	(*teeth_imgPC).clear();
	//if ((*disease_imgPC).size() != 0);
	//	(*disease_imgPC).clear();
	//if ((*meat_imgPC).size() != 0);
	//	(*meat_imgPC).clear();
	//if ((*raw_input).size() != 0)
	//	(*raw_input).clear();
	//if ((*Result_Image).size() != 0)
	//	(*Result_Image).clear();
	//
	//if ((*tmp_pic_teeth).size() != 0);
	//	(*tmp_pic_teeth).clear();
	//if ((*tmp_pic_meat).size() != 0);
	//	(*tmp_pic_meat).clear();
	//if ((*tmp_pic_disease).size() != 0);
	//	(*tmp_pic_disease).clear();

	#ifdef TEST_NO_OCT
	clear_PC_buffer();
	load_rawImg();
	#endif // TEST_NO_OCT


	Teeth_PC tmp_teeth_PC;
	tmp_teeth_PC.raw_input = (*raw_input);
	//std::cout << "load OK\n";
	for (int i = 0; i < (*raw_input).size(); i++) {// raw_input->RGB_image
		cv::Mat tmp_image = NetworkModel->Predict((*raw_input)[i]);//  raw_input->RGB_image
		(*Result_Image).push_back(tmp_image);
		progressBar->Value++;
		std::cout << "num = " << i << " is OK\n";
	}

	tmp_teeth_PC.result_input = (*Result_Image);
	classify_result();

	this->control_pic->Enabled = true;
	int index = this->control_pic->Value;

	tmp_teeth_PC.Model_mask = (*tmp_Mask_Model);

	tmp_teeth_PC.teeth_imgPC = (*teeth_imgPC);
	tmp_teeth_PC.disease_imgPC = (*disease_imgPC);
	tmp_teeth_PC.meat_imgPC = (*meat_imgPC);

	tmp_teeth_PC.teeth_pic_index = (*tmp_pic_teeth);
	tmp_teeth_PC.meat_pic_index = (*tmp_pic_meat);
	tmp_teeth_PC.disease_pic_index = (*tmp_pic_disease);

	if (Choose_teeth->SelectedIndex == 0) {
		(*teeth_1).push_back(tmp_teeth_PC);
		glManager->Clear_all();

		show_Teeth_PC((*teeth_1)[(*teeth_1).size() - 1]);
		(*tmp_display_PC) = (*teeth_1)[(*teeth_1).size() - 1];

		glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
			(*tmp_display_PC).disease_pic_index[index],
			(*tmp_display_PC).meat_pic_index[index],
			(*tmp_display_PC).teeth_imgPC.size(),
			(*tmp_display_PC).disease_imgPC.size(), meat_alpha);

		glManager->Initial_vert();

	}
	else if (Choose_teeth->SelectedIndex == 1) {
		(*teeth_2).push_back(tmp_teeth_PC);

		glManager->Clear_all();
		show_Teeth_PC((*teeth_2)[(*teeth_2).size() - 1]);
		(*tmp_display_PC) = (*teeth_2)[(*teeth_2).size() - 1];

		glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
			(*tmp_display_PC).disease_pic_index[index],
			(*tmp_display_PC).meat_pic_index[index],
			(*tmp_display_PC).teeth_imgPC.size(),
			(*tmp_display_PC).disease_imgPC.size(), meat_alpha);

		glManager->Initial_vert();
	}
	else if (Choose_teeth->SelectedIndex == 2) {
		(*teeth_3).push_back(tmp_teeth_PC);

		glManager->Clear_all();

		show_Teeth_PC((*teeth_3)[(*teeth_3).size() - 1]);
		(*tmp_display_PC) = (*teeth_3)[(*teeth_3).size() - 1];

		glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
			(*tmp_display_PC).disease_pic_index[index],
			(*tmp_display_PC).meat_pic_index[index],
			(*tmp_display_PC).teeth_imgPC.size(),
			(*tmp_display_PC).disease_imgPC.size(), meat_alpha);

		glManager->Initial_vert();
	}
	else if (Choose_teeth->SelectedIndex == 3) {
		(*teeth_4).push_back(tmp_teeth_PC);

		glManager->Clear_all();

		show_Teeth_PC((*teeth_4)[(*teeth_4).size() - 1]);
		(*tmp_display_PC) = (*teeth_4)[(*teeth_4).size() - 1];

		glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
			(*tmp_display_PC).disease_pic_index[index],
			(*tmp_display_PC).meat_pic_index[index],
			(*tmp_display_PC).teeth_imgPC.size(),
			(*tmp_display_PC).disease_imgPC.size(), meat_alpha);

		glManager->Initial_vert();
	}
	else if (Choose_teeth->SelectedIndex == 4) {
		(*teeth_5).push_back(tmp_teeth_PC);

		glManager->Clear_all();

		show_Teeth_PC((*teeth_5)[(*teeth_5).size() - 1]);
		(*tmp_display_PC) = (*teeth_5)[(*teeth_5).size() - 1];

		glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
			(*tmp_display_PC).disease_pic_index[index],
			(*tmp_display_PC).meat_pic_index[index],
			(*tmp_display_PC).teeth_imgPC.size(),
			(*tmp_display_PC).disease_imgPC.size(), meat_alpha);

		glManager->Initial_vert();
	}


	
	ShowImage(index);
	teeth_Model->Invalidate();
	/////////////////////////////////////// TEST
}


//////////////////////////////////////////////////////////////////////////
// Dark Part
//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
// Scroll Image
//////////////////////////////////////////////////////////////////////////
void DentistDemo::MyForm::ScrollChangeEvent(System::Object^ sender, System::EventArgs^ e)
{
	int index = this->control_pic->Value;
	cout << index << endl;
	ShowImage(index);
	teeth_Model->Invalidate();
	glManager->control_pic((*tmp_display_PC).teeth_pic_index[index],
		(*tmp_display_PC).disease_pic_index[index],
		(*tmp_display_PC).meat_pic_index[index],
		(*tmp_display_PC).teeth_imgPC.size(),
		(*tmp_display_PC).disease_imgPC.size(), meat_alpha);
	//slice_quad(index);
}
void  DentistDemo::MyForm::ShowImage(int index)
{
	 Mat InputMat = (*raw_input)[index];
	 cout << "InputMat.row:" << InputMat.rows << "\n";
	 cout << "InputMat.col:" << InputMat.cols << "\n";
	 Mat ResultMat = NetworkModel->Visualization((*Result_Image)[index]);

	 // 轉型態
	 cvtColor(InputMat.clone(), InputMat, CV_BGR2BGRA);
	 cvtColor(ResultMat.clone(), ResultMat, CV_BGR2BGRA);

	 HBITMAP hBit = CreateBitmap(InputMat.cols, InputMat.rows, 1, 32, InputMat.data);
	 Bitmap^ bitMap = Bitmap::FromHbitmap((IntPtr)hBit);
	 this->OCT_Img->Image = bitMap;

	 hBit = CreateBitmap(ResultMat.cols, ResultMat.rows, 1, 32, ResultMat.data);
	 bitMap = Bitmap::FromHbitmap((IntPtr)hBit);
	 this->Result_Img->Image = bitMap;
}
void DentistDemo::MyForm::TestGetData_Click(System::Object^ sender, System::EventArgs^ e)
{
	cout << "Testss" << endl;
	pin_ptr<int32_t> tmp_deviceID = &deviceID;
	InitADC(4, tmp_deviceID);
	clock_t scan_t1 = clock();

	//AboutADC(deviceID);
	/////////////////////////////////////////////////////////H_StartCap///////////////////////////////////////////////
	float LV_65 = 65;
	char SaveName[] = "V:\\OCT20170928";
	unsigned int SampRec = 2048;

	char SaveName2[1024];
	std::string name_title = "V:\\";

	(*out_fileName) = name_title + (*out_fileName);

	strcpy(SaveName2, out_fileName->c_str());

#ifndef TEST_NO_OCT
	SerialPort port("COM6", 9600);
	port.Open();

	if (!port.IsOpen) {
		std::cout << "COM6 fail to open!" << std::endl;
	}
#endif // !TEST_NO_OCT
	Sleep(100);

	Savedata = true;
	ErrorBoolean = false;
	ByteLen = 1;


	pin_ptr<int32_t> tmp_ErrorString_len_out = &ErrorString_len_out;
	pin_ptr<uint32_t> tmp_Handle = &HandleOut;
	pin_ptr<uint32_t> tmp_ByteLen = &ByteLen;
	pin_ptr<LVBoolean> tmp_ErrorBoolean = &ErrorBoolean;


#ifndef TEST_NO_OCT
	StartCap(deviceID, tmp_Handle, LV_65, SampRec, tmp_ByteLen, Savedata, SaveName, tmp_ErrorBoolean, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
	port.RtsEnable = true;
#else
	test_start_cap(deviceID, tmp_Handle, LV_65, SampRec, tmp_ByteLen, Savedata, SaveName2, tmp_ErrorBoolean, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
#endif // !TEST_NO_OCT



	/////////////////////////////////////////////////////////H_StartCap///////////////////////////////////////////////


	/////////////////////////////////////////////////////////H_ScanADC///////////////////////////////////////////////

	AllDatabyte = ByteLen * 2;
	ArrSize = new unsigned short[PIC_SIZE];
	outarr = new unsigned short[PIC_SIZE];
	final_oct_arr = new unsigned short[PIC_SIZE * 125];
	final_oct_char = new char[PIC_SIZE * 250];

	OutArrLenIn = 2048 * 500 * 2;
	OutArrLenOut = 0;
	//button_stop_enable = true;
	pin_ptr<int32_t> tmp_OutArrLenOut = &OutArrLenOut;
	int pic_count = 0;
	clock_t t3, t4, t5;
	bool change_point_cloud = false;

	//Sleep(100);

	t3 = clock();

	unsigned short *interator;
	interator = new unsigned short;
	interator = final_oct_arr;
	//final_oct_arr = final_oct_arr + PIC_SIZE * 62;


	while (pic_count < 125) {
#ifdef TEST_NO_OCT
		test_scanADC(HandleOut, AllDatabyte, ArrSize, ByteLen, outarr, OutArrLenIn, tmp_OutArrLenOut, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
#else
		ScanADC(HandleOut, AllDatabyte, ArrSize, ByteLen, outarr, OutArrLenIn, tmp_OutArrLenOut, ErrorString, ErrorString_len_in, tmp_ErrorString_len_out);
#endif // TEST_NO_OCT

		memcpy(final_oct_arr, outarr, sizeof(unsigned short) * PIC_SIZE);

		pic_count++;

		final_oct_arr = final_oct_arr + PIC_SIZE;

	}
#ifndef TEST_NO_OCT
	AboutADC(deviceID);
	port.RtsEnable = false;
	port.Close();
	//std::cout << "tmp_char_finish " << std::endl;
#endif // !TEST_NO_OCT

	t4 = clock();

	unsigned_short_to_char(interator, PIC_SIZE * 125, final_oct_char);
	t5 = clock();

	std::vector<char> tmp_char(final_oct_char, final_oct_char + PIC_SIZE * 250);

	std::cout << "to array time:" << (t4 - t3) / (double)(CLOCKS_PER_SEC) << "s" << std::endl;
	std::cout << "short to char time:" << (t5 - t4) / (double)(CLOCKS_PER_SEC) << "s" << std::endl;

	theTRcuda->RawToPointCloud(tmp_char.data(), tmp_char.size(), 250, 2048);
	scan_count++;

	int PCidx, Mapidx;
	float ratio = 1;
	float zRatio = DManager->zRatio;
	PointCloudArray tmpPC_T;

	for (int x = 2; x < theTRcuda->VolumeSize_X - 2; x++)
	{
		for (int y = 0; y < theTRcuda->VolumeSize_Y; y++)
		{
			if (x > DManager->Mapping_X || y > DManager->Mapping_Y)
				continue;

			Mapidx = ((y * theTRcuda->sample_Y * theTRcuda->VolumeSize_X) + x) * theTRcuda->sample_X;
			PCidx = ((x * theTRcuda->VolumeSize_Y) + y) * theTRcuda->VolumeSize_Z;
			if (theTRcuda->PointType[PCidx + 3] == 1 && theTRcuda->PointType[PCidx + 1] != 0)
			{
				GlobalRegistration::Point3D tmp;
				PointData tmp_Data;

				tmp_Data.Position.x() = DManager->MappingMatrix[Mapidx * 2 + 1] * ratio + 0.2;
				tmp_Data.Position.y() = DManager->MappingMatrix[Mapidx * 2] * ratio;
				tmp_Data.Position.z() = theTRcuda->PointType[PCidx + 1] * zRatio / theTRcuda->VolumeSize_Z * ratio;

				tmpPC_T.mPC.push_back(tmp_Data);
				if (x == 124) {
					point_z_124 = point_z_124 + tmp_Data.Position.z();
				}
				if (x == 125) {
					point_z_125 = point_z_125 + tmp_Data.Position.z();
				}
			}
		}
	}

	if (tmpPC_T.mPC.size() > 10000) {
		(*PointCloudArr).push_back(tmpPC_T);
		//(*pointCloudSet).push_back(tmpPC);
		PointCloud_idx_show = (*PointCloudArr).size() - 1;

		(*PointCloudArr)[PointCloud_idx_show].mPC_noGyro = tmpPC_T.mPC;
		std::cout << "tmpPC_T.mPC.size(): " << tmpPC_T.mPC.size() << std::endl;
		//std::cout << "(*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size(): " << (*PointCloudArr)[PointCloud_idx_show].mPC_noGyro.size() << std::endl;
		///////////////////////////////
		//if (pointCloudSet->size() == 1) {
		//	CombinePC(0);
		//}

		std::cout << "point_z_124 Total: " << point_z_124 << std::endl;
		std::cout << "point_z_125 Total: " << point_z_125 << std::endl;

		Find_max_min();
		draw_before_mapping();
		//numericUpDown4->Maximum = (*PointCloudArr).size() - 1;
		std::cout << "now point cloud size:" << (*PointCloudArr).size() << std::endl;
		std::cout << "Scan count:" << scan_count << std::endl;
	}

	//std::cout << "PointCloud_idx_show: " << PointCloud_idx_show << std::endl;

	is_camera_move = true;

	if ((*PointCloudArr).size() == 1) {
		//*quat1 = sysManager->GetGloveDataLPtr()->GetQuaternion();
		//*quat1 = quat1->Inverse();
		//std::cout << "quat1" << (*quat1) << "\n";


		//*acc_vector1 = sysManager->GetGloveDataLPtr()->GetAcceleration();
		first_t = clock();

		//Radian *tmp_diff;
		//Vector3 *tmp_rotation;
		//std::vector<GlobalRegistration::Point3D> *tmp_PC;
		//tmp_PC = new std::vector<GlobalRegistration::Point3D>;
		int tmp_nearest = 0;
		//tmp_diff = new Radian;
		//tmp_rotation = new Vector3;
		//(*pointCloud_radian).push_back(*tmp_diff);
		//(*pointCloud_quat_vector).push_back(*tmp_rotation);
		//(*nearest_idx).push_back(tmp_nearest);
		//(*nearest_idx).push_back(tmp_nearest);
		//(*pointCloud_aligned).push_back(*tmp_PC);

		//(*cloud_vec_arr).push_back(*tmp_rotation);
		//(*near_idx2).push_back(tmp_nearest);
		//(*near_idx2).push_back(tmp_nearest);
	}
	else if ((*PointCloudArr).size() == 2) {
		//Rotate2();
		Rotate_cloud();
		//rotate_quat2();
		//RotatePC();
		//Find_Plane();
		//MovePC();
		std::cout << "now idx:" << PointCloud_idx_show << "     now size is:" << (*PointCloudArr).size() << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr;
		//std::cout << "now idx:" << PointCloud_idx_show << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr << std::endl;
	}
	else if ((*PointCloudArr).size()>2) {
		Rotate_cloud();
		Find_min_quat3();
		std::cout << "now idx:" << PointCloud_idx_show << "     now size is:" << (*PointCloudArr).size() << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr;
		//std::cout << "now idx:" << PointCloud_idx_show << "   mPC size is:" << (*PointCloudArr)[PointCloud_idx_show].mPC.size() << std::endl;
		//std::cout << "now idx:" << PointCloud_idx_show << "   degree is:" << (*PointCloudArr)[PointCloud_idx_show].cloud_degree_arr << std::endl;
	}
	//if ((*PointCloudArr).size() > 1) {
	//	Point_cloud_idx_sec = (*pointCloudSet).size() - 2;
	//}
	clear_PC_buffer();
	color_img();

	delete[] interator;
	//delete[] final_oct_arr;
	delete[] final_oct_char;
	delete[] ArrSize;
	delete[] outarr;

	clock_t scan_t2 = clock();
	full_scan_time = (scan_t2 - scan_t1) / (double)(CLOCKS_PER_SEC);



	all_time = all_time + full_scan_time;

	std::cout << "full_scan_time:" << full_scan_time << "s" << std::endl;
	std::cout << "all_time:" << all_time << "s" << std::endl;
}