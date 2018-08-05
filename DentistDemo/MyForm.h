#pragma once

#include "TRcuda.cuh"
#include "DotNetUtilities.h"
#include <Windows.h>
#include "DevUtils.h"
#include "GLManager.h"
#include "SystemManager.h"
#include "OCT64C.h"

#include "super4pcs/algorithms/4pcs.h"
#include "super4pcs/shared4pcs.h"

#include "Eigen/Dense"
//#include "opencv/opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "DataManager.h"
#include <iostream>
#include <fstream>
#include "super4pcs/utils/geometry.h"
#include "super4pcs/algorithms/super4pcs.h"
#include <bitset>
#include <string.h>
//#include <msclr/marshal.h>
//#define PIC_SIZE PIC_SIZE

#include "super4pcs/io/io.h"
//#include "stdafx.h"

struct PointData
{
	GlobalRegistration::Point3D Position;
};
struct myPointCloud
{
	std::vector<GlobalRegistration::Point3D> mPC;
};
struct PointCloudArray{
	std::vector<PointData> mPC;
	Vector3 Point_cloud_center;
	Vector3 outside_center;
	Vector3 Point_cloud_Max, noGyro_PC_Max = Vector3(0, 0, 0);
	Vector3 Point_cloud_Min, noGyro_PC_Min = Vector3(999999, 999999, 999999);
	int near_idx2;
	Vector3 cloud_vec_arr;
	double cloud_degree_arr;
	Vector3 cube_vec_arr;
	double cube_degree_arr;
	std::vector<PointData> origin_volumePC;
	std::vector<PointData> mPC_noGyro;
	std::vector<PointData> choose_PC;
};
struct file_name {
	char name[20];

};

struct TransformVisitor {
	inline void operator() (
		float fraction,
		float best_LCP,
		Eigen::Ref<GlobalRegistration::Match4PCSBase::MatrixType> /*transformation*/) {
		printf("done: %d%c best: %f                  \r",
			static_cast<int>(fraction * 100), '%', best_LCP);
		fflush(stdout);
	}
	constexpr bool needsGlobalTransformation() const { return false; }
};

namespace DentistDemo {

	using namespace System;
	
	//using namespace System::Configuration;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Collections::Generic;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO::Ports;
	using namespace dllBLE;
	using namespace GlobalRegistration;
	using namespace System::IO;
	#define PIC_SIZE 2048000
	/// <summary>
	/// MyForm 的摘要
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//finalPC = new std::vector<GlobalRegistration::Point3D>;

			//
			//TODO:  在此加入建構函式程式碼
			//
			
		}

	protected:
		/// <summary>
		/// 清除任何使用中的資源。
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::ComboBox^  cbComPortL;
	private: System::Windows::Forms::Button^  btnComUpdate;
	protected:

	protected:

	private: System::Windows::Forms::ComboBox^  cbBleDeviceL;
	private: System::Windows::Forms::Button^  btnBleScan;
	private: System::Windows::Forms::Button^  btnComOpen;






	private: System::ComponentModel::BackgroundWorker^  backgroundWorker1;
	private: System::Windows::Forms::Button^  btnBleEstablish;
	private: System::Windows::Forms::Button^  Get_Quat;
	private: System::Windows::Forms::Button^  Initial_scan;
	private: System::Windows::Forms::Button^  Single_scan;
	private: System::Windows::Forms::Button^  Full_scan;
	private: HKOGLPanel::HKOGLPanelControl^  hkoglPanelControl1;
	private: System::Windows::Forms::Button^  Test_file;
	private: System::Windows::Forms::Button^  Clear_cloud;

	private: System::Windows::Forms::Button^  Alignment;
	private: System::Windows::Forms::Button^  Save_PointCloud;
	private: System::Windows::Forms::SaveFileDialog^  saveFileDialog1;
	private: System::Windows::Forms::Button^  Save_point_s;
	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Button^  read_point;
	private: System::Windows::Forms::Button^  Combine;
	private: System::Windows::Forms::Button^  Part_Align;
	private: System::Windows::Forms::Label^  Threshold;
	private: System::Windows::Forms::TextBox^  Theshold_in;
	private: System::Windows::Forms::Label^  Max_time;
	private: System::Windows::Forms::TextBox^  Max_time_in;
	private: System::Windows::Forms::Label^  Max_angle;
	private: System::Windows::Forms::TextBox^  Max_angle_in;
	private: System::Windows::Forms::Label^  Sample_points;
	private: System::Windows::Forms::TextBox^  Sample_points_in;
	private: System::Windows::Forms::Label^  Overlap;
	private: System::Windows::Forms::TextBox^  Overlap_in;
	private: System::Windows::Forms::Label^  Delta;
	private: System::Windows::Forms::TextBox^  Delta_in;
	private: System::Windows::Forms::Label^  Transform;
	private: System::Windows::Forms::TextBox^  Transform_in;
	private: System::Windows::Forms::Label^  Constant1;
	private: System::Windows::Forms::TextBox^  Constant1_in;
	private: System::Windows::Forms::Label^  Conatant2;
	private: System::Windows::Forms::TextBox^  Constant2_in;
	private: System::Windows::Forms::Button^  Gyro_test;
	private: System::Windows::Forms::Button^  Chose_Align;
	private: System::Windows::Forms::TrackBar^  overlap_constant1;
	private: System::Windows::Forms::TrackBar^  overlap_constant2;
	private: System::Windows::Forms::Button^  Read_point_v2;
	private: System::Windows::Forms::Button^  Rot_test;
	private: System::Windows::Forms::Button^  Reset_quat;
	private: System::Windows::Forms::Button^  Aligned_rot;
	private: System::Windows::Forms::Button^  Save_Points2;
	private: System::Windows::Forms::Button^  First_cloud;
	private: System::Windows::Forms::Button^  Second_cloud;
	private: System::Windows::Forms::Button^  Aligned12;
	private: System::Windows::Forms::Button^  next_cloud;
	private: System::Windows::Forms::Button^  Aligned_near;
	private: System::Windows::Forms::Button^  reset_rot;
	private: System::Windows::Forms::Timer^  timer1;
	private: System::Windows::Forms::Button^  Aligned_Pre;
	private: System::Windows::Forms::Label^  Iter_treshold;
	private: System::Windows::Forms::TextBox^  Iter_thresh;
	private: System::Windows::Forms::Label^  Iter_count;
	private: System::Windows::Forms::TextBox^  Iter_count_in;
	private: System::Windows::Forms::Button^  Combine_cloud;
private: System::Windows::Forms::Button^  Save_combine;
private: System::Windows::Forms::Label^  output_name;
private: System::Windows::Forms::TextBox^  name_output;
private: System::Windows::Forms::Button^  Add2Combine;
private: System::Windows::Forms::PictureBox^  pictureBox1;
private: System::Windows::Forms::PictureBox^  pictureBox2;
private: System::Windows::Forms::PictureBox^  pictureBox3;
private: System::Windows::Forms::PictureBox^  pictureBox4;
private: System::Windows::Forms::PictureBox^  pictureBox5;
private: System::Windows::Forms::Label^  label4;
private: System::Windows::Forms::Label^  label5;
private: System::Windows::Forms::Label^  label6;
private: System::Windows::Forms::Label^  label7;
private: System::Windows::Forms::Label^  label8;
private: System::Windows::Forms::NumericUpDown^  numericUpDown1;
private: System::Windows::Forms::NumericUpDown^  numericUpDown2;
private: System::Windows::Forms::NumericUpDown^  numericUpDown3;
private: System::Windows::Forms::Label^  label9;
private: System::Windows::Forms::Label^  label10;
private: System::Windows::Forms::Label^  label11;
private: System::Windows::Forms::Button^  Apply_rot;
private: System::Windows::Forms::PictureBox^  pictureBox6;
private: System::Windows::Forms::Label^  label12;
private: System::Windows::Forms::NumericUpDown^  numericUpDown4;
private: System::Windows::Forms::Label^  label13;
private: System::Windows::Forms::Button^  Aligned_target;



	private: System::ComponentModel::IContainer^  components;













	private:
		/// <summary>
		/// 設計工具所需的變數。
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改
		/// 這個方法的內容。
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			HKOGLPanel::HKCOGLPanelCameraSetting^  hkcoglPanelCameraSetting1 = (gcnew HKOGLPanel::HKCOGLPanelCameraSetting());
			HKOGLPanel::HKCOGLPanelPixelFormat^  hkcoglPanelPixelFormat1 = (gcnew HKOGLPanel::HKCOGLPanelPixelFormat());
			this->cbComPortL = (gcnew System::Windows::Forms::ComboBox());
			this->btnComUpdate = (gcnew System::Windows::Forms::Button());
			this->cbBleDeviceL = (gcnew System::Windows::Forms::ComboBox());
			this->btnBleScan = (gcnew System::Windows::Forms::Button());
			this->btnComOpen = (gcnew System::Windows::Forms::Button());
			this->backgroundWorker1 = (gcnew System::ComponentModel::BackgroundWorker());
			this->btnBleEstablish = (gcnew System::Windows::Forms::Button());
			this->Get_Quat = (gcnew System::Windows::Forms::Button());
			this->Initial_scan = (gcnew System::Windows::Forms::Button());
			this->Single_scan = (gcnew System::Windows::Forms::Button());
			this->Full_scan = (gcnew System::Windows::Forms::Button());
			this->hkoglPanelControl1 = (gcnew HKOGLPanel::HKOGLPanelControl());
			this->Test_file = (gcnew System::Windows::Forms::Button());
			this->Clear_cloud = (gcnew System::Windows::Forms::Button());
			this->Alignment = (gcnew System::Windows::Forms::Button());
			this->Save_PointCloud = (gcnew System::Windows::Forms::Button());
			this->saveFileDialog1 = (gcnew System::Windows::Forms::SaveFileDialog());
			this->Save_point_s = (gcnew System::Windows::Forms::Button());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->read_point = (gcnew System::Windows::Forms::Button());
			this->Combine = (gcnew System::Windows::Forms::Button());
			this->Part_Align = (gcnew System::Windows::Forms::Button());
			this->Threshold = (gcnew System::Windows::Forms::Label());
			this->Theshold_in = (gcnew System::Windows::Forms::TextBox());
			this->Max_time = (gcnew System::Windows::Forms::Label());
			this->Max_time_in = (gcnew System::Windows::Forms::TextBox());
			this->Max_angle = (gcnew System::Windows::Forms::Label());
			this->Max_angle_in = (gcnew System::Windows::Forms::TextBox());
			this->Sample_points = (gcnew System::Windows::Forms::Label());
			this->Sample_points_in = (gcnew System::Windows::Forms::TextBox());
			this->Overlap = (gcnew System::Windows::Forms::Label());
			this->Overlap_in = (gcnew System::Windows::Forms::TextBox());
			this->Delta = (gcnew System::Windows::Forms::Label());
			this->Delta_in = (gcnew System::Windows::Forms::TextBox());
			this->Transform = (gcnew System::Windows::Forms::Label());
			this->Transform_in = (gcnew System::Windows::Forms::TextBox());
			this->Constant1 = (gcnew System::Windows::Forms::Label());
			this->Constant1_in = (gcnew System::Windows::Forms::TextBox());
			this->Conatant2 = (gcnew System::Windows::Forms::Label());
			this->Constant2_in = (gcnew System::Windows::Forms::TextBox());
			this->Gyro_test = (gcnew System::Windows::Forms::Button());
			this->Chose_Align = (gcnew System::Windows::Forms::Button());
			this->overlap_constant1 = (gcnew System::Windows::Forms::TrackBar());
			this->overlap_constant2 = (gcnew System::Windows::Forms::TrackBar());
			this->Read_point_v2 = (gcnew System::Windows::Forms::Button());
			this->Rot_test = (gcnew System::Windows::Forms::Button());
			this->Reset_quat = (gcnew System::Windows::Forms::Button());
			this->Aligned_rot = (gcnew System::Windows::Forms::Button());
			this->Save_Points2 = (gcnew System::Windows::Forms::Button());
			this->First_cloud = (gcnew System::Windows::Forms::Button());
			this->Second_cloud = (gcnew System::Windows::Forms::Button());
			this->Aligned12 = (gcnew System::Windows::Forms::Button());
			this->next_cloud = (gcnew System::Windows::Forms::Button());
			this->Aligned_near = (gcnew System::Windows::Forms::Button());
			this->reset_rot = (gcnew System::Windows::Forms::Button());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->Aligned_Pre = (gcnew System::Windows::Forms::Button());
			this->Iter_treshold = (gcnew System::Windows::Forms::Label());
			this->Iter_thresh = (gcnew System::Windows::Forms::TextBox());
			this->Iter_count = (gcnew System::Windows::Forms::Label());
			this->Iter_count_in = (gcnew System::Windows::Forms::TextBox());
			this->Combine_cloud = (gcnew System::Windows::Forms::Button());
			this->Save_combine = (gcnew System::Windows::Forms::Button());
			this->output_name = (gcnew System::Windows::Forms::Label());
			this->name_output = (gcnew System::Windows::Forms::TextBox());
			this->Add2Combine = (gcnew System::Windows::Forms::Button());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBox2 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBox3 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBox4 = (gcnew System::Windows::Forms::PictureBox());
			this->pictureBox5 = (gcnew System::Windows::Forms::PictureBox());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->numericUpDown1 = (gcnew System::Windows::Forms::NumericUpDown());
			this->numericUpDown2 = (gcnew System::Windows::Forms::NumericUpDown());
			this->numericUpDown3 = (gcnew System::Windows::Forms::NumericUpDown());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->Apply_rot = (gcnew System::Windows::Forms::Button());
			this->pictureBox6 = (gcnew System::Windows::Forms::PictureBox());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->numericUpDown4 = (gcnew System::Windows::Forms::NumericUpDown());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->Aligned_target = (gcnew System::Windows::Forms::Button());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->overlap_constant1))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->overlap_constant2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox3))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox4))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox5))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown1))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown3))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox6))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown4))->BeginInit();
			this->SuspendLayout();
			// 
			// cbComPortL
			// 
			this->cbComPortL->FormattingEnabled = true;
			this->cbComPortL->Location = System::Drawing::Point(12, 12);
			this->cbComPortL->Name = L"cbComPortL";
			this->cbComPortL->Size = System::Drawing::Size(121, 20);
			this->cbComPortL->TabIndex = 0;
			this->cbComPortL->SelectedIndexChanged += gcnew System::EventHandler(this, &MyForm::comboBox1_SelectedIndexChanged);
			// 
			// btnComUpdate
			// 
			this->btnComUpdate->Location = System::Drawing::Point(139, 10);
			this->btnComUpdate->Name = L"btnComUpdate";
			this->btnComUpdate->Size = System::Drawing::Size(75, 23);
			this->btnComUpdate->TabIndex = 1;
			this->btnComUpdate->Text = L"Update COM";
			this->btnComUpdate->TextImageRelation = System::Windows::Forms::TextImageRelation::TextAboveImage;
			this->btnComUpdate->UseVisualStyleBackColor = true;
			this->btnComUpdate->Click += gcnew System::EventHandler(this, &MyForm::btnComUpdate_Click);
			// 
			// cbBleDeviceL
			// 
			this->cbBleDeviceL->FormattingEnabled = true;
			this->cbBleDeviceL->Location = System::Drawing::Point(333, 11);
			this->cbBleDeviceL->Name = L"cbBleDeviceL";
			this->cbBleDeviceL->Size = System::Drawing::Size(121, 20);
			this->cbBleDeviceL->TabIndex = 2;
			// 
			// btnBleScan
			// 
			this->btnBleScan->Location = System::Drawing::Point(460, 8);
			this->btnBleScan->Name = L"btnBleScan";
			this->btnBleScan->Size = System::Drawing::Size(75, 23);
			this->btnBleScan->TabIndex = 3;
			this->btnBleScan->Text = L"BleScan";
			this->btnBleScan->UseVisualStyleBackColor = true;
			this->btnBleScan->Click += gcnew System::EventHandler(this, &MyForm::btnBleScan_Click);
			// 
			// btnComOpen
			// 
			this->btnComOpen->Location = System::Drawing::Point(220, 10);
			this->btnComOpen->Name = L"btnComOpen";
			this->btnComOpen->Size = System::Drawing::Size(75, 23);
			this->btnComOpen->TabIndex = 4;
			this->btnComOpen->Text = L"Open COM";
			this->btnComOpen->UseVisualStyleBackColor = true;
			this->btnComOpen->Click += gcnew System::EventHandler(this, &MyForm::btnComOpen_Click);
			// 
			// backgroundWorker1
			// 
			this->backgroundWorker1->DoWork += gcnew System::ComponentModel::DoWorkEventHandler(this, &MyForm::backgroundWorker1_DoWork);
			// 
			// btnBleEstablish
			// 
			this->btnBleEstablish->Location = System::Drawing::Point(552, 8);
			this->btnBleEstablish->Name = L"btnBleEstablish";
			this->btnBleEstablish->Size = System::Drawing::Size(75, 23);
			this->btnBleEstablish->TabIndex = 5;
			this->btnBleEstablish->Text = L"BleEstablish";
			this->btnBleEstablish->UseVisualStyleBackColor = true;
			this->btnBleEstablish->Click += gcnew System::EventHandler(this, &MyForm::btnBleEstablish_Click);
			// 
			// Get_Quat
			// 
			this->Get_Quat->Location = System::Drawing::Point(645, 8);
			this->Get_Quat->Name = L"Get_Quat";
			this->Get_Quat->Size = System::Drawing::Size(75, 23);
			this->Get_Quat->TabIndex = 6;
			this->Get_Quat->Text = L"Get Quat";
			this->Get_Quat->UseVisualStyleBackColor = true;
			this->Get_Quat->Click += gcnew System::EventHandler(this, &MyForm::Get_Quat_Click);
			// 
			// Initial_scan
			// 
			this->Initial_scan->Location = System::Drawing::Point(755, 114);
			this->Initial_scan->Name = L"Initial_scan";
			this->Initial_scan->Size = System::Drawing::Size(75, 23);
			this->Initial_scan->TabIndex = 7;
			this->Initial_scan->Text = L"Initial_scan";
			this->Initial_scan->UseVisualStyleBackColor = true;
			this->Initial_scan->Click += gcnew System::EventHandler(this, &MyForm::Initial_scan_Click);
			// 
			// Single_scan
			// 
			this->Single_scan->Location = System::Drawing::Point(755, 223);
			this->Single_scan->Name = L"Single_scan";
			this->Single_scan->Size = System::Drawing::Size(75, 23);
			this->Single_scan->TabIndex = 8;
			this->Single_scan->Text = L"Single_scan";
			this->Single_scan->UseVisualStyleBackColor = true;
			this->Single_scan->Click += gcnew System::EventHandler(this, &MyForm::Single_scan_Click);
			// 
			// Full_scan
			// 
			this->Full_scan->Location = System::Drawing::Point(755, 185);
			this->Full_scan->Name = L"Full_scan";
			this->Full_scan->Size = System::Drawing::Size(75, 23);
			this->Full_scan->TabIndex = 9;
			this->Full_scan->Text = L"Full_scan";
			this->Full_scan->UseVisualStyleBackColor = true;
			this->Full_scan->Click += gcnew System::EventHandler(this, &MyForm::Full_scan_Click);
			// 
			// hkoglPanelControl1
			// 
			hkcoglPanelCameraSetting1->Far = 1000;
			hkcoglPanelCameraSetting1->Fov = 45;
			hkcoglPanelCameraSetting1->Near = -1000;
			hkcoglPanelCameraSetting1->Type = HKOGLPanel::HKCOGLPanelCameraSetting::CAMERATYPE::ORTHOGRAPHIC;
			this->hkoglPanelControl1->Camera_Setting = hkcoglPanelCameraSetting1;
			this->hkoglPanelControl1->Location = System::Drawing::Point(14, 49);
			this->hkoglPanelControl1->Name = L"hkoglPanelControl1";
			hkcoglPanelPixelFormat1->Accumu_Buffer_Bits = HKOGLPanel::HKCOGLPanelPixelFormat::PIXELBITS::BITS_0;
			hkcoglPanelPixelFormat1->Alpha_Buffer_Bits = HKOGLPanel::HKCOGLPanelPixelFormat::PIXELBITS::BITS_0;
			hkcoglPanelPixelFormat1->Stencil_Buffer_Bits = HKOGLPanel::HKCOGLPanelPixelFormat::PIXELBITS::BITS_0;
			this->hkoglPanelControl1->Pixel_Format = hkcoglPanelPixelFormat1;
			this->hkoglPanelControl1->Size = System::Drawing::Size(706, 482);
			this->hkoglPanelControl1->TabIndex = 10;
			this->hkoglPanelControl1->Load += gcnew System::EventHandler(this, &MyForm::hkoglPanelControl1_Load);
			this->hkoglPanelControl1->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &MyForm::hkoglPanelControl1_Paint);
			this->hkoglPanelControl1->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::hkoglPanelControl1_KeyDown);
			this->hkoglPanelControl1->KeyPress += gcnew System::Windows::Forms::KeyPressEventHandler(this, &MyForm::hkoglPanelControl1_KeyPress);
			this->hkoglPanelControl1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::hkoglPanelControl1_MouseDown);
			this->hkoglPanelControl1->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::hkoglPanelControl1_MouseMove);
			this->hkoglPanelControl1->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::hkoglPanelControl1_MouseUp);
			this->hkoglPanelControl1->MouseWheel += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::hkoglPanelControl1_MouseWheel);
			// 
			// Test_file
			// 
			this->Test_file->Location = System::Drawing::Point(755, 73);
			this->Test_file->Name = L"Test_file";
			this->Test_file->Size = System::Drawing::Size(75, 23);
			this->Test_file->TabIndex = 11;
			this->Test_file->Text = L"Read raw";
			this->Test_file->UseVisualStyleBackColor = true;
			this->Test_file->Click += gcnew System::EventHandler(this, &MyForm::Test_file_Click);
			// 
			// Clear_cloud
			// 
			this->Clear_cloud->Location = System::Drawing::Point(754, 9);
			this->Clear_cloud->Name = L"Clear_cloud";
			this->Clear_cloud->Size = System::Drawing::Size(75, 23);
			this->Clear_cloud->TabIndex = 12;
			this->Clear_cloud->Text = L"Clear cloud";
			this->Clear_cloud->UseVisualStyleBackColor = true;
			this->Clear_cloud->Click += gcnew System::EventHandler(this, &MyForm::Clear_cloud_Click);
			// 
			// Alignment
			// 
			this->Alignment->Location = System::Drawing::Point(755, 265);
			this->Alignment->Name = L"Alignment";
			this->Alignment->Size = System::Drawing::Size(75, 23);
			this->Alignment->TabIndex = 13;
			this->Alignment->Text = L"Alignment";
			this->Alignment->UseVisualStyleBackColor = true;
			this->Alignment->Click += gcnew System::EventHandler(this, &MyForm::Alignment_Click);
			// 
			// Save_PointCloud
			// 
			this->Save_PointCloud->Location = System::Drawing::Point(755, 310);
			this->Save_PointCloud->Name = L"Save_PointCloud";
			this->Save_PointCloud->Size = System::Drawing::Size(75, 23);
			this->Save_PointCloud->TabIndex = 14;
			this->Save_PointCloud->Text = L"SavePoint";
			this->Save_PointCloud->UseVisualStyleBackColor = true;
			this->Save_PointCloud->Click += gcnew System::EventHandler(this, &MyForm::Save_PointCloud_Click);
			// 
			// saveFileDialog1
			// 
			this->saveFileDialog1->DefaultExt = L"txt";
			this->saveFileDialog1->RestoreDirectory = true;
			this->saveFileDialog1->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &MyForm::saveFileDialog1_FileOk);
			// 
			// Save_point_s
			// 
			this->Save_point_s->Location = System::Drawing::Point(755, 346);
			this->Save_point_s->Name = L"Save_point_s";
			this->Save_point_s->Size = System::Drawing::Size(75, 23);
			this->Save_point_s->TabIndex = 15;
			this->Save_point_s->Text = L"Save Point s";
			this->Save_point_s->UseVisualStyleBackColor = true;
			this->Save_point_s->Click += gcnew System::EventHandler(this, &MyForm::Save_point_s_Click);
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(862, 34);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(100, 22);
			this->textBox1->TabIndex = 16;
			this->textBox1->TextChanged += gcnew System::EventHandler(this, &MyForm::textBox1_TextChanged);
			this->textBox1->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::textBox1_KeyDown);
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(879, 19);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(46, 12);
			this->label1->TabIndex = 17;
			this->label1->Text = L"peakGap";
			this->label1->Click += gcnew System::EventHandler(this, &MyForm::label1_Click);
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(879, 59);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(56, 12);
			this->label2->TabIndex = 18;
			this->label2->Text = L"energyGap";
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(862, 75);
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(100, 22);
			this->textBox2->TabIndex = 19;
			this->textBox2->TextChanged += gcnew System::EventHandler(this, &MyForm::textBox2_TextChanged);
			this->textBox2->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::textBox2_KeyDown);
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(892, 100);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(33, 12);
			this->label3->TabIndex = 20;
			this->label3->Text = L"label3";
			this->label3->Click += gcnew System::EventHandler(this, &MyForm::label3_Click);
			// 
			// read_point
			// 
			this->read_point->Location = System::Drawing::Point(756, 43);
			this->read_point->Name = L"read_point";
			this->read_point->Size = System::Drawing::Size(75, 23);
			this->read_point->TabIndex = 21;
			this->read_point->Text = L"Read point";
			this->read_point->UseVisualStyleBackColor = true;
			this->read_point->Click += gcnew System::EventHandler(this, &MyForm::read_point_Click);
			// 
			// Combine
			// 
			this->Combine->Location = System::Drawing::Point(754, 387);
			this->Combine->Name = L"Combine";
			this->Combine->Size = System::Drawing::Size(75, 23);
			this->Combine->TabIndex = 22;
			this->Combine->Text = L"Combine";
			this->Combine->UseVisualStyleBackColor = true;
			this->Combine->Click += gcnew System::EventHandler(this, &MyForm::Combine_Click);
			// 
			// Part_Align
			// 
			this->Part_Align->Location = System::Drawing::Point(881, 191);
			this->Part_Align->Name = L"Part_Align";
			this->Part_Align->Size = System::Drawing::Size(88, 23);
			this->Part_Align->TabIndex = 23;
			this->Part_Align->Text = L"Part Align";
			this->Part_Align->UseVisualStyleBackColor = true;
			this->Part_Align->Click += gcnew System::EventHandler(this, &MyForm::Part_Align_Click);
			// 
			// Threshold
			// 
			this->Threshold->AutoSize = true;
			this->Threshold->Location = System::Drawing::Point(888, 221);
			this->Threshold->Name = L"Threshold";
			this->Threshold->Size = System::Drawing::Size(52, 12);
			this->Threshold->TabIndex = 24;
			this->Threshold->Text = L"Threshold";
			// 
			// Theshold_in
			// 
			this->Theshold_in->Location = System::Drawing::Point(862, 237);
			this->Theshold_in->Name = L"Theshold_in";
			this->Theshold_in->Size = System::Drawing::Size(100, 22);
			this->Theshold_in->TabIndex = 25;
			this->Theshold_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Theshold_in_TextChanged);
			this->Theshold_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Theshold_in_KeyDown_1);
			// 
			// Max_time
			// 
			this->Max_time->AutoSize = true;
			this->Max_time->Location = System::Drawing::Point(890, 266);
			this->Max_time->Name = L"Max_time";
			this->Max_time->Size = System::Drawing::Size(49, 12);
			this->Max_time->TabIndex = 26;
			this->Max_time->Text = L"Max time";
			// 
			// Max_time_in
			// 
			this->Max_time_in->Location = System::Drawing::Point(862, 281);
			this->Max_time_in->Name = L"Max_time_in";
			this->Max_time_in->Size = System::Drawing::Size(100, 22);
			this->Max_time_in->TabIndex = 27;
			this->Max_time_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Max_time_in_TextChanged);
			this->Max_time_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Max_time_in_KeyDown);
			// 
			// Max_angle
			// 
			this->Max_angle->AutoSize = true;
			this->Max_angle->Location = System::Drawing::Point(890, 310);
			this->Max_angle->Name = L"Max_angle";
			this->Max_angle->Size = System::Drawing::Size(54, 12);
			this->Max_angle->TabIndex = 28;
			this->Max_angle->Text = L"Max angle";
			// 
			// Max_angle_in
			// 
			this->Max_angle_in->Location = System::Drawing::Point(862, 325);
			this->Max_angle_in->Name = L"Max_angle_in";
			this->Max_angle_in->Size = System::Drawing::Size(100, 22);
			this->Max_angle_in->TabIndex = 29;
			this->Max_angle_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Max_angle_in_TextChanged);
			this->Max_angle_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Max_angle_in_KeyDown);
			// 
			// Sample_points
			// 
			this->Sample_points->AutoSize = true;
			this->Sample_points->Location = System::Drawing::Point(882, 354);
			this->Sample_points->Name = L"Sample_points";
			this->Sample_points->Size = System::Drawing::Size(70, 12);
			this->Sample_points->TabIndex = 30;
			this->Sample_points->Text = L"Sample points";
			// 
			// Sample_points_in
			// 
			this->Sample_points_in->Location = System::Drawing::Point(862, 370);
			this->Sample_points_in->Name = L"Sample_points_in";
			this->Sample_points_in->Size = System::Drawing::Size(100, 22);
			this->Sample_points_in->TabIndex = 31;
			this->Sample_points_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Sample_points_in_TextChanged);
			this->Sample_points_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Sample_points_in_KeyDown);
			// 
			// Overlap
			// 
			this->Overlap->AutoSize = true;
			this->Overlap->Location = System::Drawing::Point(891, 399);
			this->Overlap->Name = L"Overlap";
			this->Overlap->Size = System::Drawing::Size(42, 12);
			this->Overlap->TabIndex = 32;
			this->Overlap->Text = L"Overlap";
			// 
			// Overlap_in
			// 
			this->Overlap_in->Location = System::Drawing::Point(862, 414);
			this->Overlap_in->Name = L"Overlap_in";
			this->Overlap_in->Size = System::Drawing::Size(100, 22);
			this->Overlap_in->TabIndex = 33;
			this->Overlap_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Overlap_in_TextChanged);
			this->Overlap_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Overlap_in_KeyDown);
			// 
			// Delta
			// 
			this->Delta->AutoSize = true;
			this->Delta->Location = System::Drawing::Point(894, 443);
			this->Delta->Name = L"Delta";
			this->Delta->Size = System::Drawing::Size(29, 12);
			this->Delta->TabIndex = 34;
			this->Delta->Text = L"Delta";
			// 
			// Delta_in
			// 
			this->Delta_in->Location = System::Drawing::Point(862, 458);
			this->Delta_in->Name = L"Delta_in";
			this->Delta_in->Size = System::Drawing::Size(100, 22);
			this->Delta_in->TabIndex = 35;
			this->Delta_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Delta_in_TextChanged);
			this->Delta_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Delta_in_KeyDown);
			// 
			// Transform
			// 
			this->Transform->AutoSize = true;
			this->Transform->Location = System::Drawing::Point(890, 487);
			this->Transform->Name = L"Transform";
			this->Transform->Size = System::Drawing::Size(54, 12);
			this->Transform->TabIndex = 36;
			this->Transform->Text = L"Transform";
			// 
			// Transform_in
			// 
			this->Transform_in->Location = System::Drawing::Point(862, 505);
			this->Transform_in->Name = L"Transform_in";
			this->Transform_in->Size = System::Drawing::Size(100, 22);
			this->Transform_in->TabIndex = 37;
			this->Transform_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Transform_in_TextChanged);
			this->Transform_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Transform_in_KeyDown);
			// 
			// Constant1
			// 
			this->Constant1->AutoSize = true;
			this->Constant1->Location = System::Drawing::Point(768, 427);
			this->Constant1->Name = L"Constant1";
			this->Constant1->Size = System::Drawing::Size(52, 12);
			this->Constant1->TabIndex = 38;
			this->Constant1->Text = L"Constant1";
			// 
			// Constant1_in
			// 
			this->Constant1_in->Location = System::Drawing::Point(742, 442);
			this->Constant1_in->Name = L"Constant1_in";
			this->Constant1_in->Size = System::Drawing::Size(100, 22);
			this->Constant1_in->TabIndex = 39;
			this->Constant1_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Constant1_in_TextChanged);
			this->Constant1_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Constant1_in_KeyDown);
			// 
			// Conatant2
			// 
			this->Conatant2->AutoSize = true;
			this->Conatant2->Location = System::Drawing::Point(768, 472);
			this->Conatant2->Name = L"Conatant2";
			this->Conatant2->Size = System::Drawing::Size(53, 12);
			this->Conatant2->TabIndex = 40;
			this->Conatant2->Text = L"Conatant2";
			// 
			// Constant2_in
			// 
			this->Constant2_in->Location = System::Drawing::Point(742, 487);
			this->Constant2_in->Name = L"Constant2_in";
			this->Constant2_in->Size = System::Drawing::Size(100, 22);
			this->Constant2_in->TabIndex = 41;
			this->Constant2_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Constant2_in_TextChanged);
			this->Constant2_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Constant2_in_KeyDown);
			// 
			// Gyro_test
			// 
			this->Gyro_test->Location = System::Drawing::Point(881, 115);
			this->Gyro_test->Name = L"Gyro_test";
			this->Gyro_test->Size = System::Drawing::Size(88, 23);
			this->Gyro_test->TabIndex = 42;
			this->Gyro_test->Text = L"Gyro test";
			this->Gyro_test->UseVisualStyleBackColor = true;
			this->Gyro_test->Click += gcnew System::EventHandler(this, &MyForm::Gyro_test_Click);
			// 
			// Chose_Align
			// 
			this->Chose_Align->Location = System::Drawing::Point(754, 519);
			this->Chose_Align->Name = L"Chose_Align";
			this->Chose_Align->Size = System::Drawing::Size(75, 23);
			this->Chose_Align->TabIndex = 43;
			this->Chose_Align->Text = L"Chose Align";
			this->Chose_Align->UseVisualStyleBackColor = true;
			this->Chose_Align->Click += gcnew System::EventHandler(this, &MyForm::Chose_Align_Click);
			// 
			// overlap_constant1
			// 
			this->overlap_constant1->Location = System::Drawing::Point(40, 548);
			this->overlap_constant1->Name = L"overlap_constant1";
			this->overlap_constant1->Size = System::Drawing::Size(151, 45);
			this->overlap_constant1->TabIndex = 44;
			this->overlap_constant1->Scroll += gcnew System::EventHandler(this, &MyForm::overlap_constant1_Scroll);
			// 
			// overlap_constant2
			// 
			this->overlap_constant2->Location = System::Drawing::Point(410, 548);
			this->overlap_constant2->Name = L"overlap_constant2";
			this->overlap_constant2->Size = System::Drawing::Size(104, 45);
			this->overlap_constant2->TabIndex = 45;
			this->overlap_constant2->Scroll += gcnew System::EventHandler(this, &MyForm::overlap_constant2_Scroll);
			// 
			// Read_point_v2
			// 
			this->Read_point_v2->Location = System::Drawing::Point(881, 152);
			this->Read_point_v2->Name = L"Read_point_v2";
			this->Read_point_v2->Size = System::Drawing::Size(88, 23);
			this->Read_point_v2->TabIndex = 46;
			this->Read_point_v2->Text = L"Read point v2";
			this->Read_point_v2->UseVisualStyleBackColor = true;
			this->Read_point_v2->Click += gcnew System::EventHandler(this, &MyForm::Read_point_v2_Click);
			// 
			// Rot_test
			// 
			this->Rot_test->Location = System::Drawing::Point(756, 548);
			this->Rot_test->Name = L"Rot_test";
			this->Rot_test->Size = System::Drawing::Size(75, 23);
			this->Rot_test->TabIndex = 47;
			this->Rot_test->Text = L"Rot test";
			this->Rot_test->UseVisualStyleBackColor = true;
			this->Rot_test->Click += gcnew System::EventHandler(this, &MyForm::Rot_test_Click);
			// 
			// Reset_quat
			// 
			this->Reset_quat->Location = System::Drawing::Point(674, 548);
			this->Reset_quat->Name = L"Reset_quat";
			this->Reset_quat->Size = System::Drawing::Size(75, 23);
			this->Reset_quat->TabIndex = 48;
			this->Reset_quat->Text = L"Reset quat";
			this->Reset_quat->UseVisualStyleBackColor = true;
			this->Reset_quat->Click += gcnew System::EventHandler(this, &MyForm::Reset_quat_Click);
			// 
			// Aligned_rot
			// 
			this->Aligned_rot->Location = System::Drawing::Point(881, 577);
			this->Aligned_rot->Name = L"Aligned_rot";
			this->Aligned_rot->Size = System::Drawing::Size(75, 23);
			this->Aligned_rot->TabIndex = 49;
			this->Aligned_rot->Text = L"Aligned rot";
			this->Aligned_rot->UseVisualStyleBackColor = true;
			this->Aligned_rot->Click += gcnew System::EventHandler(this, &MyForm::Aligned_rot_Click);
			// 
			// Save_Points2
			// 
			this->Save_Points2->Location = System::Drawing::Point(582, 548);
			this->Save_Points2->Name = L"Save_Points2";
			this->Save_Points2->Size = System::Drawing::Size(75, 23);
			this->Save_Points2->TabIndex = 50;
			this->Save_Points2->Text = L"Save Points2";
			this->Save_Points2->UseVisualStyleBackColor = true;
			this->Save_Points2->Click += gcnew System::EventHandler(this, &MyForm::Save_Points2_Click);
			// 
			// First_cloud
			// 
			this->First_cloud->Location = System::Drawing::Point(997, 23);
			this->First_cloud->Name = L"First_cloud";
			this->First_cloud->Size = System::Drawing::Size(83, 23);
			this->First_cloud->TabIndex = 51;
			this->First_cloud->Text = L"First cloud";
			this->First_cloud->UseVisualStyleBackColor = true;
			this->First_cloud->Click += gcnew System::EventHandler(this, &MyForm::First_cloud_Click);
			// 
			// Second_cloud
			// 
			this->Second_cloud->Location = System::Drawing::Point(756, 579);
			this->Second_cloud->Name = L"Second_cloud";
			this->Second_cloud->Size = System::Drawing::Size(83, 23);
			this->Second_cloud->TabIndex = 52;
			this->Second_cloud->Text = L"Second cloud";
			this->Second_cloud->UseVisualStyleBackColor = true;
			this->Second_cloud->Click += gcnew System::EventHandler(this, &MyForm::Second_cloud_Click);
			// 
			// Aligned12
			// 
			this->Aligned12->Location = System::Drawing::Point(997, 238);
			this->Aligned12->Name = L"Aligned12";
			this->Aligned12->Size = System::Drawing::Size(83, 23);
			this->Aligned12->TabIndex = 53;
			this->Aligned12->Text = L"Aligned to1";
			this->Aligned12->UseVisualStyleBackColor = true;
			this->Aligned12->Click += gcnew System::EventHandler(this, &MyForm::Aligned12_Click);
			// 
			// next_cloud
			// 
			this->next_cloud->Location = System::Drawing::Point(997, 533);
			this->next_cloud->Name = L"next_cloud";
			this->next_cloud->Size = System::Drawing::Size(83, 23);
			this->next_cloud->TabIndex = 54;
			this->next_cloud->Text = L"Next cloud";
			this->next_cloud->UseVisualStyleBackColor = true;
			this->next_cloud->Click += gcnew System::EventHandler(this, &MyForm::next_cloud_Click);
			// 
			// Aligned_near
			// 
			this->Aligned_near->Location = System::Drawing::Point(997, 299);
			this->Aligned_near->Name = L"Aligned_near";
			this->Aligned_near->Size = System::Drawing::Size(83, 23);
			this->Aligned_near->TabIndex = 55;
			this->Aligned_near->Text = L"Aligned near";
			this->Aligned_near->UseVisualStyleBackColor = true;
			this->Aligned_near->Click += gcnew System::EventHandler(this, &MyForm::Aligned_near_Click);
			// 
			// reset_rot
			// 
			this->reset_rot->Location = System::Drawing::Point(997, 69);
			this->reset_rot->Name = L"reset_rot";
			this->reset_rot->Size = System::Drawing::Size(83, 23);
			this->reset_rot->TabIndex = 56;
			this->reset_rot->Text = L"reset rot";
			this->reset_rot->UseVisualStyleBackColor = true;
			this->reset_rot->Click += gcnew System::EventHandler(this, &MyForm::reset_rot_Click);
			// 
			// timer1
			// 
			this->timer1->Enabled = true;
			this->timer1->Interval = 15;
			this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
			// 
			// Aligned_Pre
			// 
			this->Aligned_Pre->Location = System::Drawing::Point(997, 268);
			this->Aligned_Pre->Name = L"Aligned_Pre";
			this->Aligned_Pre->Size = System::Drawing::Size(83, 23);
			this->Aligned_Pre->TabIndex = 57;
			this->Aligned_Pre->Text = L"Aligned Pre";
			this->Aligned_Pre->UseVisualStyleBackColor = true;
			this->Aligned_Pre->Click += gcnew System::EventHandler(this, &MyForm::Aligned_Pre_Click);
			// 
			// Iter_treshold
			// 
			this->Iter_treshold->AutoSize = true;
			this->Iter_treshold->Location = System::Drawing::Point(888, 534);
			this->Iter_treshold->Name = L"Iter_treshold";
			this->Iter_treshold->Size = System::Drawing::Size(61, 12);
			this->Iter_treshold->TabIndex = 58;
			this->Iter_treshold->Text = L"Iter treshold";
			// 
			// Iter_thresh
			// 
			this->Iter_thresh->Location = System::Drawing::Point(862, 548);
			this->Iter_thresh->Name = L"Iter_thresh";
			this->Iter_thresh->Size = System::Drawing::Size(100, 22);
			this->Iter_thresh->TabIndex = 59;
			this->Iter_thresh->TextChanged += gcnew System::EventHandler(this, &MyForm::Iter_thresh_TextChanged);
			this->Iter_thresh->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Iter_thresh_KeyDown);
			// 
			// Iter_count
			// 
			this->Iter_count->AutoSize = true;
			this->Iter_count->Location = System::Drawing::Point(1018, 566);
			this->Iter_count->Name = L"Iter_count";
			this->Iter_count->Size = System::Drawing::Size(50, 12);
			this->Iter_count->TabIndex = 60;
			this->Iter_count->Text = L"Iter count";
			// 
			// Iter_count_in
			// 
			this->Iter_count_in->Location = System::Drawing::Point(992, 580);
			this->Iter_count_in->Name = L"Iter_count_in";
			this->Iter_count_in->Size = System::Drawing::Size(100, 22);
			this->Iter_count_in->TabIndex = 61;
			this->Iter_count_in->TextChanged += gcnew System::EventHandler(this, &MyForm::Iter_count_in_TextChanged);
			this->Iter_count_in->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::Iter_count_in_KeyDown);
			// 
			// Combine_cloud
			// 
			this->Combine_cloud->Location = System::Drawing::Point(997, 453);
			this->Combine_cloud->Name = L"Combine_cloud";
			this->Combine_cloud->Size = System::Drawing::Size(83, 23);
			this->Combine_cloud->TabIndex = 62;
			this->Combine_cloud->Text = L"Combine";
			this->Combine_cloud->UseVisualStyleBackColor = true;
			this->Combine_cloud->Click += gcnew System::EventHandler(this, &MyForm::Combine_cloud_Click);
			// 
			// Save_combine
			// 
			this->Save_combine->Location = System::Drawing::Point(997, 488);
			this->Save_combine->Name = L"Save_combine";
			this->Save_combine->Size = System::Drawing::Size(83, 23);
			this->Save_combine->TabIndex = 63;
			this->Save_combine->Text = L"Save combine";
			this->Save_combine->UseVisualStyleBackColor = true;
			this->Save_combine->Click += gcnew System::EventHandler(this, &MyForm::Save_combine_Click);
			// 
			// output_name
			// 
			this->output_name->AutoSize = true;
			this->output_name->Location = System::Drawing::Point(778, 142);
			this->output_name->Name = L"output_name";
			this->output_name->Size = System::Drawing::Size(32, 12);
			this->output_name->TabIndex = 64;
			this->output_name->Text = L"Name";
			// 
			// name_output
			// 
			this->name_output->Location = System::Drawing::Point(743, 157);
			this->name_output->Name = L"name_output";
			this->name_output->Size = System::Drawing::Size(100, 22);
			this->name_output->TabIndex = 65;
			this->name_output->TextChanged += gcnew System::EventHandler(this, &MyForm::name_output_TextChanged);
			this->name_output->KeyDown += gcnew System::Windows::Forms::KeyEventHandler(this, &MyForm::name_output_KeyDown);
			// 
			// Add2Combine
			// 
			this->Add2Combine->Location = System::Drawing::Point(992, 394);
			this->Add2Combine->Name = L"Add2Combine";
			this->Add2Combine->Size = System::Drawing::Size(93, 23);
			this->Add2Combine->TabIndex = 66;
			this->Add2Combine->Text = L"Add to combine";
			this->Add2Combine->UseVisualStyleBackColor = true;
			this->Add2Combine->Click += gcnew System::EventHandler(this, &MyForm::Add2Combine_Click);
			// 
			// pictureBox1
			// 
			this->pictureBox1->BackColor = System::Drawing::Color::Gold;
			this->pictureBox1->Location = System::Drawing::Point(992, 59);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(93, 38);
			this->pictureBox1->TabIndex = 67;
			this->pictureBox1->TabStop = false;
			// 
			// pictureBox2
			// 
			this->pictureBox2->BackColor = System::Drawing::SystemColors::ControlDark;
			this->pictureBox2->Location = System::Drawing::Point(975, 230);
			this->pictureBox2->Name = L"pictureBox2";
			this->pictureBox2->Size = System::Drawing::Size(128, 200);
			this->pictureBox2->TabIndex = 68;
			this->pictureBox2->TabStop = false;
			// 
			// pictureBox3
			// 
			this->pictureBox3->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(192)),
				static_cast<System::Int32>(static_cast<System::Byte>(128)));
			this->pictureBox3->Location = System::Drawing::Point(992, 441);
			this->pictureBox3->Name = L"pictureBox3";
			this->pictureBox3->Size = System::Drawing::Size(93, 79);
			this->pictureBox3->TabIndex = 69;
			this->pictureBox3->TabStop = false;
			// 
			// pictureBox4
			// 
			this->pictureBox4->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(255)),
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->pictureBox4->Location = System::Drawing::Point(738, 108);
			this->pictureBox4->Name = L"pictureBox4";
			this->pictureBox4->Size = System::Drawing::Size(111, 109);
			this->pictureBox4->TabIndex = 70;
			this->pictureBox4->TabStop = false;
			// 
			// pictureBox5
			// 
			this->pictureBox5->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(255)),
				static_cast<System::Int32>(static_cast<System::Byte>(192)));
			this->pictureBox5->Location = System::Drawing::Point(742, 303);
			this->pictureBox5->Name = L"pictureBox5";
			this->pictureBox5->Size = System::Drawing::Size(100, 73);
			this->pictureBox5->TabIndex = 71;
			this->pictureBox5->TabStop = false;
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Font = (gcnew System::Drawing::Font(L"新細明體", 11.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(136)));
			this->label4->Location = System::Drawing::Point(735, 100);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(14, 15);
			this->label4->TabIndex = 72;
			this->label4->Text = L"1";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Font = (gcnew System::Drawing::Font(L"新細明體", 11.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(136)));
			this->label5->Location = System::Drawing::Point(735, 295);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(14, 15);
			this->label5->TabIndex = 73;
			this->label5->Text = L"2";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Font = (gcnew System::Drawing::Font(L"新細明體", 11.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(136)));
			this->label6->Location = System::Drawing::Point(984, 49);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(14, 15);
			this->label6->TabIndex = 74;
			this->label6->Text = L"3";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Font = (gcnew System::Drawing::Font(L"新細明體", 11.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(136)));
			this->label7->Location = System::Drawing::Point(975, 222);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(14, 15);
			this->label7->TabIndex = 75;
			this->label7->Text = L"5";
			this->label7->Click += gcnew System::EventHandler(this, &MyForm::label7_Click);
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Font = (gcnew System::Drawing::Font(L"新細明體", 11.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(136)));
			this->label8->Location = System::Drawing::Point(985, 433);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(14, 15);
			this->label8->TabIndex = 76;
			this->label8->Text = L"6";
			// 
			// numericUpDown1
			// 
			this->numericUpDown1->Location = System::Drawing::Point(1024, 112);
			this->numericUpDown1->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 360, 0, 0, 0 });
			this->numericUpDown1->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 360, 0, 0, System::Int32::MinValue });
			this->numericUpDown1->Name = L"numericUpDown1";
			this->numericUpDown1->Size = System::Drawing::Size(73, 22);
			this->numericUpDown1->TabIndex = 77;
			this->numericUpDown1->ValueChanged += gcnew System::EventHandler(this, &MyForm::numericUpDown1_ValueChanged);
			// 
			// numericUpDown2
			// 
			this->numericUpDown2->Location = System::Drawing::Point(1024, 141);
			this->numericUpDown2->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 360, 0, 0, 0 });
			this->numericUpDown2->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 360, 0, 0, System::Int32::MinValue });
			this->numericUpDown2->Name = L"numericUpDown2";
			this->numericUpDown2->Size = System::Drawing::Size(73, 22);
			this->numericUpDown2->TabIndex = 78;
			this->numericUpDown2->ValueChanged += gcnew System::EventHandler(this, &MyForm::numericUpDown2_ValueChanged);
			// 
			// numericUpDown3
			// 
			this->numericUpDown3->Location = System::Drawing::Point(1024, 169);
			this->numericUpDown3->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 360, 0, 0, 0 });
			this->numericUpDown3->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 360, 0, 0, System::Int32::MinValue });
			this->numericUpDown3->Name = L"numericUpDown3";
			this->numericUpDown3->Size = System::Drawing::Size(73, 22);
			this->numericUpDown3->TabIndex = 79;
			this->numericUpDown3->ValueChanged += gcnew System::EventHandler(this, &MyForm::numericUpDown3_ValueChanged);
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Font = (gcnew System::Drawing::Font(L"新細明體", 9, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(136)));
			this->label9->Location = System::Drawing::Point(976, 116);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(44, 12);
			this->label9->TabIndex = 80;
			this->label9->Text = L"Angle X";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(976, 145);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(44, 12);
			this->label10->TabIndex = 81;
			this->label10->Text = L"Angle Y";
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(976, 174);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(43, 12);
			this->label11->TabIndex = 82;
			this->label11->Text = L"Angle Z";
			// 
			// Apply_rot
			// 
			this->Apply_rot->Location = System::Drawing::Point(997, 197);
			this->Apply_rot->Name = L"Apply_rot";
			this->Apply_rot->Size = System::Drawing::Size(75, 23);
			this->Apply_rot->TabIndex = 83;
			this->Apply_rot->Text = L"Apply rot";
			this->Apply_rot->UseVisualStyleBackColor = true;
			this->Apply_rot->Click += gcnew System::EventHandler(this, &MyForm::Apply_rot_Click);
			// 
			// pictureBox6
			// 
			this->pictureBox6->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(192)), static_cast<System::Int32>(static_cast<System::Byte>(192)),
				static_cast<System::Int32>(static_cast<System::Byte>(255)));
			this->pictureBox6->Location = System::Drawing::Point(972, 103);
			this->pictureBox6->Name = L"pictureBox6";
			this->pictureBox6->Size = System::Drawing::Size(131, 124);
			this->pictureBox6->TabIndex = 84;
			this->pictureBox6->TabStop = false;
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Font = (gcnew System::Drawing::Font(L"新細明體", 11.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(136)));
			this->label12->Location = System::Drawing::Point(975, 93);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(14, 15);
			this->label12->TabIndex = 85;
			this->label12->Text = L"4";
			// 
			// numericUpDown4
			// 
			this->numericUpDown4->Location = System::Drawing::Point(1024, 328);
			this->numericUpDown4->Name = L"numericUpDown4";
			this->numericUpDown4->Size = System::Drawing::Size(73, 22);
			this->numericUpDown4->TabIndex = 86;
			this->numericUpDown4->ValueChanged += gcnew System::EventHandler(this, &MyForm::numericUpDown4_ValueChanged);
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(1001, 333);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(17, 12);
			this->label13->TabIndex = 87;
			this->label13->Text = L"ID";
			// 
			// Aligned_target
			// 
			this->Aligned_target->Location = System::Drawing::Point(997, 358);
			this->Aligned_target->Name = L"Aligned_target";
			this->Aligned_target->Size = System::Drawing::Size(83, 23);
			this->Aligned_target->TabIndex = 88;
			this->Aligned_target->Text = L"Aligned Target";
			this->Aligned_target->UseVisualStyleBackColor = true;
			this->Aligned_target->Click += gcnew System::EventHandler(this, &MyForm::Aligned_target_Click);
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1109, 614);
			this->Controls->Add(this->Aligned_target);
			this->Controls->Add(this->label13);
			this->Controls->Add(this->numericUpDown4);
			this->Controls->Add(this->label12);
			this->Controls->Add(this->Apply_rot);
			this->Controls->Add(this->label11);
			this->Controls->Add(this->label10);
			this->Controls->Add(this->label9);
			this->Controls->Add(this->numericUpDown3);
			this->Controls->Add(this->numericUpDown2);
			this->Controls->Add(this->numericUpDown1);
			this->Controls->Add(this->label8);
			this->Controls->Add(this->label7);
			this->Controls->Add(this->label6);
			this->Controls->Add(this->label5);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->Add2Combine);
			this->Controls->Add(this->name_output);
			this->Controls->Add(this->output_name);
			this->Controls->Add(this->Save_combine);
			this->Controls->Add(this->Combine_cloud);
			this->Controls->Add(this->Iter_count_in);
			this->Controls->Add(this->Iter_count);
			this->Controls->Add(this->Iter_thresh);
			this->Controls->Add(this->Iter_treshold);
			this->Controls->Add(this->Aligned_Pre);
			this->Controls->Add(this->reset_rot);
			this->Controls->Add(this->Aligned_near);
			this->Controls->Add(this->next_cloud);
			this->Controls->Add(this->Aligned12);
			this->Controls->Add(this->Second_cloud);
			this->Controls->Add(this->First_cloud);
			this->Controls->Add(this->Save_Points2);
			this->Controls->Add(this->Aligned_rot);
			this->Controls->Add(this->Reset_quat);
			this->Controls->Add(this->Rot_test);
			this->Controls->Add(this->Read_point_v2);
			this->Controls->Add(this->overlap_constant2);
			this->Controls->Add(this->overlap_constant1);
			this->Controls->Add(this->Chose_Align);
			this->Controls->Add(this->Gyro_test);
			this->Controls->Add(this->Constant2_in);
			this->Controls->Add(this->Conatant2);
			this->Controls->Add(this->Constant1_in);
			this->Controls->Add(this->Constant1);
			this->Controls->Add(this->Transform_in);
			this->Controls->Add(this->Transform);
			this->Controls->Add(this->Delta_in);
			this->Controls->Add(this->Delta);
			this->Controls->Add(this->Overlap_in);
			this->Controls->Add(this->Overlap);
			this->Controls->Add(this->Sample_points_in);
			this->Controls->Add(this->Sample_points);
			this->Controls->Add(this->Max_angle_in);
			this->Controls->Add(this->Max_angle);
			this->Controls->Add(this->Max_time_in);
			this->Controls->Add(this->Max_time);
			this->Controls->Add(this->Theshold_in);
			this->Controls->Add(this->Threshold);
			this->Controls->Add(this->Part_Align);
			this->Controls->Add(this->Combine);
			this->Controls->Add(this->read_point);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->textBox2);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->textBox1);
			this->Controls->Add(this->Save_point_s);
			this->Controls->Add(this->Save_PointCloud);
			this->Controls->Add(this->Alignment);
			this->Controls->Add(this->Clear_cloud);
			this->Controls->Add(this->Test_file);
			this->Controls->Add(this->hkoglPanelControl1);
			this->Controls->Add(this->Full_scan);
			this->Controls->Add(this->Single_scan);
			this->Controls->Add(this->Initial_scan);
			this->Controls->Add(this->Get_Quat);
			this->Controls->Add(this->btnBleEstablish);
			this->Controls->Add(this->btnComOpen);
			this->Controls->Add(this->btnBleScan);
			this->Controls->Add(this->cbBleDeviceL);
			this->Controls->Add(this->btnComUpdate);
			this->Controls->Add(this->cbComPortL);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->pictureBox2);
			this->Controls->Add(this->pictureBox3);
			this->Controls->Add(this->pictureBox4);
			this->Controls->Add(this->pictureBox5);
			this->Controls->Add(this->pictureBox6);
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			this->Load += gcnew System::EventHandler(this, &MyForm::MyForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->overlap_constant1))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->overlap_constant2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox3))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox4))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox5))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown1))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown3))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox6))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numericUpDown4))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void MyForm_Load(System::Object^  sender, System::EventArgs^  e);
	private:
		//
		// Enum Declarations
		//
		enum class State_Connection
		{
			Start, DeviceReady, ComOpened, BleEstablished
		};

		enum class State_Processing
		{
			Idle, Recording, Recognizing, Playback_Playing, Playback_Paused
		};
		private:
			 //
			 // Form Function Declarations
			 //
			delegate void Callback_DeviceDiscoveredDelegate(DeviceInfo ^deviceInfo);

			delegate void UpdateConnectionStatusDelegate(State_Connection state);
			void UpdateConnectionStatus(State_Connection state);
	private:
		//
		// BLE Callback Function Declarations
		//
		void Callback_DeviceInitDone(String ^address);
		void Callback_DeviceCloseDone(String ^comport);
		void Callback_DeviceDiscovered(DeviceInfo ^deviceInfo);
		void Callback_EstablishLinkDone(DeviceInfo ^deviceInfo, unsigned char status);
		void Callback_TerminateLinkDone(DeviceInfo ^deviceInfo, unsigned char status, bool byHost);
		void Callback_HandleValueNotification(DeviceInfo ^deviceInfo, cli::array<unsigned char> ^payload);
		void Callback_HandleCheckNotifyStatus(DeviceInfo ^deviceInfo, bool isNotifying);
		void Callback_HandleChangeNotifyStatus(DeviceInfo ^deviceInfo, bool isSuccess);
	private:
		void Initial_scan_Click(System::Object^  sender, System::EventArgs^  e);
		void btnComUpdate_Click(System::Object^  sender, System::EventArgs^  e);
		void btnComOpen_Click(System::Object^  sender, System::EventArgs^  e);
		void btnBleScan_Click(System::Object^  sender, System::EventArgs^  e);
		void btnBleEstablish_Click(System::Object^  sender, System::EventArgs^  e);
		void Single_scan_Click(System::Object^  sender, System::EventArgs^  e);
		void hkoglPanelControl1_Load(System::Object^  sender, System::EventArgs^  e);
		void Full_scan_Click(System::Object^  sender, System::EventArgs^  e);
		void hkoglPanelControl1_Paint(System::Object ^ sender, System::Windows::Forms::PaintEventArgs ^ e);
		void hkoglPanelControl1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e);
		void hkoglPanelControl1_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e);
		void hkoglPanelControl1_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e);
		void hkoglPanelControl1_MouseWheel(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e);
		void hkoglPanelControl1_KeyPress(System::Object^  sender, System::Windows::Forms::KeyPressEventArgs^  e);
		void hkoglPanelControl1_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Test_file_Click(System::Object^  sender, System::EventArgs^  e);
		void Get_Quat_Click(System::Object^  sender, System::EventArgs^  e);
		void Clear_cloud_Click(System::Object^  sender, System::EventArgs^  e);
		void Alignment_Click(System::Object^  sender, System::EventArgs^  e);
		void Save_PointCloud_Click(System::Object^  sender, System::EventArgs^  e);
		void saveFileDialog1_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e);
		void Save_point_s_Click(System::Object^  sender, System::EventArgs^  e);
		void label1_Click(System::Object^  sender, System::EventArgs^  e);
		void textBox1_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void textBox1_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void textBox2_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void textBox2_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void label3_Click(System::Object^  sender, System::EventArgs^  e);
		void read_point_Click(System::Object^  sender, System::EventArgs^  e);
		void Combine_Click(System::Object^  sender, System::EventArgs^  e);
		void Part_Align_Click(System::Object^  sender, System::EventArgs^  e);
		void Theshold_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Theshold_in_KeyDown_1(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Max_time_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Max_time_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Max_angle_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Max_angle_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Sample_points_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Sample_points_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Overlap_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Overlap_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Delta_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Delta_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Transform_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Transform_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Iter_thresh_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Iter_thresh_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Iter_count_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Iter_count_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);

		void Constant1_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Constant2_in_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void Constant1_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void Constant2_in_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);

		void Add2Combine_Click(System::Object^  sender, System::EventArgs^  e);
		void Gyro_test_Click(System::Object^  sender, System::EventArgs^  e);
		void Chose_Align_Click(System::Object^  sender, System::EventArgs^  e);
		void overlap_constant1_Scroll(System::Object^  sender, System::EventArgs^  e);
		void overlap_constant2_Scroll(System::Object^  sender, System::EventArgs^  e);
		void Read_point_v2_Click(System::Object^  sender, System::EventArgs^  e);
		void Rot_test_Click(System::Object^  sender, System::EventArgs^  e);
		void Aligned_rot_Click(System::Object^  sender, System::EventArgs^  e);
		void First_cloud_Click(System::Object^  sender, System::EventArgs^  e);
		void Second_cloud_Click(System::Object^  sender, System::EventArgs^  e);
		void Aligned12_Click(System::Object^  sender, System::EventArgs^  e);
		void next_cloud_Click(System::Object^  sender, System::EventArgs^  e);
		void Aligned_near_Click(System::Object^  sender, System::EventArgs^  e);
		void Aligned_Pre_Click(System::Object^  sender, System::EventArgs^  e);
		void Combine_cloud_Click(System::Object^  sender, System::EventArgs^  e);
		void Save_combine_Click(System::Object^  sender, System::EventArgs^  e);
		void name_output_TextChanged(System::Object^  sender, System::EventArgs^  e);
		void name_output_KeyDown(System::Object^  sender, System::Windows::Forms::KeyEventArgs^  e);
		void numericUpDown1_ValueChanged(System::Object^  sender, System::EventArgs^  e);
		void numericUpDown2_ValueChanged(System::Object^  sender, System::EventArgs^  e);
		void numericUpDown3_ValueChanged(System::Object^  sender, System::EventArgs^  e);
		void Aligned_target_Click(System::Object^  sender, System::EventArgs^  e);
		void Apply_rot_Click(System::Object^  sender, System::EventArgs^  e);
		void numericUpDown4_ValueChanged(System::Object^  sender, System::EventArgs^  e);
		void RotatePC();
		void unsigned_short_to_char(unsigned short *input, int inputlen, char *output);
		void test_start_cap(int32_t DevIn, uint32_t *Handles, float Lv_65,
			uint32_t SampRec, uint32_t *ByteLen, LVBoolean SaveDat, char SaveName[],
			LVBoolean *ErrBool, char ADCStat[], int32_t len, int32_t *len2);
		void test_scanADC(uint32_t Handles, uint32_t ByteBuf, uint16_t ArrSize[],
			int32_t lenArr, uint16_t OutArr[], int32_t lenOut, int32_t *lenOut2,
			char ADCStat[], int32_t lenStat, int32_t *lenStatOut);
		void draw_volumeData(TRcuda *theTRcuda);
		void drawPointType(TRcuda *theTRcuda);
		void Rotate_quat(float angle, float x, float y, float z);
		void drawBoardTemp(TRcuda *theTRcuda);
		void drawPointTypeCloud(TRcuda *theTRcuda);
		void drawZAxisValue(TRcuda *theTRcuda);
		void drawPCSet();
		void MovePC();
		void super4PCS_Align(std::vector<GlobalRegistration::Point3D> PC1, std::vector<GlobalRegistration::Point3D> *PC2, int max_time_seconds);
		void CombinePC(int idx);
		void Find_max_min();
		void Find_Plane();
		void Find_over_maxmin();
		void Push_back_file();
		void read_point_v2(int file_id);
		void reset_rotation(float angle, float x, float y, float z);
		void Rotate2();
		void Rotate_cloud();
		void camera_cube(float length,float width,float height);
		void Find_min_quat();
		void Find_min_quat2();
		void Find_min_quat3();
		void drawPCSet2();
		void drawPCSet3();
		void rotate_quat2();
		void Find_combine_maxmin();
		void draw_before_camera();
		void draw_before_mapping();
		void mani_rotate(float angle, float x, float y, float z);
		void mani_rotate_cloud(int rot_idx);
	private:
		//
		// System Maintenance Variables
		//
		State_Connection mState_Connection;
		State_Processing mState_Processing;


	private:
		//
		// BLE Variables
		//
		BluetoothLeDevice ^bleDeviceL, ^bleDeviceR;
		Dictionary<String^, DeviceInfo^> ^bleDeviceList; // <address, device name>

														 //
														 // Processing Threads
														 //
	private:
		//
		// System Managers
		//
		System::ComponentModel::ComponentResourceManager ^mResources;
		GLManager *glManager;   // OpenGL manager
		SystemManager *sysManager;
		char *ErrorString, *octstatus;

		
		DataManager *DManager;

		unsigned int HandleOut, AllDatabyte, ByteLen;
		LVBoolean ErrorBoolean, Savedata;
		unsigned short *outarr, *RealOctData, *ArrSize, *short_arr, *final_oct_arr;
		char *char_arr, *final_oct_char;
		
		public:
			int32_t ErrorString_len_in, ErrorString_len_out, fftnumber, deviceID, OutArrLenIn, OutArrLenOut;
			TRcuda *theTRcuda;
			bool mouse_move;
			float thePointSize;
			int volumeDataIdx, PointCloud_idx_now, PointCloud_idx_show, readRaw_count, Point_cloud_idx_sec;
			int volumeZ_now, Min_quat_idx, scan_count;

			bool showVolumeData, showPointType, showBoardTemp, showPointCloud, can_rotate, Initial_OK, show_first_point;
			bool show_second_point, enable_review, velocity_offset_enable, showoverlap1, showoverlap2;
			bool change_constant1, change_constant2, gyro_test, show_third_point, show_tmp_align1, show_tmp_align2;
			bool is_camera_move, show_aligned_cloud, is_reset, timer_rot, show_combine_cloud, is_before_camera,show_noGyro_PC, show_rawData;
			clock_t begin_time, end_time;

			//std::vector<std::vector<GlobalRegistration::Point3D>> *pointCloudSet/*, *overlap_cloud1, *overlap_cloud2*/, *pointCloud_aligned;
			//std::vector<GlobalRegistration::Point3D> *finalPC, *aligned, *tmp_alignedPC1, *tmp_alignedPC2;
			std::vector<GlobalRegistration::Point3D> *tmp_over_aligned1, *tmp_over_aligned2;
			//std::vector<int> *finalPCid, *nearest_idx;
			std::vector<file_name> *filename_array;
			//std::vector<Quaternion> *tmp_input_quat;
			//std::vector<Radian> *pointCloud_radian;
			//std::vector<Vector3> *pointCloud_quat_vector;
			
			//std::vector <Vector3> *Point_cloud_max;

			Radian *theta_diff,*output_diff;
			Vector3 *rotationAxix_diff,*output_rotVec;
			Quaternion *quat1, *quat2, *quat3, *all_quat, *quat_tmp, *camera_quat, *all_quat1, *preQuaternL;
			Matrix3 *tmp_rotMatrix;
			glm::mat4 *tmp_MM, *reset_M, *mani_MM;
			
			glm::mat4 *LocalTransform_M;
			GLfloat *float_rotate_M;
			Vector3 *tmp_center/*, *acc_vector1, *acc_vector2, *acc_vector3, *velocity_offset, *movement_vector, *result_movement, *final_max, *final_min, *pre_velocity, *now_velocity*/;
			Vector3 *pre_acc, *now_acc, *draw_plane1, *draw_plane2, *draw_plane3, *draw_plane4;
			Vector3 *combine_max, *combine_min;
			float max_x, max_y, max_z, min_x, min_y, min_z, tmp_peak_gap, tmp_energy_gap, frame_rate;
			double tmp_theshold, super_theshold, tmp_max_time, super_max_time, tmp_max_angle, super_max_angle, tmp_sample_point, super_sample_point;
			double tmp_overlap, super_overlap, tmp_delta, super_delta, tmp_transform, super_transform, tmp_constant1, tmp_constant2;
			double super_constant1, super_constant2, tmp_aligned1_max, tmp_aligned1_min, tmp_aligned2_max, tmp_aligned2_min;
			double tmp_iterThre, super_iterTre, point_z_124, point_z_125;

			float timer_1, timer_2, timer_3;
			clock_t first_t, second_t;
			Point3D::Scalar final_score;
			std::vector<Vector3> /**Point_cloud_Max, *Point_cloud_Min, *Point_cloud_center, *outside_center,*/ *Plane_vector, *input_rotVec;
			//std::vector<Vector3> *cloud_vec_arr;
			std::vector<std::vector<int>> *overlap_idx1, *overlap_idx2;
			std::vector<GlobalRegistration::Match4PCSBase::MatrixType> *translate_mat;
			std::vector<float> *input_dgree;
			//std::vector<int> *near_idx2;
			int file_idx,tmp_iterCount, super_iterCount, Aligned_target_id, show_Cloud_1ID;
			std::vector<PointCloudArray> *PointCloudArr;
			std::vector<PointData> *Combine_cloud_PC;
			std::string *tmp_fileName, *out_fileName;
			//std::vector<Vector3> *overlap1_Max, *overlap1_Min;
			//std::vector<Vector3> *overlap2_Max, *overlap2_Min;
			float full_scan_time, all_time,tmp_rotate_x,tmp_rotate_y,tmp_rotate_z;


			//float theta_diff, rotationAxix_diff_x, rotationAxix_diff_y, rotationAxix_diff_z;
			
		private:

		//Quaternion *preQuaternL, *preQuaternR;


private: System::Void comboBox1_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {
	}

private: System::Void backgroundWorker1_DoWork(System::Object^  sender, System::ComponentModel::DoWorkEventArgs^  e) {
}
private: System::Void Reset_quat_Click(System::Object^  sender, System::EventArgs^  e) {
	//(*test_diff) = Quaternion(0.7f, 0.7f, 0.0f, 0.0f);
	sysManager->GetGloveDataL().Zero();
}

private: System::Void Save_Points2_Click(System::Object^  sender, System::EventArgs^  e) {
	//clock_t tmp_file_name;
	//std::string filename;
	//std::string fileidx;
	//tmp_file_name = clock();
	//int tnp_idx = PointCloud_idx_show;
	//
	//Quaternion test_quat;
	//Radian *test_diff, *quat1_diff;
	//Vector3 *test_rotation, *quat1_rotation;
	//test_diff = new Radian((*pointCloud_radian)[PointCloud_idx_show]);
	//test_rotation = new Vector3((*pointCloud_quat_vector)[PointCloud_idx_show].x,
	//	(*pointCloud_quat_vector)[PointCloud_idx_show].y,
	//	(*pointCloud_quat_vector)[PointCloud_idx_show].z);
	//quat1_diff = new Radian;
	//quat1_rotation = new Vector3;
	////test_quat = sysManager->GetGloveDataL().GetQuaternion();
	////test_quat = test_quat * (*quat1);
	////test_quat = test_quat.Inverse();
	//
	////test_quat.ToAngleAxis(*test_diff, *test_rotation);
	//(*quat1).ToAngleAxis(*quat1_diff, *quat1_rotation);
	//
	//
	//
	//std::vector<GlobalRegistration::Point3D> tmpPC;
	//for (int j = 0; j < (*PointCloudArr)[tnp_idx].mPC.size(); j++) {
	//	GlobalRegistration::Point3D tmp;
	//
	//	tmp.x() = (*PointCloudArr)[tnp_idx][j].x();
	//	tmp.y() = (*PointCloudArr)[tnp_idx][j].y();
	//	tmp.z() = (*PointCloudArr)[tnp_idx][j].z();
	//	tmpPC.push_back(tmp);
	//}
	//
	////std::cout << "test_diff" << (*test_diff).valueDegrees() << std::endl;
	////std::cout << "test_rotation:(" << test_rotation->x << "," << test_rotation->y << "," << test_rotation->z << ")" << std::endl;
	////std::cout << "(*Point_cloud_center)[0]:(" << (*Point_cloud_center)[0].x << "," << (*Point_cloud_center)[0].y << "," << (*Point_cloud_center)[0].z << ")" << std::endl;
	////glTranslatef(-(*Point_cloud_center)[0].x,-(*Point_cloud_center)[0].y, -(*Point_cloud_center)[0].z);
	//std::cout << "(*test_diff).valueDegrees():(" << (*output_diff).valueDegrees() << std::endl;
	//std::cout << "(*test_rotation):(" << (*output_rotVec).x << "," << (*output_rotVec).y << "," << (*output_rotVec).z << ")" << std::endl;
	//
	//std::cout << "tmp_file_name: " << tmp_file_name << ", PointCloud_idx_show:" << tnp_idx << std::endl;
	//MarshalString(tmp_file_name.ToString(), filename);
	//MarshalString(tnp_idx.ToString(), fileidx);
	//
	//std::ofstream  output_file;
	//output_file.open(filename + "__" + fileidx + ".txt");
	////output_file << "1234" << std::endl;
	//
	//output_file << (*quat1_diff).valueDegrees() << " ";
	//output_file << (*quat1_rotation).x << " ";
	//output_file << (*quat1_rotation).y << " ";
	//output_file << (*quat1_rotation).z << " ";
	//output_file << "\n";
	//
	//output_file << (*output_diff).valueDegrees() << " ";
	//output_file << (*output_rotVec).x << " ";
	//output_file << (*output_rotVec).y << " ";
	//output_file << (*output_rotVec).z << " ";
	//output_file << "\n";
	//
	///*output_file << (*quat3).x << " ";
	//output_file << (*quat3).y << " ";
	//output_file << (*quat3).z << " ";
	//output_file << (*quat3).w << " ";
	//output_file << "\n";*/
	//
	//for (int i = 0; i < (*PointCloudArr)[tnp_idx].size(); i++)
	//{
	//	//swap x and y
	//	output_file << tmpPC[i].x() << " ";
	//	output_file << tmpPC[i].y() << " ";
	//	output_file << tmpPC[i].z() << " ";
	//	output_file << "0 0";
	//	output_file << "\n";
	//}
	//output_file.close();
	////saveFileDialog1->ShowDialog();
	//std::cout << "Save_OK" << std::endl;
}

private: System::Void reset_rot_Click(System::Object^  sender, System::EventArgs^  e) {

	is_reset = true;
}
private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
	timer_rot = true;


	//std::cout << "timer:" << timer_rot << "\n";
	hkoglPanelControl1->Invalidate();
}










private: System::Void domainUpDown1_SelectedItemChanged(System::Object^  sender, System::EventArgs^  e) {
}


private: System::Void label7_Click(System::Object^  sender, System::EventArgs^  e) {
}


};


}
