#pragma once

#include "TRcuda.cuh"
#include "DotNetUtilities.h"
#include <Windows.h>
#include "DevUtils.h"
#include "GLManager.h"
#include "SystemManager.h"
#include "OCT64C.h"

#include "4pcs.h"
#include "super4pcs/shared4pcs.h"

#include "Eigen/Dense"
#include "DataManager.h"

#include <iostream>
#include <fstream>
#include "super4pcs/utils/geometry.h"
#include "super4pcs.h"
#include <bitset>
#include <string>
//#include <msclr/marshal.h>
//#define PIC_SIZE PIC_SIZE

#include "super4pcs/io/io.h"
#include "SegnetModel.h"

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
struct objData {
	std::vector<glm::vec3> *out_vertices;
	std::vector<glm::vec2> *out_uvs;
	std::vector<glm::vec3> *out_normals;
	std::vector<unsigned int> *out_materialIndices;
	std::vector<std::string> *out_mtls;
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

namespace DentistDemo 
{
	using namespace System;
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
		// TestModel
		SegnetModel* NetworkModel;

		MyForm(void)
		{
			InitializeComponent();

			// 模型建置
			NetworkModel = new SegnetModel();
			NetworkModel->Load("./Models/segnet_inference.prototxt", "./Models/segnet_iter_40000.caffemodel");
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
	private: System::Windows::Forms::ComboBox^  cbBleDeviceL;
	private: System::Windows::Forms::Button^  btnBleScan;
	private: System::Windows::Forms::Button^  btnComOpen;
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
	private: System::Windows::Forms::Button^  Aligned_rot;
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
	private: System::Windows::Forms::Button^  Test_Model;
	private: System::ComponentModel::IContainer^  components;

	private:
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
			this->btnBleEstablish = (gcnew System::Windows::Forms::Button());
			this->Get_Quat = (gcnew System::Windows::Forms::Button());
			this->Full_scan = (gcnew System::Windows::Forms::Button());
			this->hkoglPanelControl1 = (gcnew HKOGLPanel::HKOGLPanelControl());
			this->Clear_cloud = (gcnew System::Windows::Forms::Button());
			this->saveFileDialog1 = (gcnew System::Windows::Forms::SaveFileDialog());
			this->Aligned12 = (gcnew System::Windows::Forms::Button());
			this->Aligned_near = (gcnew System::Windows::Forms::Button());
			this->reset_rot = (gcnew System::Windows::Forms::Button());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->Aligned_Pre = (gcnew System::Windows::Forms::Button());
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
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->pictureBox6 = (gcnew System::Windows::Forms::PictureBox());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->Aligned_target = (gcnew System::Windows::Forms::Button());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox3))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox4))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox5))->BeginInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox6))->BeginInit();
			this->SuspendLayout();
			// 
			// cbComPortL
			// 
			this->cbComPortL->FormattingEnabled = true;
			this->cbComPortL->Location = System::Drawing::Point(12, 12);
			this->cbComPortL->Name = L"cbComPortL";
			this->cbComPortL->Size = System::Drawing::Size(121, 20);
			this->cbComPortL->TabIndex = 0;
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
			// Full_scan
			// 
			this->Full_scan->Location = System::Drawing::Point(755, 110);
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
			// saveFileDialog1
			// 
			this->saveFileDialog1->DefaultExt = L"txt";
			this->saveFileDialog1->RestoreDirectory = true;
			this->saveFileDialog1->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &MyForm::saveFileDialog1_FileOk);
			// 
			// Aligned12
			// 
			this->Aligned12->Location = System::Drawing::Point(755, 153);
			this->Aligned12->Name = L"Aligned12";
			this->Aligned12->Size = System::Drawing::Size(75, 23);
			this->Aligned12->TabIndex = 53;
			this->Aligned12->Text = L"Aligned to1";
			this->Aligned12->UseVisualStyleBackColor = true;
			this->Aligned12->Click += gcnew System::EventHandler(this, &MyForm::Aligned12_Click);
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
			this->output_name->Location = System::Drawing::Point(778, 62);
			this->output_name->Name = L"output_name";
			this->output_name->Size = System::Drawing::Size(32, 12);
			this->output_name->TabIndex = 64;
			this->output_name->Text = L"Name";
			// 
			// name_output
			// 
			this->name_output->Location = System::Drawing::Point(743, 77);
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
			this->ClientSize = System::Drawing::Size(870, 614);
			this->Controls->Add(this->Aligned_target);
			this->Controls->Add(this->label13);
			this->Controls->Add(this->label12);
			this->Controls->Add(this->label8);
			this->Controls->Add(this->label7);
			this->Controls->Add(this->label6);
			this->Controls->Add(this->name_output);
			this->Controls->Add(this->output_name);
			this->Controls->Add(this->Aligned12);
			this->Controls->Add(this->Clear_cloud);
			this->Controls->Add(this->hkoglPanelControl1);
			this->Controls->Add(this->Full_scan);
			this->Controls->Add(this->Get_Quat);
			this->Controls->Add(this->btnBleEstablish);
			this->Controls->Add(this->btnComOpen);
			this->Controls->Add(this->btnBleScan);
			this->Controls->Add(this->cbBleDeviceL);
			this->Controls->Add(this->btnComUpdate);
			this->Controls->Add(this->cbComPortL);
			this->Name = L"MyForm";
			this->Text = L"MyForm";
			this->Load += gcnew System::EventHandler(this, &MyForm::MyForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox2))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox3))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox5))->EndInit();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox6))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

			#pragma region Dark Add Part
			// Test_Model
			// 
			this->Test_Model = (gcnew System::Windows::Forms::Button());
			this->Test_Model->Location = System::Drawing::Point(755, 196);
			this->Test_Model->Name = L"Test_Model";
			this->Test_Model->Size = System::Drawing::Size(75, 23);
			this->Test_Model->TabIndex = 66;
			this->Test_Model->Text = L"Test Model";
			this->Test_Model->UseVisualStyleBackColor = true;
			this->Test_Model->Click += gcnew System::EventHandler(this, &MyForm::Test_Model_Click);
			this->Controls->Add(this->Test_Model);

			//////////////////////////////////////////////////////////////////////////
			// Model Init
			//////////////////////////////////////////////////////////////////////////

			#pragma endregion
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
		void Callback_DeviceInitDone(System::String ^address);
		void Callback_DeviceCloseDone(System::String ^comport);
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
		void Save_PointCloud_Click(System::Object^  sender, System::EventArgs^  e);
		void saveFileDialog1_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e);
		void Save_point_s_Click(System::Object^  sender, System::EventArgs^  e);
		void read_point_Click(System::Object^  sender, System::EventArgs^  e);
		void Add2Combine_Click(System::Object^  sender, System::EventArgs^  e);
		void Gyro_test_Click(System::Object^  sender, System::EventArgs^  e);
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
		void Aligned_target_Click(System::Object^  sender, System::EventArgs^  e);
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
		void super4PCS_Align(std::vector<GlobalRegistration::Point3D> PC1, std::vector<GlobalRegistration::Point3D> *PC2, int max_time_seconds);
		void Find_max_min();
		void Push_back_file();
		void read_point_v2(int file_id);
		void reset_rotation(float angle, float x, float y, float z);
		void Rotate_cloud();
		void camera_cube(float length, float width, float height);
		void Find_min_quat2();
		void Find_min_quat3();
		void drawPCSet3();
		void rotate_quat2();
		void Find_combine_maxmin();
		void draw_before_camera();
		void draw_before_mapping();
		void mani_rotate(float angle, float x, float y, float z);
		void mani_rotate_cloud(int rot_idx);
		void load_obj();
		void load_result();
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
		Dictionary<System::String^, DeviceInfo^> ^bleDeviceList; // <address, device name>

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
		bool is_camera_move, show_aligned_cloud, is_reset, timer_rot, show_combine_cloud, is_before_camera, show_noGyro_PC, show_rawData;
		clock_t begin_time, end_time;
		std::vector<GlobalRegistration::Point3D> *tmp_over_aligned1, *tmp_over_aligned2;
		std::vector<file_name> *filename_array;
		Radian *theta_diff, *output_diff;
		Vector3 *rotationAxix_diff, *output_rotVec;
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
		int file_idx, tmp_iterCount, super_iterCount, Aligned_target_id, show_Cloud_1ID;
		std::vector<PointCloudArray> *PointCloudArr;
		std::vector<PointData> *Combine_cloud_PC;
		std::string *tmp_fileName, *out_fileName;
		float full_scan_time, all_time, tmp_rotate_x, tmp_rotate_y, tmp_rotate_z;
		std::vector<cv::Mat> *result_input;

		objData *obj1, *obj2, *obj3, *obj4, *obj5;

	private:

		//Quaternion *preQuaternL, *preQuaternR;
		private: System::Void reset_rot_Click(System::Object^  sender, System::EventArgs^  e) {

			is_reset = true;
		}

		private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
			timer_rot = true;


			//std::cout << "timer:" << timer_rot << "\n";
			hkoglPanelControl1->Invalidate();
		}

		// Test
		int indexA = 0;
		private: System::Void Test_Model_Click(System::Object^  sender, System::EventArgs^  e)
		{
			Mat img;
			if (indexA == 0)
				img = imread("./95.png");
			else if (indexA == 1)
				img = imread("./76_reverse.png");
			indexA = (indexA + 1) % 2;

			img = NetworkModel->Predict(img);

			cv::imshow("Display window", img);
			cv::waitKey(0);
		}
	};
}