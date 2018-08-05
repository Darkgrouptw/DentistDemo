#pragma once

#include <array>
#include <memory>
#include <limits>
#include <string>
#include <sstream>
#include <Math/Quaternion.h>
#include <Math/Vector3.h>
#include "KalmanFilter.h"

#define NUM_Q   4
#define NUM_A   3
#define NUM_F   10
#define NUM_T   10

#define NUM_FEATURE (NUM_Q + NUM_A + NUM_F)

#define HAND_L  0
#define HAND_R  1
#define HAND_SIZE 2

enum class DataType
{
	TYPE_NONE, TYPE_ALL, TYPE_QUAT, TYPE_ACCE, TYPE_FLEX, TYPE_TOUCH, TYPE_NUM
};

class GloveData
{
public:
    //
    // Finger Indices
    //
    static const int Thumb_Top  = 0;
    static const int Thumb_Btm  = 1;
    static const int Index_Top  = 2;
    static const int Index_Btm  = 3;
    static const int Middle_Top = 4;
    static const int Middle_Btm = 5;
    static const int Ring_Top   = 6;
    static const int Ring_Btm   = 7;
    static const int Pinky_Top  = 8;
    static const int Pinky_Btm  = 9;
    static const int Finger_Size = 10;
    
private:
    //
    // Sensor Values
    //
    Quaternion               quat;          // Orientation of the hand
    Vector3                  acce;          // Acceleration of the hand
    std::array<float, NUM_F> flex;          // Finger flex raw angle
    std::array<float, NUM_F> touch;         // Finger touch force

    std::array<float, NUM_F> flexMax;       // Finger flex max angle
    std::array<float, NUM_F> flexFiltered;  // Finger normalized and filtered angle

    bool    mIsValid;                       // Whether this data is valid

    std::array<KalmanFilter, NUM_F> flexFilter = {}; // Kalman filter for filtering finger flex

public:
    //
    // Constructor
    //

	// Default constructor with Kalman filter
    GloveData()
        : mIsValid(true)
    {
        Zero();

       /* for (int i = 0; i < NUM_F; i++)
        {
            flexMax[i] = std::numeric_limits<float>::min();
            flexFilter[i] = KalmanFilter(0.05, 0.1);
        }*/
    }

    // Assign values for all features
    GloveData(Quaternion _q, Vector3 _a, std::array<float, NUM_F> _f, std::array<float, NUM_F> _t)
        : quat(_q), acce(_a), flexFiltered(_f), touch(_t), mIsValid(true) 
    {
		
	}

	// Assign values for only quaternion feature
	GloveData(Quaternion _q)
		: quat(_q), acce(Vector3(0, 0, 0))
		, flexFiltered(std::array<float, NUM_F>())
		, touch(std::array<float, NUM_F>()), mIsValid(true)
	{

	}

	// Assign values for only acceleration feature
	GloveData(Vector3 _a)
		: quat(Quaternion(0,0,0,0)), acce(_a)
		, flexFiltered(std::array<float, NUM_F>())
		, touch(std::array<float, NUM_F>()), mIsValid(true)
	{

	}

	// Assign values for only flex or touch feature
	GloveData(std::array<float, NUM_F> _array, int type)
		: quat(Quaternion(0, 0, 0, 0)), acce(Vector3(0, 0, 0)), mIsValid(true)
	{
		if (type == (int)DataType::TYPE_FLEX)
		{
			flexFiltered = _array;
			touch = std::array<float, NUM_F>();
		}
		else if (type == (int)DataType::TYPE_TOUCH)
		{
			flexFiltered = std::array<float, NUM_F>();
			touch = _array;
		}
		else
		{
			flexFiltered = std::array<float, NUM_F>();
			touch = std::array<float, NUM_F>();
		}
	}

	~GloveData()
	{
	}

    bool Update()
    {
		/*for (int i = 0; i < NUM_F; i++)
		{
			flexMax[i] = std::max(flex[i], flexMax[i]);
			flexFiltered[i] = (float)flexFilter[i].Update((flex[i] / flexMax[i]) * 90.0f);
		}*/
		// Update succeeded
		return true;
    }

    static const GloveData INVALID;

public:
    //
    // Getters and setters
    //
    Quaternion               GetQuaternion()                 { return this->quat; }
    void                     SetQuaternion(Quaternion _quat) { this->quat = _quat; }

    Vector3                  GetAcceleration()              { return this->acce; }
    void                     SetAcceleration(Vector3 _acce) { this->acce = _acce; }

    std::array<float, NUM_F> GetFlexRaw()                               { return this->flex; }
    void                     SetFlexRaw(std::array<float, NUM_F> _flex) { this->flex = _flex; }
    float                    GetFlexRawAt(int i)                        { return this->flex[i]; }
    void                     SetFlexRawAt(int i, float v)               { this->flex[i] = v; }

    std::array<float, NUM_F> GetFlexFiltered()                               { return this->flexFiltered; }
    void                     SetFlexFiltered(std::array<float, NUM_F> _flex) { this->flexFiltered = _flex; }
    float                    GetFlexFilteredAt(int i)                        { return this->flexFiltered[i]; }
    void                     SetFlexFilteredAt(int i, float v)               { this->flexFiltered[i] = v; }

    std::array<float, NUM_F> GetTouch()                                 { return this->touch; }
    void                     SetTouch(std::array<float, NUM_F> _touch)  { this->touch = touch; }
    float                    GetTouchAt(int i)                          { return this->touch[i]; }
    void                     SetTouchAt(int i, float v)                 { this->touch[i] = v; }

private:
    GloveData(bool flag) : mIsValid(flag)
    {}

public:
    //
    // Utility Functions
    //
    void Zero();                        // Zero all values
    void ZeroQuat();                    // Zero hand orientation
    void ZeroAcce();                    // Zero hand acceleration
    void ZeroFlex();                    // Zero finger flex angle
    void ZeroTouch();                   // Zero finger touch

    bool IsValid() { return mIsValid; }

    //
    // ToString Functions
    //
	std::string GloveData::ToCsvString(int type);	// GloveData to std::string
    std::string ToCsvString_Quat();					// Quaternion to std::string 
    std::string ToCsvString_Acce();					// Acceleration to std::string
    std::string ToCsvString_Flex();					// Finger flex to std::string
    std::string ToCsvString_Touch();				// Finger touch to std::string
    std::string ToCsvString_All();					// All features in one
};