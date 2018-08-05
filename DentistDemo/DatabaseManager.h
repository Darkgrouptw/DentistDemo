#pragma once

#include <vector>

#include "SignClip.h"

class DatabaseManager
{
public:
    DatabaseManager()
    {
		std::cout << "Loading database..." << std::endl;
		LoadDatabase();
		std::cout << "Merged database size:             " << mSignClip_ori.Length() << std::endl;
        std::cout << "Integrated sign database size:    " << mSignClip_integrated_all.Length() << std::endl;
        std::cout << "Integrated quat database size:    " << mSignClip_integrated_quat.Length() << std::endl;
        std::cout << "Integrated acce database size:    " << mSignClip_integrated_acce.Length() << std::endl;
        std::cout << "Integrated flex database size:    " << mSignClip_integrated_flex.Length() << std::endl;
        std::cout << "Integrated touch database size:   " << mSignClip_integrated_touch.Length() << std::endl;
        std::cout << "Database loaded" << std::endl;
    }

    void LoadRecordFilenames(std::vector<std::string> &records, bool ext);
	
	bool LoadDatabase();

    void Integrate();

private:
    
    void LoadSignClipNames();
    void LoadSignClipLengths();

	void SaveSignClipNames();
	void SaveSignClipLengths();

    void ClearDatabase()
    {
        mSignClip_ori.Clear();
        mSignClip_integrated_all.Clear();
        mSignClip_integrated_quat.Clear();
        mSignClip_integrated_acce.Clear();
        mSignClip_integrated_flex.Clear();
        mSignClip_integrated_touch.Clear();
        
		mSignFileNames.swap(std::vector<std::string>());
        mSignClipLength.swap(std::vector<unsigned int>());
        mSignClipLengthSum.swap(std::vector<unsigned int>());
    }

public:
    //
    // Sign Motion Clips
    //
    SignClip mSignClip_ori;
    SignClip mSignClip_integrated_all;
    SignClip mSignClip_integrated_quat;
    SignClip mSignClip_integrated_acce;
    SignClip mSignClip_integrated_flex;
    SignClip mSignClip_integrated_touch;

    std::vector<std::string> mSignFileNames;
    unsigned int mMaxSignClipLength;
    std::vector<unsigned int> mSignClipLength;
    std::vector<unsigned int> mSignClipLengthSum;

public:
    static const std::string PATH_RECORD;
    static const std::string PATH_INTEGRATE;
    
	static const std::string FILENAME_ORI;
	static const std::string FILENAME_INTEGRATED_ALL;
	static const std::string FILENAME_INTEGRATED_QUAT;
	static const std::string FILENAME_INTEGRATED_ACCE;
	static const std::string FILENAME_INTEGRATED_FLEX;
	static const std::string FILENAME_INTEGRATED_TOUCH;

    static const std::string FILENAME_SIGNNAME;
    static const std::string FILENAME_SIGNLENGTH;
};