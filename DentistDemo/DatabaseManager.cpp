#include "DatabaseManager.h"

void DatabaseManager::LoadRecordFilenames(std::vector<std::string> &records, bool ext = true)
{
    // Get path
    boost::filesystem::path dir(PATH_RECORD);
    
    // Get every file under the path
    for (boost::filesystem::directory_iterator it = boost::filesystem::directory_iterator(dir);
        it != boost::filesystem::directory_iterator(); ++it)
    {
        // Get filepath
        boost::filesystem::path recrod = it->path();

		// Get csv files only
		if (recrod.extension().string().compare(".csv") != 0)
			continue;

        // Extract filename
        std::string filename = recrod.filename().string();
        
        // Get filename w/o file extension
        if (!ext)
        {
            // Find the first dot of the filename (name.ext)
            size_t dotfound = filename.find(".");
            // Get name and push to vector
            records.push_back(filename.substr(0, dotfound));
        }
        else
        {
            records.push_back(filename);
        }
    }
}

bool DatabaseManager::LoadDatabase()
{
    // Load sign clip files
    mSignClip_ori.LoadClip(PATH_INTEGRATE + FILENAME_ORI);
    mSignClip_integrated_all.LoadClip(PATH_INTEGRATE + FILENAME_INTEGRATED_ALL);
    mSignClip_integrated_quat.LoadClip(PATH_INTEGRATE + FILENAME_INTEGRATED_QUAT);
    mSignClip_integrated_acce.LoadClip(PATH_INTEGRATE + FILENAME_INTEGRATED_ACCE);
    mSignClip_integrated_flex.LoadClip(PATH_INTEGRATE + FILENAME_INTEGRATED_FLEX);
    // TODO TOUCH
    // Load Touch data if needed

    LoadSignClipNames();
    LoadSignClipLengths();

	return false;
}

void DatabaseManager::Integrate()
{
	// Start the integration process,
	// In this process, each SignClip (mSignClip), Sign file names (mSignFileNames)
	// and sign clip length (mSignClipLength) will be created, assigned and saves to database.
	
    // Clear black window
    system("cls");

	// Clear previous database
	ClearDatabase();

    std::cout << "Data integration started..." << std::endl;

    // File stream for read record
    std::fstream fs_clip;

    // Load existing record files
    LoadRecordFilenames(mSignFileNames);

    // Iterate through each record
    for (unsigned int i = 0; i < mSignFileNames.size(); i++)
    {
		// Meaning of i: SignClip index, used for SignFrames to know which SignClip belong to

        // Full file path
        std::string filename = PATH_RECORD + mSignFileNames[i];
        // Open file for read
        fs_clip.open(filename, std::fstream::in);

        // Check if file opened, should be exist normally...
        if (!fs_clip.is_open())
        {
            std::cout << "<DatabaseManager::Integrate> File does not exist: " << filename << std::endl;
            return;
        }

        // File opened successfully, start process integration
        std::cout << "Processing file: " << filename << std::endl;

        // Create and load SignClip from file
        SignClip signClip;
        signClip.LoadClip(filename);

        // Get each frame
		SignFrame tmpSignFrame;
        for (unsigned int j = 0; j < signClip.Length(); j++)
        {
			signClip.GetSignFrame(j, tmpSignFrame);
			// Assign sign clip index as sign frame index
			tmpSignFrame.SetIndex(i);
			// Add self index to same index vector
			int self_idx = mSignClip_ori.Length();
			tmpSignFrame.AddSameIndex(self_idx);

			// Meaning of self_idx: the index of tmpSignFrame in mSignClip_ori (the overall merged SignFrames)

			// Original SignFrames w/o searching equal frames
			mSignClip_ori.AddSignFrame(tmpSignFrame);

			// Integrated SignFrames with searching equal frames
			mSignClip_integrated_all.AddSignFrame(tmpSignFrame, true, (int)DataType::TYPE_ALL);

			// Integrated quaternion SignFrames with searching equal frames
			SignFrame quatSignFrame(i, tmpSignFrame.GetGloveDataL().GetQuaternion(), tmpSignFrame.GetGloveDataR().GetQuaternion());
			quatSignFrame.AddSameIndex(self_idx);
			mSignClip_integrated_quat.AddSignFrame(quatSignFrame, true, (int)DataType::TYPE_QUAT);

			// Integrated acceleration SignFrames with searching equal frames
			SignFrame acceSignFrame(i, tmpSignFrame.GetGloveDataL().GetAcceleration(), tmpSignFrame.GetGloveDataR().GetAcceleration());
			acceSignFrame.AddSameIndex(self_idx);
			mSignClip_integrated_acce.AddSignFrame(acceSignFrame, true, (int)DataType::TYPE_ACCE);

			// Integrated flex SignFrames with searching equal frames
			SignFrame flexSignFrame(i, tmpSignFrame.GetGloveDataL().GetFlexFiltered(), tmpSignFrame.GetGloveDataR().GetFlexFiltered(), (int)DataType::TYPE_FLEX);
			flexSignFrame.AddSameIndex(self_idx);
			mSignClip_integrated_flex.AddSignFrame(flexSignFrame, true, (int)DataType::TYPE_FLEX);

			// Integrated touch SignFrames with searching equal frames
			// TODO TOUCH
        }

		// Add length of this clip to mSignClipLength
		mSignClipLength.push_back(signClip.Length());
		std::cout << "Motion Length: " << mSignFileNames[i] << " " << signClip.Length() << std::endl;

		// Close reading file
		fs_clip.close();
    }

    // Check saving directory (PATH_INTEGRATE)
    boost::filesystem::path path_integrate(PATH_INTEGRATE);
    if (!boost::filesystem::exists(path_integrate))
    {
        if (!boost::filesystem::create_directory(path_integrate))
        {
            std::cout << "Database directory create failed." << std::endl;
            return;
        }
    }

	// Save SignClips
	mSignClip_ori.SaveClip(PATH_INTEGRATE + FILENAME_ORI);
	mSignClip_integrated_all.SaveClip(PATH_INTEGRATE + FILENAME_INTEGRATED_ALL);
	mSignClip_integrated_quat.SaveClip(PATH_INTEGRATE + FILENAME_INTEGRATED_QUAT);
	mSignClip_integrated_acce.SaveClip(PATH_INTEGRATE + FILENAME_INTEGRATED_ACCE);
	mSignClip_integrated_flex.SaveClip(PATH_INTEGRATE + FILENAME_INTEGRATED_FLEX);
	// TODO TOUCH

	SaveSignClipNames();
	SaveSignClipLengths();

    std::cout << "Data integration finished..." << std::endl;
}

void DatabaseManager::LoadSignClipNames()
{
    // Clear vector
    mSignFileNames.swap(std::vector<std::string>());

    // Open file to read
    std::fstream fs(PATH_INTEGRATE + FILENAME_SIGNNAME, std::fstream::in);
    // String buffer
    std::string str_buf;

    // Check file open state
    if (!fs.is_open())
    {
        std::cout << "<DatabaseManager::LoadClipNames> File does not exist." << std::endl;
        return;
    }

    // Read each line
    while (!fs.eof())
    {
        // Get line
        std::getline(fs, str_buf);

        // Skip zero length, usually for last line
        if (str_buf.length() == 0) continue;

        // Push name to vector
        mSignFileNames.push_back(str_buf);
    }
}

void DatabaseManager::LoadSignClipLengths()
{
    // Clear vector
    mSignClipLength.swap(std::vector<unsigned int>());
    mSignClipLengthSum.swap(std::vector<unsigned int>());

    // Reset values
    mMaxSignClipLength = 0;
    int sumLen = 0;

    // Open file to read
    std::fstream fs(PATH_INTEGRATE + FILENAME_SIGNLENGTH, std::fstream::in);
    // String buffer
    std::string str_buf;

    // Check file open state
    if (!fs.is_open())
    {
        std::cout << "<DatabaseManager::LoadClipLengths> File does not exist." << std::endl;
        return;
    }

    // Read each line
    while (!fs.eof())
    {
        // Get line
        std::getline(fs, str_buf);

        // Skip zero length, usually for last line
        if (str_buf.length() == 0) continue;

        // Get number and push to vector
        unsigned int length = stoi(str_buf);
        mSignClipLength.push_back(length);

        // Get max length
        if (mMaxSignClipLength < length)
            mMaxSignClipLength = length;
        
        // Get sum length
        sumLen += length;
        mSignClipLengthSum.push_back(sumLen);
    }
}

void DatabaseManager::SaveSignClipNames()
{
	// Filename for this operation
	std::string filename = PATH_INTEGRATE + FILENAME_SIGNNAME;

	// Remove previous file if exists
	boost::filesystem::remove(boost::filesystem::path(filename));

	// Open file for writing
	std::fstream fs(filename, std::fstream::out);

	if (!fs.is_open())
	{
		std::cout << "<DatabaseManager::SaveClipNames> File opened failed: " << filename << std::endl;
		return;
	}

	// Start writing
	for each (std::string name in mSignFileNames)
	{
		fs << name << std::endl;
	}
}

void DatabaseManager::SaveSignClipLengths()
{
	// Filename for this operation
	std::string filename = PATH_INTEGRATE + FILENAME_SIGNLENGTH;

	// Remove previous file if exists
	boost::filesystem::remove(boost::filesystem::path(filename));

	// Open file for writing
	std::fstream fs(filename, std::fstream::out);

	if (!fs.is_open())
	{
		std::cout << "<DatabaseManager::SaveSignClipLengths> File opened failed: " << filename << std::endl;
		return;
	}

	// Start writing
	for each (unsigned int length in mSignClipLength)
	{
		fs << length << std::endl;
	}
}

const std::string DatabaseManager::PATH_RECORD		= ".\\Database\\Records\\";
const std::string DatabaseManager::PATH_INTEGRATE	= ".\\Database\\Integrate\\";

const std::string DatabaseManager::FILENAME_ORI	             = "OriSignData.csv";
const std::string DatabaseManager::FILENAME_INTEGRATED_ALL	 = "SignData.csv";
const std::string DatabaseManager::FILENAME_INTEGRATED_QUAT  = "SignQuaterData.csv";
const std::string DatabaseManager::FILENAME_INTEGRATED_ACCE  = "SignAcceData.csv";
const std::string DatabaseManager::FILENAME_INTEGRATED_FLEX  = "SignFingerData.csv";
const std::string DatabaseManager::FILENAME_INTEGRATED_TOUCH = "SignTouchData.csv";

const std::string DatabaseManager::FILENAME_SIGNNAME = "SignName.csv";
const std::string DatabaseManager::FILENAME_SIGNLENGTH = "SignMotionLength.csv";