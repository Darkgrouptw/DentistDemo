#include "DataHandleManager.h"

bool RecordManager::Create(std::string filename)
{
    mFstream.open(filename, std::fstream::out);
    if (mFstream.is_open())
    {
        mIsRecording = true;
        mCounter = 0;
        return true;
    }
    else
    {
        return false;
    }
}

int RecordManager::Record(GloveData &dataL, GloveData &dataR)
{
    mFstream << mCounter << "," + dataL.ToCsvString_All() + "," + dataR.ToCsvString_All() << std::endl;
    return ++mCounter;
}

int RecordManager::Stop()
{
    if (mFstream.is_open())
    {
        mFstream.close();
        mIsRecording = false;
        return mCounter;
    }
    else
    {
        return -1;
    }
}

bool PlaybackManager::Prepare(std::string filename)
{
    std::cout << "Prepare file for playback: " << filename << std::endl;
	if (mSignClip.LoadClip(filename))
	{
		std::cout << "Playback preparation done, " << mSignClip.Length() << " frames loaded" << std::endl;
	}
	else
	{
		std::cout << "Playback preparation failed." << std::endl;
		return false;
	}
	
    if (mSignClip.Length() > 0)
        mIndex = 0;

    return true;
}

bool PlaybackManager::Retrieve(SignFrame & signFrame)
{
    if (mIndex >= mSignClip.Length())
        return false;

	mSignClip.GetSignFrame(mIndex++, signFrame);
    
    return true;
}

bool PlaybackManager::Retrieve(GloveData & dataL, GloveData & dataR)
{
    SignFrame signFrame;
    bool result = Retrieve(signFrame);

    if (result == false) return result;

    dataL = signFrame.GetGloveDataL();
    dataR = signFrame.GetGloveDataR();

    return true;
}