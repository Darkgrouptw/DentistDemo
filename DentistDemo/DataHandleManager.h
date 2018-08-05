#pragma once

#include <fstream>
#include "SignClip.h"

class RecordManager
{
public:
    //
    // Constructors
    //
    RecordManager()
    {

    }

    //
    // Record Functions
    //
    bool Create(std::string filename);
    int Record(GloveData &dataL, GloveData &dataR);
    int Stop();
    bool IsRecording()
    {
        return mIsRecording;
    }

private: 
    //
    // Variables
    //
    bool mIsRecording;
    std::fstream mFstream;
    int mCounter;
};

class PlaybackManager
{
public:
    //
    // Constructors
    //
    PlaybackManager()
    {
        // Create default buffer 
		mSignClip = SignClip();

        // Set -1 for no frames
        mIndex = -1;
    }

    //
    // Playback Functions
    //
    bool Prepare(std::string filename);     // Read data as SignFrames into buffer
    bool Retrieve(SignFrame &signFrame);    // Retrieve current frame and add up index
    bool Retrieve(GloveData &dataL, GloveData &dataR);

    unsigned int GetFrameSize() { return mSignClip.Length(); }
    bool SetFrameIndex(unsigned int _index)
    {
        if (_index >= mSignClip.Length()) return false; // greater than frame size

        mIndex = _index;
        return true;
    }

    SignClip GetSignClip()
    {
        return mSignClip;
    }

private:

    unsigned int mIndex;
    SignClip mSignClip;

};
