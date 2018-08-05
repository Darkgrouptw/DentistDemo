#pragma once
#include "GloveData.h"
//#include "DataHandleManager.h"
//#include "DatabaseManager.h"

class SystemManager
{
public:
    //
    // Constructors
    //
    SystemManager()
    {
       // mRecordManager = new RecordManager();
        //mPlaybackManager = new PlaybackManager();
        //mDatabaseManager = new DatabaseManager();
    }

    ~SystemManager()
    {
        //delete mRecordManager;
        //delete mPlaybackManager;
        //delete mDatabaseManager;
    }

    //
    // Getters and Setters
    //
    //RecordManager	*GetRecordManager()		{ return mRecordManager; }
    //PlaybackManager *GetPlaybackManager()	{ return mPlaybackManager; }
    //DatabaseManager *GetDatabaseManager()	{ return mDatabaseManager; }

    GloveData GetGloveDataL() { return mCurGloveDataL; }
    GloveData GetGloveDataR() { return mCurGloveDataR; }
    GloveData GetGloveData(int side)
    {
        if (side == HAND_L)
            return mCurGloveDataL;
        else if (side == HAND_R)
            return mCurGloveDataR;
        else
            return GloveData::INVALID;
    }

    GloveData *GetGloveDataLPtr() { return &mCurGloveDataL; }
    GloveData *GetGloveDataRPtr() { return &mCurGloveDataR; }
    GloveData *GetGloveDataPtr(int side)
    {
        if (side == HAND_L)
            return &mCurGloveDataL;
        else if (side == HAND_R)
            return &mCurGloveDataR;
        else
            return NULL;
    }

    bool SetGloveDataL(GloveData &data) { mCurGloveDataL = data; return true; }
    bool SetGloveDataR(GloveData &data) { mCurGloveDataR = data; return true; }
    bool SetGloveData(GloveData &data, int side)
    {
        if (side == HAND_L)
            mCurGloveDataL = data;
        else if (side == HAND_R)
            mCurGloveDataR = data;
        else
            return false;
        return true;
    }

private:
    //
    // Other Functional Managers
    //
    //RecordManager *mRecordManager;
    //PlaybackManager *mPlaybackManager;
    //DatabaseManager *mDatabaseManager;

    //
    // Class Variables
    //
    GloveData mCurGloveDataL, mCurGloveDataR;

};
