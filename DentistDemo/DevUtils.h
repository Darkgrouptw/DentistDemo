#pragma once

#include <Math/Quaternion.h>
#include <Math/Vector3.h>
#include <array>
#include "GloveData.h"

using namespace System;

public class DevUtils
{
public:
    static String^ LoadAddress(array<unsigned char>^ addr_uc)
    {
        String^ addr = String::Empty;

        for (int i = addr_uc->Length-1; i >= 0; i--)
        {
            addr += String::Format("{0:X2}", addr_uc[i]);
            if (i > 0)
                addr += ":";
        }

        return addr;
    }

    static String^ ReverseAddress(String^ addr)
    {
        String^ addr_rev = String::Empty;

        array<String^>^ addr_s = addr->Split(':');
        for (int i = addr_s->Length-1; i >= 0; i--)
        {
            addr_rev += addr_s[i];

            if (i > 0) addr_rev += ":";
        }

        return addr_rev;
    }

    // Convert array<unsigned char>^ to String^ with given splitter (Default " ")
    static String^ Bytes2String(array<unsigned char>^ data, String^ splitter = " ")
    {
        String^ result = String::Empty;

        for (int i = 0; i < data->Length; i++)
        {
            result += String::Format("{0:X2}", data[i]);
            if (i < data->Length - 1)
                result += splitter;
        }

        return result;
    }

    static unsigned char GetPacketIndex(array<unsigned char>^ data)
    {
        unsigned char index = (data[0] >> 4) & 0x0F;

        return index;
    }

    static unsigned char GetPacketType(array<unsigned char>^ data)
    {
        unsigned char type = data[0] & 0x0F;

        return type;
    }

    static Quaternion GetQuaternion(array<unsigned char>^ data, int startIndex)
    {
        Quaternion quat = Quaternion();

        quat.w = BitConverter::ToSingle(data, startIndex);
        quat.x = BitConverter::ToSingle(data, startIndex + 4);
        quat.y = BitConverter::ToSingle(data, startIndex + 8);
        quat.z = BitConverter::ToSingle(data, startIndex + 12);

        return quat;
    }

    static Vector3 GetAcceleration(array<unsigned char>^ data, int startIndex)
    {
        Vector3 acce = Vector3();

        float sensitivity = data[startIndex + 6];

        acce.x = (float)(BitConverter::ToInt16(data, startIndex + 0) * sensitivity / 1000.0f);
        acce.y = (float)(BitConverter::ToInt16(data, startIndex + 2) * sensitivity / 1000.0f);
        acce.z = (float)(BitConverter::ToInt16(data, startIndex + 4) * sensitivity / 1000.0f);

        return acce;
    }

    static std::array<float, NUM_F> GetFingerFlex(array<unsigned char>^ data, int startIndex)
    {
        std::array<float, NUM_F> flex;

        for (int i = startIndex, j = 0; j < 10; i++, j += 2)
        {
            flex[j] = (float)((data[i] >> 4) & 0x0F);
            flex[j + 1] = (float)(data[i] & 0x0F);
        }

        return flex;
    }

    static std::array<float, NUM_T> GetFingerTouch(array<unsigned char>^ data, int startIndex)
    {
        std::array<float, NUM_T> touch;

        for (int i = startIndex, j = 0; j < 10; i++, j += 2)
        {
            touch[j] = (float)((data[i] >> 4) & 0x0F);
            touch[j + 1] = (float)(data[i] & 0x0F);
        }

        return touch;
    }
};