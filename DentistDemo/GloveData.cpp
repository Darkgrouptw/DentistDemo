#include "GloveData.h"

void GloveData::Zero()
{
    ZeroQuat();
    ZeroAcce();
    ZeroFlex();
    ZeroTouch();
}

void GloveData::ZeroQuat()
{
    quat = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

void GloveData::ZeroAcce()
{
    acce = Vector3();
}

void GloveData::ZeroFlex()
{
    for (int i = 0; i < NUM_F; i++)
    {
        flex[i] = 0;
        flexFiltered[i] = 0;
    }
}

void GloveData::ZeroTouch()
{
    for (int i = 0; i < NUM_T; i++)
        touch[i] = 0;
}

std::string GloveData::ToCsvString(int type = (int)DataType::TYPE_ALL)
{
	switch (type)
	{
	case (int)DataType::TYPE_ALL:
		return ToCsvString_All();
	case (int)DataType::TYPE_QUAT:
		return ToCsvString_Quat();
	case (int)DataType::TYPE_ACCE:
		return ToCsvString_Acce();
	case (int)DataType::TYPE_FLEX:
		return ToCsvString_Flex();
	case (int)DataType::TYPE_TOUCH:
		return ToCsvString_Touch();
	default:
		return ToCsvString_All();
	}
}

std::string GloveData::ToCsvString_Quat()
{
    return std::to_string(quat.w) + "," + std::to_string(quat.x) + "," +
           std::to_string(quat.y) + "," + std::to_string(quat.z);
}

std::string GloveData::ToCsvString_Acce()
{
    return std::to_string(acce.x) +","+ std::to_string(acce.y) +","+ std::to_string(acce.z);
}

std::string GloveData::ToCsvString_Flex()
{
    return std::to_string(flexFiltered[0]) + "," + std::to_string(flexFiltered[1]) + "," + std::to_string(flexFiltered[2]) + "," +
           std::to_string(flexFiltered[3]) + "," + std::to_string(flexFiltered[4]) + "," + std::to_string(flexFiltered[5]) + "," +
           std::to_string(flexFiltered[6]) + "," + std::to_string(flexFiltered[7]) + "," + std::to_string(flexFiltered[8]) + "," +
           std::to_string(flexFiltered[9]);
}

std::string GloveData::ToCsvString_Touch()
{
    return std::to_string(touch[0]) + "," + std::to_string(touch[1]) + "," + std::to_string(touch[2]) + "," +
           std::to_string(touch[3]) + "," + std::to_string(touch[4]) + "," + std::to_string(touch[5]) + "," +
           std::to_string(touch[6]) + "," + std::to_string(touch[7]) + "," + std::to_string(touch[8]) + "," +
           std::to_string(touch[9]);
}

std::string GloveData::ToCsvString_All()
{
    return ToCsvString_Flex() + "," + ToCsvString_Quat() + "," + ToCsvString_Acce(); // No touch for now
}

const GloveData GloveData::INVALID(false);