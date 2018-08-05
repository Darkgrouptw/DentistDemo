#pragma once
#include <GL/glm/glm.hpp>

char* HandRight[] =
{
    "HandBone/R_Hand_Palm.obj",  // Palm
    "HandBone/R_Thumb_MCP.obj",  // Thumb
    "HandBone/R_Thumb_PIP.obj",
    "HandBone/R_Thumb_DIP.obj",
    "HandBone/R_Index_MCP.obj",  // Index
    "HandBone/R_Index_PIP.obj",
    "HandBone/R_Index_DIP.obj",
    "HandBone/R_Middle_MCP.obj",  // Middle
    "HandBone/R_Middle_PIP.obj",
    "HandBone/R_Middle_DIP.obj",
    "HandBone/R_Ring_MCP.obj",  // Ring
    "HandBone/R_Ring_PIP.obj",
    "HandBone/R_Ring_DIP.obj",
    "HandBone/R_Pinky_MCP.obj",  // Pinky
    "HandBone/R_Pinky_PIP.obj",
    "HandBone/R_Pinky_DIP.obj"
};

char *HandLeft[] =
{
    "HandBone/L_Hand_Palm.obj",  // Palm
    "HandBone/L_Thumb_MCP.obj",  // Thumb
    "HandBone/L_Thumb_PIP.obj",
    "HandBone/L_Thumb_DIP.obj",
    "HandBone/L_Index_MCP.obj",  // Index
    "HandBone/L_Index_PIP.obj",
    "HandBone/L_Index_DIP.obj",
    "HandBone/L_Middle_MCP.obj",  // Middle
    "HandBone/L_Middle_PIP.obj",
    "HandBone/L_Middle_DIP.obj",
    "HandBone/L_Ring_MCP.obj",  // Ring
    "HandBone/L_Ring_PIP.obj",
    "HandBone/L_Ring_DIP.obj",
    "HandBone/L_Pinky_MCP.obj",  // Pinky
    "HandBone/L_Pinky_PIP.obj",
    "HandBone/L_Pinky_DIP.obj"
};

class HandPart
{
public:
    static const int Hand_Palm  = 0;
    static const int Thumb_MCP  = 1;
    static const int Thumb_PIP  = 2;
    static const int Thumb_DIP  = 3;
    static const int Index_MCP  = 4;
    static const int Index_PIP  = 5;
    static const int Index_DIP  = 6;
    static const int Middle_MCP = 7;
    static const int Middle_PIP = 8;
    static const int Middle_DIP = 9;
    static const int Ring_MCP   = 10;
    static const int Ring_PIP   = 11;
    static const int Ring_DIP   = 12;
    static const int Pinky_MCP  = 13;
    static const int Pinky_PIP  = 14;
    static const int Pinky_DIP  = 15;
    static const int Finger_Size = 16;
};

// Left Hand Local Position
glm::vec3 HandLeftLocalPos[] =
{
    glm::vec3(0,0,0),
    -glm::vec3(-0.578,0.19,0.339),
    -glm::vec3(-0.469,0.455,1.137),
    -glm::vec3(-0.302,0.329,0.744),
    -glm::vec3(-0.529,0.425,2.475),
    -glm::vec3(-0.084,0.224,1.23),
    -glm::vec3(-0.05,0.165,0.62),
    -glm::vec3(0.054,0.446,2.471),
    -glm::vec3(-0.033,0.231,1.412),
    -glm::vec3(0.014,0.197,0.772),
    -glm::vec3(0.623,0.464,2.244),
    -glm::vec3(0.087,0.23,1.262),
    -glm::vec3(0.027,0.202,0.718),
    -glm::vec3(1.108,0.441,1.985),
    -glm::vec3(0.068,0.196,0.961),
    -glm::vec3(0.022,0.115,0.454)
};

// Right Hand Local Position
glm::vec3 HandRightLocalPos[] =
{
    glm::vec3(0,0,0),
    -glm::vec3(0.578,0.19,0.339),
    -glm::vec3(0.469,0.455,1.137),
    -glm::vec3(0.302,0.329,0.744),
    -glm::vec3(0.529,0.425,2.475),
    -glm::vec3(0.084,0.224,1.23),
    -glm::vec3(0.05,0.165,0.62),
    -glm::vec3(-0.054,0.446,2.471),
    -glm::vec3(0.033,0.231,1.412),
    -glm::vec3(-0.014,0.197,0.772),
    -glm::vec3(-0.623,0.464,2.244),
    -glm::vec3(-0.087,0.23,1.262),
    -glm::vec3(-0.027,0.202,0.718),
    -glm::vec3(-1.108,0.441,1.985),
    -glm::vec3(-0.068,0.196,0.961),
    -glm::vec3(-0.022,0.115,0.454)
};