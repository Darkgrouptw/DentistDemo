#pragma once
#include "GloveData.h"
#include "GLModelNode.h"
#include "GLCamera.h"
#include <iostream>

namespace DentistDemo
{
    class GLManager
    {
    public:
        GLManager();

        void Draw();
        void MotionUpdate(GloveData &left, GloveData &right);

        GLCamera *GetCamera();
		glm::vec4 rotateAxis_R;

    private:

        void BuildLeftHand();
        void BuildRightHand();


    private:
        //
        // OpenGL Buffer Object
        //
        GLuint VAO;

        //
        // OpenGL Camera
        //
        GLCamera *glCamera;

        //
        // OpenGL Models
        //
        GLModelNode **modelHandL;
        GLModelNode **modelHandR;
    };
}