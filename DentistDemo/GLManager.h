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
		bool teeth1_draw, teeth2_draw, teeth3_draw;

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
		GLModelNode *tooth1;
		GLModelNode *tooth2;
		GLModelNode *tooth3;
    };
}