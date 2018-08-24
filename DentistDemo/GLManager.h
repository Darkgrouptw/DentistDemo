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
		
		void read_point(float x,float y, float z);
		void read_color(float x, float y, float z);
		void read_alpha(double alpha, int start);
		void initial_alpha();
		void Initial_vert();
        void MotionUpdate(GloveData &left, GloveData &right);

        GLCamera *GetCamera();
		glm::vec4 rotateAxis_R;
		bool is_PC;

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
		GLModelNode *teeth_vertex;
    };
}