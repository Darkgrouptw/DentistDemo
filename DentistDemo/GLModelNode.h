#pragma once

#include <GL/glew.h>
#include <GL/glm/glm.hpp>
#include <GL/glm/gtc/type_ptr.hpp>
#include <GL/glm/gtc/matrix_transform.hpp>
#include <GL/glm/gtc/constants.hpp>
#include "LoadShaders.h"
#include "objloader.hpp"

class GLModelNode
{
public:
    //
    // Constructors
    //
    GLModelNode(std::string name, unsigned int maxChildren = 10) : 
        NodeName(name), MaxChildrenCount(maxChildren), LocalTransform(glm::mat4(1.0)), WorldTransform(glm::mat4(1.0))
    {
        // Load shader
        ShaderInfo shaders[] =
        {
            { GL_VERTEX_SHADER,		"./Shader/Base.vert" },
            { GL_FRAGMENT_SHADER,	"./Shader/Base.frag" },
            { GL_NONE, NULL }
        };
        Program = LoadShaders(shaders);

        // Use shader program to get uniform parameters
        glUseProgram(Program);
        ViewMatID = glGetUniformLocation(Program, "View");
        ProjectMatID = glGetUniformLocation(Program, "Projection");
        ModelMatID = glGetUniformLocation(Program, "Model");

        // Clear shader program
        glUseProgram(0);

        // Initialize hierarchy structure
        //ParentNode = NULL;
        //if (maxChildren > 0)
        //{
        //    ChildNodes = new GLModelNode *[maxChildren];
        //    for (unsigned int i = 0; i < maxChildren; i++)
        //        ChildNodes[i] = NULL;
        //}
        //else
        //{
        //    ChildNodes = NULL;
        //}
        //CurChildrenCount = 0;
    };
	GLModelNode() :
		LocalTransform(glm::mat4(1.0)), WorldTransform(glm::mat4(1.0))
	{
		// Load shader
		ShaderInfo shaders[] =
		{
			{ GL_VERTEX_SHADER,		"./Shader/Base.vert" },
			{ GL_FRAGMENT_SHADER,	"./Shader/Base.frag" },
			{ GL_NONE, NULL }
		};
		Program = LoadShaders(shaders);

		// Use shader program to get uniform parameters
		glUseProgram(Program);
		ViewMatID = glGetUniformLocation(Program, "View");
		ProjectMatID = glGetUniformLocation(Program, "Projection");
		ModelMatID = glGetUniformLocation(Program, "Model");

		// Clear shader program
		glUseProgram(0);

		// Initialize hierarchy structure
		ParentNode = NULL;
		//if (maxChildren > 0)
		//{
		//    ChildNodes = new GLModelNode *[maxChildren];
		//    for (unsigned int i = 0; i < maxChildren; i++)
		//        ChildNodes[i] = NULL;
		//}
		//else
		//{
		//    ChildNodes = NULL;
		//}
		//CurChildrenCount = 0;
	};

    //
    // Main Functions
    //

    // Model rendering
    void Render(glm::mat4 viewMat, glm::mat4 projMat);
    // Model transform
    void Translate(float x, float y, float z);
    void Translate(glm::vec3 vec);
    void Rotate(float angle, float x, float y, float z);
    void Rotate(float angle, glm::vec3 vec);
    void Scale(float x, float y, float z);
    void Scale(glm::vec3 vec);

    glm::mat4 GetWorldTransform();
    glm::mat4 GetLocalTransform();
    void UpdateTransform();
    void ResetTransform();

    //
    // Hierarchy Structure
    //
    bool AddChild(GLModelNode *node);
    bool RemoveChild(GLModelNode *node);
    GLModelNode *GetParent();
    void SetParent(GLModelNode* parent);

    bool LoadModel(char* objfilename);

private:
    //
    // Node Information
    //
    std::string NodeName;           // Name of this node
    glm::mat4   LocalTransform;     // Local transform of this node
    glm::mat4   WorldTransform;     // Global transform (according to parent, if exists) of this node
    // Hierarchy structure
    GLModelNode *ParentNode;
    GLModelNode **ChildNodes;
    unsigned int MaxChildrenCount;
    unsigned int CurChildrenCount;

    //
    // OpenGL Buffer Objects
    //
    GLuint Program;         // Shader program
    GLuint VBO_vertices;    // OpenGL array buffer for vertices
    GLuint VBO_normals;     // OpenGL array buffer for normals

    //
    // OpenGL Shader Parameters
    //
    GLuint ViewMatID;
    GLuint ProjectMatID;
    GLuint ModelMatID;

    //
    // Object Primitives
    //
    std::vector<glm::vec3> vertices;            // Vertex vectors
    std::vector<glm::vec3> normals;             // Normal vectors
    std::vector<unsigned int> materialIndices;  // Face group by materials (not used for now, record only)
    std::vector<std::string> materialStrings;   // Material name (not use for now, record only)
};