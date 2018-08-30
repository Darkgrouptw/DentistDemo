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
		//AlphaID = glGetUniformLocation(Program, "Alpha");

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
	void Render_PC(glm::mat4 viewMat, glm::mat4 projMat);
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

	void read_vert(double x, double y, double z);
	void read_color(double x, double y, double z);
	void change_color(double color_idt);
	//void change_color_PC(double color_idt);
	//void change_vert_PC(double x, double y, double z);
	void clear_all();
	void read_alpha(double alpha, int start, int end);
	void control_slice(int teeth_idx, int disease_idx, int meat_idx, int teeth_size, int disease_size, int meat_alpha);
	void initial_alpha(double alpha);
	void initial_vert();

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

	GLuint VBO_colors;    // OpenGL array buffer for color
	GLuint VBO_alpha;

    //
    // OpenGL Shader Parameters
    //
    GLuint ViewMatID;
    GLuint ProjectMatID;
    GLuint ModelMatID;
	GLuint AlphaID;
    //
    // Object Primitives
    //
    std::vector<glm::vec3> vertices;            // Vertex vectors
    std::vector<glm::vec3> normals;             // Normal vectors
    std::vector<unsigned int> materialIndices;  // Face group by materials (not used for now, record only)
    std::vector<std::string> materialStrings;   // Material name (not use for now, record only)

	std::vector<glm::vec3> point_vertices;
	std::vector<glm::vec3> point_colors;
	std::vector<glm::vec3> tmp_point_colors;
	std::vector<float> point_alpha;
};