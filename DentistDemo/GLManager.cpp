#include "GLManager.h"
#include "ModelList.h"

using namespace DentistDemo;

GLManager::GLManager()
{
    // Clear background to back
    glClearColor(0, 0, 0, 1);

    glCamera = new GLCamera();

    // VAO generation and binding
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
	is_PC = false;

    // Load models
	tooth1 = new GLModelNode();
	teeth_vertex = new GLModelNode();
	//tooth1->LoadModel("./Objx/teeth_test.objx");
	tooth1->LoadModel("./Objx/T01_02.objx");
	//tooth1->LoadModel("D:/2018_02/DentistDemo/x64/Release/Objx/teeth_test.objx");
    //modelHandL = new GLModelNode*[HandPart::Finger_Size];
    //modelHandR = new GLModelNode*[HandPart::Finger_Size];
	//
    //for (int i = 0; i < HandPart::Finger_Size; i++)
    //{
    //    modelHandL[i] = new GLModelNode(std::string(HandLeft[i]));
    //    modelHandL[i]->LoadModel(HandLeft[i]);
	//
    //    modelHandR[i] = new GLModelNode(std::string(HandRight[i]));
    //    modelHandR[i]->LoadModel(HandRight[i]);
    //}
	//
    //BuildLeftHand();
    //BuildRightHand();
}

void GLManager::Draw()
{
    //glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//std::cout << "clear color" << std::endl;
    glBindVertexArray(VAO);

    glCamera->UpdateView();

    glm::mat4 viewMat = glCamera->GetViewMatrix();
    glm::mat4 projMat = glCamera->GetProjecyionMatrix();

	if (is_PC == true) {
		glPointSize(6);
		teeth_vertex->Render_PC(viewMat, projMat);
	}
	
	//tooth1->Render(viewMat, projMat);

    //for (int i = 0; i < HandPart::Finger_Size; i++)
    // {
    //     modelHandL[i]->Render(viewMat, projMat);
    //     modelHandR[i]->Render(viewMat, projMat);
    // }
}
void DentistDemo::GLManager::read_point(float x, float y, float z) 
{
	//std::cout << "in GLManager read_point\n";
	teeth_vertex->read_vert(x, y, z);
	
}
void DentistDemo::GLManager::read_color(float x, float y, float z)
{
	//std::cout << "in GLManager read_point\n";
	teeth_vertex->read_color(x, y, z);

}
void DentistDemo::GLManager::change_color(float color_idt)
{
	teeth_vertex->change_color(color_idt);
}
void DentistDemo::GLManager::read_alpha(double alpha, int start, int end) {
	teeth_vertex->read_alpha(alpha, start, end);
}
void DentistDemo::GLManager::initial_alpha() {
	teeth_vertex->initial_alpha(1.0f);
}
void DentistDemo::GLManager::Clear_all() {

	teeth_vertex->clear_all();
}
void DentistDemo::GLManager::Initial_vert() {
	
	teeth_vertex->initial_vert();

}
void DentistDemo::GLManager::Rotate_90() {
	teeth_vertex->Rotate(90, 0, 0, 1);

}
void DentistDemo::GLManager::control_pic(int teeth_idx, int disease_idx, int meat_idx, int teeth_size, int disease_size, float meat_alpha) {
	teeth_vertex->control_slice(teeth_idx, disease_idx, meat_idx, teeth_size, disease_size, meat_alpha);

}
void DentistDemo::GLManager::MotionUpdate(GloveData &left, GloveData &right)
{
    // Reset all transform
    for (int i = 0; i < HandPart::Finger_Size; i++)
    {
        modelHandL[i]->ResetTransform();
        modelHandR[i]->ResetTransform();
    }

    //
    // Update left hand
    //
    // Left hand palm (Quaternion)
    int curPart = HandPart::Hand_Palm;
    modelHandL[curPart]->Translate(glm::vec3(-2, 0, 0));
    Radian theta;
    Vector3 rotationAxix;
    Quaternion qq;
    left.GetQuaternion().ToAngleAxis(theta, rotationAxix);
    modelHandL[curPart]->Rotate(theta.valueDegrees(), rotationAxix.x, rotationAxix.y, rotationAxix.z);
    
    // Left hand thumb MCP (None)
    curPart = HandPart::Thumb_MCP;
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    // No transform for this part

    // Left hand thumb PIP (1)
    curPart = HandPart::Thumb_PIP;
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(1), -0.5, 1, -0.5);

    // Left hand thumb DIP (0)
    curPart = HandPart::Thumb_DIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(0), -0.5, 1, -0.5);

    // Left hand index MCP (3)
    curPart = HandPart::Index_MCP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(3), -1, 0, 0);

    // Left hand index PIP (2)
    curPart = HandPart::Index_PIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(2), -1, 0, 0);

    // Left hand index DIP (2)
    curPart = HandPart::Index_DIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(2), -1, 0, 0);
    
    // Left hand middle MCP (5)
    curPart = HandPart::Middle_MCP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(5), -1, 0, 0);

    // Left hand middle PIP (4)
    curPart = HandPart::Middle_PIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(4), -1, 0, 0);

    // Left hand middle DIP (4)
    curPart = HandPart::Middle_DIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(4), -1, 0, 0);

    // Left hand ring MCP (7)
    curPart = HandPart::Ring_MCP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(7), -1, 0, 0);

    // Left hand ring PIP (6)
    curPart = HandPart::Ring_PIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(6), -1, 0, 0);

    // Left hand ring DIP (6)
    curPart = HandPart::Ring_DIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(6), -1, 0, 0);

    // Left hand pinky MCP (9)
    curPart = HandPart::Pinky_MCP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(9), -1, 0, 0);

    // Left hand pinky PIP (8)
    curPart = HandPart::Pinky_PIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(8), -1, 0, 0);

    // Left hand pinky DIP (8)
    curPart = HandPart::Pinky_DIP; // current part
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
    modelHandL[curPart]->Rotate(left.GetFlexFilteredAt(8), -1, 0, 0);

    //
    // Update right hand
    //
    // Right hand palm (Quaternion)
    curPart = HandPart::Hand_Palm; // current part
    modelHandR[curPart]->Translate(glm::vec3(2, 0, 0));
    right.GetQuaternion().ToAngleAxis(theta, rotationAxix);
    modelHandR[curPart]->Rotate(theta.valueDegrees(), rotationAxix.x, rotationAxix.y, rotationAxix.z);

    // Right hand thumb MCP (None)
    curPart = HandPart::Thumb_MCP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand thumb PIP (1)
    curPart = HandPart::Thumb_PIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(1), -0.5, -1, 0.5);

    // Right hand thumb DIP (0)
    curPart = HandPart::Thumb_DIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(0), -0.5, -1, 0.5);

    // Right hand index MCP (3)
    curPart = HandPart::Index_MCP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(3), -1, 0, 0);

    // Right hand index PIP (2)
    curPart = HandPart::Index_PIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(2), -1, 0, 0);

    // Right hand index DIP (2)
    curPart = HandPart::Index_DIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(2), -1, 0, 0);

    // Right hand middle MCP (5)
    curPart = HandPart::Middle_MCP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(5), -1, 0, 0);

    // Right hand middle PIP (4)
    curPart = HandPart::Middle_PIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(4), -1, 0, 0);

    // Right hand middle DIP (4)
    curPart = HandPart::Middle_DIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(4), -1, 0, 0);

    // Right hand ring MCP (7)
    curPart = HandPart::Ring_MCP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(7), -1, 0, 0);

    // Right hand ring PIP (6)
    curPart = HandPart::Ring_PIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(6), -1, 0, 0);

    // Right hand ring DIP (6)
    curPart = HandPart::Ring_DIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(6), -1, 0, 0);

    // Right hand pinky MCP (9)
    curPart = HandPart::Pinky_MCP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(9), -1, 0, 0);

    // Right hand pinky PIP (8)
    curPart = HandPart::Pinky_PIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(8), -1, 0, 0);

    // Right hand pinky DIP (8)
    curPart = HandPart::Pinky_DIP; // current part
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
    modelHandR[curPart]->Rotate(right.GetFlexFilteredAt(8), -1, 0, 0);
}

GLCamera *DentistDemo::GLManager::GetCamera()
{
    return glCamera;
}

void DentistDemo::GLManager::BuildLeftHand()
{
    // Build Left hand hierarchy
    // Left hand root is modelHandLeft[HandPart::Hand_Palm]

    // Left hand thumb MCP
    int curPart = HandPart::Thumb_MCP; // current part
    int parPart = HandPart::Hand_Palm; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);

    // Left hand thumb PIP
    curPart = HandPart::Thumb_PIP; // current part
    parPart = HandPart::Thumb_MCP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);

    // Left hand thumb DIP
    curPart = HandPart::Thumb_DIP; // current part
    parPart = HandPart::Thumb_PIP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand index MCP
    curPart = HandPart::Index_MCP; // current part
    parPart = HandPart::Hand_Palm; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand index PIP
    curPart = HandPart::Index_PIP; // current part
    parPart = HandPart::Index_MCP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand index DIP
    curPart = HandPart::Index_DIP; // current part
    parPart = HandPart::Index_PIP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand middle MCP
    curPart = HandPart::Middle_MCP; // current part
    parPart = HandPart::Hand_Palm; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand middle PIP
    curPart = HandPart::Middle_PIP; // current part
    parPart = HandPart::Middle_MCP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand middle DIP
    curPart = HandPart::Middle_DIP; // current part
    parPart = HandPart::Middle_PIP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand ring MCP
    curPart = HandPart::Ring_MCP; // current part
    parPart = HandPart::Hand_Palm; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand ring PIP
    curPart = HandPart::Ring_PIP; // current part
    parPart = HandPart::Ring_MCP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand ring DIP
    curPart = HandPart::Ring_DIP; // current part
    parPart = HandPart::Ring_PIP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand pinky MCP
    curPart = HandPart::Pinky_MCP; // current part
    parPart = HandPart::Hand_Palm; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand pinky PIP
    curPart = HandPart::Pinky_PIP; // current part
    parPart = HandPart::Pinky_MCP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);

    // Left hand pinky DIP
    curPart = HandPart::Pinky_DIP; // current part
    parPart = HandPart::Pinky_PIP; // parent part
    modelHandL[parPart]->AddChild(modelHandL[curPart]);
    modelHandL[curPart]->Translate(HandLeftLocalPos[curPart]);
}

void DentistDemo::GLManager::BuildRightHand()
{
    // Build Right hand hierarchy
    // Right hand root is modelHandRight[HandPart::Hand_Palm]
    modelHandR[HandPart::Hand_Palm]->Translate(glm::vec3(2, 0, 0));

    // Right hand thumb MCP
    int curPart = HandPart::Thumb_MCP; // current part
    int parPart = HandPart::Hand_Palm; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand thumb PIP
    curPart = HandPart::Thumb_PIP; // current part
    parPart = HandPart::Thumb_MCP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand thumb DIP
    curPart = HandPart::Thumb_DIP; // current part
    parPart = HandPart::Thumb_PIP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand index MCP
    curPart = HandPart::Index_MCP; // current part
    parPart = HandPart::Hand_Palm; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand index PIP
    curPart = HandPart::Index_PIP; // current part
    parPart = HandPart::Index_MCP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand index DIP
    curPart = HandPart::Index_DIP; // current part
    parPart = HandPart::Index_PIP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand middle MCP
    curPart = HandPart::Middle_MCP; // current part
    parPart = HandPart::Hand_Palm; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand middle PIP
    curPart = HandPart::Middle_PIP; // current part
    parPart = HandPart::Middle_MCP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand middle DIP
    curPart = HandPart::Middle_DIP; // current part
    parPart = HandPart::Middle_PIP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand ring MCP
    curPart = HandPart::Ring_MCP; // current part
    parPart = HandPart::Hand_Palm; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand ring PIP
    curPart = HandPart::Ring_PIP; // current part
    parPart = HandPart::Ring_MCP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand ring DIP
    curPart = HandPart::Ring_DIP; // current part
    parPart = HandPart::Ring_PIP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand pinky MCP
    curPart = HandPart::Pinky_MCP; // current part
    parPart = HandPart::Hand_Palm; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand pinky PIP
    curPart = HandPart::Pinky_PIP; // current part
    parPart = HandPart::Pinky_MCP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);

    // Right hand pinky DIP
    curPart = HandPart::Pinky_DIP; // current part
    parPart = HandPart::Pinky_PIP; // parent part
    modelHandR[parPart]->AddChild(modelHandR[curPart]);
    modelHandR[curPart]->Translate(HandRightLocalPos[curPart]);
}