#pragma once

#include <GL/glew.h>
#include <GL/glm/glm.hpp>
#include <GL/glm/gtc/type_ptr.hpp>
#include <GL/glm/gtc/matrix_transform.hpp>
#include <GL/glm/gtc/constants.hpp>

class GLCamera
{
public:
    //
    // Constructor
    //
    GLCamera()
    {
        Reset();
    };

    //
    // Camera Operation
    //
    void StartRotate(int x, int y);
    void StartTranslate(int x, int y);
    void StopRotate(int x, int y);
    void StopTranslate(int x, int y);
    void Move(int x, int y);
    void ZoomIn(int delta);

    bool IsMoving();
	bool mouse_move;

    //
    // Matrices Update
    //
    void UpdateView();

    void Reset();

    //
    // Matrices Get
    //
    glm::mat4 GetViewMatrix();
    glm::mat4 GetProjecyionMatrix();
	float* GLCamera::GetViewMatrixF();
	float* GLCamera::GetProjectionMatrixF();

private:
    //
    // OpenGL Matrices
    //
    glm::mat4 ViewMat;
    glm::mat4 ProjMat;
	float* ViewMatF;
	float* ProjMatF;

    //
    // Camera Parameters
    //

    // Camera/eye position
    glm::vec3 cameraPosition;

    // Target position
    glm::vec3 targetPosition;

    // Zoom in/out
    double rotateRadius;
    double minRadius;

    // Translate
    bool isTranslate;
    glm::vec3 translateDelta;

    // Rotate
    bool isRotate;
    double minAngleV, maxAngleV;
    double angleV, angleH;

    int xTemp, yTemp;

};