#include "GLCamera.h"

void GLCamera::StartRotate(int x, int y)
{
    isRotate = true;
    xTemp = x; yTemp = y;
}

void GLCamera::StartTranslate(int x, int y)
{
    isTranslate = true;
    xTemp = x; yTemp = y;
}

void GLCamera::StopRotate(int x, int y)
{
    isRotate = false;
}

void GLCamera::StopTranslate(int x, int y)
{
    isTranslate = false;
    translateDelta = glm::vec3(0);
}

void GLCamera::Move(int x, int y)
{
    if (isRotate)
    {
        angleV -= (y - yTemp) / 10.0f;
        angleH += (x - xTemp) / 10.0f;
        angleV = (((angleV) > (minAngleV)) ? (angleV) : (minAngleV));
        angleV = (((angleV) < (maxAngleV)) ? (angleV) : (maxAngleV));
        xTemp = x;
        yTemp = y;
    }
    else if (isTranslate)
    {
        double phi = angleH * (3.14159f / 180.0f);
        translateDelta[0] = (x - xTemp) / 30.0f;
        translateDelta[1] = (y - yTemp) / 30.0f;
        translateDelta[2] = 0.0f;

        translateDelta[2] = translateDelta[0] * cos(phi);
        translateDelta[0] = -1.0*translateDelta[0] * sin(phi);
        xTemp = x;
        yTemp = y;
    }

	if (!fabs(x - xTemp) > 1.0f)
		mouse_move = false;
}

void GLCamera::ZoomIn(int delta)
{
    if (delta > 0)
    {
        rotateRadius -= 0.05f * abs(rotateRadius);
    }
    else
    {
        rotateRadius += 0.05f * abs(rotateRadius);
    }

    rotateRadius = std::fmax(rotateRadius, minRadius);
}

bool GLCamera::IsMoving()
{
    return isTranslate || isRotate;
}

void GLCamera::UpdateView()
{
    double theta = angleV * (glm::pi<float>() / 180.0f);
    double phi = angleH * (glm::pi<float>() / 180.0f);

    cameraPosition.x = rotateRadius * sin(theta) * cos(phi);
    cameraPosition.y = rotateRadius * cos(theta);
    cameraPosition.z = rotateRadius * sin(theta) * sin(phi);

    targetPosition += translateDelta;
    cameraPosition += targetPosition;

    ViewMat = glm::lookAt
        (
            cameraPosition,     // Camera in World Space
            targetPosition,     // and looks at the origin
            glm::vec3(0, 1, 0)  // Head is up (set to 0,1,0 to look upside-down)
        );

	for (int i = 0; i < ViewMat.row_size(); i++)
	{
		for (int j = 0; j < ViewMat.col_size(); j++)
		{
			ViewMatF[i*ViewMat.col_size() + j] = ViewMat[i][j];
		}
	}

	//printf("cameraPosition");
	//printf("%fx:", cameraPosition.x);
	//printf("%fy:", cameraPosition.y);
	//printf("%fz:\n\n", cameraPosition.z);
	//
	//printf("targetPosition");
	//printf("%fx:", targetPosition.x);
	//printf("%fy:", targetPosition.y);
	//printf("%fz:\n\n", targetPosition.z);
	//
	//
	//printf("%frotateRadius\n", rotateRadius);
	//printf("%fangleV\n", angleV);
	//printf("%fangleH\n\n\n", angleH);
	//cameraPosition = glm::vec3(5.0f, 5.0f, 0.0f);
	//targetPosition = glm::vec3(5.0f, 5.0f, -5.0f);

	//printf("ViewMatF\n");
	//for (int i = 0; i < 16; i++) {
	//	printf("%f ", ViewMatF[i]);
	//}
	//
	//printf("\n");
	////
	//printf("ProjMatF\n");
	//for (int i = 0; i < 16; i++) {
	//	printf("%f ", ProjMatF[i]);
	//}
	//printf("\n");
	//printf("\n");
}

void GLCamera::Reset()
{
    // Set up projection and view matrix
    ProjMat = glm::perspective(60.0f, 16.0f / 9.0f, 0.1f, 2000.0f);

	ViewMatF = new float[16];
	ProjMatF = new float[16];
	for (int i = 0; i < ProjMat.row_size(); i++)
	{
		for (int j = 0; j < ProjMat.col_size(); j++)
		{
			ProjMatF[i*ProjMat.col_size() + j] = ProjMat[i][j];
		}
	}

	//for (int i = 0; i < ProjMat.row_size(); i++)
	//{
	//	for (int j = 0; j < ProjMat.col_size(); j++)
	//	{
	//		ProjMatF[i*ProjMat.col_size() + j] = ProjMat[i][j];
	//	}
	//}

	//float tmp[16] = {-0.991216, -0.031996, 0.128328, 0.0, 0.0, 0.970296, 0.241922, 0.0, -0.132256, 0.239797, -0.961772, 0.0, 6.455243, -6.063533, -9.372509, 1.0};

	//for (int i = 0; i < 16; i++) {
	//	ViewMatF[i] = tmp[i];
	//}

	cameraPosition = glm::vec3(-10.112521, 9.628954, -8.63829);
	targetPosition = glm::vec3(-10.278182, 5.566662, 3.225534);
	rotateRadius = 12.541135;
	angleV = 71.6000;
	angleH = -89.2;

    translateDelta = glm::vec3(0);
    //rotateRadius = 10;
    minRadius = 0.1;
   // angleV = angleH = 90;
    minAngleV = 0;
    maxAngleV = 180;
    isTranslate = false;
    isRotate = false;

    UpdateView();
}

glm::mat4 GLCamera::GetViewMatrix()
{
    return ViewMat;
}

glm::mat4 GLCamera::GetProjecyionMatrix()
{
    return ProjMat;
}

float* GLCamera::GetViewMatrixF()
{
	return ViewMatF;
}

float* GLCamera::GetProjectionMatrixF()
{
	return ProjMatF;
}
