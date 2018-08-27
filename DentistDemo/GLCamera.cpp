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
	//printf("cameraPosition:\n");
	//printf("%f x:", cameraPosition.x);
	//printf("%f y:", cameraPosition.y);
	//printf("%f z:", cameraPosition.z);
	//printf("\n\n");
	//
	//printf("targetPosition:\n");
	//printf("%f x:", targetPosition.x);
	//printf("%f y:", targetPosition.y);
	//printf("%f z:", targetPosition.z);
	//printf("\n\n");
	//
	//printf("%frotateRadius:\n", rotateRadius);
	//printf("%fangleV:\n", angleV);
	//printf("%fangleH:\n", angleH);
	//printf("\n\n");

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

    translateDelta = glm::vec3(0);
    //rotateRadius = 10;
	rotateRadius = 6.965914;
    minRadius = 0.1;
    //angleV = angleH = 90;
	angleV = 72.5;
	angleH = -103.2;
    minAngleV = 0;
    maxAngleV = 180;
    isTranslate = false;
    isRotate = false;
	cameraPosition = glm::vec3(-9.834746, 9.261356, -5.892487);
	targetPosition = glm::vec3(-8.317695, 7.166666, 0.575495);

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
