#include "GLModelNode.h"

void GLModelNode::Render(glm::mat4 viewMat, glm::mat4 projMat)
{
    // Update global transform to parent node
    UpdateTransform();

    // Use shader program
    glUseProgram(Program);

    // Pass uniform parameters
    glUniformMatrix4fv(ViewMatID, 1, GL_FALSE, glm::value_ptr(viewMat));
    glUniformMatrix4fv(ProjectMatID, 1, GL_FALSE, glm::value_ptr(projMat));
    glUniformMatrix4fv(ModelMatID, 1, GL_FALSE, glm::value_ptr(WorldTransform));

    // Bind vertices buffer
    glBindBuffer(GL_ARRAY_BUFFER, VBO_vertices);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    // Bind normals buffer
    glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
    // Draw the object
    glDrawArrays(GL_TRIANGLES, 0, vertices.size());

    // Disable attribute array
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    // Clear shader program
    glUseProgram(0);
}
void GLModelNode::Render_PC(glm::mat4 viewMat, glm::mat4 projMat) 
{
	UpdateTransform();
	glUseProgram(Program);
	//std::cout << "glUseProgram OK\n";

	glUniformMatrix4fv(ViewMatID, 1, GL_FALSE, glm::value_ptr(viewMat));
	glUniformMatrix4fv(ProjectMatID, 1, GL_FALSE, glm::value_ptr(projMat));
	glUniformMatrix4fv(ModelMatID, 1, GL_FALSE, glm::value_ptr(WorldTransform));
	//std::cout << "glUniformMatrix4fv OK\n";

	
	
	//std::cout << "glBindBuffer OK\n";
	glBindBuffer(GL_ARRAY_BUFFER, VBO_vertices);
	glBufferData(GL_ARRAY_BUFFER, point_vertices.size() * sizeof(glm::vec3), &point_vertices[0], GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
	glBufferData(GL_ARRAY_BUFFER, point_colors.size() * sizeof(glm::vec3), &point_colors[0], GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	//glUniform1f(glGetUniformLocation(Program, "Alpha"), 0.5);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_alpha);
	glBufferData(GL_ARRAY_BUFFER, point_alpha.size() * sizeof(float), &point_alpha[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(2);

	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);

	glDrawArrays(GL_POINTS, 0, point_vertices.size());
	//std::cout << "glEnableVertexAttribArray OK\n";
	
	glDisableVertexAttribArray(0);
	//std::cout << "glDisableVertexAttribArray 0 OK\n";
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	//std::cout << "glDisableVertexAttribArray 1 OK\n";

	glDisable(GL_BLEND);
	glUseProgram(0);
}
void GLModelNode::read_vert(double x, double y, double z)
{
	glm::vec3 tmp_v;
	tmp_v.x = x;
	tmp_v.y = y;
	tmp_v.z = z;
	
	point_vertices.push_back(tmp_v);
	//std::cout << "tmp_v OK";
}

void GLModelNode::read_color(double x, double y, double z)
{
	glm::vec3 tmp_v;
	tmp_v.x = x;
	tmp_v.y = y;
	tmp_v.z = z;

	point_colors.push_back(tmp_v);
	tmp_point_colors.push_back(tmp_v);
	//std::cout << "tmp_v OK";
}
void GLModelNode::read_alpha(double alpha, int start) {
	for (int i = start; i < point_alpha.size(); i++) {
		point_alpha[i] = alpha;
	}
	std::cout << "mode node alpha:" << alpha << "\n";
}
void GLModelNode::change_color(double color_idt) {
	for (int i = 0; i < point_colors.size(); i++) {
		point_colors[i].x = tmp_point_colors[i].x * color_idt;
		point_colors[i].y = tmp_point_colors[i].y * color_idt;
		point_colors[i].z = tmp_point_colors[i].z * color_idt;
	}
	std::cout << "mode node color:" << color_idt <<"\n";
}
void GLModelNode::initial_alpha(double alpha) {
	point_alpha.push_back(alpha);
}

void GLModelNode::Translate(float x, float y, float z)
{
    glm::vec4 c0 = glm::vec4(1, 0, 0, 0);
    glm::vec4 c1 = glm::vec4(0, 1, 0, 0);
    glm::vec4 c2 = glm::vec4(0, 0, 1, 0);
    glm::vec4 c3 = glm::vec4(x, y, z, 1);
    glm::mat4 M = glm::mat4(c0, c1, c2, c3);
    
    LocalTransform *= M;
}

void GLModelNode::Translate(glm::vec3 vec)
{
    Translate(vec.x, vec.y, vec.z);
}

void GLModelNode::Rotate(float angle, float x, float y, float z)
{
    float r = angle * glm::pi<float>() / 180.0f;
    glm::mat4 M = glm::mat4(1);

    glm::vec4 c0 = glm::vec4(cos(r) + (1 - cos(r))*x*x, (1 - cos(r))*y*x + sin(r)*z, (1 - cos(r))*z*x - sin(r)*y, 0);
    glm::vec4 c1 = glm::vec4((1 - cos(r))*y*x - sin(r)*z, cos(r) + (1 - cos(r))*y*y, (1 - cos(r))*z*y + sin(r)*x, 0);
    glm::vec4 c2 = glm::vec4((1 - cos(r))*z*x + sin(r)*y, (1 - cos(r))*z*y - sin(r)*x, cos(r) + (1 - cos(r))*z*z, 0);
    glm::vec4 c3 = glm::vec4(0, 0, 0, 1);
    M = glm::mat4(c0, c1, c2, c3);

    LocalTransform *= M;
}

void GLModelNode::Rotate(float angle, glm::vec3 vec)
{
    Rotate(angle, vec.x, vec.y, vec.z);
}

void GLModelNode::Scale(float x, float y, float z)
{
    glm::vec4 c0 = glm::vec4(x, 0, 0, 0);
    glm::vec4 c1 = glm::vec4(0, y, 0, 0);
    glm::vec4 c2 = glm::vec4(0, 0, z, 0);
    glm::vec4 c3 = glm::vec4(0, 0, 0, 1);
    glm::mat4 M = glm::mat4(c0, c1, c2, c3);

    LocalTransform *= M;
}

void GLModelNode::Scale(glm::vec3 vec)
{
    Scale(vec.x, vec.y, vec.z);
}

glm::mat4 GLModelNode::GetWorldTransform()
{
    if (ParentNode != NULL)
    {
        return ParentNode->GetWorldTransform() * LocalTransform;
    }
    else
    {
        return LocalTransform;
    }
}

glm::mat4 GLModelNode::GetLocalTransform()
{
    return LocalTransform;
}

void GLModelNode::UpdateTransform()
{
    if (ParentNode != NULL)
    {
		//printf("ParentNode != NULL\n");
        WorldTransform = ParentNode->GetWorldTransform() * LocalTransform;
    }
    else
    {
		//printf("ParentNode == NULL\n");
        WorldTransform = LocalTransform;
    }
}

void GLModelNode::ResetTransform()
{
    LocalTransform = glm::mat4(1.0);
    WorldTransform = glm::mat4(1.0);
}

bool GLModelNode::AddChild(GLModelNode *node)
{
    if (CurChildrenCount >= MaxChildrenCount)
        return false;   // No more space

    for (unsigned int i = 0; i < MaxChildrenCount; i++)
    {
        if (ChildNodes[i] == NULL)
        {
            ChildNodes[i] = node;
            ChildNodes[i]->SetParent(this);
            CurChildrenCount++;

            return true;    // Add success
        }
    }

    return false;   // Add failed
}

bool GLModelNode::RemoveChild(GLModelNode *node)
{
    if (node == NULL)
        return false;   // Input failure

    for (unsigned int i = 0; i < MaxChildrenCount; i++)
    {
        if (ChildNodes[i] == node)
        {
            ChildNodes[i]->SetParent(NULL);
            ChildNodes[i] = NULL;
            CurChildrenCount--;
            return true;    // Remove success
        }
    }

    return false;   // Remove failed
}

GLModelNode * GLModelNode::GetParent()
{
    return ParentNode;
}

void GLModelNode::SetParent(GLModelNode * parent)
{
    ParentNode = parent;
}

bool GLModelNode::LoadModel(char * filename)
{
    bool result = loadOBJ(filename, vertices, normals, materialIndices, materialStrings);
    if (!result)
    {
        std::cout << "Load failed";
        return result;
    }

    // Vertices VBO generation and binding
    glGenBuffers(1, &VBO_vertices);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_vertices);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

    // Normal VBO generation and binding
    glGenBuffers(1, &VBO_normals);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);

    return true;
}
void GLModelNode::initial_vert() {

	glGenBuffers(1, &VBO_vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_vertices);
	glBufferData(GL_ARRAY_BUFFER, point_vertices.size() * sizeof(glm::vec3), &point_vertices[0], GL_STATIC_DRAW);
	

	glGenBuffers(1, &VBO_colors);
	std::cout << "glGenBuffers OK\n";
	glBindBuffer(GL_ARRAY_BUFFER, VBO_colors);
	std::cout << "glBindBuffer OK\n";
	glBufferData(GL_ARRAY_BUFFER, point_colors.size() * sizeof(glm::vec3), &point_colors[0], GL_STATIC_DRAW);
	std::cout << "glBufferData OK\n";


	glGenBuffers(1, &VBO_alpha);
	glBindBuffer(GL_ARRAY_BUFFER, VBO_alpha);
	glBufferData(GL_ARRAY_BUFFER, point_alpha.size() * sizeof(float), &point_alpha[0], GL_DYNAMIC_DRAW);


}
