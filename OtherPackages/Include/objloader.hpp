#pragma once

#include <GL/glm/glm.hpp>
#include <string>
#include <vector>

bool loadOBJ(
	const char * path, 
	std::vector<glm::vec3> & out_vertices, 
	std::vector<glm::vec2> & out_uvs,
	std::vector<glm::vec3> & out_normals,
	std::vector<unsigned int> &out_materialIndices,
	std::vector<std::string> &out_mtls
);

// Loader without uvs
bool loadOBJ(
    const char * path,
    std::vector<glm::vec3> & out_vertices,
    std::vector<glm::vec3> & out_normals,
    std::vector<unsigned int> &out_materialIndices,
    std::vector<std::string> &out_mtls
);

bool loadMTL(const char * path,
			 std::vector<glm::vec3> &Kd,
			 std::vector<glm::vec3> &Ka,
			 std::vector<glm::vec3> &Ks,
			 std::vector<std::string> &out_name,
			 std::vector<std::string> &texture
);

bool loadAssImp(
	const char * path, 
	std::vector<unsigned short> & indices,
	std::vector<glm::vec3> & vertices,
	std::vector<glm::vec2> & uvs,
	std::vector<glm::vec3> & normals
);