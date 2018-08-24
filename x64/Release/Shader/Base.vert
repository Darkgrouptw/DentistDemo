#version 430

layout (location = 0) in vec3 vertexPosition;
//layout (location = 1) in vec3 vertexNormal;
layout (location = 1) in vec3 vertexColor;
layout (location = 2) in float vertexAlpha;

uniform mat4 View;
uniform	mat4 Projection;
uniform mat4 Model;
//uniform float Alpha;

vec3 vLightPosition = vec3(0.0f, 10.0f, 10.0f);

out vec3 vNormal;
out vec3 vLightDir;
out vec3 vColor;
out float vAlpha;

void main(void) 
{ 
	mat4 MV = View * Model;
	mat4 MVP = Projection * MV;
    
    // Get surface normal in eye coordinates
	mat3 normalMatrix = mat3(MV);//normal matrix is MV matrix's 3*3 excluding 'w' 
    //vNormal = normalMatrix * vertexNormal;
	vNormal = normalMatrix * vec3(0,0,-1);
	
	// Get vertex position in eye coordinates
    vec4 vPosition4 = MV * vec4(vertexPosition,1);
    vec3 vPosition3 = vPosition4.xyz;
    
    // Get vector to light source
    vLightDir = vLightPosition - vPosition3;

    // Don't forget to transform the geometry!
    gl_Position = MVP * vec4(vertexPosition,1);
	
	vColor = vertexColor;
	vAlpha = vertexAlpha;
}