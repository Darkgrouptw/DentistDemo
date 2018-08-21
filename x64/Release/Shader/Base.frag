#version 430

in vec3 vNormal;
in vec3 vLightDir;

// Lighting color
vec4 ambientColor  = vec4(0.2, 0.2, 0.2, 1.0);
vec4 diffuseColor  = vec4(0.8, 0.8, 0.8, 1.0);   
vec4 specularColor = vec4(1.0, 1.0, 1.0, 1.0);
float shininess = 128.0;

void main()
{
	vec4 baseColor = vec4(0.8, 0.8, 0.8, 1.0);

	vec3 eyeDir = vec3(0.0, 0.0, 1.0);
	vec3 lightDir = normalize(vLightDir);
    vec3 normal = normalize(vNormal);
    vec3 halfVector = normalize(lightDir + normalize(eyeDir));
    
	
	if(!gl_FrontFacing)
	{
		normal = -normal;
	
	}
	
	
    float diff = max(0.0, dot(normal, lightDir));
    float spec = pow(max(0.0, dot(normal, halfVector)), shininess);
    
	
	
    vec4 ambient = baseColor * ambientColor;
    vec4 diffuse = baseColor * diffuseColor * diff;
    vec4 specular = baseColor * specularColor * spec;
    
	
	gl_FragColor = ambient + diffuse + specular;
	//gl_FragColor = vec4(0.5,0.5,0.5,0.5); 
	gl_FragColor.a = 0.5;
}