#version 330 core

uniform mat4 MV;
uniform vec3 LightPosition_worldspace;

// from vertex shader
in vec3 Position_worldspace;
flat in vec3 Normal_cameraspace;
in vec3 EyeDirection_cameraspace;
in vec3 LightDirection_cameraspace;

// outputs
out vec3 color;

void main(){

	// Light properties
	vec3 LightColor = vec3(1,1,1);
	float LightPower = 5.0f;
	
	// Material properties
	vec3 MaterialDiffuseColor = vec3(0.5, 0.5, 0.5);
	vec3 MaterialAmbientColor = vec3(0.2,0.2,0.2) * MaterialDiffuseColor;
	vec3 MaterialSpecularColor = vec3(0.0,0.0,0.0);

	// Distance to the light
	float distance = length( LightPosition_worldspace - Position_worldspace );

	// Phong Calculations
	vec3 n = normalize( Normal_cameraspace );
	vec3 l = normalize( LightDirection_cameraspace );
	float cosTheta = clamp( dot( n,l ), 0,1 );
	
	vec3 E = normalize(EyeDirection_cameraspace);
	vec3 R = reflect(-l,n);
	float cosAlpha = clamp( dot( E,R ), 0,1 );
	
	color = MaterialAmbientColor
		+ MaterialDiffuseColor * LightColor * LightPower * cosTheta / distance
		+ MaterialSpecularColor * LightColor * LightPower * pow(cosAlpha,5) / distance;



}
