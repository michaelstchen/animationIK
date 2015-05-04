#version 330 core

uniform mat4 MV;
uniform vec3 LightPosition_worldspace;

// From vertex shader
in vec3 Position_worldspace;
in vec3 Normal_cameraspace;
in vec3 EyeDirection_cameraspace;
in vec3 LightDirection_cameraspace;

// Ouputs
out vec3 color;

void main(){

	// Light properties
	vec3 LightColor = vec3(1,1,1);
	float LightPower = 30.0f;
	
	// Material properties
	vec3 MaterialDiffuseColor = vec3(1.0, 0.0, 0.0);
	vec3 MaterialAmbientColor = vec3(0.1,0.1,0.1) * MaterialDiffuseColor;
	vec3 MaterialSpecularColor = vec3(0.0,0.0,0.0);

	// Distance to the light
	float distance = length( LightPosition_worldspace - Position_worldspace );

	// Phong calculations
	vec3 n = normalize( Normal_cameraspace );
	vec3 l = normalize( LightDirection_cameraspace );
	float cosTheta = clamp( dot( n,l ), 0,1 );
	
	vec3 E = normalize(EyeDirection_cameraspace);
	vec3 R = reflect(-l,n);
	float cosAlpha = clamp( dot( E,R ), 0,1 );
	
	color = MaterialAmbientColor
		+ MaterialDiffuseColor * LightColor * LightPower * cosTheta / (distance*distance)
		+ MaterialSpecularColor * LightColor * LightPower * pow(cosAlpha,5) / (distance*distance);

}
