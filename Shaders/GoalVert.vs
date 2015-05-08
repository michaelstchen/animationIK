#version 330 core

uniform mat4 MVP;

// Input vertex data, different for all executions of this shader.
in vec3 position;

void main(){

    gl_Position = MVP * vec4(position, 1);

}
