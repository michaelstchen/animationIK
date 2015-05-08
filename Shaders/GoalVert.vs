#version 330 core

// Input vertex data, different for all executions of this shader.
in vec3 position;

void main(){

    gl_Position.xyz = position;
    gl_Position.w = 1.0;

}
