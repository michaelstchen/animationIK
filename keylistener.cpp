#include <stdlib.h>
#include <stdio.h>

#include <GL/glut.h>

#include "keylistener.h"

bool wireframemode = false;

vec3 camPos = vec3(0, 0, 30);
vec3 origin = vec3(10, 0, 0);
vec3 up = vec3(1,0,0);
vec3 right = glm::normalize(glm::cross(origin - camPos, up)); 

mat4 viewMat = glm::lookAt(camPos, origin, up);

void normalKeys(unsigned char key, int x, int y) {
    if (key == 27) {
        exit(0);
    } else if (key == 61) {
        //viewMat = glm::scale(viewMat, vec3(1.05f, 1.05f, 1.05f));
        mat4 rot = glm::scale(mat4(1.0f), vec3(0.95f, 0.95f, 0.95f));
        vec4 camPos4 = rot * vec4(camPos, 1.0f);
        camPos = vec3(camPos4[0], camPos4[1], camPos4[2]);
        viewMat = glm::lookAt(camPos, origin, up);
    } else if (key == 45) {
        //viewMat = glm::scale(viewMat, vec3(0.95f, 0.95f, 0.95f));
        mat4 rot = glm::scale(mat4(1.0f), vec3(1.05f, 1.05f, 1.05f));
        vec4 camPos4 = rot * vec4(camPos, 1.0f);
        camPos = vec3(camPos4[0], camPos4[1], camPos4[2]);
        viewMat = glm::lookAt(camPos, origin, up);
    } else if (key == 99) {
        wireframemode = !wireframemode;
    }
}

void specialKeys(int key, int x, int y) {
    int mod = glutGetModifiers();

    if (key == GLUT_KEY_RIGHT) {
        if (mod == GLUT_ACTIVE_SHIFT) {
            viewMat = glm::translate(viewMat, right * 0.25f);
        } else {
            mat4 rot4 = glm::rotate(mat4(1.0f), degToRad(2.0f), up);
            
        }
    } else if (key == GLUT_KEY_LEFT) {
        if (mod == GLUT_ACTIVE_SHIFT) {
            viewMat = glm::translate(viewMat, -right * 0.25f);
        } else {

        }
    } else if (key == GLUT_KEY_UP) {
        if (mod == GLUT_ACTIVE_SHIFT) {
            viewMat = glm::translate(viewMat, up * 0.25f);
        } else {

        }
    } else if (key == GLUT_KEY_DOWN) {
        if (mod == GLUT_ACTIVE_SHIFT) {
            viewMat = glm::translate(viewMat, -up * 0.25f);
        } else {
            
        }
    }
}

glm::mat4 getViewMat() {
    return viewMat;
}

bool isWireFrame() {
    return wireframemode;
}
