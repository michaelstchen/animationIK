#include <stdlib.h>
#include <stdio.h>

#include <GL/glut.h>

#include <cmath>

#include "keylistener.h"

bool wireframemode = false;

vec3 camPos = vec3(20, 0, 30);
vec3 origin = vec3(10, 0, 0);
vec3 up = vec3(1,0,0);
vec3 right = glm::normalize(glm::cross(origin - camPos, up)); 

mat4 viewMat = glm::lookAt(camPos, origin, up);

void normalKeys(unsigned char key, int x, int y) {
    if (key == 27) {
        exit(0);
    } else if (key == 61) {
        vec3 cam_to_origin = origin - camPos;
        camPos = camPos + (cam_to_origin * 0.05f);
        viewMat = glm::lookAt(camPos, origin, up);
    } else if (key == 45) {
        vec3 cam_to_origin = origin - camPos;
        camPos = camPos - (cam_to_origin * 0.05f);
        viewMat = glm::lookAt(camPos, origin, up);
    } else if (key == 99) {
        wireframemode = !wireframemode;
    }
}

void specialKeys(int key, int x, int y) {
    int mod = glutGetModifiers();

    if (key == GLUT_KEY_RIGHT) {
        if (mod == GLUT_ACTIVE_SHIFT) {
            camPos = camPos + (-right * 0.25f);
            origin = origin + (-right * 0.25f);
            viewMat = glm::lookAt(camPos, origin, up);
        } else {
            camPos = camPos - origin;
            mat4 rot = glm::rotate(mat4(1.0f), degToRad(2.0f), vec3(1.0f, 0.0f, 0.0f));
            right = vec3(rot * vec4(right, 1.0f));
            camPos = vec3(rot * vec4(camPos, 1.0f));
            camPos = camPos + origin;
            viewMat = glm::lookAt(camPos, origin, up);
        }
    } else if (key == GLUT_KEY_LEFT) {
        if (mod == GLUT_ACTIVE_SHIFT) {
            camPos = camPos + (right * 0.25f);
            origin = origin + (right * 0.25f);
            viewMat = glm::lookAt(camPos, origin, up);
        } else {
            camPos = camPos - origin;
            mat4 rot = glm::rotate(mat4(1.0f), degToRad(-2.0f), up);
            right = vec3(rot * vec4(right, 1.0f));
            camPos = vec3(rot * vec4(camPos, 1.0f));
            camPos = camPos + origin;
            viewMat = glm::lookAt(camPos, origin, up);
        }
    } else if (key == GLUT_KEY_UP) {
        if (mod == GLUT_ACTIVE_SHIFT) {
            camPos = camPos + (-up * 0.25f);
            origin = origin + (-up * 0.25f);
            viewMat = glm::lookAt(camPos, origin, up);
        } else {
            camPos = camPos - origin;
            mat4 rot = glm::rotate(mat4(1.0f), degToRad(-2.0f), right);
            up = vec3(rot * vec4(up, 1.0f));
            camPos = vec3(rot * vec4(camPos, 1.0f));
            camPos = camPos + origin;
            viewMat = glm::lookAt(camPos, origin, up);
        }
    } else if (key == GLUT_KEY_DOWN) {
        if (mod == GLUT_ACTIVE_SHIFT) {
            camPos = camPos + (up * 0.25f);
            origin = origin + (up * 0.25f);
            viewMat = glm::lookAt(camPos, origin, up);
        } else {
            camPos = camPos - origin;
            mat4 rot = glm::rotate(mat4(1.0f), degToRad(2.0f), right);
            up = vec3(rot * vec4(up, 1.0f));
            camPos = vec3(rot * vec4(camPos, 1.0f));
            camPos = camPos + origin;
            viewMat = glm::lookAt(camPos, origin, up);
        }
    }
}

glm::mat4 getViewMat() {
    return viewMat;
}

bool isWireFrame() {
    return wireframemode;
}
