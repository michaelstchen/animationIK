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
        viewMat = glm::scale(viewMat, vec3(1.05f, 1.05f, 1.05f));
    } else if (key == 45) {
        // 45 represents '+' key
        viewMat = glm::scale(viewMat, vec3(0.95f, 0.95f, 0.95f));
    } else if (key == 99) {
        wireframemode = !wireframemode;
    }
}

void specialKeys(int key, int x, int y) {

    if (key == GLUT_KEY_RIGHT) {
        viewMat = glm::translate(viewMat, right * 0.25f);
    } else if (key == GLUT_KEY_LEFT) {
        viewMat = glm::translate(viewMat, -right * 0.25f);
    } else if (key == GLUT_KEY_UP) {
        viewMat = glm::translate(viewMat, up * 0.25f);
    } else if (key == GLUT_KEY_DOWN) {
        viewMat = glm::translate(viewMat, -up * 0.25f);
    }
}

glm::mat4 getViewMat() {
    return viewMat;
}

bool isWireFrame() {
    return wireframemode;
}
