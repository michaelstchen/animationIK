#ifndef KEYLISTENER_H
#define KEYLISTENER_H

#define degToRad(x) x*(3.141592f/180.0f)
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

void normalKeys(unsigned char key, int x, int y);
void specialKeys(int key, int x, int y);

mat4 getViewMat();

#endif
