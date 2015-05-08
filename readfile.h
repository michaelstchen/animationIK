#ifndef READFILE_H
#define READFILE_H

#include <vector>
using namespace std;

#define PI 3.141592f
#define degToRad(x) x*(3.141592f/180.0f)
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/vector_angle.hpp>
using namespace glm;


bool loadOBJ(char* filename,
             vector<vec3> & out_vertices,
             vector<vec3> & out_normals,
             float len);

bool loadOBJ(char* filename,
             vector<vec3> & out_vertices,
             vector<vec3> & out_normals);


#endif
