#ifndef JOINT_H
#define JOINT_H

#include <vector>
using namespace std;

#define degToRad(x) x*(3.141592f/180.0f)
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

class Joint {
 public:
    Joint(Joint* p, float length);

    Joint* parent;
    float len;

    std::vector< vec3 > vertices;
    std::vector< vec3 > normals;
    
    mat4 Model;

};

#endif
