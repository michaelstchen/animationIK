#ifndef JOINT_H
#define JOINT_H

#include <vector>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

#define PI 3.141592f
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
    
    Vector4f rot;

    mat4 modelMat();

};



#endif
