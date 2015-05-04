#include "joint.h"
#include "readfile.h"


Joint::Joint(Joint* p, float length) {
    
    parent = p;
    len = length;

    Model = mat4(1.0f);

    loadOBJ("Inputs/joint.obj", vertices, normals);

}
