#include <iostream>
#include <cmath>

#include "joint.h"


Joint::Joint(Joint* p, float length) {
    
    parent = p;
    len = length;

    rot << 0.0f, 0.0f, 0.0f, 0.0f;
}


mat4 Joint::modelMat() {
    //return transl * rot;
    return mat4(1.0f);
}


Matrix4f crossMat(Vector4f & v) {
    
    Matrix4f newCrossMat;
    newCrossMat << 
            0, -v[2],  v[1], 0,
         v[2],     0, -v[0], 0,
        -v[1],  v[0],     0, 0,
            0,     0,     0, 1;       

    return newCrossMat;
}


Matrix4f rodriguez(Vector4f & r) {
    float angle = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);

    Vector4f r_norm = r / angle;
    Matrix4f r_cross = crossMat(r_norm);

    return r_norm * r_norm.transpose()
        + r_cross * sin(angle)
        - r_cross * r_cross * cos(angle);
    
}


int main(int argc, char **argv) {
    Joint joint0 = Joint(NULL, 5.0f);
    Vector4f v; Vector4f r;
    v << 0.0, 1.0, 0.0, 0.0;
    r << 0.0, 0.0, PI / 4.0f, 0.0;
    Matrix4f rotM = rodriguez(r);
    cout << "\n";
    cout << rotM * v;
    cout << "\n";
}
