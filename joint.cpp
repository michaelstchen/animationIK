#include <iostream>
#include <cmath>

#include "joint.h"


Joint::Joint(Joint* p, Joint* n, float length) {
    
    prev = p; next = n;
    len = length;
    rot << 0.0f, 0.0f, 0.0f, 0.0f;
}

mat4 eigen_to_glm(Matrix4f m) {
    mat4 model = mat4(1.0f);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            model[i][j] = m(i, j);
        }
    }

    return model;
    
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

    Vector4f r_norm;
    if (angle != 0) {
        r_norm = r / angle;
    } else {
        r_norm.setIdentity();
    }
    
    Matrix4f r_cross = crossMat(r_norm);

    Matrix4f rot = r_norm * r_norm.transpose()
        + r_cross * sin(angle)
        - r_cross * r_cross * cos(angle);

    rot(3, 3) = 1.0f;
    
    return rot;
    
}

Matrix4f Joint::X() {
    Matrix4f transform = R();
    transform(2, 3) = len;
    return transform;
}

Matrix4f Joint::R() {
    return rodriguez(rot);
}

mat4 Joint::modelMat() {
    if (prev == NULL) {
        return eigen_to_glm(X());
    }

    return prev->modelMat() * eigen_to_glm(X());

}




// int main(int argc, char **argv) {
//     return 0;
// }
