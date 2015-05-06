#include <iostream>
#include <stdio.h>
#include <cmath>

#include "joint.h"


Joint::Joint(Joint* p, Joint* n, float length) {
    
    prev = p; next = n;
    len = length;
    rot << 0.0f, 0.0f, 0.0f, 0.0f;
}

// glm matrix accesses are m[col][row] while
// eigen matrix accesses are m[row][col]
mat4 eigen_to_glm(Matrix4f m) {
    mat4 model = mat4(1.0f);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            model[i][j] = m(j, i);
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
    Vector4f length_v;
    length_v << len, 0.0, 0.0, 1.0;
    Vector4f p_i = R() * length_v;

    Matrix4f transform = R();
    transform(0, 3) = p_i(0);
    transform(1, 3) = p_i(1);
    transform(2, 3) = p_i(2);

    return transform;
}

Matrix4f Joint::R() {
    return rodriguez(rot);
}


Matrix3f Joint::J() {
    Vector4f length_v;
    length_v << len, 0.0, 0.0, 1.0;
    Vector4f p_n = R() * length_v;

    Matrix4f X_ni = X();
    for (Joint* n = next; n != NULL; n = n->next) {
        X_ni = X_ni * n->X();
    }

    Matrix4f R_i0 = Matrix4f::Identity();
    for (Joint* n = prev; n != NULL; n = n->prev) {
        R_i0 = R_i0 * prev->R();
    }

    Vector4f p_e = X_ni * p_n;

    Matrix4f jacob = -R_i0 * crossMat(p_e);

    return jacob.block(1, 1, 3, 3);
}


mat4 modelMatHelper(Joint & j) {
    if (j.prev == NULL) {
        return eigen_to_glm(j.X());
    }

    return modelMatHelper(*j.prev) * eigen_to_glm(j.X());
}


mat4 Joint::modelMat() {
    if (prev == NULL) {
        return eigen_to_glm(R());
    }

    mat4 toWorld = modelMatHelper(*prev);

    return toWorld * eigen_to_glm(R());

}


int main(int argc, char **argv) {
    Joint* joint0; Joint* joint1;
    Joint* joint2; Joint* joint3;
    
    joint0 = new Joint(NULL, joint1, 1.0f);
    joint1 = new Joint(joint0, joint2, 2.0f);
    joint2 = new Joint(joint1, joint3, 3.0f);
    joint3 = new Joint(joint2, NULL, 4.0f);
    
    joint0->next = joint1;
    joint1->next = joint2;
    joint2->next = joint3;
    joint3->prev = joint2;
    joint2->prev = joint1;
    joint1->prev = joint0;

    cout << joint0->X();
    cout << "\n";
    cout << "\n";
    cout << joint0->next->X();
    cout << "\n";
    cout << "\n";
    cout << joint0->next->next->X();
    cout << "\n";
    cout << "\n";
    cout << joint0->next->next->next->X();
    cout << "\n";
    cout << "\n";
    cout << joint0->J();

    
}
