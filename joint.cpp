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

Matrix4f glm_to_eigen(mat4 m) {
    Matrix4f model;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            model(j, i) = m[i][j];
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

    if (angle == 0.0f) {
        return Matrix4f::Identity();
    }

    Vector4f r_norm;
    r_norm = r / angle;
    Matrix4f r_cross = crossMat(r_norm);

    Matrix4f rot = r_norm * r_norm.transpose()
        + r_cross * sin(angle)
        - r_cross * r_cross * cos(angle);

    rot(3, 3) = 1.0f;
    rot(2, 3) = 0.0f; rot(1, 3) = 0.0f; rot(0, 3) = 0.0f;
    rot(3, 2) = 0.0f; rot(3, 1) = 0.0f; rot(3, 0) = 0.0f;
    
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
    Joint* end = this;
    while (end->next != NULL) {
        end = end->next;
    }

    Vector4f length_v;
    length_v << end->len, 0.0, 0.0, 1.0;
    Vector4f p_n = end->R() * length_v;

    Matrix4f X_ni = Matrix4f::Identity();
    for (Joint* n = this; n->next !=NULL; n = n->next) {
        X_ni = X_ni * n->X();
    }

    Matrix4f R_i0 = Matrix4f::Identity();
    for (Joint* n = prev; n != NULL; n = n->prev) {
        //R_i0 = n->R() * R_i0;
        R_i0 = n->R() * R_i0;
    }

    Vector4f p_e = X_ni * p_n;
    Matrix4f jacob = -R_i0 * crossMat(p_e);

    return jacob.block(0, 0, 3, 3);
}
MatrixXf jacobian2(vector<Joint*> & skel) {
    MatrixXf jac(3, 12);

    float eps = 0.00001f;
    int index = 0;
    for (int i = 0; i < skel.size(); i++) {
        for (int j = 0; j < 3; j++) {
            skel[i]->rot(j) += eps;
            Vector4f plus = getEffector(skel);
            skel[i]->rot(j) -= 2.0f * eps;

            Vector4f minus = getEffector(skel);
            skel[i]->rot(j) += eps;

            Vector3f der = ((plus - minus) / (2.0f * eps)).block(0,0,3,1);
            jac.col(index) = ((plus - minus) / (2.0f * eps)).block(0,0,3,1);
            index++;

        }
    }

    return jac;
}

MatrixXf jacobian(vector<Joint*> & skel) {
    //MatrixXf jac(3, 12);
    //jac << skel[0]->J(), skel[1]->J(), skel[2]->J(), skel[3]->J();

    MatrixXf jac(3, 6);
    jac << skel[0]->J(), skel[1]->J();    
    return jac;   
}

Vector4f getEffector(vector<Joint*> & skel) {
    Vector4f length_v;
    length_v << skel[3]->len, 0.0, 0.0, 1.0;
    return glm_to_eigen(skel[3]->modelMat()) * length_v;

    // length_v << skel[1]->len, 0.0, 0.0, 1.0;
    // return glm_to_eigen(skel[1]->modelMat()) * length_v;
}


int IKsolver(vector<Joint*> & skel, Vector4f & goal, float delta) {
    Vector3f currDist = (goal - getEffector(skel)).block(0,0,3,1);
    if (currDist.norm() < 0.01) {
        return 0;
    }

    Vector3f dp = currDist;
    dp.normalize();
    dp = dp * delta;

    MatrixXf dr = jacobian2(skel).jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(dp);

    for (int i = 0; i < skel.size(); i++) {
        skel[i]->rot(0) += dr(i*3 + 0);
        skel[i]->rot(1) += dr(i*3 + 1);
        skel[i]->rot(2) += dr(i*3 + 2);
    }

    // Vector3f newDist = (goal - getEffector(skel)).block(0,0,3,1);
    // if (newDist.norm() > currDist.norm()) {
    //     cout << "\nin herein herein herein herein herein here\n";
    //     for (int i = 0; i < skel.size(); i++) {
    //         skel[i]->rot(0) -= dr(i*3 + 0);
    //         skel[i]->rot(1) -= dr(i*3 + 1);
    //         skel[i]->rot(2) -= dr(i*3 + 2);
    //     }
        
    //     int retval = IKsolver(skel, goal, delta / 5.0f);
    //     return retval;
    // }

    return 1;
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


// int main(int argc, char **argv) {
//     Joint* joint0; Joint* joint1;
//     Joint* joint2; Joint* joint3;
    
//     joint0 = new Joint(NULL, joint1, 1.0f);
//     joint1 = new Joint(joint0, joint2, 2.0f);
//     joint2 = new Joint(joint1, joint3, 3.0f);
//     joint3 = new Joint(joint2, NULL, 4.0f);
    
//     joint0->next = joint1;
//     joint1->next = joint2;
//     joint2->next = joint3;

//     vector<Joint*> skel;
//     skel.push_back(joint0);
//     skel.push_back(joint1);
//     skel.push_back(joint2);
//     skel.push_back(joint3);

//     // Vector4f goal;
//     // goal << 7.0f, 0.0f, 5.0f, 1.0f;

//     // for (int i = 0; i < 4; i++) {
//     //     IKsolver(skel, goal, 0.01f);
        
//     // }
    
// }
