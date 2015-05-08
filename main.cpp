#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
using namespace Eigen;

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#define degToRad(x) x*(3.141592f/180.0f)
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "shader.h"
#include "joint.h"
#include "readfile.h"
#include "keylistener.h"

//****************************************************
// Global variables
//****************************************************
int windowWidth = 800;
int windowHeight = 600;

GLuint smoothprogram;
GLuint flatprogram;
GLuint VertexArrayID;

GLuint vertexbuffer;
GLuint normalbuffer;

std::vector< vec3 > goal_verts;
std::vector< vec3 > vertices;
std::vector< vec3 > normals;

std::vector< Joint* > skeleton;
Joint* joint0; Joint* joint1;
Joint* joint2; Joint* joint3;

int currgoalInd;

mat4 MVP;

//****************************************************
// Callback Functions
//****************************************************
void renderScene() {

    // clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Choose shader
    GLuint programID = flatprogram;
    glUseProgram(programID);

    // Get a handles
    GLuint MatrixID = glGetUniformLocation(programID, "MVP");
    GLuint ViewMatrixID = glGetUniformLocation(programID, "V");
    GLuint ModelMatrixID = glGetUniformLocation(programID, "M");
    GLuint LightID = glGetUniformLocation(programID,
                                          "LightPosition_worldspace");

    mat4 Projection = glm::perspective(degToRad(45.0f), 4.0f/3.0f,
                                       0.1f, 100.0f);
    // Camera matrix
    mat4 View = getViewMat();

    vec3 lightPos = vec3(5, 5, 10);
    glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);

    // 1st attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

    // 2nd attribute buffer : normals
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0 );
    
    if (isWireFrame()) {
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    } else {
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    }

    vec3 goal_glm = goal_verts[currgoalInd];
    Vector4f goal_eigen;
    goal_eigen << goal_glm[0], goal_glm[1], goal_glm[2], 1.0f;

    if (IKsolver(skeleton, goal_eigen, 0.03f) == 0) {
        if (currgoalInd >= goal_verts.size() - 1) {
            currgoalInd = 0;
        } else {
            currgoalInd++;
        }
    }

    // Draw the joints
    for (int i = 0; i < skeleton.size(); i++) {
        // Model matrix
        mat4 Model = skeleton[i]->modelMat();
        // Our ModelViewProjection : multiplication of our 3 matrices
        MVP = Projection * View * Model;

        // Send our transformation to the currently bound shader, 
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &Model[0][0]);
        glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &View[0][0]);

        glDrawArrays(GL_TRIANGLES, 0, vertices.size());
    }

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);

    // swap buffers
    glutSwapBuffers();
}


void windowResize(int w, int h) {
    /* Don't change the viewport */
}

void populateGoalVerts(vector<vec3> & v) {
    for (float theta = 0.0f; theta < 2.0f * PI + 0.1f; theta += 0.1f) {
        v.push_back(vec3(10.0f, 10.0f*cos(theta), 10.0f*sin(theta)));
    }
    currgoalInd = 0;
}

//****************************************************
// Program Start Point
//****************************************************
int main(int argc, char **argv) {

    // initiate GLUT and create window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

    glutInitContextVersion(3, 3);
    glutInitContextProfile(GLUT_CORE_PROFILE );
    glutInitContextFlags(GLUT_DEBUG);

    glutInitWindowPosition(0, 0);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow("CS184 Assignment 4: Inverse Kinematics");

    // register callbacks
    glutDisplayFunc(renderScene);
    glutReshapeFunc(windowResize);
    glutIdleFunc(renderScene);

    // user input processing
    glutKeyboardFunc(normalKeys);
    glutSpecialFunc(specialKeys);

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    glewInit();

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS); 

    // Cull triangles which normal is not towards the camera
    //glEnable(GL_CULL_FACE);

    // Create and compile our GLSL program from the shaders
    smoothprogram = LoadShaders("Shaders/SmoothVertex.vs",
                            "Shaders/SmoothFragment.fs" );
    flatprogram = LoadShaders("Shaders/FlatVertex.vs",
                            "Shaders/FlatFragment.fs" );

    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    loadOBJ("Inputs/joint.obj", vertices, normals);

    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &normalbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);

    joint0 = new Joint(NULL, joint1, 5.0f);
    joint1 = new Joint(joint0, joint2, 5.0f);
    joint2 = new Joint(joint1, joint3, 5.0f);
    joint3 = new Joint(joint2, NULL, 5.0f);

    skeleton.push_back(joint0);
    skeleton.push_back(joint1);
    skeleton.push_back(joint2);
    skeleton.push_back(joint3);

    populateGoalVerts(goal_verts);

    // enter GLUT event processing loop
    glutMainLoop();

    return 1;

}
