#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include "readfile.h"


bool loadOBJ(char* filename,
             vector<vec3> & out_vertices,
             vector<vec3> & out_normals,
             float len) {

    FILE* finput = fopen(filename, "r");
    if (finput == NULL) {
        fprintf(stderr, "ERROR: cannot open obj file '%s'\n", filename);
        return false;
    }

    vector< vec3 > temp_v;
    bool first_time = true;

    char line[100];
    while(fscanf(finput, "%s", line) != EOF) {

        if (strcmp(line, "v") == 0 ){
            vec3 vertex;
            fscanf(finput, "%f %f %f\n",
                   &vertex.x, &vertex.y, &vertex.z );
            
            if (first_time) {
                vertex[0] = len;
                first_time = false;
            }

            temp_v.push_back(vertex);
        } else if (strcmp(line, "f") == 0 ){
            int vInd[3];
            int matches = fscanf(finput, "%d %d %d\n",
                                 &vInd[0], &vInd[1], &vInd[2]);

            vec3 facenorm = glm::normalize(glm::cross(temp_v[vInd[1] - 1] - temp_v[vInd[0] - 1], temp_v[vInd[2] - 1] - temp_v[vInd[0] - 1]));

            for (int i = 0; i < 3; i++) {
                out_vertices.push_back(temp_v[vInd[i] - 1]);
                out_normals.push_back(facenorm);
            }
            
        } else {
            fprintf(stderr, "ERROR: reading obj line '%s'\n", line);
        }
    }

    return true;
}
