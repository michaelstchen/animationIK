#include "joint.h"


Joint::Joint(Joint* p, float length) {
    
    parent = p;
    len = length;

    if (parent == NULL) {
        Model = mat4(1.0f);
    } else {
        Model = glm::translate(parent->Model, vec3(0, 0, parent->len));
    }

}
