#include "scVec3.h"

namespace sc {

double * Vec3::alloc(){
    return (double*)malloc(3*sizeof(double));
}

void Vec3::free(double* v){
    free(v);
}

double Vec3::dot(double* u, double *v){
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
}

void Vec3::cross(double* u, double *v, double* target){
    target[0] = (u[1] * v[2]) - (u[2] * v[1]);
    target[1] = (u[2] * v[0]) - (u[0] * v[2]);
    target[2] = (u[0] * v[1]) - (u[1] * v[0]);
}

void Vec3::set(double* out, double x, double y, double z){
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

void Vec3::add(double* out, double * u, double * v){
    out[0] = v[0] + u[0];
    out[1] = v[1] + u[1];
    out[2] = v[2] + u[2];
}

void Vec3::subtract(double* out, double * u, double * v){
    out[0] = u[0] - v[0];
    out[1] = u[1] - v[1];
    out[2] = u[2] - v[2];
}

void Vec3::copy(double* out, double* v){
    out[0] = v[0];
    out[1] = v[1];
    out[2] = v[2];
}

void Vec3::scale(double* out, double scalar){
    out[0] *= scalar;
    out[1] *= scalar;
    out[2] *= scalar;
}

void Vec3::multiplyElementWise(double* out, double * u, double * v){
    out[0] = v[0] * u[0];
    out[1] = v[1] * u[1];
    out[2] = v[2] * u[2];
}

}
