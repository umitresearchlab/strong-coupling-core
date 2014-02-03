#include "Quat.h"
#include "Vec3.h"
#include "math.h"

namespace sc {

double * Quat::alloc(){
    double * q = (double*)malloc(4*sizeof(double));
    Quat::set(q,0,0,0,0);
    return q;
}

void Quat::free(double* q){
    free(q);
}

void Quat::multiply(double* p, double *q, double* target){
    double vaxvb[3];
    /*
    va.set(this[0],this[1],this[2]);
    vb.set(q[0],q[1],q[2]);
    */

    target[3] = p[3]*q[3] - Vec3::dot(p,q);

    Vec3::cross(p,q,vaxvb);

    target[0] = q[3] * q[0] + p[3]*p[0] + vaxvb[0];
    target[1] = q[3] * q[1] + p[3]*p[1] + vaxvb[1];
    target[2] = q[3] * q[2] + p[3]*p[2] + vaxvb[2];

}

void Quat::normalize(double* q, double *out){
    double l = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if ( l == 0 ) {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        out[3] = 0;
    } else {
        l = 1 / l;
        out[0] *= l;
        out[1] *= l;
        out[2] *= l;
        out[3] *= l;
    }
}

void Quat::set(double* out, double x, double y, double z, double w){
    out[0] = x;
    out[1] = y;
    out[2] = z;
    out[3] = w;
}

void Quat::copy(double* out, double* q){
    out[0] = q[0];
    out[1] = q[1];
    out[2] = q[2];
    out[3] = q[3];
}

}
