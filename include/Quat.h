#ifndef SCQUAT_H
#define SCQUAT_H

#include "stdlib.h"

namespace sc {

class Quat {
public:
    static double * alloc();
    static void free(double* q);
    static void multiply(double* p, double *q, double *target);
    static void normalize(double* q, double *out);
    static void set(double* q, double x, double y, double z, double w);
    static void copy(double* out, double* q);
};

}

#endif
