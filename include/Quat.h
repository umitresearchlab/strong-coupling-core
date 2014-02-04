#ifndef SCQUAT_H
#define SCQUAT_H

#include "stdlib.h"
#include "Vec3.h"

namespace sc {

class Quat {
private:
    double m_data[4];

public:
    Quat();
    virtual ~Quat();

    double& operator[] (const int i) const  {
        this->m_data[i];
    };

    Quat multiply(const Quat& p, const Quat& q) const;
    Vec3 getAxis() const;
    void normalize();
    void set(double x, double y, double z, double w);
    void copy(const Quat& q);


};

}

#endif
