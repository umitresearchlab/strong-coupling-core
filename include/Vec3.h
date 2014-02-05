#ifndef SCVEC3_H
#define SCVEC3_H

#include "stdlib.h"

namespace sc {

class Vec3 {
private:
    double m_data[3];

public:
    Vec3();
    Vec3(double,double,double);
    virtual ~Vec3();

    Vec3 cross(const Vec3& u) const;
    Vec3 add(const Vec3& v) const;
    Vec3 subtract(const Vec3& u) const;
    double dot(const Vec3& u) const;
    double x() const;
    double y() const;
    double z() const;
    void set(double x, double y, double z);
    void copy(const Vec3&);

    double& operator[] (const int i) {
        return m_data[i];
    };

    void operator += (const Vec3& v) {
        this->m_data[0] += v.x();
        this->m_data[1] += v.y();
        this->m_data[2] += v.z();
    };

    Vec3 operator * (float s) const {
        return Vec3(this->m_data[0]*s,
                    this->m_data[1]*s,
                    this->m_data[2]*s);
    };

    Vec3 operator + (const Vec3 v) const {
        return Vec3(this->m_data[0] + v.x(),
                    this->m_data[1] + v.y(),
                    this->m_data[2] + v.z());
    };

    Vec3 operator - (const Vec3 v) const {
        return Vec3(this->m_data[0] - v.x(),
                    this->m_data[1] - v.y(),
                    this->m_data[2] - v.z());
    };
};

}

#endif
