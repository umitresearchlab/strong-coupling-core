#include "Quat.h"
#include "Vec3.h"
#include "math.h"
#include "stdio.h"

using namespace sc;

Quat::Quat(){
    m_data[0] = 0; // x
    m_data[1] = 0; // y
    m_data[2] = 0; // z

    m_data[3] = 1; // w
}
Quat::Quat( double x, double y, double z, double w){
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;

    m_data[3] = w;
}
Quat::~Quat(){}

double Quat::x() const {
    return m_data[0];
}
double Quat::y() const {
    return m_data[1];
}
double Quat::z() const {
    return m_data[2];
}
double Quat::w() const {
    return m_data[3];
}

Vec3 Quat::getAxis() const {
    return Vec3(m_data[0],m_data[1],m_data[2]);
};

Quat Quat::multiply(const Quat& q) const {
    Quat result;
    Vec3 qAxis = q.getAxis();
    Vec3 pAxis = this->getAxis();
    double x = m_data[0];
    double y = m_data[1];
    double z = m_data[2];
    double w = m_data[3];

    result[3] = w * q[3] - pAxis.dot(qAxis);

    Vec3 vaxvb = this->getAxis().cross(q.getAxis());

    result[0] = q[3] * q[0] + w*x + vaxvb[0];
    result[1] = q[3] * q[1] + w*y + vaxvb[1];
    result[2] = q[3] * q[2] + w*z + vaxvb[2];

    return result;

}

Vec3 Quat::multiplyVector(const Vec3& v) const {
    Vec3 result;

    double  x = v.x(),
            y = v.y(),
            z = v.z();

    double  qx = this->x(),
            qy = this->y(),
            qz = this->z(),
            qw = this->w();

    // q*v
    double  ix =  qw * x + qy * z - qz * y,
            iy =  qw * y + qz * x - qx * z,
            iz =  qw * z + qx * y - qy * x,
            iw = -qx * x - qy * y - qz * z;

    result.set( ix * qw + iw * -qx + iy * -qz - iz * -qy,
                iy * qw + iw * -qy + iz * -qx - ix * -qz,
                iz * qw + iw * -qz + ix * -qy - iy * -qx);

    return result;
}

void Quat::normalize(){
    double l = sqrt(m_data[0]*m_data[0]+m_data[1]*m_data[1]+m_data[2]*m_data[2]+m_data[3]*m_data[3]);
    if ( l == 0 ) {
        m_data[0] = 0;
        m_data[1] = 0;
        m_data[2] = 0;
        m_data[3] = 1;
    } else {
        l = 1 / l;
        m_data[0] *= l;
        m_data[1] *= l;
        m_data[2] *= l;
        m_data[3] *= l;
    }
}

void Quat::set(double x, double y, double z, double w){
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
    m_data[3] = w;
}

void Quat::copy(const Quat& q){
    m_data[0] = q[0];
    m_data[1] = q[1];
    m_data[2] = q[2];
    m_data[3] = q[3];
}
