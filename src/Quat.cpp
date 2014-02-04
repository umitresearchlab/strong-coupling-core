#include "Quat.h"
#include "Vec3.h"
#include "math.h"

using namespace sc;

Quat::Quat(){
    m_data[0] = 0;
    m_data[1] = 0;
    m_data[2] = 0;
    m_data[3] = 1;
}
Quat::~Quat(){}

Vec3 Quat::getAxis() const {
    return Vec3(m_data[0],m_data[1],m_data[2]);
};

Quat Quat::multiply(const Quat& p, const Quat& q) const {
    Quat result;
    Vec3 qAxis = q.getAxis();
    Vec3 pAxis = p.getAxis();

    result[3] = p[3]*q[3] - pAxis.dot(qAxis);

    Vec3 vaxvb = p.getAxis().cross(q.getAxis());

    result[0] = q[3] * q[0] + p[3]*p[0] + vaxvb[0];
    result[1] = q[3] * q[1] + p[3]*p[1] + vaxvb[1];
    result[2] = q[3] * q[2] + p[3]*p[2] + vaxvb[2];

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
