#include "Vec3.h"

using namespace sc;

Vec3::Vec3(){
    set(0,0,0);
}
Vec3::Vec3(double x, double y, double z){
    set(x,y,z);
}
Vec3::~Vec3(){}

double Vec3::x() const {
    return m_data[0];
}
double Vec3::y() const {
    return m_data[1];
}
double Vec3::z() const {
    return m_data[2];
}

Vec3 Vec3::cross(const Vec3& u) const {
    Vec3 result;
    result.set( (u.x() * m_data[2]) - (u.x() * m_data[1]),
                (u.y() * m_data[0]) - (u.y() * m_data[2]),
                (u.z() * m_data[1]) - (u.z() * m_data[0])  );
    return result;
}

double Vec3::dot(const Vec3& u) const {
    return  u.x() * m_data[0] +
            u.y() * m_data[1] +
            u.z() * m_data[2];
}

void Vec3::set(double x, double y, double z){
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

void Vec3::copy(const Vec3& v){
    this->m_data[0] = v.x();
    this->m_data[1] = v.y();
    this->m_data[2] = v.z();
}

Vec3 Vec3::add(const Vec3& v) const {
    return Vec3(v.x()+m_data[0],
                v.y()+m_data[1],
                v.z()+m_data[2]);
}

Vec3 Vec3::subtract(const Vec3& v) const {
    return Vec3(-v.x()+m_data[0],
                -v.y()+m_data[1],
                -v.z()+m_data[2]);
}
