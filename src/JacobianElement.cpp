#include "JacobianElement.h"
#include "stdio.h"

using namespace sc;

JacobianElement::JacobianElement(){}
JacobianElement::~JacobianElement(){}

double JacobianElement::multiply(const Vec3& spatial, const Vec3& rotational){
    return spatial.dot(m_spatial) + rotational.dot(m_rotational);
}
double JacobianElement::multiply(const JacobianElement& e){
    return e.getSpatial().dot(getSpatial()) + e.getRotational().dot(getRotational());
}

void JacobianElement::setSpatial(double x, double y, double z){
    m_spatial.set(x,y,z);
}

void JacobianElement::setRotational(double x, double y, double z){
    m_rotational.set(x,y,z);
}

Vec3 JacobianElement::getSpatial() const {
    return m_spatial;
}

Vec3 JacobianElement::getRotational() const {
    return m_rotational;
}

void JacobianElement::print(){
    printf("%g %g %g %g %g %g\n",m_spatial.x(),m_spatial.y(),m_spatial.z(),m_rotational.x(),m_rotational.y(),m_rotational.z());
}
