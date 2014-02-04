#include "JacobianElement.h"

using namespace sc;

JacobianElement::JacobianElement(){}
JacobianElement::~JacobianElement(){}

double JacobianElement::multiply(const Vec3& spatial, const Vec3& rotational){
    return 0;
}
double JacobianElement::multiply(const JacobianElement& e){
    return 0;
}
