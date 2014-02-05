#include "Connector.h"
#include "BallJointConstraint.h"
#include "Equation.h"
#include "stdio.h"
#include "stdlib.h"

using namespace sc;

BallJointConstraint::BallJointConstraint(
    Connector* connA,
    Connector* connB,
    const Vec3& localAnchorA,
    const Vec3& localAnchorB
) : Constraint(connA,connB){
    addEquation(&m_x);
    addEquation(&m_y);
    addEquation(&m_z);

    m_localAnchorA.copy(localAnchorA);
    m_localAnchorB.copy(localAnchorB);

    for(int i=0; i<getNumEquations(); i++){
        getEquation(i)->setConnectors(connA,connB);
        getEquation(i)->setDefault();
    }
}

BallJointConstraint::~BallJointConstraint(){}

void BallJointConstraint::update(){


    // 3 equations

    // Gx = [ -x   -(rj x x)   x   (ri x x)]
    // Gy = [ -y   -(rj x y)   y   (ri x y)]
    // Gz = [ -z   -(rj x z)   z   (ri x z)]

    // or:

    // G = [    I   rj*    -I   -ri*    ]

    Vec3 x(1,0,0);
    Vec3 y(0,1,0);
    Vec3 z(0,0,1);

    // Get world oriented attachment vectors
    Vec3 ri = m_connA->m_quaternion.multiplyVector(m_localAnchorA);
    Vec3 rj = m_connB->m_quaternion.multiplyVector(m_localAnchorB);

    // gvec = ( xj + rj - xi - ri )
    // gx = gvec . dot ( x )
    // gy = gvec . dot ( y )
    // gz = gvec . dot ( z )
    Vec3 gvec = m_connB->m_position + rj - m_connA->m_position - ri;
    m_x.setViolation(gvec.dot(x));
    m_y.setViolation(gvec.dot(y));
    m_z.setViolation(gvec.dot(z));

    Vec3 ri_x_x = ri.cross(x);
    Vec3 ri_x_y = ri.cross(y);
    Vec3 ri_x_z = ri.cross(z);
    Vec3 rj_x_x = rj.cross(x);
    Vec3 rj_x_y = rj.cross(y);
    Vec3 rj_x_z = rj.cross(z);

    m_x.setG(-1, 0, 0, -rj_x_x.x(), -rj_x_x.y(), -rj_x_x.z(),    1, 0, 0,  ri_x_x.x(), ri_x_x.y(), ri_x_x.z());
    m_y.setG( 0,-1, 0, -rj_x_y.x(), -rj_x_y.y(), -rj_x_y.z(),    0, 1, 0,  ri_x_y.x(), ri_x_y.y(), ri_x_y.z());
    m_z.setG( 0, 0,-1, -rj_x_z.x(), -rj_x_z.y(), -rj_x_z.z(),    0, 0, 1,  ri_x_z.x(), ri_x_z.y(), ri_x_z.z());
}
