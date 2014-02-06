#include "Connector.h"
#include "LockConstraint.h"
#include "BallJointConstraint.h"
#include "Equation.h"
#include "stdio.h"
#include "stdlib.h"

using namespace sc;

LockConstraint::~LockConstraint(){}

LockConstraint::LockConstraint(
    Connector* connA,
    Connector* connB,
    const Vec3& localAnchorA,
    const Vec3& localAnchorB,
    const Quat& localOrientationA,
    const Quat& localOrientationB
) : BallJointConstraint(connA,connB,localAnchorA,localAnchorB) {
    addEquation(&m_xr);
    addEquation(&m_yr);
    addEquation(&m_zr);

    // 3 equations, one for each rotational DOF
    m_xr.setG( 0, 0, 0,-1, 0, 0, 0, 0, 0, 1, 0, 0);
    m_yr.setG( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0, 1, 0);
    m_zr.setG( 0, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0, 1);

    m_xr.setConnectors(connA,connB);
    m_yr.setConnectors(connA,connB);
    m_zr.setConnectors(connA,connB);
    m_xr.setDefault();
    m_yr.setDefault();
    m_zr.setDefault();
}

void LockConstraint::update(){
    // Todo fix
    // m_xr.setDefaultViolation();
    // m_yr.setDefaultViolation();
    // m_zr.setDefaultViolation();

    // Rotational violation:
    //      g = ni .dot( nj )  (3 rotational DOFs)
    // where
    //      ni is a unit vector that rotates with body i
    //      nj is a unit vector that rotates with body j

    Vec3 x(1,0,0);
    Vec3 y(0,1,0);
    Vec3 z(0,0,1);
    Vec3 zero(0,0,0);

    Vec3 xi = m_connA->m_quaternion . multiplyVector( x );
    Vec3 yi = m_connA->m_quaternion . multiplyVector( y );
    Vec3 zi = m_connA->m_quaternion . multiplyVector( z );

    Vec3 xj = m_connB->m_quaternion . multiplyVector( x );
    Vec3 yj = m_connB->m_quaternion . multiplyVector( y );
    Vec3 zj = m_connB->m_quaternion . multiplyVector( z );



    m_xr.setG(zero, zj.cross(yi), zero, yi.cross(zj));
    m_xr.setViolation(yi.dot(zj));

    m_yr.setG(zero, xj.cross(zi), zero, zi.cross(xj));
    m_yr.setViolation(zi.dot(xj));

    m_zr.setG(zero, yj.cross(xi), zero, xi.cross(yj));
    m_zr.setViolation(xi.dot(yj));

    BallJointConstraint::update();
}
